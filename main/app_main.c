#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/jpeg_encode.h"
#include "usb_device_uvc.h"
#include "thermal.h"
#include "sensor.h"
#include "camera_pins.h"
#include "driver/gpio.h"

#define JPEG_CAPACITY (IMAGE_BYTES)
static const char *TAG="thermalcam";
static SemaphoreHandle_t state_lock,jpeg_lock;
static QueueHandle_t capture_queue;
static thermal_frame_t latest;
static thermal_mode_t mode;
static bool acquisition_error=true;
static uvc_fb_t published;
static uint8_t *rgb,*encoded;
static size_t rgb_capacity,jpeg_capacity;
static jpeg_encoder_handle_t encoder;

/* The I2C owner never performs full-frame thermal calculations. */
static struct {
    uint32_t good, rejected, queue_drops;
    uint32_t max_capture_ms, max_read_us, max_calculate_us, opposite_ready;
} counters;

static void sensor_task(void *arg)
{
    (void)arg;
    sensor_pair_t *snapshot=malloc(sizeof(*snapshot));
    assert(snapshot);
    bool configured=false;
    unsigned consecutive_bus_errors=0;
    for(;;) {
        if(!configured) {
            int rc=sensor_configure();
            if(rc) {
                ESP_LOGW(TAG,"Sensor configuration failed (%d); retrying",rc);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            configured=true;
            ESP_LOGI(TAG,"MLX90640 configured: 18-bit, 32 subpages/s (up to 16 full fps), 400 kHz I2C");
        }
        int64_t start=esp_timer_get_time();
        sensor_diag_t diag;
        bool ok=sensor_capture(snapshot,&diag);
        uint32_t capture_ms=(uint32_t)((esp_timer_get_time()-start)/1000);
        bool sent=ok && xQueueSend(capture_queue,snapshot,0)==pdTRUE;

        xSemaphoreTake(state_lock,portMAX_DELAY);
        if(capture_ms>counters.max_capture_ms) counters.max_capture_ms=capture_ms;
        if(diag.read_us>counters.max_read_us) counters.max_read_us=diag.read_us;
        counters.opposite_ready+=diag.opposite_ready;
        if(!ok) counters.rejected++;
        else if(!sent) counters.queue_drops++;
        xSemaphoreGive(state_lock);
        if(!ok) {
            /* Missed subpages are sampling losses, NOT sensor failures.
             * Reconfigure only after successive transport/control faults. */
            bool bus_fault=(strstr(diag.failure,"I2C")!=NULL) ||
                           (strstr(diag.failure,"deadline")!=NULL) ||
                           (strcmp(diag.failure,"sensor changed to chess mode")==0);
            if(bus_fault) {
                if(++consecutive_bus_errors>=3) {
                    configured=false;
                    consecutive_bus_errors=0;
                }
            } else consecutive_bus_errors=0;
            ESP_LOGW(TAG,"Pair rejected: %s; capture=%lu ms read=%lu us wait=%lu us gap=%lu us page=%d",
                     diag.failure,(unsigned long)capture_ms,
                     (unsigned long)diag.read_us,(unsigned long)diag.ready_wait_us,
                     (unsigned long)diag.page_gap_us,diag.last_page);
        } else {
            consecutive_bus_errors=0;
            if(!sent) ESP_LOGW(TAG,"Raw pair queue full; discarded capture");
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void process_task(void *arg)
{
    (void)arg;
    sensor_pair_t *snapshot=malloc(sizeof(*snapshot));
    thermal_frame_t *candidate=calloc(1,sizeof(*candidate));
    assert(snapshot && candidate);
    int64_t stats_since=esp_timer_get_time();
    uint32_t sequence=0;
    for(;;) {
        if(xQueueReceive(capture_queue,snapshot,portMAX_DELAY)!=pdTRUE) continue;
        uint32_t calculation_us=0;
        bool ok=sensor_process(snapshot,candidate,&calculation_us);
        xSemaphoreTake(state_lock,portMAX_DELAY);
        if(calculation_us>counters.max_calculate_us)
            counters.max_calculate_us=calculation_us;
        if(ok) {
            candidate->sequence=++sequence;
            latest=*candidate;
            acquisition_error=false;
            ++counters.good;
        } else {
            ++counters.rejected;
        }
        int64_t now=esp_timer_get_time();
        if(now-stats_since>=5000000) {
            uint32_t dt=(uint32_t)(now-stats_since);
            uint32_t fps10=(uint32_t)((uint64_t)counters.good*10000000ULL/dt);
            ESP_LOGI(TAG,"Acquisition: %u.%u full fps good=%lu rejected=%lu dropped=%lu max_pair=%lu ms max_read=%lu us max_calc=%lu us next_ready=%lu",
                     fps10/10,fps10%10,
                     (unsigned long)counters.good,(unsigned long)counters.rejected,
                     (unsigned long)counters.queue_drops,
                     (unsigned long)counters.max_capture_ms,
                     (unsigned long)counters.max_read_us,
                     (unsigned long)counters.max_calculate_us,
                     (unsigned long)counters.opposite_ready);
            memset(&counters,0,sizeof(counters));
            stats_since=now;
        }
        xSemaphoreGive(state_lock);
        if(!ok) ESP_LOGW(TAG,"Thermal calculation rejected raw pair");
    }
}

static void button_task(void *arg)
{
    (void)arg;
    bool last=false, stable=false;
    int64_t changed=0;
    for(;;) {
        int64_t now=esp_timer_get_time();
        bool down=!gpio_get_level(PIN_BUTTON);
        if(down!=last) { last=down; changed=now; }
        if(now-changed>=50000 && stable!=down) {
            stable=down;
            if(down) {
                xSemaphoreTake(state_lock,portMAX_DELAY);
                mode=(mode+1)%MODE_COUNT;
                xSemaphoreGive(state_lock);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t stream_start(uvc_format_t fmt,int w,int h,int rate,void *ctx)
{
    (void)ctx;
    if(fmt!=UVC_FORMAT_JPEG || w!=IMAGE_W || h!=IMAGE_H || rate!=16)
        return ESP_ERR_NOT_SUPPORTED;
    ESP_LOGI(TAG,"UVC started: %dx%d at %d fps",w,h,rate);
    return ESP_OK;
}
static void stream_stop(void *ctx) { (void)ctx; ESP_LOGI(TAG,"UVC stopped"); }
static uvc_fb_t *stream_get(void *ctx)
{
    (void)ctx;
    if(xSemaphoreTake(jpeg_lock,pdMS_TO_TICKS(20))!=pdTRUE) return NULL;
    if(!published.len) { xSemaphoreGive(jpeg_lock); return NULL; }
    /* Driver copies to its own transfer buffer, then immediately calls return. */
    return &published;
}
static void stream_return(uvc_fb_t *fb,void *ctx)
{
    (void)fb; (void)ctx; xSemaphoreGive(jpeg_lock);
}

void app_main(void)
{
    state_lock=xSemaphoreCreateMutex(); jpeg_lock=xSemaphoreCreateMutex();
    capture_queue=xQueueCreate(4,sizeof(sensor_pair_t));
    assert(state_lock && jpeg_lock && capture_queue);
    jpeg_encode_memory_alloc_cfg_t input={.buffer_direction=JPEG_ENC_ALLOC_INPUT_BUFFER};
    jpeg_encode_memory_alloc_cfg_t output={.buffer_direction=JPEG_ENC_ALLOC_OUTPUT_BUFFER};
    rgb=jpeg_alloc_encoder_mem(IMAGE_BYTES,&input,&rgb_capacity);
    encoded=jpeg_alloc_encoder_mem(JPEG_CAPACITY,&output,&jpeg_capacity);
    published.buf=heap_caps_malloc(JPEG_CAPACITY,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    uint8_t *transfer_buffer=heap_caps_malloc(JPEG_CAPACITY,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    assert(rgb && encoded && published.buf && transfer_buffer);
    published.width=IMAGE_W; published.height=IMAGE_H; published.format=UVC_FORMAT_JPEG;
    jpeg_encode_engine_cfg_t engine_cfg={.timeout_ms=200};
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&engine_cfg,&encoder));
    gpio_config_t button={.pin_bit_mask=1ULL<<PIN_BUTTON,.mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&button));
    ESP_ERROR_CHECK(sensor_bus_init());
    uvc_device_config_t uvc={.uvc_buffer=transfer_buffer,.uvc_buffer_size=JPEG_CAPACITY,
        .start_cb=stream_start,.stop_cb=stream_stop,.fb_get_cb=stream_get,.fb_return_cb=stream_return};
    ESP_ERROR_CHECK(uvc_device_config(0,&uvc));
    ESP_ERROR_CHECK(uvc_device_init());
    BaseType_t task_ok=xTaskCreate(sensor_task,"sensor",12288,NULL,5,NULL);
    ESP_ERROR_CHECK(task_ok==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
    task_ok=xTaskCreate(process_task,"thermal_calc",12288,NULL,4,NULL);
    ESP_ERROR_CHECK(task_ok==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
    task_ok=xTaskCreate(button_task,"button",4096,NULL,4,NULL);
    ESP_ERROR_CHECK(task_ok==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
    thermal_frame_t *view=calloc(1,sizeof(*view)); assert(view);
    jpeg_encode_cfg_t cfg={.width=IMAGE_W,.height=IMAGE_H,.src_type=JPEG_ENCODE_IN_FORMAT_RGB888,
        .sub_sample=JPEG_DOWN_SAMPLING_YUV444,.image_quality=80};
    TickType_t next=xTaskGetTickCount();
    for(;;) {
        bool fault; thermal_mode_t active_mode;
        xSemaphoreTake(state_lock,portMAX_DELAY);
        fault=acquisition_error; active_mode=mode; *view=latest;
        xSemaphoreGive(state_lock);
        int64_t age=view->valid?(esp_timer_get_time()-view->captured_us)/1000:0;
        bool stale=fault || age>1000;
        thermal_render(rgb,view,active_mode,stale,age>UINT32_MAX?UINT32_MAX:(uint32_t)age);
        uint32_t bytes=0;
        esp_err_t e=jpeg_encoder_process(encoder,&cfg,rgb,IMAGE_BYTES,encoded,jpeg_capacity,&bytes);
        if(e==ESP_OK && bytes>0 && bytes<=JPEG_CAPACITY) {
            xSemaphoreTake(jpeg_lock,portMAX_DELAY);
            memcpy(published.buf,encoded,bytes); published.len=bytes;
            int64_t now=esp_timer_get_time();
            published.timestamp.tv_sec=now/1000000;
            published.timestamp.tv_usec=now%1000000;
            xSemaphoreGive(jpeg_lock);
        } else {
            /* Never silently keep serving old video if encoding fails. */
            xSemaphoreTake(jpeg_lock,portMAX_DELAY); published.len=0; xSemaphoreGive(jpeg_lock);
            ESP_LOGE(TAG,"JPEG failed: %s, %lu bytes",esp_err_to_name(e),(unsigned long)bytes);
        }
        vTaskDelayUntil(&next,pdMS_TO_TICKS(1000/16));
    }
}
