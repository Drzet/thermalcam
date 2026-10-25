#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "sdkconfig.h"
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
#include "uvc_mode_control.h"
#include "sensor.h"
#include "camera_pins.h"
#include "driver/gpio.h"

#define JPEG_CAPACITY (IMAGE_BYTES)
static const char *TAG="thermalcam";
static SemaphoreHandle_t state_lock,jpeg_lock;
static QueueHandle_t capture_queue;
static thermal_frame_t latest;
static thermal_mode_t mode;

/* UVC control callback and physical button share the same atomic mode. */
uint8_t thermalcam_uvc_get_mode(void)
{
    return (uint8_t)__atomic_load_n(&mode, __ATOMIC_RELAXED);
}

void thermalcam_uvc_set_mode(uint8_t value)
{
    if (value < MODE_COUNT)
        __atomic_store_n(&mode, (thermal_mode_t)value, __ATOMIC_RELAXED);
}

static bool acquisition_error=true;
static uvc_fb_t published;
static uint8_t *rgb,*encoded;
static size_t rgb_capacity,jpeg_capacity;
static jpeg_encoder_handle_t encoder;

/* USB callback counts, reported outside USB context every five seconds.
 * "returned" means the UVC stack has copied the frame, not host delivery. */
static portMUX_TYPE uvc_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static struct {
    uint32_t requested, offered, returned, distinct_thermal;
    uint32_t last_thermal_sequence;
} uvc_stats;
static uint32_t published_thermal_sequence;

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
            ESP_LOGI(TAG,"MLX90640 configured: 18-bit, 32 subpages/s (up to 16 full fps), I3C legacy-I2C 1 MHz (EEPROM 400 kHz)");
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
                thermal_mode_t before, after;
                do {
                    before = __atomic_load_n(&mode, __ATOMIC_RELAXED);
                    after = (thermal_mode_t)((before + 1) % MODE_COUNT);
                } while (!__atomic_compare_exchange_n(&mode, &before, after,
                                                       false, __ATOMIC_RELAXED,
                                                       __ATOMIC_RELAXED));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t stream_start(uvc_format_t fmt,int w,int h,int rate,void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG,"UVC host request: fmt=%d %dx%d at %d fps (descriptor=%d fps)",
             (int)fmt,w,h,rate,CONFIG_UVC_CAM1_FRAMERATE);
    if(fmt!=UVC_FORMAT_JPEG || w!=IMAGE_W || h!=IMAGE_H) {
        ESP_LOGW(TAG,"UVC unsupported host format/resolution");
        return ESP_ERR_NOT_SUPPORTED;
    }
    /* The local sdkconfig may retain an older descriptor rate even if
     * sdkconfig.defaults has changed. Never reject an otherwise valid
     * stream solely because the host requested that advertised rate. */
    if(rate!=CONFIG_UVC_CAM1_FRAMERATE)
        ESP_LOGW(TAG,"UVC rate differs from compiled descriptor: %d vs %d",
                 rate,CONFIG_UVC_CAM1_FRAMERATE);
    ESP_LOGI(TAG,"UVC started: %dx%d at %d fps",w,h,rate);
    return ESP_OK;
}
static void stream_stop(void *ctx) { (void)ctx; ESP_LOGI(TAG,"UVC stopped"); }
static uvc_fb_t *stream_get(void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&uvc_stats_lock);
    ++uvc_stats.requested;
    portEXIT_CRITICAL(&uvc_stats_lock);

    if(xSemaphoreTake(jpeg_lock,pdMS_TO_TICKS(20))!=pdTRUE) return NULL;
    if(!published.len) { xSemaphoreGive(jpeg_lock); return NULL; }

    portENTER_CRITICAL(&uvc_stats_lock);
    ++uvc_stats.offered;
    portEXIT_CRITICAL(&uvc_stats_lock);
    /* Driver copies to its own transfer buffer, then immediately calls return. */
    return &published;
}
static void stream_return(uvc_fb_t *fb,void *ctx)
{
    (void)fb; (void)ctx;
    portENTER_CRITICAL(&uvc_stats_lock);
    ++uvc_stats.returned;
    if(published_thermal_sequence &&
       published_thermal_sequence!=uvc_stats.last_thermal_sequence) {
        ++uvc_stats.distinct_thermal;
        uvc_stats.last_thermal_sequence=published_thermal_sequence;
    }
    portEXIT_CRITICAL(&uvc_stats_lock);
    xSemaphoreGive(jpeg_lock);
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
    thermal_init();
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
    thermalcam_uvc_control_init(); /* Retain the wrapped UVC handlers at link time. */
    ESP_ERROR_CHECK(uvc_device_init());
    ESP_LOGI(TAG,"UVC compiled descriptor: MJPEG %dx%d at %d fps",IMAGE_W,IMAGE_H,
             CONFIG_UVC_CAM1_FRAMERATE);
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
    int64_t video_since=esp_timer_get_time();
    uint32_t last_rendered_sequence=0;
    uint32_t renders=0, fresh=0, jpeg_ok=0, publishes=0;
    uint64_t render_total=0, jpeg_total=0, publish_total=0;
    uint32_t render_max=0, jpeg_max=0, publish_max=0;
    for(;;) {
        bool fault; thermal_mode_t active_mode;
        xSemaphoreTake(state_lock,portMAX_DELAY);
        fault=acquisition_error; active_mode=__atomic_load_n(&mode, __ATOMIC_RELAXED); *view=latest;
        xSemaphoreGive(state_lock);
        if(view->valid && view->sequence!=last_rendered_sequence) {
            ++fresh;
            last_rendered_sequence=view->sequence;
        }
        int64_t age=view->valid?(esp_timer_get_time()-view->captured_us)/1000:0;
        bool stale=fault || age>1000;

        int64_t start=esp_timer_get_time();
        thermal_render(rgb,view,active_mode,stale,age>UINT32_MAX?UINT32_MAX:(uint32_t)age);
        uint32_t render_us=(uint32_t)(esp_timer_get_time()-start);
        ++renders;
        render_total+=render_us;
        if(render_us>render_max) render_max=render_us;

        uint32_t bytes=0;
        start=esp_timer_get_time();
        esp_err_t e=jpeg_encoder_process(encoder,&cfg,rgb,IMAGE_BYTES,encoded,jpeg_capacity,&bytes);
        uint32_t jpeg_us=(uint32_t)(esp_timer_get_time()-start);
        jpeg_total+=jpeg_us;
        if(jpeg_us>jpeg_max) jpeg_max=jpeg_us;

        if(e==ESP_OK && bytes>0 && bytes<=JPEG_CAPACITY) {
            ++jpeg_ok;
            static bool first_jpeg_logged;
            if(!first_jpeg_logged) {
                ESP_LOGI(TAG,"First JPEG encoded: %lu bytes",(unsigned long)bytes);
                first_jpeg_logged=true;
            }
            start=esp_timer_get_time();
            xSemaphoreTake(jpeg_lock,portMAX_DELAY);
            memcpy(published.buf,encoded,bytes); published.len=bytes;
            int64_t now=esp_timer_get_time();
            published.timestamp.tv_sec=now/1000000;
            published.timestamp.tv_usec=now%1000000;
            published_thermal_sequence=view->valid?view->sequence:0;
            xSemaphoreGive(jpeg_lock);
            uint32_t publish_us=(uint32_t)(esp_timer_get_time()-start);
            ++publishes;
            publish_total+=publish_us;
            if(publish_us>publish_max) publish_max=publish_us;
        } else {
            /* Never silently keep serving old video if encoding fails. */
            xSemaphoreTake(jpeg_lock,portMAX_DELAY); published.len=0; xSemaphoreGive(jpeg_lock);
            ESP_LOGE(TAG,"JPEG failed: %s, %lu bytes",esp_err_to_name(e),(unsigned long)bytes);
        }

        int64_t now=esp_timer_get_time();
        if(now-video_since>=5000000) {
            uint32_t period_us=(uint32_t)(now-video_since);
            uint32_t render_fps10=(uint32_t)((uint64_t)renders*10000000ULL/period_us);
            uint32_t fresh_fps10=(uint32_t)((uint64_t)fresh*10000000ULL/period_us);
            uint32_t jpeg_fps10=(uint32_t)((uint64_t)jpeg_ok*10000000ULL/period_us);
            uint32_t publish_fps10=(uint32_t)((uint64_t)publishes*10000000ULL/period_us);
            ESP_LOGI(TAG,"Video: render=%lu.%lu/s fresh=%lu.%lu/s jpeg=%lu.%lu/s publish=%lu.%lu/s "
                     "render_avg/max=%lu/%lu us jpeg_avg/max=%lu/%lu us publish_avg/max=%lu/%lu us",
                     (unsigned long)(render_fps10/10),(unsigned long)(render_fps10%10),
                     (unsigned long)(fresh_fps10/10),(unsigned long)(fresh_fps10%10),
                     (unsigned long)(jpeg_fps10/10),(unsigned long)(jpeg_fps10%10),
                     (unsigned long)(publish_fps10/10),(unsigned long)(publish_fps10%10),
                     (unsigned long)(render_total/renders),(unsigned long)render_max,
                     (unsigned long)(jpeg_total/renders),(unsigned long)jpeg_max,
                     (unsigned long)(publishes?publish_total/publishes:0),(unsigned long)publish_max);
            uint32_t requested,offered,returned,distinct;
            portENTER_CRITICAL(&uvc_stats_lock);
            requested=uvc_stats.requested; offered=uvc_stats.offered;
            returned=uvc_stats.returned; distinct=uvc_stats.distinct_thermal;
            uvc_stats.requested=uvc_stats.offered=uvc_stats.returned=0;
            uvc_stats.distinct_thermal=0;
            portEXIT_CRITICAL(&uvc_stats_lock);
            ESP_LOGI(TAG,"UVC callbacks/5s: requested=%lu offered=%lu returned=%lu "
                     "distinct_thermal=%lu (not USB completion)",
                     (unsigned long)requested,(unsigned long)offered,
                     (unsigned long)returned,(unsigned long)distinct);

            renders=fresh=jpeg_ok=publishes=0;
            render_total=jpeg_total=publish_total=0;
            render_max=jpeg_max=publish_max=0;
            video_since=now;
        }
        vTaskDelayUntil(&next,pdMS_TO_TICKS(1000/16));
    }
}
