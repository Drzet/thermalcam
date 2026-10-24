#include "sensor.h"
#include "camera_pins.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "driver/i3c_master.h"
#include "driver/i3c_master_i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG="mlx90640_bus";
#define SENSOR_ADDR 0x33
#define EEPROM_BUS_HZ 400000u
#define ACQUISITION_BUS_HZ 1000000u
static i3c_master_bus_handle_t bus;
static i3c_master_i2c_device_handle_t dev, dev_slow, dev_fast;
static paramsMLX90640 params;
static SemaphoreHandle_t params_mutex;
static int64_t deadline;
/* IDF v6.0.3 I3C FIFO uses 32-bit words when draining RX data. */
static uint16_t raw[834] __attribute__((aligned(4)));
static uint16_t eeprom[832] __attribute__((aligned(4)));
static float temperatures[THERMAL_PIXELS];

esp_err_t sensor_bus_init(void)
{
    params_mutex=xSemaphoreCreateMutex();
    if(!params_mutex) return ESP_ERR_NO_MEM;
    /* The P4's I3C controller also drives ordinary 7-bit I2C devices.
     * Same physical SDA/SCL pins; no I3C dynamic-address assignment.
     * Use blocking FIFO transfers initially (no DMA alignment requirements).
     */
    i3c_master_bus_config_t cfg={
        .sda_io_num=PIN_SDA,.scl_io_num=PIN_SCL,
        .clock_source=I3C_MASTER_CLK_SRC_DEFAULT
    };
    esp_err_t err=i3c_new_master_bus(&cfg,&bus);
    if(err!=ESP_OK) return err;
    /* Unlike the normal I2C master, IDF 6.0.3's I3C pin setup does not
     * set GPIO open-drain. On the non-dedicated GPIO41/42 pads, legacy I2C
     * requires this explicitly. Do not drive SDA/SCL high push-pull. */
    err=gpio_od_enable(PIN_SDA);
    if(err!=ESP_OK) return err;
    err=gpio_od_enable(PIN_SCL);
    if(err!=ESP_OK) return err;
    /* The proven I2C baseline enabled internal pull-ups. The IDF 6.0.3
     * I3C legacy-I2C driver only routes GPIOs and does NOT turn these on.
     * Restore the same weak pull-ups for bootstrap. External low-value
     * pull-ups are still needed for reliable 1 MHz Fast-mode Plus. */
    err=gpio_set_pull_mode(PIN_SDA,GPIO_PULLUP_ONLY);
    if(err!=ESP_OK) return err;
    err=gpio_set_pull_mode(PIN_SCL,GPIO_PULLUP_ONLY);
    if(err!=ESP_OK) return err;

    /* The MLX90640's EEPROM requires <=400 kHz. Other registers/RAM allow
     * 1 MHz. Two handles refer to the same 7-bit address with distinct
     * transaction timings, selected only by the sensor task. */
    i3c_device_i2c_config_t dc={
        .device_address=SENSOR_ADDR,.scl_freq_hz=EEPROM_BUS_HZ
    };
    err=i3c_master_bus_add_i2c_device(bus,&dc,&dev_slow);
    if(err!=ESP_OK) return err;
    dc.scl_freq_hz=ACQUISITION_BUS_HZ;
    err=i3c_master_bus_add_i2c_device(bus,&dc,&dev_fast);
    if(err!=ESP_OK) return err;
    dev=dev_slow;
    return ESP_OK;
}

/* Single sensor task owns these callbacks. Deadline bounds upstream ready polling. */
int MLX90640_I2CRead(uint8_t addr,uint16_t reg,uint16_t words,uint16_t *data)
{
    if(addr!=SENSOR_ADDR || esp_timer_get_time()>deadline) return -1;
    /* IDF 6.0.3 I3C FIFO holds 128 bytes. Split the 1664-byte EEPROM
     * dump (and any other longer reads) into <=128-byte transactions;
     * this also avoids relying on multi-watermark FIFO draining. */
    for(unsigned pos=0;pos<words;) {
        unsigned n=words-pos;
        if(n>64) n=64;
        uint16_t at=reg+pos;
        uint8_t cmd[2] __attribute__((aligned(4)))={at>>8,at};
        /* ESP-IDF's FIFO reads 32-bit words: pad 2-byte register reads. */
        uint32_t short_read=0;
        uint8_t *dst=(n==1)?(uint8_t *)&short_read:(uint8_t *)&data[pos];
        esp_err_t err=i3c_master_i2c_device_transmit_receive(dev,cmd,2,dst,n*2,50);
        if(err!=ESP_OK) {
            ESP_LOGE(TAG,"I3C-I2C read failed: reg=0x%04X words=%u clock=%lu Hz: %s",
                     at,n,(unsigned long)(dev==dev_slow?EEPROM_BUS_HZ:ACQUISITION_BUS_HZ),
                     esp_err_to_name(err));
            return -1;
        }
        if(n==1) memcpy(&data[pos],&short_read,sizeof(uint16_t));
        for(unsigned i=pos;i<pos+n;i++)
            data[i]=__builtin_bswap16(data[i]);
        pos+=n;
    }
    if(reg==MLX90640_STATUS_REG && words==1 && !MLX90640_GET_DATA_READY(data[0]))
        vTaskDelay(pdMS_TO_TICKS(2));
    return 0;
}
int MLX90640_I2CWrite(uint8_t addr,uint16_t reg,uint16_t value)
{
    if(addr!=SENSOR_ADDR || esp_timer_get_time()>deadline) return -1;
    uint8_t cmd[4] __attribute__((aligned(4)))={reg>>8,reg,value>>8,value};
    esp_err_t err=i3c_master_i2c_device_transmit(dev,cmd,4,50);
    if(err!=ESP_OK)
        ESP_LOGE(TAG,"I3C-I2C write failed: reg=0x%04X clock=%lu Hz: %s",
                 reg,(unsigned long)(dev==dev_slow?EEPROM_BUS_HZ:ACQUISITION_BUS_HZ),
                 esp_err_to_name(err));
    return err==ESP_OK?0:-1;
}
void MLX90640_I2CInit(void) {}
void MLX90640_I2CFreqSet(int freq) { (void)freq; }
int MLX90640_I2CGeneralReset(void) { return -1; } /* Never broadcast reset on shared bus. */

int sensor_configure(void)
{
    /* Every reinitialisation re-reads EEPROM at its rated <=400 kHz. */
    dev=dev_slow;
    deadline=esp_timer_get_time()+2000000;
    /* Non-destructive one-word EEPROM probe distinguishes a fundamental
     * I3C legacy-I2C transaction failure from a long FIFO read failure. */
    uint16_t probe=0;
    if(MLX90640_I2CRead(SENSOR_ADDR,0x2400,1,&probe)) {
        ESP_LOGE(TAG,"400 kHz 1-word EEPROM probe failed before DumpEE");
        return -1;
    }
    static bool logged_probe;
    if(!logged_probe) {
        ESP_LOGI(TAG,"400 kHz EEPROM single-word probe succeeded: 0x%04X",probe);
        logged_probe=true;
    }
    int rc=MLX90640_DumpEE(SENSOR_ADDR,eeprom);
    if(rc) {
        ESP_LOGE(TAG,"MLX90640_DumpEE failed at 400 kHz: %d",rc);
        return rc;
    }
    xSemaphoreTake(params_mutex,portMAX_DELAY);
    rc=MLX90640_ExtractParameters(eeprom,&params);
    xSemaphoreGive(params_mutex);
    if(rc) return rc;
    rc=MLX90640_SetInterleavedMode(SENSOR_ADDR); if(rc) return rc;
    rc=MLX90640_SetResolution(SENSOR_ADDR,2); if(rc) return rc; /* 18 bit */
    rc=MLX90640_SetRefreshRate(SENSOR_ADDR,7); if(rc) return rc; /* 64 subpages/s */
    if(MLX90640_GetCurMode(SENSOR_ADDR)!=0 || MLX90640_GetCurResolution(SENSOR_ADDR)!=2 ||
       MLX90640_GetRefreshRate(SENSOR_ADDR)!=7) return -1;

    /* Switch to 1 MHz only after EEPROM and configuration operations.
     * Verify that fast-mode-plus register reads actually work. */
    dev=dev_fast;
    if(MLX90640_GetRefreshRate(SENSOR_ADDR)!=7) {
        dev=dev_slow;
        return -1;
    }
    return 0;
}

/*
 * TV/interleaved mode updates one complete 32-pixel row out of every two.
 * Reading only those twelve rows halves pixel I2C traffic. At 64
 * subpages/s, this keeps bus occupancy below the 15.625 ms page interval
 * when the P4 I3C peripheral operates in 1 MHz legacy-I2C mode.
 * Keep Melexis' 834-word frame layout for its temperature-calculation API.
 */
static int read_updated_subpage(uint16_t *frame, sensor_diag_t *diag)
{
    uint16_t status;
    int64_t wait_start=esp_timer_get_time();
    do {
        if (MLX90640_I2CRead(SENSOR_ADDR, MLX90640_STATUS_REG, 1, &status)) {
            diag->failure="ready-status I2C/deadline";
            diag->ready_wait_us += (uint32_t)(esp_timer_get_time()-wait_start);
            return -1;
        }
    } while (!MLX90640_GET_DATA_READY(status));
    diag->ready_wait_us += (uint32_t)(esp_timer_get_time()-wait_start);

    int page = MLX90640_GET_FRAME(status);
    if (MLX90640_I2CWrite(SENSOR_ADDR, MLX90640_STATUS_REG,
                          MLX90640_INIT_STATUS_VALUE)) {
        diag->failure="clear status I2C";
        return -1;
    }

    /* Read common calibration/ambient values before the pixel rows, so a
     * newly completed opposite subpage cannot change them mid-read. */
    if (MLX90640_I2CRead(SENSOR_ADDR, MLX90640_AUX_DATA_START_ADDRESS,
                         MLX90640_AUX_NUM, &frame[768])) {
        diag->failure="auxiliary I2C/deadline";
        return -1;
    }
    if (MLX90640_I2CRead(SENSOR_ADDR, MLX90640_CTRL_REG, 1, &frame[832])) {
        diag->failure="control-register I2C/deadline";
        return -1;
    }
    frame[833] = (uint16_t)page;

    for (unsigned row = (unsigned)page; row < 24; row += 2) {
        unsigned offset = row * 32;
        if (MLX90640_I2CRead(SENSOR_ADDR, MLX90640_PIXEL_DATA_START_ADDRESS + offset,
                             32, &frame[offset])) {
            diag->failure="pixel row I2C/deadline";
            return -1;
        }
        if (frame[offset] == 0x7FFF) {
            diag->failure="invalid pixel row";
            return -1;
        } /* Upstream frame-data check. */
    }

    /* Upstream auxiliary-data validation, retained for the shortened read. */
    const unsigned ranges[][2] = {
        {0, 1}, {8, 19}, {20, 23}, {24, 33},
        {40, 51}, {52, 55}, {56, 64}
    };
    for (unsigned i = 0; i < sizeof(ranges) / sizeof(ranges[0]); i++)
        for (unsigned j = ranges[i][0]; j < ranges[i][1]; j++)
            if (frame[768 + j] == 0x7FFF) {
                diag->failure="invalid auxiliary data";
                return -1;
            }
    if (frame[832] & MLX90640_CTRL_MEAS_MODE_MASK) {
        diag->failure="sensor changed to chess mode";
        return -1;
    }
    return page;
}

bool sensor_capture(sensor_pair_t *out, sensor_diag_t *diag)
{
    *diag=(sensor_diag_t){.failure="none",.last_page=-1};
    unsigned seen=0, count=0;
    int last_page=-1;
    int64_t last_page_us=0;
    deadline=esp_timer_get_time()+750000;
    for(unsigned attempt=0;attempt<4 && seen!=3;attempt++) {
        diag->attempts=attempt+1;
        int64_t read_start=esp_timer_get_time();
        int page=read_updated_subpage(raw,diag);
        diag->read_us+=(uint32_t)(esp_timer_get_time()-read_start);
        if(page<0 || page>1) {
            if(page>1) diag->failure="invalid page number";
            return false;
        }
        diag->last_page=page;
        int64_t now=esp_timer_get_time();
        if(last_page_us) diag->page_gap_us=(uint32_t)(now-last_page_us);
        if(last_page==page) { diag->failure="repeated subpage"; return false; }
        if(last_page_us && now-last_page_us>50000) {
            diag->failure="subpage gap exceeds 50ms";
            return false;
        }
        /* Confirm the just-read rows were not overwritten by a later
         * measurement of the SAME page. Opposite-page ready is harmless. */
        uint16_t status;
        if(MLX90640_I2CRead(SENSOR_ADDR,MLX90640_STATUS_REG,1,&status)) {
            diag->failure="post-read status I2C/deadline";
            return false;
        }
        if(MLX90640_GET_DATA_READY(status)) {
            if(MLX90640_GET_FRAME(status)==page) {
                diag->failure="same subpage overwritten during read";
                return false;
            }
            diag->opposite_ready++;
        }
        /* The worker gets a complete private raw snapshot. No I2C transfer
         * waits for expensive per-pixel floating-point calculations. */
        memcpy(out->raw[count],raw,sizeof(raw));
        seen |= 1u<<page;
        ++count;
        last_page=page;
        last_page_us=now;
    }
    if(seen!=3 || count!=2) {
        diag->failure="missing complete subpage pair";
        return false;
    }
    out->captured_us=esp_timer_get_time();
    return true;
}

bool sensor_process(const sensor_pair_t *pair, thermal_frame_t *out,
                    uint32_t *calculation_us)
{
    for(unsigned i=0;i<THERMAL_PIXELS;i++) temperatures[i]=NAN;
    int64_t start=esp_timer_get_time();
    xSemaphoreTake(params_mutex,portMAX_DELAY);
    bool ok=true;
    for(unsigned i=0;i<2;i++) {
        uint16_t *data=(uint16_t *)pair->raw[i];
        float ambient=MLX90640_GetTa(data,&params);
        if(!isfinite(ambient)) { ok=false; break; }
        MLX90640_CalculateTo(data,&params,0.95f,ambient-8.0f,temperatures);
    }
    if(ok) {
        for(unsigned i=0;i<5;i++) {
            if(params.brokenPixels[i]<THERMAL_PIXELS)
                temperatures[params.brokenPixels[i]]=NAN;
            if(params.outlierPixels[i]<THERMAL_PIXELS)
                temperatures[params.outlierPixels[i]]=NAN;
        }
    }
    xSemaphoreGive(params_mutex);
    *calculation_us=(uint32_t)(esp_timer_get_time()-start);
    if(!ok || !thermal_validate(temperatures,out)) return false;
    out->captured_us=pair->captured_us;
    return true;
}
