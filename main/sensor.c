#include "sensor.h"
#include "camera_pins.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define SENSOR_ADDR 0x33
static i2c_master_dev_handle_t dev;
static paramsMLX90640 params;
static int64_t deadline;
static uint16_t raw[834], eeprom[832];
static float temperatures[THERMAL_PIXELS];

esp_err_t sensor_bus_init(void)
{
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t cfg = {.i2c_port=I2C_NUM_0,.sda_io_num=PIN_SDA,
        .scl_io_num=PIN_SCL,.clk_source=I2C_CLK_SRC_DEFAULT,.glitch_ignore_cnt=7,
        .flags.enable_internal_pullup=true};
    esp_err_t err=i2c_new_master_bus(&cfg,&bus);
    if(err!=ESP_OK) return err;
    i2c_device_config_t dc={.dev_addr_length=I2C_ADDR_BIT_LEN_7,
        .device_address=SENSOR_ADDR,.scl_speed_hz=400000};
    return i2c_master_bus_add_device(bus,&dc,&dev);
}

/* Single sensor task owns these callbacks. Deadline bounds upstream ready polling. */
int MLX90640_I2CRead(uint8_t addr,uint16_t reg,uint16_t words,uint16_t *data)
{
    if(addr!=SENSOR_ADDR || esp_timer_get_time()>deadline) return -1;
    uint8_t cmd[2]={reg>>8,reg};
    esp_err_t err=i2c_master_transmit_receive(dev,cmd,2,(uint8_t*)data,words*2,50);
    if(err!=ESP_OK) return -1;
    for(unsigned i=0;i<words;i++) data[i]=__builtin_bswap16(data[i]);
    if(reg==MLX90640_STATUS_REG && words==1 && !MLX90640_GET_DATA_READY(data[0]))
        vTaskDelay(pdMS_TO_TICKS(2));
    return 0;
}
int MLX90640_I2CWrite(uint8_t addr,uint16_t reg,uint16_t value)
{
    if(addr!=SENSOR_ADDR || esp_timer_get_time()>deadline) return -1;
    uint8_t cmd[4]={reg>>8,reg,value>>8,value};
    return i2c_master_transmit(dev,cmd,4,50)==ESP_OK?0:-1;
}
void MLX90640_I2CInit(void) {}
void MLX90640_I2CFreqSet(int freq) { (void)freq; }
int MLX90640_I2CGeneralReset(void) { return -1; } /* Never broadcast reset on shared bus. */

int sensor_configure(void)
{
    deadline=esp_timer_get_time()+2000000;
    int rc=MLX90640_DumpEE(SENSOR_ADDR,eeprom);
    if(rc) return rc;
    rc=MLX90640_ExtractParameters(eeprom,&params); if(rc) return rc;
    rc=MLX90640_SetInterleavedMode(SENSOR_ADDR); if(rc) return rc;
    rc=MLX90640_SetResolution(SENSOR_ADDR,2); if(rc) return rc; /* 18 bit */
    rc=MLX90640_SetRefreshRate(SENSOR_ADDR,6); if(rc) return rc; /* 32 subpages/s */
    if(MLX90640_GetCurMode(SENSOR_ADDR)!=0 || MLX90640_GetCurResolution(SENSOR_ADDR)!=2 ||
       MLX90640_GetRefreshRate(SENSOR_ADDR)!=6) return -1;
    return 0;
}

/*
 * TV/interleaved mode updates one complete 32-pixel row out of every two.
 * Reading only those twelve rows halves pixel I2C traffic.  This is necessary
 * for 32 subpages/s on the P4's documented 400 kHz master interface.
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

bool sensor_capture(thermal_frame_t *out, sensor_diag_t *diag)
{
    *diag=(sensor_diag_t){.failure="none",.last_page=-1};
    for(unsigned i=0;i<THERMAL_PIXELS;i++) temperatures[i]=NAN;
    unsigned seen=0;
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
        if(last_page_us)
            diag->page_gap_us=(uint32_t)(now-last_page_us);
        /* Both pages must be consecutive; do not combine old and new scenes. */
        if(last_page==page) { diag->failure="repeated subpage"; return false; }
        if(last_page_us && now-last_page_us>50000) {
            diag->failure="subpage gap exceeds 50ms";
            return false;
        }
        last_page=page;
        last_page_us=now;
        /* Reject a read overlapped by the next conversion. */
        uint16_t status;
        if(MLX90640_I2CRead(SENSOR_ADDR,MLX90640_STATUS_REG,1,&status)) {
            diag->failure="post-read status I2C/deadline";
            return false;
        }
        if(MLX90640_GET_DATA_READY(status)) {
            diag->failure="new data ready after read";
            return false;
        }
        float ambient=MLX90640_GetTa(raw,&params);
        if(!isfinite(ambient)) { diag->failure="invalid ambient"; return false; }
        int64_t calculate_start=esp_timer_get_time();
        MLX90640_CalculateTo(raw,&params,0.95f,ambient-8.0f,temperatures);
        diag->calculate_us+=(uint32_t)(esp_timer_get_time()-calculate_start);
        seen |= 1u<<page;
    }
    if(seen!=3) { diag->failure="missing complete subpage pair"; return false; }
    /* Mark factory defects before interpolation; never trust their finite values. */
    for(unsigned i=0;i<5;i++) {
        if(params.brokenPixels[i]<THERMAL_PIXELS) temperatures[params.brokenPixels[i]]=NAN;
        if(params.outlierPixels[i]<THERMAL_PIXELS) temperatures[params.outlierPixels[i]]=NAN;
    }
    if(!thermal_validate(temperatures,out)) {
        diag->failure="invalid calculated pixels";
        return false;
    }
    out->captured_us=esp_timer_get_time();
    return true;
}
