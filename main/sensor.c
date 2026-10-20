#include "sensor.h"
#include "board.h"
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
    rc=MLX90640_SetRefreshRate(SENSOR_ADDR,4); if(rc) return rc; /* 8 subpages/s */
    if(MLX90640_GetCurMode(SENSOR_ADDR)!=0 || MLX90640_GetCurResolution(SENSOR_ADDR)!=2 ||
       MLX90640_GetRefreshRate(SENSOR_ADDR)!=4) return -1;
    return 0;
}

bool sensor_capture(thermal_frame_t *out)
{
    for(unsigned i=0;i<THERMAL_PIXELS;i++) temperatures[i]=NAN;
    unsigned seen=0;
    deadline=esp_timer_get_time()+750000;
    for(unsigned attempt=0;attempt<4 && seen!=3;attempt++) {
        int page=MLX90640_GetFrameData(SENSOR_ADDR,raw);
        if(page<0 || page>1) return false;
        /* Reject a read overlapped by the next conversion. */
        uint16_t status;
        if(MLX90640_I2CRead(SENSOR_ADDR,MLX90640_STATUS_REG,1,&status)) return false;
        if(MLX90640_GET_DATA_READY(status)) return false;
        float ambient=MLX90640_GetTa(raw,&params);
        if(!isfinite(ambient)) return false;
        MLX90640_CalculateTo(raw,&params,0.95f,ambient-8.0f,temperatures);
        seen |= 1u<<page;
    }
    if(seen!=3) return false;
    /* Mark factory defects before interpolation; never trust their finite values. */
    for(unsigned i=0;i<5;i++) {
        if(params.brokenPixels[i]<THERMAL_PIXELS) temperatures[params.brokenPixels[i]]=NAN;
        if(params.outlierPixels[i]<THERMAL_PIXELS) temperatures[params.outlierPixels[i]]=NAN;
    }
    if(!thermal_validate(temperatures,out)) return false;
    out->captured_us=esp_timer_get_time();
    return true;
}
