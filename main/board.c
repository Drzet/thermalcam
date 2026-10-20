#include "board.h"
#include "thermal.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static spi_device_handle_t lcd,touch;
static uint8_t *line;
static esp_err_t transfer(spi_device_handle_t d,const void *tx,void *rx,size_t n)
{
    spi_transaction_t t={.length=n*8,.tx_buffer=tx,.rx_buffer=rx};
    return spi_device_polling_transmit(d,&t);
}
static esp_err_t command(uint8_t cmd,const uint8_t *data,size_t len)
{
    gpio_set_level(PIN_TFT_DC,0);
    esp_err_t err=transfer(lcd,&cmd,NULL,1);
    gpio_set_level(PIN_TFT_DC,1);
    return err==ESP_OK && len?transfer(lcd,data,NULL,len):err;
}
#define CMD(c,...) do { const uint8_t a[]={__VA_ARGS__}; ESP_ERROR_CHECK(command(c,a,sizeof(a))); } while(0)
esp_err_t board_init(void)
{
    gpio_config_t out={.pin_bit_mask=(1ULL<<PIN_TFT_DC)|(1ULL<<PIN_TFT_RST)|(1ULL<<PIN_TFT_BL),.mode=GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_config_t in={.pin_bit_mask=(1ULL<<PIN_BUTTON)|(1ULL<<PIN_TOUCH_IRQ),.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&in));
    spi_bus_config_t bus={.mosi_io_num=PIN_TFT_MOSI,.miso_io_num=PIN_TFT_MISO,.sclk_io_num=PIN_TFT_SCK,
        .quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=IMAGE_W*3};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&bus,SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dc={.clock_speed_hz=40000000,.mode=0,.spics_io_num=PIN_TFT_CS,.queue_size=1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST,&dc,&lcd));
    bus.mosi_io_num=PIN_TOUCH_MOSI; bus.miso_io_num=PIN_TOUCH_MISO; bus.sclk_io_num=PIN_TOUCH_CLK;
    bus.max_transfer_sz=32;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST,&bus,SPI_DMA_CH_AUTO));
    dc.clock_speed_hz=2000000; dc.spics_io_num=PIN_TOUCH_CS;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST,&dc,&touch));
    line=heap_caps_malloc(IMAGE_W*3,MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
    if(!line) return ESP_ERR_NO_MEM;
    gpio_set_level(PIN_TFT_BL,0);
    gpio_set_level(PIN_TFT_RST,0); vTaskDelay(pdMS_TO_TICKS(150));
    gpio_set_level(PIN_TFT_RST,1); vTaskDelay(pdMS_TO_TICKS(150));
    /* ILI9488 register setup follows the original Arduino_GFX panel configuration. */
    CMD(0xe0,0x00,0x03,0x09,0x08,0x16,0x0a,0x3f,0x78,0x4c,0x09,0x0a,0x08,0x16,0x1a,0x0f);
    CMD(0xe1,0x00,0x16,0x19,0x03,0x0f,0x05,0x32,0x45,0x46,0x04,0x0e,0x0d,0x35,0x37,0x0f);
    CMD(0xc0,0x17,0x15); CMD(0xc1,0x41); CMD(0xc5,0x00,0x12,0x80);
    CMD(0xb0,0x80); CMD(0xb1,0xa0); CMD(0xb4,0x02); CMD(0xb6,0x02,0x02);
    CMD(0xe9,0x00); CMD(0xf7,0xa9,0x51,0x2c,0x82);
    CMD(0x3a,0x66); CMD(0x36,0x28); /* 18-bit, original rotation 1 + BGR */
    ESP_ERROR_CHECK(command(0x21,NULL,0)); /* original IPS inversion */
    ESP_ERROR_CHECK(command(0x11,NULL,0)); vTaskDelay(pdMS_TO_TICKS(150));
    ESP_ERROR_CHECK(command(0x29,NULL,0));
    gpio_set_level(PIN_TFT_BL,1);
    return ESP_OK;
}

esp_err_t board_draw(const uint8_t *bgr)
{
    for(int y=0;y<IMAGE_H;y++) {
        uint8_t xaddr[]={0,0,(IMAGE_W-1)>>8,(IMAGE_W-1)&255};
        uint8_t yaddr[]={y>>8,y&255,y>>8,y&255};
        esp_err_t e=command(0x2a,xaddr,4); if(e) return e;
        e=command(0x2b,yaddr,4); if(e) return e;
        e=command(0x2c,NULL,0); if(e) return e;
        for(int x=0;x<IMAGE_W;x++) {
            const uint8_t *p=bgr+(y*IMAGE_W+x)*3;
            line[x*3]=p[2]&0xfc; line[x*3+1]=p[1]&0xfc; line[x*3+2]=p[0]&0xfc;
        }
        e=transfer(lcd,line,NULL,IMAGE_W*3); if(e) return e;
    }
    return ESP_OK;
}
static int adc(uint8_t cmd)
{
    uint8_t tx[3]={cmd,0,0},rx[3];
    if(transfer(touch,tx,rx,3)!=ESP_OK) return -1;
    return ((rx[1]<<8)|rx[2])>>3;
}
bool board_touch(int *x,int *y)
{
    if(gpio_get_level(PIN_TOUCH_IRQ)) return false;
    /* XPT2046 12-bit differential channels. PD=0 restores PENIRQ after each sample. */
    int z1=adc(0xb0),z2=adc(0xc0);
    if(z1<0 || z2<0 || z1+4095-z2<400) return false;
    /* Match XPT2046_Touchscreen default rotation 1 (X=0x90, Y=0xd0). */
    (void)adc(0x90);
    int rx=adc(0x90),ry=adc(0xd0);
    if(rx<0 || ry<0 || gpio_get_level(PIN_TOUCH_IRQ)) return false;
    /* Original sketch mapping retained; calibration still requires the physical panel. */
    *x=rx*(IMAGE_W-1)/4095; *y=(4095-ry)*(IMAGE_H-1)/4095;
    return true;
}
