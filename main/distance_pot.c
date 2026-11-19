#include "distance_pot.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uvc_mode_control.h"

static const char *TAG = "distance_pot";

static void task(void *arg)
{
    adc_oneshot_unit_handle_t unit_handle = (adc_oneshot_unit_handle_t)arg;
    adc_unit_t unit;
    adc_channel_t channel;
    int gpio = CONFIG_THERMALCAM_ALIGN_POT_GPIO;
    if (adc_oneshot_io_to_channel(gpio, &unit, &channel) != ESP_OK) {
        ESP_LOGE(TAG,"ADC GPIO mapping lost");
        vTaskDelete(NULL);
        return;
    }
    int filtered=-1, last_cm=-1;
    for (;;) {
        int raw;
        if (adc_oneshot_read(unit_handle,channel,&raw)==ESP_OK) {
            filtered=filtered<0?raw:(3*filtered+raw+2)/4;
            int low=CONFIG_THERMALCAM_ALIGN_POT_ADC_MIN;
            int high=CONFIG_THERMALCAM_ALIGN_POT_ADC_MAX;
            if (filtered<low) raw=low;
            else if(filtered>high) raw=high;
            else raw=filtered;
#ifdef CONFIG_THERMALCAM_ALIGN_POT_REVERSE
            raw=high-(raw-low);
#endif
            int cm=CONFIG_THERMALCAM_ALIGN_POT_MIN_CM+
                  ((raw-low)*(CONFIG_THERMALCAM_ALIGN_POT_MAX_CM-
                              CONFIG_THERMALCAM_ALIGN_POT_MIN_CM)+
                   (high-low)/2)/(high-low);
            if(cm!=last_cm) {
                thermalcam_uvc_set_distance((uint16_t)cm);
                last_cm=cm;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void distance_pot_start(void)
{
    const int gpio=CONFIG_THERMALCAM_ALIGN_POT_GPIO;
    if (gpio==-1) {
        ESP_LOGI(TAG,"No ADC knob configured; UVC contrast selects distance");
        return;
    }
    /* The ESP32-P4 ADC1 channels are GPIO16..22; availability on board
     * must be checked manually. Do not assume a pin from board layout. */
    if (gpio<16 || gpio>22 ||
        CONFIG_THERMALCAM_ALIGN_POT_ADC_MAX<=CONFIG_THERMALCAM_ALIGN_POT_ADC_MIN ||
        CONFIG_THERMALCAM_ALIGN_POT_MAX_CM<=CONFIG_THERMALCAM_ALIGN_POT_MIN_CM) {
        ESP_LOGE(TAG,"Invalid knob pin or endpoint calibration");
        return;
    }
    adc_unit_t unit;
    adc_channel_t channel;
    esp_err_t err=adc_oneshot_io_to_channel(gpio,&unit,&channel);
    if (err!=ESP_OK || unit!=ADC_UNIT_1) {
        ESP_LOGE(TAG,"Invalid ADC mapping on GPIO%d",gpio);
        return;
    }
    adc_oneshot_unit_init_cfg_t cfg={.unit_id=unit};
    adc_oneshot_unit_handle_t handle=NULL;
    err=adc_oneshot_new_unit(&cfg,&handle);
    if(err!=ESP_OK) {
        ESP_LOGE(TAG,"ADC unit initialization: %s",esp_err_to_name(err));
        return;
    }
    adc_oneshot_chan_cfg_t chan_cfg={
        .bitwidth=ADC_BITWIDTH_DEFAULT,.atten=ADC_ATTEN_DB_12
    };
    err=adc_oneshot_config_channel(handle,channel,&chan_cfg);
    if(err!=ESP_OK) {
        ESP_LOGE(TAG,"ADC channel: %s",esp_err_to_name(err));
        adc_oneshot_del_unit(handle);
        return;
    }
    if(xTaskCreate(task,"distance_knob",3072,handle,2,NULL)!=pdPASS) {
        ESP_LOGE(TAG,"Cannot create ADC task");
        adc_oneshot_del_unit(handle);
        return;
    }
    ESP_LOGI(TAG,"GPIO%d distance knob %d..%d cm",gpio,
             CONFIG_THERMALCAM_ALIGN_POT_MIN_CM,
             CONFIG_THERMALCAM_ALIGN_POT_MAX_CM);
}
