/*
 * UVC Processing Unit brightness -> ThermalCam display mode.
 *
 * usb_device_uvc 1.3.1 / TinyUSB 0.19 has no processing-unit control
 * callback and its stock descriptor connects input terminal directly to
 * output terminal. The two --wrap link options in main/CMakeLists.txt
 * change only those two external TinyUSB entry points:
 *   - add a real PU to the existing single-camera configuration descriptor
 *   - respond to PU brightness requests, forward every other UVC request
 *
 * The upstream component and video streaming implementation are unchanged.
 */
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "uvc_mode_control.h"

#define VC_INTERFACE        0u
#define CAMERA_TERMINAL_ID  1u
#define OUTPUT_TERMINAL_ID  2u
#define PROCESSING_UNIT_ID  3u
#define PU_BRIGHTNESS       2u
#define CONFIG_BUFFER_SIZE  512u

#define CS_INTERFACE        0x24u
#define VC_HEADER           0x01u
#define VC_INPUT_TERMINAL   0x02u
#define VC_OUTPUT_TERMINAL  0x03u
#define VC_PROCESSING_UNIT  0x05u

static const char *TAG = "uvc_brightness";

/* UVC 1.5 PU: brightness is bmControls bit 0; all other PU controls off.
 * bControlSize=2, bmVideoStandards=0. Unit 3 sits between camera 1
 * and streaming output terminal 2. */
static const uint8_t brightness_pu[] = {
    12, CS_INTERFACE, VC_PROCESSING_UNIT, PROCESSING_UNIT_ID,
    CAMERA_TERMINAL_ID, 0, 0,  /* source and wMaxMultiplier */
    2, 1, 0,                 /* bControlSize, bmControls */
    0, 0                     /* iProcessing, bmVideoStandards */
};

static uint8_t config_with_pu[CONFIG_BUFFER_SIZE];
static bool config_ready;

uint8_t const *__real_tud_descriptor_configuration_cb(uint8_t index);
bool __real_videod_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                  tusb_control_request_t const *request);

uint8_t const *__wrap_tud_descriptor_configuration_cb(uint8_t index)
{
    uint8_t const *original = __real_tud_descriptor_configuration_cb(index);
    if (!original || index != 0) return original;
    if (config_ready) return config_with_pu;

    /* Locate the existing VC header and output terminal in the stock
     * single-camera descriptor. Never modify the read-only upstream
     * array or assume fixed descriptor offsets. */
    if (original[0] != 9 || original[1] != TUSB_DESC_CONFIGURATION)
        return original;
    uint16_t length = (uint16_t)(original[2] | ((uint16_t)original[3] << 8));
    if (length < 9 || length + sizeof(brightness_pu) > CONFIG_BUFFER_SIZE)
        return original;

    int vc_header = -1, camera = -1, output = -1;
    bool in_vc = false;
    for (uint16_t p = 0; p < length;) {
        uint8_t n = original[p];
        if (n < 2 || p + n > length) return original;
        if (original[p + 1] == TUSB_DESC_INTERFACE && n >= 9) {
            in_vc = original[p + 2] == VC_INTERFACE &&
                    original[p + 5] == TUSB_CLASS_VIDEO &&
                    original[p + 6] == VIDEO_SUBCLASS_CONTROL;
        }
        if (in_vc && original[p + 1] == CS_INTERFACE && n >= 3) {
            if (original[p + 2] == VC_HEADER && n >= 7)
                vc_header = (int)p;
            if (original[p + 2] == VC_INPUT_TERMINAL && n >= 4 &&
                original[p + 3] == CAMERA_TERMINAL_ID)
                camera = (int)p;
            if (original[p + 2] == VC_OUTPUT_TERMINAL && n >= 9 &&
                original[p + 3] == OUTPUT_TERMINAL_ID &&
                original[p + 7] == CAMERA_TERMINAL_ID)
                output = (int)p;
        }
        p += n;
    }
    if (vc_header < 0 || camera < 0 || output < 0 || camera >= output)
        return original;
    uint16_t vc_length = (uint16_t)(original[vc_header + 5] |
                                    ((uint16_t)original[vc_header + 6] << 8));
    if (vc_length > UINT16_MAX - sizeof(brightness_pu))
        return original;

    memcpy(config_with_pu, original, (size_t)output);
    memcpy(config_with_pu + output, brightness_pu, sizeof(brightness_pu));
    memcpy(config_with_pu + output + sizeof(brightness_pu),
           original + output, (size_t)(length - output));

    uint16_t new_length = (uint16_t)(length + sizeof(brightness_pu));
    uint16_t new_vc_length = (uint16_t)(vc_length + sizeof(brightness_pu));
    config_with_pu[2] = (uint8_t)new_length;
    config_with_pu[3] = (uint8_t)(new_length >> 8);
    config_with_pu[vc_header + 5] = (uint8_t)new_vc_length;
    config_with_pu[vc_header + 6] = (uint8_t)(new_vc_length >> 8);
    /* Output terminal's bSourceID points at our inserted PU. */
    config_with_pu[output + sizeof(brightness_pu) + 7] = PROCESSING_UNIT_ID;
    config_ready = true;
    ESP_LOGI(TAG, "UVC PU brightness enabled: AUTO=0 BODY=1 HOME=2");
    return config_with_pu;
}

/* EP0 control buffer must survive SETUP and DATA/ACK stages. TinyUSB
 * serialises control transfers, so one static buffer is sufficient. */
static uint16_t brightness_word;
static uint8_t brightness_info;

void thermalcam_uvc_control_init(void)
{
    /* Call from app_main before USB is initialized. Also keeps the linker
     * from discarding this translation unit and its --wrap entry points. */
    brightness_word = thermalcam_uvc_get_mode();
}

bool __wrap_videod_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                  tusb_control_request_t const *request)
{
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS ||
        request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE ||
        request->wIndex != ((PROCESSING_UNIT_ID << 8) | VC_INTERFACE) ||
        request->wValue != (PU_BRIGHTNESS << 8))
        return __real_videod_control_xfer_cb(rhport, stage, request);

    if (!config_ready) return false;

    if (request->bRequest == VIDEO_REQUEST_SET_CUR) {
        if (request->wLength != 2) return false;
        if (stage == CONTROL_STAGE_SETUP)
            return tud_control_xfer(rhport, request, &brightness_word,
                                    sizeof(brightness_word));
        if (stage == CONTROL_STAGE_DATA) {
            if (brightness_word >= THERMAL_MODE_COUNT) return false;
            thermalcam_uvc_set_mode((uint8_t)brightness_word);
            ESP_LOGI(TAG, "UVC brightness %u -> thermal mode %u",
                     (unsigned)brightness_word, (unsigned)brightness_word);
        }
        return true;
    }

    if (stage != CONTROL_STAGE_SETUP) return true;

    switch (request->bRequest) {
    case VIDEO_REQUEST_GET_INFO:
        if (request->wLength != 1) return false;
        brightness_info = 0x03; /* supports GET_CUR and SET_CUR */
        return tud_control_xfer(rhport, request, &brightness_info, 1);
    case VIDEO_REQUEST_GET_LEN:
        brightness_word = 2;
        break;
    case VIDEO_REQUEST_GET_CUR:
        brightness_word = thermalcam_uvc_get_mode();
        break;
    case VIDEO_REQUEST_GET_MIN:
    case VIDEO_REQUEST_GET_DEF:
        brightness_word = 0;
        break;
    case VIDEO_REQUEST_GET_MAX:
        brightness_word = THERMAL_MODE_COUNT - 1;
        break;
    case VIDEO_REQUEST_GET_RES:
        brightness_word = 1;
        break;
    default:
        return false;
    }
    if (request->wLength != 2) return false;
    return tud_control_xfer(rhport, request, &brightness_word,
                            sizeof(brightness_word));
}
