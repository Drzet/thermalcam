#include "fusion.h"
#include <string.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "visible_camera.h"

static const char *TAG = "fusion";

/* Image crop chosen so PPA 5/8 scaling is exact at its 1/16 step:
 * OV5647 800x640 -> central 768x512 -> UVC 480x320.
 * This is an image crop, NOT a thermal-to-visible optical calibration. */
#define CROP_W 768
#define CROP_H 512
#define CROP_X 16
#define CROP_Y 64
#define SCALE_RATIO 0.625f

static uint16_t *new_buffer(size_t size)
{
    return heap_caps_aligned_alloc(64, size,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void fusion_deinit(fusion_t *fusion)
{
    if (!fusion) return;
    if (fusion->scale_client) ppa_unregister_client(fusion->scale_client);
    if (fusion->blend_client) ppa_unregister_client(fusion->blend_client);
    free(fusion->visible565);
    free(fusion->mixed565);
    free(fusion->thermal_argb);
    memset(fusion, 0, sizeof(*fusion));
}

esp_err_t fusion_init(fusion_t *fusion)
{
    if (!fusion) return ESP_ERR_INVALID_ARG;
    memset(fusion, 0, sizeof(*fusion));
    fusion->align = (fusion_alignment_t) {
        .center_x = IMAGE_W / 2 + CONFIG_THERMALCAM_FUSION_OFFSET_X,
        .center_y = IMAGE_H / 2 + CONFIG_THERMALCAM_FUSION_OFFSET_Y,
        .width = CONFIG_THERMALCAM_FUSION_WIDTH,
        .height = CONFIG_THERMALCAM_FUSION_HEIGHT,
        .alpha = CONFIG_THERMALCAM_FUSION_ALPHA,
        .mirror_x = CONFIG_THERMALCAM_FUSION_MIRROR_X,
        .mirror_y = CONFIG_THERMALCAM_FUSION_MIRROR_Y,
    };
    size_t px = IMAGE_W * IMAGE_H;
    fusion->visible565 = new_buffer(px * sizeof(uint16_t));
    fusion->mixed565 = new_buffer(px * sizeof(uint16_t));
    fusion->thermal_argb = (uint32_t *)new_buffer(px * sizeof(uint32_t));
    if (!fusion->visible565 || !fusion->mixed565 || !fusion->thermal_argb) {
        fusion_deinit(fusion);
        return ESP_ERR_NO_MEM;
    }
    ppa_client_config_t scale = { .oper_type = PPA_OPERATION_SRM };
    ppa_client_config_t blend = { .oper_type = PPA_OPERATION_BLEND };
    esp_err_t err = ppa_register_client(&scale, &fusion->scale_client);
    if (err == ESP_OK) err = ppa_register_client(&blend, &fusion->blend_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPA client registration failed: %s", esp_err_to_name(err));
        fusion_deinit(fusion);
        return err;
    }
    fusion->overlay_sequence = UINT32_MAX;
    ESP_LOGI(TAG, "PPA scale 800x640 -> 480x320; overlay %dx%d, alpha %u/255",
             fusion->align.width, fusion->align.height, fusion->align.alpha);
    return ESP_OK;
}

static void to_bgr888(uint8_t *dest, const uint16_t *src)
{
    for (size_t i = 0; i < (size_t)IMAGE_W * IMAGE_H; ++i) {
        uint16_t p = src[i];
        dest[3 * i] = (uint8_t)((p & 0x1f) * 255 / 31);
        dest[3 * i + 1] = (uint8_t)(((p >> 5) & 0x3f) * 255 / 63);
        dest[3 * i + 2] = (uint8_t)(((p >> 11) & 0x1f) * 255 / 31);
    }
}

esp_err_t fusion_render(fusion_t *fusion, uint8_t *bgr888,
                        const uint16_t *camera800x640,
                        const thermal_frame_t *thermal,
                        thermal_mode_t mode, bool stale, uint32_t age_ms)
{
    if (!fusion || !bgr888 || !camera800x640 || !thermal)
        return ESP_ERR_INVALID_ARG;

    ppa_srm_oper_config_t scale = {
        .in = {
            .buffer = camera800x640,
            .pic_w = VISIBLE_W, .pic_h = VISIBLE_H,
            .block_w = CROP_W, .block_h = CROP_H,
            .block_offset_x = CROP_X, .block_offset_y = CROP_Y,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = fusion->visible565,
            .buffer_size = IMAGE_W * IMAGE_H * sizeof(uint16_t),
            .pic_w = IMAGE_W, .pic_h = IMAGE_H,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .scale_x = SCALE_RATIO, .scale_y = SCALE_RATIO,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    esp_err_t err = ppa_do_scale_rotate_mirror(fusion->scale_client, &scale);
    if (err != ESP_OK) return err;

    /* Thermal is acquired at 8 Hz. Regenerate its projected colour layer
     * only on a new sensor frame, range-mode change or stale-state change;
     * reuse it over intermediate 30 Hz visible frames. */
    if (fusion->overlay_sequence != thermal->sequence ||
        fusion->overlay_mode != mode || fusion->overlay_stale != stale) {
        thermal_render_argb_overlay(fusion->thermal_argb, thermal, mode,
                                    stale, &fusion->align);
        fusion->overlay_sequence = thermal->sequence;
        fusion->overlay_mode = mode;
        fusion->overlay_stale = stale;
    }

    ppa_blend_oper_config_t blend = {
        .in_bg = {
            .buffer = fusion->visible565,
            .pic_w = IMAGE_W, .pic_h = IMAGE_H,
            .block_w = IMAGE_W, .block_h = IMAGE_H,
            .blend_cm = PPA_BLEND_COLOR_MODE_RGB565,
        },
        .in_fg = {
            .buffer = fusion->thermal_argb,
            .pic_w = IMAGE_W, .pic_h = IMAGE_H,
            .block_w = IMAGE_W, .block_h = IMAGE_H,
            .blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888,
        },
        .out = {
            .buffer = fusion->mixed565,
            .buffer_size = IMAGE_W * IMAGE_H * sizeof(uint16_t),
            .pic_w = IMAGE_W, .pic_h = IMAGE_H,
            .blend_cm = PPA_BLEND_COLOR_MODE_RGB565,
        },
        .bg_alpha_update_mode = PPA_ALPHA_FIX_VALUE,
        .bg_alpha_fix_val = 255,
        .fg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    err = ppa_do_blend(fusion->blend_client, &blend);
    if (err != ESP_OK) return err;

    to_bgr888(bgr888, fusion->mixed565);
    thermal_render_fusion_status(bgr888, thermal, mode, stale, age_ms);
    return ESP_OK;
}
