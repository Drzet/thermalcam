#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/ppa.h"
#include "thermal.h"

/* PPA SRM: crop OV5647 800x640 to central 768x512 (3:2), scale 5/8
 * to 480x320. PPA BLEND: RGB565 visible + per-pixel-alpha ARGB thermal. */
typedef struct {
    ppa_client_handle_t scale_client;
    ppa_client_handle_t blend_client;
    uint16_t *visible565;
    uint16_t *mixed565;
    uint32_t *thermal_argb;
    fusion_alignment_t align;
    uint32_t overlay_sequence;
    thermal_mode_t overlay_mode;
    bool overlay_stale;
} fusion_t;

esp_err_t fusion_init(fusion_t *fusion);
esp_err_t fusion_render(fusion_t *fusion, uint8_t *bgr888,
                        const uint16_t *camera800x640,
                        const thermal_frame_t *thermal,
                        thermal_mode_t mode, bool stale, uint32_t age_ms);
void fusion_deinit(fusion_t *fusion);
