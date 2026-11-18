#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* OV5647 RAW8 -> P4 ISP -> RGB565 through Espressif esp_video/V4L2.
 * GPIO7 SDA and GPIO8 SCL are the WT9932P4-TINY CSI connector pins. */
#define VISIBLE_W 800
#define VISIBLE_H 640
#define VISIBLE_BUFFER_COUNT 2

typedef struct {
    int fd;
    uint8_t *buffer[VISIBLE_BUFFER_COUNT];
    size_t size[VISIBLE_BUFFER_COUNT];
    unsigned count;
} visible_camera_t;

esp_err_t visible_camera_init(visible_camera_t *cam);
/* Frame stays owned by V4L2 until visible_camera_release(). */
bool visible_camera_capture(visible_camera_t *cam, const uint16_t **pixels,
                            unsigned *index);
void visible_camera_release(visible_camera_t *cam, unsigned index);
void visible_camera_close(visible_camera_t *cam);
