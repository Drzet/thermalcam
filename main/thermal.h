#pragma once
#include <stdbool.h>
#include <stdint.h>
#define THERMAL_PIXELS 768
#define IMAGE_W 480
#define IMAGE_H 320
#define IMAGE_BYTES (IMAGE_W * IMAGE_H * 3)
typedef struct {
    float pixels[THERMAL_PIXELS];
    float min, max;
    int64_t captured_us;
    uint32_t sequence;
    unsigned repaired;
    bool valid;
} thermal_frame_t;
typedef enum { MODE_AUTO, MODE_BODY, MODE_HOME, MODE_COUNT } thermal_mode_t;
bool thermal_validate(const float *input, thermal_frame_t *output);
void thermal_range(const thermal_frame_t *frame, thermal_mode_t mode, float *lo, float *hi);
/* BGR888 byte order: matches the P4 JPEG encoder's native RGB888 layout. */
void thermal_render(uint8_t *bgr, const thermal_frame_t *frame, thermal_mode_t mode,
                    bool frozen, bool stale, uint32_t age_ms);
