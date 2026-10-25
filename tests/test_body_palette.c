/* Host-side Body palette regression test; run from repository root:
 *   cc -std=c11 -O2 tests/test_body_palette.c -lm -o test_body_palette
 *   ./test_body_palette
 *
 * The production thermal.c is included directly so this exercises the actual
 * LUT indexing, clipping, rendering and colour-bar implementation.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../main/thermal.c"

static uint32_t output_rgb(const uint8_t *image, int x, int y)
{
    const uint8_t *bgr = image + ((size_t)y * IMAGE_W + x) * 3;
    return ((uint32_t)bgr[2] << 16) | ((uint32_t)bgr[1] << 8) | bgr[0];
}

static void fill_frame(thermal_frame_t *frame, float temperature)
{
    for (unsigned i = 0; i < THERMAL_PIXELS; ++i)
        frame->pixels[i] = temperature;
    frame->valid = true;
    frame->min = 20.0f;
    frame->max = 40.0f;
}

int main(void)
{
    thermal_init();
    const float body_scale = 255.0f / 15.0f;

    float lo, hi;
    thermal_frame_t frame = {0};
    fill_frame(&frame, 32.0f);
    thermal_range(&frame, MODE_BODY, &lo, &hi);
    assert(lo == 30.0f && hi == 45.0f);
    thermal_range(&frame, MODE_HOME, &lo, &hi);
    assert(lo == -20.0f && hi == 100.0f);
    thermal_range(&frame, MODE_AUTO, &lo, &hi);
    assert(lo == 20.0f && hi == 40.0f);

    /* Canonical Inferno cold/hot direction and all 256 distinct entries. */
    assert(bodyPalette[0] == 0x000004u);
    assert(bodyPalette[255] == 0xFCFFA4u);
    for (int i = 0; i < PALETTE_SIZE; ++i) {
        for (int j = 0; j < i; ++j)
            assert(bodyPalette[i] != bodyPalette[j]);
        float t = 30.0f + 15.0f * ((float)i / 255.0f);
        assert(tempToColorFast(t, 30.0f, body_scale, bodyPalette) == bodyPalette[i]);
        /* AUTO/HOME original gradient generation must remain intact. */
        assert(tempPalette[i] == makeGradientColor(1.0f - i / 255.0f));
    }

    assert(tempToColorFast(-50.0f, 30.0f, body_scale, bodyPalette) == bodyPalette[0]);
    assert(tempToColorFast(29.999f, 30.0f, body_scale, bodyPalette) == bodyPalette[0]);
    assert(tempToColorFast(30.0f, 30.0f, body_scale, bodyPalette) == bodyPalette[0]);
    assert(tempToColorFast(45.0f, 30.0f, body_scale, bodyPalette) == bodyPalette[255]);
    assert(tempToColorFast(45.001f, 30.0f, body_scale, bodyPalette) == bodyPalette[255]);
    assert(tempToColorFast(100.0f, 30.0f, body_scale, bodyPalette) == bodyPalette[255]);

    uint8_t *image = (uint8_t *)malloc(IMAGE_BYTES);
    assert(image);
    const int pixel_x = 128, pixel_y = 150;

    /* All pixels and sidebar endpoints use the Body palette, with clipping. */
    const float temperatures[] = {-10.0f, 30.0f, 45.0f, 70.0f};
    const uint32_t expected[] = {0x000004u, 0x000004u, 0xFCFFA4u, 0xFCFFA4u};
    for (unsigned k = 0; k < sizeof(temperatures)/sizeof(temperatures[0]); ++k) {
        fill_frame(&frame, temperatures[k]);
        thermal_render(image, &frame, MODE_BODY, false, 0);
        assert(output_rgb(image, pixel_x, pixel_y) == expected[k]);
        assert(output_rgb(image, 394, 95) == bodyPalette[255]);
        assert(output_rgb(image, 394, 295) == bodyPalette[0]);
    }

    /* AUTO and HOME keep exactly the previous LUT and their own ranges. */
    fill_frame(&frame, 32.0f);
    thermal_render(image, &frame, MODE_AUTO, false, 0);
    assert(output_rgb(image, pixel_x, pixel_y) ==
           tempToColorFast(32.0f, 20.0f, 255.0f / 20.0f, tempPalette));
    assert(output_rgb(image, 394, 95) == tempPalette[255]);
    assert(output_rgb(image, 394, 295) == tempPalette[0]);

    thermal_render(image, &frame, MODE_HOME, false, 0);
    assert(output_rgb(image, pixel_x, pixel_y) ==
           tempToColorFast(32.0f, -20.0f, 255.0f / 120.0f, tempPalette));
    assert(output_rgb(image, 394, 95) == tempPalette[255]);
    assert(output_rgb(image, 394, 295) == tempPalette[0]);

    free(image);
    puts("PASS: Body Inferno 256-index mapping, clipping, direction, sidebar; AUTO/HOME unchanged");
    return 0;
}
