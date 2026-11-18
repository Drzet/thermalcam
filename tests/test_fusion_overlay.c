/* Host regression test for the 480x320 transparent overlay; compile:
 * cc -std=c11 -O2 tests/test_fusion_overlay.c -lm -o /tmp/fusion_overlay_test
 * /tmp/fusion_overlay_test
 */
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../main/thermal.c"

#define COUNT ((size_t)IMAGE_W * IMAGE_H)

static void fill(thermal_frame_t *f, float temperature)
{
    memset(f, 0, sizeof(*f));
    f->valid = true;
    f->min = 30; f->max = 45;
    for (unsigned i = 0; i < THERMAL_PIXELS; ++i)
        f->pixels[i] = temperature;
}

int main(void)
{
    thermal_init();
    thermal_frame_t *f = calloc(1, sizeof(*f));
    uint32_t *guard = malloc((COUNT + 2) * sizeof(uint32_t));
    assert(f && guard);
    uint32_t *overlay = guard + 1;
    fusion_alignment_t align = {
        .center_x = 240, .center_y = 160,
        .width = 384, .height = 288, .alpha = 100,
    };

    guard[0] = guard[COUNT + 1] = 0xdeadbeefu;
    fill(f, 30.0f);
    thermal_render_argb_overlay(overlay, f, MODE_BODY, false, &align);
    assert(overlay[0] == 0); /* Outside optical projection is transparent */
    assert(overlay[160 * IMAGE_W + 240] == (100u << 24 | bodyPalette[0]));
    assert(guard[0] == 0xdeadbeefu && guard[COUNT + 1] == 0xdeadbeefu);

    fill(f, 45.0f);
    thermal_render_argb_overlay(overlay, f, MODE_BODY, false, &align);
    assert(overlay[160 * IMAGE_W + 240] == (100u << 24 | bodyPalette[255]));
    thermal_render_argb_overlay(overlay, f, MODE_BODY, true, &align);
    assert(overlay[160 * IMAGE_W + 240] == 0);
    f->valid = false;
    thermal_render_argb_overlay(overlay, f, MODE_BODY, false, &align);
    assert(overlay[160 * IMAGE_W + 240] == 0);

    fill(f, 32.5f);
    align.center_x = 0; align.center_y = 0;
    align.mirror_x = true; align.mirror_y = true;
    thermal_render_argb_overlay(overlay, f, MODE_BODY, false, &align);
    assert(overlay[0] != 0 && overlay[IMAGE_W * IMAGE_H - 1] == 0);
    assert(guard[0] == 0xdeadbeefu && guard[COUNT + 1] == 0xdeadbeefu);
    free(guard); free(f);
    puts("PASS: fusion alpha, temperature LUT, stale mask, alignment, clipping, guards");
    return 0;
}
