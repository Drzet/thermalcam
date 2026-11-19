/* S3 distance/parallax registration, expressed in 320x240 reference pixels. */
#pragma once
#include <stdint.h>
#define S3_REF_W 320
#define S3_REF_H 240
#define S3_MIN_CM 5
#define S3_MAX_CM 500
#define S3_ZOOM_MIN 100
#define S3_ZOOM_MAX 250
#define S3_PARALLAX_MAX100 9000
extern const int s3_distance_presets_cm[5];
typedef struct {
    int base_x100, base_y100, base_zx, base_zy, coefficient_100_cm;
    int offset_x100, offset_y100, offset_zx, offset_zy;
    int distance_cm, px100, py100, zx_pct, zy_pct;
} alignment_s3_t;
void alignment_s3_init(alignment_s3_t *a, int base_x, int base_y, int base_zx,
                       int base_zy, int coefficient, int distance);
void alignment_s3_set_offsets(alignment_s3_t *a, int x, int y, int zx, int zy);
void alignment_s3_set_distance(alignment_s3_t *a, int cm);
void alignment_s3_build_maps(const alignment_s3_t *a, int w, int h,
                             uint16_t *xmap, uint16_t *ymap);
