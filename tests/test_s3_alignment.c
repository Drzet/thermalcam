/* S3 reciprocal-distance registration: host-side regression test. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../main/alignment_s3.h"
#define W 480
#define H 320
int main(void)
{
    alignment_s3_t a;
    uint16_t x[W],y[H];
    alignment_s3_init(&a,125,-670,126,126,35654,30);
    assert(a.distance_cm==30);
    assert(a.px100==1313 && a.py100==-670);
    assert(a.zx_pct==126 && a.zy_pct==126);
    assert(s3_distance_presets_cm[0]==5 &&
           s3_distance_presets_cm[1]==15 &&
           s3_distance_presets_cm[2]==30 &&
           s3_distance_presets_cm[3]==100 &&
           s3_distance_presets_cm[4]==500);
    alignment_s3_build_maps(&a,W,H,x,y);
    int center30=x[W/2];
    for(int i=1;i<W;i++) assert(x[i]>=x[i-1] && x[i]<W);
    for(int i=1;i<H;i++) assert(y[i]>=y[i-1] && y[i]<H);
    alignment_s3_set_distance(&a,5);
    assert(a.px100==7256);
    alignment_s3_build_maps(&a,W,H,x,y);
    assert(x[W/2]<center30);
    alignment_s3_set_distance(&a,500);
    assert(a.px100==196);
    alignment_s3_build_maps(&a,W,H,x,y);
    assert(x[W/2]>center30);
    alignment_s3_set_distance(&a,-1);
    assert(a.distance_cm==5);
    alignment_s3_set_distance(&a,999);
    assert(a.distance_cm==500);
    alignment_s3_set_offsets(&a,100,200,10,-5);
    assert(a.px100==296 && a.py100==-470);
    assert(a.zx_pct==136 && a.zy_pct==121);
    puts("PASS: S3 manual distance presets, reciprocal parallax, visible crop, clamps");
    return 0;
}
