/* Adapted from multicamera_thermal.ino: recomputeAlignmentTransform(),
 * parallaxX100ForDistance() and cameraCropParamsFor().
 * Translates/zooms visible image only; thermal projection remains fixed. */
#include "alignment_s3.h"
#include <string.h>
const int s3_distance_presets_cm[5]={5,15,30,100,500};
static int lim(int v,int low,int high){return v<low?low:(v>high?high:v);}
void alignment_s3_set_distance(alignment_s3_t *a,int cm)
{
    if(!a)return;
    cm=lim(cm,S3_MIN_CM,S3_MAX_CM);
    a->distance_cm=cm;
    a->px100=lim(a->base_x100+(a->coefficient_100_cm+cm/2)/cm+a->offset_x100,
                  -S3_PARALLAX_MAX100,S3_PARALLAX_MAX100);
    a->py100=lim(a->base_y100+a->offset_y100,
                  -S3_PARALLAX_MAX100,S3_PARALLAX_MAX100);
    a->zx_pct=lim(a->base_zx+a->offset_zx,S3_ZOOM_MIN,S3_ZOOM_MAX);
    a->zy_pct=lim(a->base_zy+a->offset_zy,S3_ZOOM_MIN,S3_ZOOM_MAX);
}
void alignment_s3_init(alignment_s3_t *a,int bx,int by,int zx,int zy,
                       int coefficient,int cm)
{
    if(!a)return;
    memset(a,0,sizeof(*a));
    a->base_x100=bx; a->base_y100=by;
    a->base_zx=zx; a->base_zy=zy;
    a->coefficient_100_cm=coefficient;
    alignment_s3_set_distance(a,cm);
}
void alignment_s3_set_offsets(alignment_s3_t *a,int x,int y,int zx,int zy)
{
    if(!a)return;
    a->offset_x100=lim(x,-S3_PARALLAX_MAX100,S3_PARALLAX_MAX100);
    a->offset_y100=lim(y,-S3_PARALLAX_MAX100,S3_PARALLAX_MAX100);
    a->offset_zx=lim(zx,-50,50); a->offset_zy=lim(zy,-50,50);
    alignment_s3_set_distance(a,a->distance_cm);
}
/* Same Q16 source-coordinate and clamped nearest-neighbour crop as S3.
 * S3 pixel offsets use a 320x240 reference; P4 target is 480x320. */
static void build_axis(uint16_t *map,int extent,int ref,int zoom,int offset100)
{
    int crop=(extent*100+zoom/2)/zoom;
    if(crop>extent)crop=extent;
    int64_t shift=((int64_t)offset100*crop<<16)/(100*ref);
    int64_t start=((int64_t)(extent-crop)<<15)-shift;
    int64_t step=((int64_t)crop<<16)/extent;
    for(int i=0;i<extent;++i){
        int s=(int)((start+(int64_t)i*step)>>16);
        map[i]=(uint16_t)lim(s,0,extent-1);
    }
}
void alignment_s3_build_maps(const alignment_s3_t *a,int w,int h,
                             uint16_t *xmap,uint16_t *ymap)
{
    if(!a||!xmap||!ymap||w<=0||h<=0)return;
    build_axis(xmap,w,S3_REF_W,a->zx_pct,a->px100);
    build_axis(ymap,h,S3_REF_H,a->zy_pct,a->py100);
}
