#include "thermal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

bool thermal_validate(const float *in, thermal_frame_t *out)
{
    unsigned bad = 0;
    for (int i = 0; i < THERMAL_PIXELS; ++i) if (!isfinite(in[i])) ++bad;
    /* Repair isolated defects only, never manufacture a whole image. */
    if (bad > 4) return false;
    float lo = INFINITY, hi = -INFINITY;
    for (int i = 0; i < THERMAL_PIXELS; ++i) {
        float t = in[i];
        if (!isfinite(t)) {
            float sum = 0; unsigned n = 0;
            int x = i % 32, y = i / 32;
            int neighbours[4] = { x ? i-1 : -1, x < 31 ? i+1 : -1,
                                  y ? i-32 : -1, y < 23 ? i+32 : -1 };
            for (unsigned k = 0; k < 4; ++k) {
                int j = neighbours[k];
                if (j >= 0 && isfinite(in[j])) { sum += in[j]; ++n; }
            }
            if (!n) return false;
            t = sum / n;
        }
        if (!isfinite(t)) return false;
        out->pixels[i] = t;
        if (t < lo) lo = t;
        if (t > hi) hi = t;
    }
    out->min = lo; out->max = hi; out->repaired = bad; out->valid = true;
    return true;
}

void thermal_range(const thermal_frame_t *f, thermal_mode_t mode, float *lo, float *hi)
{
    if (mode == MODE_BODY) { *lo = 30; *hi = 45; }
    else if (mode == MODE_HOME) { *lo = -20; *hi = 100; }
    else { *lo = f->min; *hi = fmaxf(f->max, f->min + 0.5f); }
}

static void pixel(uint8_t *b, int x, int y, uint32_t rgb)
{
    if (x < 0 || x >= IMAGE_W || y < 0 || y >= IMAGE_H) return;
    uint8_t *p = b + (y * IMAGE_W + x) * 3;
    p[0] = rgb; p[1] = rgb >> 8; p[2] = rgb >> 16;
}

static uint32_t colour(float n)
{
    n = 1.0f - fminf(1, fmaxf(0, n)); /* Preserve original cold-white/hot-black palette. */
    float r, g, b;
    if (n <= .25f) { r=0; g=0; b=n*4; }
    else if (n <= .5f) { r=0; g=(n-.25f)*4; b=1; }
    else if (n <= .75f) { r=(n-.5f)*4; g=1; b=1-r; }
    else { r=1; g=1; b=(n-.75f)*4; }
    return ((uint32_t)(r*255)<<16) | ((uint32_t)(g*255)<<8) | (uint32_t)(b*255);
}

/* Original 3x5 bitmap alphabet; each row is three bits, top row first. */
static uint16_t glyph(char c)
{
    static const uint16_t digits[] = {0x7b6f,0x2492,0x73e7,0x73cf,0x5bc9,0x79cf,0x79ef,0x7249,0x7bef,0x7bcf};
    if(c>='0' && c<='9') return digits[c-'0'];
    switch(c) {
    case 'A': return 0x2bed; case 'B': return 0x6bae; case 'C': return 0x7927;
    case 'D': return 0x6b6e; case 'E': return 0x79e7; case 'F': return 0x79e4;
    case 'G': return 0x796f; case 'H': return 0x5bed; case 'I': return 0x7497;
    case 'J': return 0x124e; case 'K': return 0x5bad; case 'L': return 0x4927;
    case 'M': return 0x5fed; case 'N': return 0x5ffd; case 'O': return 0x7b6f;
    case 'P': return 0x7be4; case 'Q': return 0x7b7b; case 'R': return 0x6bad;
    case 'S': return 0x79cf; case 'T': return 0x7492; case 'U': return 0x5b6f;
    case 'V': return 0x5b6a; case 'W': return 0x5bfd; case 'X': return 0x5aad;
    case 'Y': return 0x5a92; case 'Z': return 0x72a7; case '-': return 0x01c0;
    case '.': return 0x0002; case ':': return 0x0410; default: return 0;
    }
}
static void text(uint8_t *b,int x,int y,const char *s,uint32_t color,int scale)
{
    for (; *s; ++s,x+=4*scale) {
        uint16_t bits=glyph(*s);
        for(int row=0;row<5;row++) for(int col=0;col<3;col++)
            if(bits & (1u << (14-row*3-col)))
                for(int dy=0;dy<scale;dy++) for(int dx=0;dx<scale;dx++)
                    pixel(b,x+col*scale+dx,y+row*scale+dy,color);
    }
}

void thermal_render(uint8_t *b,const thermal_frame_t *f,thermal_mode_t mode,
                    bool frozen,bool stale,uint32_t age_ms)
{
    static const char *names[]={"AUTO","BODY","HOME"};
    memset(b,0,IMAGE_BYTES);
    text(b,8,9,frozen?"RESUME":"FREEZE",0xffffff,2);
    text(b,128,9,!f->valid?"NO SENSOR":stale?"STALE":frozen?"FROZEN":"USB CAMERA",
         (!f->valid||stale)?0xff4040:0xffffff,2);
    text(b,368,9,names[mode],0xffffff,2);
    if(!f->valid) { text(b,40,150,"WAITING FOR VALID FRAME",0xff4040,3); return; }
    float lo,hi; thermal_range(f,mode,&lo,&hi);
    for(int y=0;y<288;y++) {
        float sy=fminf(23,fmaxf(0,(y+.5f)/12-.5f));
        int y0=(int)sy,y1=y0<23?y0+1:y0; float fy=sy-y0;
        for(int x=0;x<384;x++) {
            float sx=fminf(31,fmaxf(0,(x+.5f)/12-.5f));
            int x0=(int)sx,x1=x0<31?x0+1:x0; float fx=sx-x0;
            float a=f->pixels[y0*32+x0]*(1-fx)+f->pixels[y0*32+x1]*fx;
            float c=f->pixels[y1*32+x0]*(1-fx)+f->pixels[y1*32+x1]*fx;
            pixel(b,x,y+32,colour((a*(1-fy)+c*fy-lo)/(hi-lo)));
        }
    }
    char str[32];
    snprintf(str,sizeof(str),"MIN %.1f",f->min); text(b,390,40,str,0xffffff,1);
    snprintf(str,sizeof(str),"MAX %.1f",f->max); text(b,390,51,str,0xffffff,1);
    snprintf(str,sizeof(str),"AGE %lu MS",(unsigned long)age_ms); text(b,390,62,str,0xffffff,1);
    snprintf(str,sizeof(str),"FIX %u",f->repaired); text(b,390,73,str,0xffffff,1);
    for(int y=95;y<=295;y++) for(int x=394;x<414;x++)
        pixel(b,x,y,colour(1-(y-95)/200.f));
    for(int i=0;i<=4;i++) {
        float t=hi-(hi-lo)*i/4.f; int y=95+i*50;
        snprintf(str,sizeof(str),"%.1f",t); text(b,422,y-2,str,0xffffff,1);
    }
    text(b,420,309,"C",0xffffff,1);
    if(stale||frozen) text(b,8,300,stale?"STALE":"FROZEN",0xff4040,2);
}
