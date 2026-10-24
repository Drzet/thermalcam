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

/* Original ESP32-S3 renderer from main/sketch_dec6a.ino, adapted only
 * for C and ESP32-P4's BGR24 JPEG input (instead of RGB565 LCD output).
 * The temperature interpolation and LUT indexing are unchanged. */
enum { MLX_W = 32, MLX_H = 24, VIEW_W = 384, VIEW_H = 288 };

typedef struct {
  uint8_t i0;
  uint8_t i1;
  float w0;
  float w1;
} SampleCoord;

static SampleCoord xMap[VIEW_W];
static SampleCoord yMap[VIEW_H];

static void initResampleMaps(void) {
  const float scaleX = (float)MLX_W / (float)VIEW_W;
  const float scaleY = (float)MLX_H / (float)VIEW_H;

  for (int x = 0; x < VIEW_W; ++x) {
    float srcX = (x + 0.5f) * scaleX - 0.5f;
    if (srcX < 0.0f) srcX = 0.0f;
    if (srcX >= MLX_W - 1) srcX = MLX_W - 1.001f;

    int x0 = (int)srcX;
    int x1 = x0 + 1;
    if (x1 >= MLX_W) x1 = MLX_W - 1;
    float dx = srcX - x0;

    xMap[x].i0 = (uint8_t)x0;
    xMap[x].i1 = (uint8_t)x1;
    xMap[x].w0 = 1.0f - dx;
    xMap[x].w1 = dx;
  }

  for (int y = 0; y < VIEW_H; ++y) {
    float srcY = (y + 0.5f) * scaleY - 0.5f;
    if (srcY < 0.0f) srcY = 0.0f;
    if (srcY >= MLX_H - 1) srcY = MLX_H - 1.001f;

    int y0 = (int)srcY;
    int y1 = y0 + 1;
    if (y1 >= MLX_H) y1 = MLX_H - 1;
    float dy = srcY - y0;

    yMap[y].i0 = (uint8_t)y0;
    yMap[y].i1 = (uint8_t)y1;
    yMap[y].w0 = 1.0f - dy;
    yMap[y].w1 = dy;
  }
}

// smooth 'heat' gradient via precomputed palette
constexpr int PALETTE_SIZE = 256;
static uint32_t tempPalette[PALETTE_SIZE];

static uint32_t makeGradientColor(float n) {
  float r, g, b;

  if (n <= 0.25f) {
    float k = n / 0.25f;
    r = 0.0f; g = 0.0f; b = k;
  } else if (n <= 0.50f) {
    float k = (n - 0.25f) / 0.25f;
    r = 0.0f; g = k; b = 1.0f;
  } else if (n <= 0.75f) {
    float k = (n - 0.50f) / 0.25f;
    r = k; g = 1.0f; b = 1.0f - k;
  } else {
    float k = (n - 0.75f) / 0.25f;
    r = 1.0f; g = 1.0f; b = k;
  }

  uint8_t R = (uint8_t)(r * 255.0f + 0.5f);
  uint8_t G = (uint8_t)(g * 255.0f + 0.5f);
  uint8_t B = (uint8_t)(b * 255.0f + 0.5f);

  return ((uint32_t)R << 16) | ((uint32_t)G << 8) | B;
}

static void initTempPalette(void) {
  for (int i = 0; i < PALETTE_SIZE; ++i) {
    float normalized = i / 255.0f;
    float n = 1.0f - normalized;
    tempPalette[i] = makeGradientColor(n);
  }
}

static inline uint32_t tempToColorFast(float t, float minTemp, float maxTemp) {
  if (t < minTemp) t = minTemp;
  if (t > maxTemp) t = maxTemp;

  float n = (t - minTemp) / (maxTemp - minTemp);
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;

  int idx = (int)(n * 255.0f + 0.5f);
  if (idx < 0) idx = 0;
  if (idx >= PALETTE_SIZE) idx = PALETTE_SIZE - 1;

  return tempPalette[idx];
}

void thermal_init(void)
{
  initTempPalette();
  initResampleMaps();
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
                    bool stale,uint32_t age_ms)
{
    static const char *names[]={"AUTO","BODY","HOME"};
    memset(b,0,IMAGE_BYTES);
    text(b,8,9,"THERMALCAM",0xffffff,2);
    text(b,128,9,!f->valid?"NO SENSOR":stale?"STALE":"USB CAMERA",
         (!f->valid||stale)?0xff4040:0xffffff,2);
    text(b,368,9,names[mode],0xffffff,2);
    if(!f->valid) { text(b,40,150,"WAITING FOR VALID FRAME",0xff4040,3); return; }
    float lo,hi; thermal_range(f,mode,&lo,&hi);
    renderFrame(b, f->pixels, lo, hi);
    char str[32];
    snprintf(str,sizeof(str),"MIN %.1f",f->min); text(b,390,40,str,0xffffff,1);
    snprintf(str,sizeof(str),"MAX %.1f",f->max); text(b,390,51,str,0xffffff,1);
    snprintf(str,sizeof(str),"AGE %lu MS",(unsigned long)age_ms); text(b,390,62,str,0xffffff,1);
    snprintf(str,sizeof(str),"FIX %u",f->repaired); text(b,390,73,str,0xffffff,1);
    for(int y=95;y<=295;y++) for(int x=394;x<414;x++)
        pixel(b,x,y,tempPalette[PALETTE_SIZE-1-((y-95)*(PALETTE_SIZE-1)+100)/200]);
    for(int i=0;i<=4;i++) {
        float t=hi-(hi-lo)*i/4.f; int y=95+i*50;
        snprintf(str,sizeof(str),"%.1f",t); text(b,422,y-2,str,0xffffff,1);
    }
    text(b,420,309,"C",0xffffff,1);
    if(stale) text(b,8,300,"STALE",0xff4040,2);
}
