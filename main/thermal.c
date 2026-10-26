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
enum { PALETTE_SIZE = 256 };
static uint32_t tempPalette[PALETTE_SIZE];

/* Turbo colormap (Anton Mikhailov, Google), Apache-2.0.
 * Canonical 256-sample sRGB table, quantized once to RGB888.
 * Source: https://github.com/matplotlib/matplotlib/blob/main/lib/matplotlib/_cm_listed.py
 * Body only: index 0 = cold dark blue/purple; index 255 = hot dark red.
 * No runtime gradient generation; 30..45 deg C maps linearly to 0..255.
 */
static const uint32_t bodyPalette[PALETTE_SIZE] = {
  0x30123Bu, 0x321543u, 0x33184Au, 0x341B51u, 0x351E58u, 0x36215Fu, 0x372466u, 0x38276Du,
  0x392A73u, 0x3A2D79u, 0x3B2F80u, 0x3C3286u, 0x3D358Bu, 0x3E3891u, 0x3F3B97u, 0x3F3E9Cu,
  0x4040A2u, 0x4143A7u, 0x4146ACu, 0x4249B1u, 0x424BB5u, 0x434EBAu, 0x4451BFu, 0x4454C3u,
  0x4456C7u, 0x4559CBu, 0x455CCFu, 0x455ED3u, 0x4661D6u, 0x4664DAu, 0x4666DDu, 0x4669E0u,
  0x466BE3u, 0x476EE6u, 0x4771E9u, 0x4773EBu, 0x4776EEu, 0x4778F0u, 0x477BF2u, 0x467DF4u,
  0x4680F6u, 0x4682F8u, 0x4685FAu, 0x4687FBu, 0x458AFCu, 0x458CFDu, 0x448FFEu, 0x4391FEu,
  0x4294FFu, 0x4196FFu, 0x4099FFu, 0x3E9BFEu, 0x3D9EFEu, 0x3BA0FDu, 0x3AA3FCu, 0x38A5FBu,
  0x37A8FAu, 0x35ABF8u, 0x33ADF7u, 0x31AFF5u, 0x2FB2F4u, 0x2EB4F2u, 0x2CB7F0u, 0x2AB9EEu,
  0x28BCEBu, 0x27BEE9u, 0x25C0E7u, 0x23C3E4u, 0x22C5E2u, 0x20C7DFu, 0x1FC9DDu, 0x1ECBDAu,
  0x1CCDD8u, 0x1BD0D5u, 0x1AD2D2u, 0x1AD4D0u, 0x19D5CDu, 0x18D7CAu, 0x18D9C8u, 0x18DBC5u,
  0x18DDC2u, 0x18DEC0u, 0x18E0BDu, 0x19E2BBu, 0x19E3B9u, 0x1AE4B6u, 0x1CE6B4u, 0x1DE7B2u,
  0x1FE9AFu, 0x20EAACu, 0x22EBAAu, 0x25ECA7u, 0x27EEA4u, 0x2AEFA1u, 0x2CF09Eu, 0x2FF19Bu,
  0x32F298u, 0x35F394u, 0x38F491u, 0x3CF58Eu, 0x3FF68Au, 0x43F787u, 0x46F884u, 0x4AF880u,
  0x4EF97Du, 0x52FA7Au, 0x55FA76u, 0x59FB73u, 0x5DFC6Fu, 0x61FC6Cu, 0x65FD69u, 0x69FD66u,
  0x6DFE62u, 0x71FE5Fu, 0x75FE5Cu, 0x79FE59u, 0x7DFF56u, 0x80FF53u, 0x84FF51u, 0x88FF4Eu,
  0x8BFF4Bu, 0x8FFF49u, 0x92FF47u, 0x96FE44u, 0x99FE42u, 0x9CFE40u, 0x9FFD3Fu, 0xA1FD3Du,
  0xA4FC3Cu, 0xA7FC3Au, 0xA9FB39u, 0xACFB38u, 0xAFFA37u, 0xB1F936u, 0xB4F836u, 0xB7F735u,
  0xB9F635u, 0xBCF534u, 0xBEF434u, 0xC1F334u, 0xC3F134u, 0xC6F034u, 0xC8EF34u, 0xCBED34u,
  0xCDEC34u, 0xD0EA34u, 0xD2E935u, 0xD4E735u, 0xD7E535u, 0xD9E436u, 0xDBE236u, 0xDDE037u,
  0xDFDF37u, 0xE1DD37u, 0xE3DB38u, 0xE5D938u, 0xE7D739u, 0xE9D539u, 0xEBD339u, 0xECD13Au,
  0xEECF3Au, 0xEFCD3Au, 0xF1CB3Au, 0xF2C93Au, 0xF4C73Au, 0xF5C53Au, 0xF6C33Au, 0xF7C13Au,
  0xF8BE39u, 0xF9BC39u, 0xFABA39u, 0xFBB838u, 0xFBB637u, 0xFCB336u, 0xFCB136u, 0xFDAE35u,
  0xFDAC34u, 0xFEA933u, 0xFEA732u, 0xFEA431u, 0xFEA130u, 0xFE9E2Fu, 0xFE9B2Du, 0xFE992Cu,
  0xFE962Bu, 0xFE932Au, 0xFE9029u, 0xFD8D27u, 0xFD8A26u, 0xFC8725u, 0xFC8423u, 0xFB8122u,
  0xFB7E21u, 0xFA7B1Fu, 0xF9781Eu, 0xF9751Du, 0xF8721Cu, 0xF76F1Au, 0xF66C19u, 0xF56918u,
  0xF46617u, 0xF36315u, 0xF26014u, 0xF15D13u, 0xF05B12u, 0xEF5811u, 0xED5510u, 0xEC530Fu,
  0xEB500Eu, 0xEA4E0Du, 0xE84B0Cu, 0xE7490Cu, 0xE5470Bu, 0xE4450Au, 0xE2430Au, 0xE14109u,
  0xDF3F08u, 0xDD3D08u, 0xDC3B07u, 0xDA3907u, 0xD83706u, 0xD63506u, 0xD43305u, 0xD23105u,
  0xD02F05u, 0xCE2D04u, 0xCC2B04u, 0xCA2A04u, 0xC82803u, 0xC52603u, 0xC32503u, 0xC12302u,
  0xBE2102u, 0xBC2002u, 0xB91E02u, 0xB71D02u, 0xB41B01u, 0xB21A01u, 0xAF1801u, 0xAC1701u,
  0xA91601u, 0xA71401u, 0xA41301u, 0xA11201u, 0x9E1001u, 0x9B0F01u, 0x980E01u, 0x950D01u,
  0x920B01u, 0x8E0A01u, 0x8B0902u, 0x880802u, 0x850702u, 0x810602u, 0x7E0502u, 0x7A0403u
};


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
  /* Index increases with temperature: cold = dark/blue, hot = yellow/white.
   * Previously (1 - normalized) accidentally reversed this direction. */
  for (int i = 0; i < PALETTE_SIZE; ++i) {
    float normalized = i / 255.0f;
    tempPalette[i] = makeGradientColor(normalized);
  }
}

/* Keep the main-branch lookup table, but calculate the range conversion
 * once per image rather than dividing once per output pixel. */
static inline uint32_t tempToColorFast(float t, float minTemp, float scale,
                                       const uint32_t *palette)
{
  float index = (t - minTemp) * scale;
  if (index < 0.0f) index = 0.0f;
  if (index > (float)(PALETTE_SIZE - 1)) index = (float)(PALETTE_SIZE - 1);
  return palette[(int)(index + 0.5f)];
}

/* Two reusable horizontal scanlines: 2 * 384 * sizeof(float) = 3072 B.
 * Separability of bilinear interpolation lets each source row be
 * interpolated horizontally once, then reused across the output rows. */
static float horizontalRows[2][VIEW_W];

static void interpolateHorizontal(float *out, const float *sourceRow)
{
  for (int x = 0; x < VIEW_W; ++x) {
    const SampleCoord *xm = &xMap[x];
    out[x] = sourceRow[xm->i0] * xm->w0 +
             sourceRow[xm->i1] * xm->w1;
  }
}

static void renderFrame(uint8_t *bgr, const float *mlxFrame, float minTemp, float maxTemp,
                        const uint32_t *palette)
{
  const float colourScale = (float)(PALETTE_SIZE - 1) / (maxTemp - minTemp);
  float *top = horizontalRows[0], *bottom = horizontalRows[1];
  int topRow = -1, bottomRow = -1;

  for (int y = 0; y < VIEW_H; ++y) {
    const SampleCoord *ym = &yMap[y];

    /* The next top row is normally the previous bottom row.
     * Reuse it instead of repeating its horizontal interpolation. */
    if (ym->i0 == bottomRow) {
      float *tmp = top;
      top = bottom;
      bottom = tmp;
      topRow = bottomRow;
      bottomRow = -1;
    }

    if (topRow != ym->i0) {
      interpolateHorizontal(top, mlxFrame + ym->i0 * MLX_W);
      topRow = ym->i0;
    }
    if (bottomRow != ym->i1) {
      interpolateHorizontal(bottom, mlxFrame + ym->i1 * MLX_W);
      bottomRow = ym->i1;
    }

    uint8_t *dst = bgr + ((y + 32) * IMAGE_W) * 3;
    for (int x = 0; x < VIEW_W; ++x, dst += 3) {
      float t = top[x] * ym->w0 + bottom[x] * ym->w1;
      uint32_t rgb = tempToColorFast(t, minTemp, colourScale, palette);
      dst[0] = (uint8_t)rgb;
      dst[1] = (uint8_t)(rgb >> 8);
      dst[2] = (uint8_t)(rgb >> 16);
    }
  }
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
    const uint32_t *palette = (mode == MODE_BODY) ? bodyPalette : tempPalette;
    renderFrame(b, f->pixels, lo, hi, palette);
    char str[32];
    snprintf(str,sizeof(str),"MIN %.1f",f->min); text(b,390,40,str,0xffffff,2);
    snprintf(str,sizeof(str),"MAX %.1f",f->max); text(b,390,54,str,0xffffff,2);
    snprintf(str,sizeof(str),"AGE %lu MS",(unsigned long)age_ms); text(b,390,70,str,0xffffff,1);
    snprintf(str,sizeof(str),"FIX %u",f->repaired); text(b,390,81,str,0xffffff,1);
    for(int y=95;y<=295;y++) for(int x=394;x<414;x++)
        pixel(b,x,y,palette[PALETTE_SIZE-1-((y-95)*(PALETTE_SIZE-1)+100)/200]);
    for(int i=0;i<=4;i++) {
        float t=hi-(hi-lo)*i/4.f; int y=95+i*50;
        snprintf(str,sizeof(str),"%.1f",t); text(b,422,y-2,str,0xffffff,2);
    }
    text(b,420,309,"C",0xffffff,1);
    if(stale) text(b,8,300,"STALE",0xff4040,2);
}
