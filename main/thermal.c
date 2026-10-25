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

/* Inferno: established 256-colour perceptually uniform sequential palette.
 * Nathaniel J. Smith and Stefan van der Walt; CC0/public domain.
 * Exact published BIDS/colormap inferno RGB data quantised to RGB888.
 * https://github.com/BIDS/colormap/blob/master/colormaps.py
 * Index 0 = cold (near black), 255 = hot (yellow-white).
 * Only BODY mode uses this table; AUTO/HOME retain tempPalette.
 */
static const uint32_t bodyPalette[PALETTE_SIZE] = {
  0x000004u, 0x010005u, 0x010106u, 0x010108u, 0x02010Au, 0x02020Cu, 0x02020Eu, 0x030210u,
  0x040312u, 0x040314u, 0x050417u, 0x060419u, 0x07051Bu, 0x08051Du, 0x09061Fu, 0x0A0722u,
  0x0B0724u, 0x0C0826u, 0x0D0829u, 0x0E092Bu, 0x10092Du, 0x110A30u, 0x120A32u, 0x140B34u,
  0x150B37u, 0x160B39u, 0x180C3Cu, 0x190C3Eu, 0x1B0C41u, 0x1C0C43u, 0x1E0C45u, 0x1F0C48u,
  0x210C4Au, 0x230C4Cu, 0x240C4Fu, 0x260C51u, 0x280B53u, 0x290B55u, 0x2B0B57u, 0x2D0B59u,
  0x2F0A5Bu, 0x310A5Cu, 0x320A5Eu, 0x340A5Fu, 0x360961u, 0x380962u, 0x390963u, 0x3B0964u,
  0x3D0965u, 0x3E0966u, 0x400A67u, 0x420A68u, 0x440A68u, 0x450A69u, 0x470B6Au, 0x490B6Au,
  0x4A0C6Bu, 0x4C0C6Bu, 0x4D0D6Cu, 0x4F0D6Cu, 0x510E6Cu, 0x520E6Du, 0x540F6Du, 0x550F6Du,
  0x57106Eu, 0x59106Eu, 0x5A116Eu, 0x5C126Eu, 0x5D126Eu, 0x5F136Eu, 0x61136Eu, 0x62146Eu,
  0x64156Eu, 0x65156Eu, 0x67166Eu, 0x69166Eu, 0x6A176Eu, 0x6C186Eu, 0x6D186Eu, 0x6F196Eu,
  0x71196Eu, 0x721A6Eu, 0x741A6Eu, 0x751B6Eu, 0x771C6Du, 0x781C6Du, 0x7A1D6Du, 0x7C1D6Du,
  0x7D1E6Du, 0x7F1E6Cu, 0x801F6Cu, 0x82206Cu, 0x84206Bu, 0x85216Bu, 0x87216Bu, 0x88226Au,
  0x8A226Au, 0x8C2369u, 0x8D2369u, 0x8F2469u, 0x902568u, 0x922568u, 0x932667u, 0x952667u,
  0x972766u, 0x982766u, 0x9A2865u, 0x9B2964u, 0x9D2964u, 0x9F2A63u, 0xA02A63u, 0xA22B62u,
  0xA32C61u, 0xA52C60u, 0xA62D60u, 0xA82E5Fu, 0xA92E5Eu, 0xAB2F5Eu, 0xAD305Du, 0xAE305Cu,
  0xB0315Bu, 0xB1325Au, 0xB3325Au, 0xB43359u, 0xB63458u, 0xB73557u, 0xB93556u, 0xBA3655u,
  0xBC3754u, 0xBD3853u, 0xBF3952u, 0xC03A51u, 0xC13A50u, 0xC33B4Fu, 0xC43C4Eu, 0xC63D4Du,
  0xC73E4Cu, 0xC83F4Bu, 0xCA404Au, 0xCB4149u, 0xCC4248u, 0xCE4347u, 0xCF4446u, 0xD04545u,
  0xD24644u, 0xD34743u, 0xD44842u, 0xD54A41u, 0xD74B3Fu, 0xD84C3Eu, 0xD94D3Du, 0xDA4E3Cu,
  0xDB503Bu, 0xDD513Au, 0xDE5238u, 0xDF5337u, 0xE05536u, 0xE15635u, 0xE25734u, 0xE35933u,
  0xE45A31u, 0xE55C30u, 0xE65D2Fu, 0xE75E2Eu, 0xE8602Du, 0xE9612Bu, 0xEA632Au, 0xEB6429u,
  0xEB6628u, 0xEC6726u, 0xED6925u, 0xEE6A24u, 0xEF6C23u, 0xEF6E21u, 0xF06F20u, 0xF1711Fu,
  0xF1731Du, 0xF2741Cu, 0xF3761Bu, 0xF37819u, 0xF47918u, 0xF57B17u, 0xF57D15u, 0xF67E14u,
  0xF68013u, 0xF78212u, 0xF78410u, 0xF8850Fu, 0xF8870Eu, 0xF8890Cu, 0xF98B0Bu, 0xF98C0Au,
  0xF98E09u, 0xFA9008u, 0xFA9207u, 0xFA9407u, 0xFB9606u, 0xFB9706u, 0xFB9906u, 0xFB9B06u,
  0xFB9D07u, 0xFC9F07u, 0xFCA108u, 0xFCA309u, 0xFCA50Au, 0xFCA60Cu, 0xFCA80Du, 0xFCAA0Fu,
  0xFCAC11u, 0xFCAE12u, 0xFCB014u, 0xFCB216u, 0xFCB418u, 0xFBB61Au, 0xFBB81Du, 0xFBBA1Fu,
  0xFBBC21u, 0xFBBE23u, 0xFAC026u, 0xFAC228u, 0xFAC42Au, 0xFAC62Du, 0xF9C72Fu, 0xF9C932u,
  0xF9CB35u, 0xF8CD37u, 0xF8CF3Au, 0xF7D13Du, 0xF7D340u, 0xF6D543u, 0xF6D746u, 0xF5D949u,
  0xF5DB4Cu, 0xF4DD4Fu, 0xF4DF53u, 0xF4E156u, 0xF3E35Au, 0xF3E55Du, 0xF2E661u, 0xF2E865u,
  0xF2EA69u, 0xF1EC6Du, 0xF1ED71u, 0xF1EF75u, 0xF1F179u, 0xF2F27Du, 0xF2F482u, 0xF3F586u,
  0xF3F68Au, 0xF4F88Eu, 0xF5F992u, 0xF6FA96u, 0xF8FB9Au, 0xF9FC9Du, 0xFAFDA1u, 0xFCFFA4u
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
  for (int i = 0; i < PALETTE_SIZE; ++i) {
    float normalized = i / 255.0f;
    float n = 1.0f - normalized;
    tempPalette[i] = makeGradientColor(n);
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
    snprintf(str,sizeof(str),"MIN %.1f",f->min); text(b,390,40,str,0xffffff,1);
    snprintf(str,sizeof(str),"MAX %.1f",f->max); text(b,390,51,str,0xffffff,1);
    snprintf(str,sizeof(str),"AGE %lu MS",(unsigned long)age_ms); text(b,390,62,str,0xffffff,1);
    snprintf(str,sizeof(str),"FIX %u",f->repaired); text(b,390,73,str,0xffffff,1);
    for(int y=95;y<=295;y++) for(int x=394;x<414;x++)
        pixel(b,x,y,palette[PALETTE_SIZE-1-((y-95)*(PALETTE_SIZE-1)+100)/200]);
    for(int i=0;i<=4;i++) {
        float t=hi-(hi-lo)*i/4.f; int y=95+i*50;
        snprintf(str,sizeof(str),"%.1f",t); text(b,422,y-2,str,0xffffff,1);
    }
    text(b,420,309,"C",0xffffff,1);
    if(stale) text(b,8,300,"STALE",0xff4040,2);
}
