#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <Arduino_GFX_Library.h>
#include "ThermalModes.h"
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include "wifiman.h"
#include "webs.h"

// ---------- DISPLAY PINS ----------
#define TFT_BL    16
#define TFT_DC    19
#define TFT_CS    39
#define TFT_RST   38
#define TFT_SCK   15
#define TFT_MOSI  7
#define TFT_MISO  17

// ---------- TOUCH PINS ----------
#define TOUCH_CLK 10   // T_CLK
#define TOUCH_CS  11   // T_CS
#define TOUCH_DIN 12   // T_DIN (MOSI)
#define TOUCH_DO  13   // T_DO  (MISO)
#define TOUCH_IRQ 14   // T_IRQ

// Hardware XPT2046 touch using separate SPI bus
SPIClass touchSPI(HSPI);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

float frameMin = 0.0f;
float frameMax = 0.0f;

// simple colour defs (RGB565)
static const uint16_t COLOR_BLACK = 0x0000;
static const uint16_t COLOR_WHITE = 0xFFFF;

// SPI bus and display - pointers only, allocate in setup()
Arduino_DataBus *bus = nullptr;
Arduino_GFX *gfx = nullptr;

// ---------- MLX90640 ----------
constexpr int MLX_W = 32;
constexpr int MLX_H = 24;

Adafruit_MLX90640 mlx;
float mlxFrame[MLX_W * MLX_H];

void fixNaNsAndComputeRange() {
  frameMin = 1e6f;
  frameMax = -1e6f;

  for (int y = 0; y < MLX_H; ++y) {
    for (int x = 0; x < MLX_W; ++x) {
      int idx = y * MLX_W + x;
      float t = mlxFrame[idx];

      if (isnan(t)) {
        float sum = 0.0f;
        int count = 0;

        if (x > 0) {
          float v = mlxFrame[y * MLX_W + (x - 1)];
          if (!isnan(v)) { sum += v; count++; }
        }
        if (x < MLX_W - 1) {
          float v = mlxFrame[y * MLX_W + (x + 1)];
          if (!isnan(v)) { sum += v; count++; }
        }
        if (y > 0) {
          float v = mlxFrame[(y - 1) * MLX_W + x];
          if (!isnan(v)) { sum += v; count++; }
        }
        if (y < MLX_H - 1) {
          float v = mlxFrame[(y + 1) * MLX_W + x];
          if (!isnan(v)) { sum += v; count++; }
        }

        if (count > 0) {
          t = sum / count;
        } else {
          t = 0.0f;
        }
        mlxFrame[idx] = t;
      }

      if (!isnan(t)) {
        if (t < frameMin) frameMin = t;
        if (t > frameMax) frameMax = t;
      }
    }
  }

  if (frameMax - frameMin < 0.5f) {
    frameMax = frameMin + 0.5f;
  }
}

// ---------- RENDER CONFIG ----------
constexpr int SCREEN_W = 480;
constexpr int SCREEN_H = 320;
constexpr int VIEW_W = 384;  // thermal image width on screen
constexpr int VIEW_H = 288;  // thermal image height on screen

// Frame buffer in PSRAM for ESP32-S3
uint16_t* frameBuffer = nullptr;

struct SampleCoord {
  uint8_t i0;
  uint8_t i1;
  float w0;
  float w1;
};

SampleCoord xMap[VIEW_W];
SampleCoord yMap[VIEW_H];

// Frame synchronization between capture/render and web snapshot
SemaphoreHandle_t frameMutex;

void initResampleMaps() {
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
uint16_t tempPalette[PALETTE_SIZE];

static uint16_t makeGradientColor(float n) {
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

  return ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3);
}

void initTempPalette() {
  for (int i = 0; i < PALETTE_SIZE; ++i) {
    float normalized = i / 255.0f;
    float n = 1.0f - normalized;
    tempPalette[i] = makeGradientColor(n);
  }
}

inline uint16_t tempToColorFast(float t, float minTemp, float maxTemp) {
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

// ---------- UI / TOP BAR LAYOUT ----------

constexpr int TOP_BAR_H       = 32;
constexpr int SAVE_BUTTON_W   = 120;
constexpr int MODE_BUTTON_W   = 120;
constexpr int SAVE_BUTTON_X   = 0;
constexpr int MODE_BUTTON_X   = SCREEN_W - MODE_BUTTON_W;
constexpr int WIFI_BUTTON_X   = SAVE_BUTTON_W;
constexpr int WIFI_BUTTON_W   = SCREEN_W - SAVE_BUTTON_W - MODE_BUTTON_W;

// ---------- UI DRAWING ----------

// Draw static top bar with three button-like regions for future touch control
void drawTopBarButtons() {
  const int barH = TOP_BAR_H;

  // Clear top bar area
  gfx->fillRect(0, 0, SCREEN_W, barH, COLOR_BLACK);

  gfx->setTextColor(COLOR_WHITE, COLOR_BLACK);
  gfx->setTextSize(2);

  // Save button (left, fixed width)
  int bxSave = SAVE_BUTTON_X;
  int bwSave = SAVE_BUTTON_W;
  int by = 0;
  gfx->drawRect(bxSave + 1, by + 1, bwSave - 2, barH - 2, COLOR_WHITE);
  gfx->setCursor(bxSave + 10, by + 8);
  gfx->print("Save");

  // WiFi/status button (middle, expanded)
  int bxWifi = WIFI_BUTTON_X;
  int bwWifi = WIFI_BUTTON_W;
  gfx->drawRect(bxWifi + 1, by + 1, bwWifi - 2, barH - 2, COLOR_WHITE);

  // Initial WiFi label will be drawn by updateWifiButtonLabel()

  // Mode button (right, fixed width, aligned to screen right)
  int bxMode = MODE_BUTTON_X;
  int bwMode = MODE_BUTTON_W;
  gfx->drawRect(bxMode + 1, by + 1, bwMode - 2, barH - 2, COLOR_WHITE);
  // Initial text will be drawn by updateModeButtonLabel()
}

// Update label of the third (Mode) button to "Mode-<name>"
void updateModeButtonLabel() {
  const int barH = TOP_BAR_H;
  const int bx = MODE_BUTTON_X;   // right-aligned button
  const int bw = MODE_BUTTON_W;
  const int by = 0;

  // Clear inside of the button (keep border)
  gfx->fillRect(bx + 2, by + 2, bw - 4, barH - 4, COLOR_BLACK);

  gfx->setTextColor(COLOR_WHITE, COLOR_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(bx + 6, by + 8);
  gfx->print("Mode-");
  gfx->print(getModeName());
}

void updateWifiButtonLabel() {
  const int barH = TOP_BAR_H;
  const int bx = WIFI_BUTTON_X;
  const int bw = WIFI_BUTTON_W;
  const int by = 0;

  // Clear inside of the button (keep border)
  gfx->fillRect(bx + 2, by + 2, bw - 4, barH - 4, COLOR_BLACK);

  gfx->setTextColor(COLOR_WHITE, COLOR_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(bx + 6, by + 8);

  if (!wifiManIsConnected()) {
    gfx->print("WiFi off");
    return;
  }

  WifiModeState mode = wifiManGetMode();
  String ip = wifiManGetIp();

  if (mode == WIFI_STATE_STA_PRESET) {
    gfx->print("STA ");
  } else {
    gfx->print("AP ");
  }
  gfx->print(ip);
}

// ---------- RENDERING ----------

// Render the current MLX frame (mlxFrame) into the RGB565 frameBuffer.
// Assumes frameMutex is already held by the caller.
void renderFrame(float minTemp, float maxTemp) {
  int bufIdx = 0;
  for (int y = 0; y < VIEW_H; ++y) {
    const SampleCoord& ym = yMap[y];

    for (int x = 0; x < VIEW_W; ++x) {
      const SampleCoord& xm = xMap[x];

      float t00 = mlxFrame[ym.i0 * MLX_W + xm.i0];
      float t10 = mlxFrame[ym.i0 * MLX_W + xm.i1];
      float t01 = mlxFrame[ym.i1 * MLX_W + xm.i0];
      float t11 = mlxFrame[ym.i1 * MLX_W + xm.i1];

      float tx0 = t00 * xm.w0 + t10 * xm.w1;
      float tx1 = t01 * xm.w0 + t11 * xm.w1;
      float t   = tx0 * ym.w0 + tx1 * ym.w1;

      frameBuffer[bufIdx++] = tempToColorFast(t, minTemp, maxTemp);
    }

    // Yield periodically to keep the system responsive
    if (y % 10 == 0) {
      yield();
    }
  }
}

// Display update (SPI transfer)
void drawThermalFrame() {
  const int startX = 0;
  const int startY = (SCREEN_H - VIEW_H);

  static bool sidebarInitialized = false;
  static ThermalMode lastMode = (ThermalMode)(-1);

  // Draw thermal image on the left
  gfx->draw16bitRGBBitmap(startX, startY, frameBuffer, VIEW_W, VIEW_H);

  // Sidebar on the right
  const int sidebarX = VIEW_W + 4;
  const int sidebarW = SCREEN_W - sidebarX;
  const int sidebarY = 0;
  const int sidebarH = SCREEN_H;

  // Draw static sidebar background once
  if (!sidebarInitialized) {
    gfx->fillRect(sidebarX, sidebarY, sidebarW, sidebarH, COLOR_BLACK);
    sidebarInitialized = true;
  }

  // Update Mode button label in top bar only when mode changes
  ThermalMode mode = getCurrentMode();
  if (mode != lastMode) {
    updateModeButtonLabel();
    lastMode = mode;
  }

  // Temperature range used for current frame
  float minTemp, maxTemp;
  getTemperatureRange(minTemp, maxTemp, frameMin, frameMax);

  // Palette geometry
  const int paletteX = sidebarX + 10;
  const int paletteY = 70;
  const int paletteW = 20;
  const int paletteH = SCREEN_H - paletteY - 10;

  // Vertical color palette bar
  for (int y = 0; y < paletteH; ++y) {
    float frac = 1.0f - (float)y / (float)(paletteH - 1);  // 0 bottom, 1 top
    int idx = (int)(frac * 255.0f + 0.5f);
    if (idx < 0) idx = 0;
    if (idx >= PALETTE_SIZE) idx = PALETTE_SIZE - 1;
    uint16_t c = tempPalette[idx];
    gfx->drawFastHLine(paletteX, paletteY + y, paletteW, c);
  }

  // Tick labels at fixed Celsius steps
  gfx->setTextSize(1);

  float range = maxTemp - minTemp;
  if (range < 1.0f) range = 1.0f;

  float step;
  if (range <= 10.0f)      step = 1.0f;
  else if (range <= 20.0f) step = 2.0f;
  else if (range <= 50.0f) step = 5.0f;
  else if (range <= 100.0f) step = 10.0f;
  else                     step = 20.0f;

  // Clear full label column once before drawing new ticks
  int labelX = paletteX + paletteW + 8;
  int labelW = sidebarX + sidebarW - 2 - labelX;
  gfx->fillRect(labelX, paletteY - 6, labelW, paletteH + 12, COLOR_BLACK);

  float tStart = ceil(minTemp / step) * step;
  for (float t = tStart; t <= maxTemp + 0.001f; t += step) {
    float frac = (t - minTemp) / range;  // 0..1
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    int y = paletteY + (int)((1.0f - frac) * (paletteH - 1));

    gfx->drawFastHLine(paletteX + paletteW + 1, y, 4, COLOR_WHITE);

    gfx->setCursor(labelX, y - 3);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", t);
    gfx->print(buf);
  }
}

// ---------- SETUP / LOOP ----------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting MLX90640 Thermal Cam - ESP32-S3");

  // Initialize WiFi manager and start in first mode
  wifiManInit();
  wifiManToggleMode();
  if (wifiManIsConnected()) {
    Serial.print("WiFi up, IP=");
    Serial.println(wifiManGetIp());
    websBegin(VIEW_W, VIEW_H);
  } else {
    Serial.println("WiFi not connected");
  }

  // Initialize mode system
  modesInit();

  // Allocate frame buffer (use PSRAM if available)
  if (psramFound()) {
    frameBuffer = (uint16_t*)ps_malloc(VIEW_W * VIEW_H * sizeof(uint16_t));
    Serial.printf("PSRAM found: %d bytes free\n", ESP.getFreePsram());
    Serial.println("Running single-core mode with PSRAM");
  } else {
    frameBuffer = (uint16_t*)malloc(VIEW_W * VIEW_H * sizeof(uint16_t));
    Serial.println("PSRAM not found, using heap");
    Serial.println("Running single-core mode without PSRAM");
  }

  if (!frameBuffer) {
    Serial.println("Failed to allocate frame buffer!");
    while (1) delay(100);
  }

  initTempPalette();
  initResampleMaps();

  // Create mutex for frame synchronization
  frameMutex = xSemaphoreCreateMutex();
  if (frameMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1) delay(100);
  }
  Serial.println("Mutex created");

  // Initialize display objects AFTER PSRAM setup
  bus = new Arduino_HWSPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
  gfx = new Arduino_ILI9488_18bit(bus, TFT_RST, 1, true);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);   // backlight on

  // Configure SPI speed before initializing display
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  SPI.setFrequency(40000000);  // 40MHz SPI

  Serial.println("Initializing display...");
  yield();  // Feed watchdog

  gfx->begin();
  yield();  // Feed watchdog after display init

  gfx->fillScreen(COLOR_BLACK);
  // Draw static UI buttons in the top bar: Save, WiFi, Mode
  drawTopBarButtons();
  updateModeButtonLabel();
  updateWifiButtonLabel();
  Serial.println("Display initialized");
  yield();  // Feed watchdog

  // Initialize touch controller (hardware SPI)
  Serial.println("Initializing touch controller...");
  touchSPI.begin(TOUCH_CLK, TOUCH_DO, TOUCH_DIN, TOUCH_CS);  // SCK, MISO, MOSI, SS
  touch.begin(touchSPI);
  Serial.println("Touch initialized");

  Wire.begin(41, 42);      // SDA, SCL
  Wire.setClock(400000);   // Start at 400kHz for stability with PSRAM
  Serial.println("I2C initialized at 400kHz");
  yield();  // Feed watchdog

  delay(100);  // Allow I2C to stabilize

  Serial.println("Looking for MLX90640...");
  yield();  // Feed watchdog before slow MLX init

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 init failed!");
    gfx->println("MLX init failed");
    while (1) delay(100);
  }

  Serial.println("MLX90640 detected");
  yield();  // Feed watchdog

  mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_8_HZ);

  Serial.println("MLX configured");
  yield();  // Feed watchdog

  // Now increase I2C speed for runtime
  Wire.setClock(1500000);  // 1MHz is safer with PSRAM
  Serial.println("I2C speed increased to 1.5MHz");

  Serial.println("Setup complete! Single-core mode (capture + render + UI + web).");
}

// Basic touch processing: map taps to top-bar buttons
void processTouch() {
  static bool wasTouched = false;

  if (!touch.touched()) {
    wasTouched = false;
    return;
  }

  // Simple edge detection to avoid repeats while finger is held
  if (wasTouched) return;
  wasTouched = true;

  TS_Point p = touch.getPoint();

  // Map raw touch to screen coordinates (needs calibration) [INFERRED]
  int16_t x = map((int)p.x, 0, 4095, 0, SCREEN_W);
  int16_t y = map((int)p.y, 0, 4095, SCREEN_H, 0);

  Serial.print("Touch screen: x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.println(y);

  if (y >= TOP_BAR_H) return;  // only top bar is interactive for now

  int index = -1;
  if (x >= SAVE_BUTTON_X && x < SAVE_BUTTON_X + SAVE_BUTTON_W) {
    index = 0;
  } else if (x >= WIFI_BUTTON_X && x < WIFI_BUTTON_X + WIFI_BUTTON_W) {
    index = 1;
  } else if (x >= MODE_BUTTON_X && x < MODE_BUTTON_X + MODE_BUTTON_W) {
    index = 2;
  } else {
    return;
  }

  switch (index) {
    case 0:
      Serial.println("Save button tapped");
      // TODO: implement save action
      break;
    case 1:
      Serial.println("WiFi button tapped");
      wifiManToggleMode();
      if (wifiManIsConnected()) {
        Serial.print("WiFi active, IP=");
        Serial.println(wifiManGetIp());
        websBegin(VIEW_W, VIEW_H);
      } else {
        Serial.println("WiFi disabled or not connected");
      }
      updateWifiButtonLabel();
      break;
    case 2: {
      Serial.println("Mode button tapped");
      ThermalMode mode = getCurrentMode();
      int next = ((int)mode + 1) % NUM_MODES;
      currentMode = (ThermalMode)next;
      // updateModeButtonLabel() will be invoked on next drawThermalFrame()
      break;
    }
    default:
      break;
  }
}

void loop() {
  // Update mode logic
  modesUpdate();

  // Single-core: capture MLX frame and render into frameBuffer
  if (xSemaphoreTake(frameMutex, portMAX_DELAY)) {
    mlx.getFrame(mlxFrame);
    fixNaNsAndComputeRange();

    float minTemp, maxTemp;
    getTemperatureRange(minTemp, maxTemp, frameMin, frameMax);
    renderFrame(minTemp, maxTemp);

    xSemaphoreGive(frameMutex);
  }

  // Draw current frame to the display
  drawThermalFrame();

  // Handle touch input
  processTouch();

  // Service web server
  websHandle();

  // Small delay to prevent a too-tight loop
  delay(10);
}
