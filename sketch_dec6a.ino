#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <Arduino_GFX_Library.h>
#include "ThermalModes.h"

// ---------- DISPLAY PINS ----------
#define TFT_BL    16
#define TFT_DC    19
#define TFT_CS    39
#define TFT_RST   38
#define TFT_SCK   15
#define TFT_MOSI  7
#define TFT_MISO  17

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

// MLX90640 object temperature range
constexpr float TEMP_MIN = -40.0f;
constexpr float TEMP_MAX = 300.0f;

// ---------- RENDER CONFIG ----------
constexpr int VIEW_W = 320;  // thermal image size on screen
constexpr int VIEW_H = 240;

// Frame buffer in PSRAM for ESP32-S3 (8MB available)
uint16_t* frameBuffer = nullptr;

// Dual-core synchronization (disabled with PSRAM to avoid conflicts)
TaskHandle_t renderTaskHandle = NULL;
volatile bool frameReady = false;
SemaphoreHandle_t frameMutex;
bool useDualCore = false;  // Set based on PSRAM availability

// find min/max in current frame
void computeFrameRange() {
  frameMin = 1e6f;
  frameMax = -1e6f;

  for (int i = 0; i < MLX_W * MLX_H; ++i) {
    float t = mlxFrame[i];
    if (isnan(t)) continue;
    if (t < frameMin) frameMin = t;
    if (t > frameMax) frameMax = t;
  }

  if (frameMax - frameMin < 0.5f) {   // avoid div/0 and flat gradients
    frameMax = frameMin + 0.5f;
  }
}

// smooth 'heat' gradient: black → blue → cyan → yellow → white
uint16_t tempToColor(float t, float minTemp, float maxTemp) {
  if (t < minTemp) t = minTemp;
  if (t > maxTemp) t = maxTemp;
  float n = (t - minTemp) / (maxTemp - minTemp);
  n = 1.0f - n;  // invert so hot = white, cold = black

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

// ---------- RENDERING ----------
// Core 0: Render frame buffer (computation-heavy)
void renderTask(void* parameter) {
  while (true) {
    // Wait for new frame data
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    if (xSemaphoreTake(frameMutex, portMAX_DELAY)) {
      const float scaleX = (float)MLX_W / (float)VIEW_W;
      const float scaleY = (float)MLX_H / (float)VIEW_H;
      
      // Get temperature range based on current mode
      float minTemp, maxTemp;
      getTemperatureRange(minTemp, maxTemp, frameMin, frameMax);

      int bufIdx = 0;
      for (int y = 0; y < VIEW_H; ++y) {
        float srcY = (y + 0.5f) * scaleY - 0.5f;
        if (srcY < 0.0f) srcY = 0.0f;
        if (srcY >= MLX_H - 1) srcY = MLX_H - 1.001f;
        
        int y0 = (int)srcY;
        int y1 = y0 + 1;
        if (y1 >= MLX_H) y1 = MLX_H - 1;
        float dy = srcY - y0;
        float wy0 = 1.0f - dy;
        float wy1 = dy;

        for (int x = 0; x < VIEW_W; ++x) {
          float srcX = (x + 0.5f) * scaleX - 0.5f;
          if (srcX < 0.0f) srcX = 0.0f;
          if (srcX >= MLX_W - 1) srcX = MLX_W - 1.001f;
          
          int x0 = (int)srcX;
          int x1 = x0 + 1;
          if (x1 >= MLX_W) x1 = MLX_W - 1;
          float dx = srcX - x0;
          
          float t00 = mlxFrame[y0 * MLX_W + x0];
          float t10 = mlxFrame[y0 * MLX_W + x1];
          float t01 = mlxFrame[y1 * MLX_W + x0];
          float t11 = mlxFrame[y1 * MLX_W + x1];
          
          float t = (t00 * (1.0f - dx) + t10 * dx) * wy0 +
                    (t01 * (1.0f - dx) + t11 * dx) * wy1;
          
          frameBuffer[bufIdx++] = tempToColor(t, minTemp, maxTemp);
        }
        
        // Feed watchdog every 10 lines
        if (y % 10 == 0) {
          vTaskDelay(1);
        }
      }
      
      xSemaphoreGive(frameMutex);
      frameReady = true;
    }
  }
}

// Core 1: Display update (SPI transfer)
void drawThermalFrame() {
  const int screenW = 480;
  const int screenH = 320;
  const int startX = (screenW - VIEW_W) / 2;
  const int startY = (screenH - VIEW_H) / 2;
  
  if (frameReady) {
    gfx->draw16bitRGBBitmap(startX, startY, frameBuffer, VIEW_W, VIEW_H);
    frameReady = false;
  }
}

// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting MLX90640 Thermal Cam - ESP32-S3");
  
  // Initialize mode system
  modesInit();
  
  // Allocate frame buffer in PSRAM
  if (psramFound()) {
    frameBuffer = (uint16_t*)ps_malloc(VIEW_W * VIEW_H * sizeof(uint16_t));
    Serial.printf("PSRAM found: %d bytes free\n", ESP.getFreePsram());
    useDualCore = false;  // Disable dual-core with PSRAM to avoid conflicts
    Serial.println("Running in single-core mode due to PSRAM");
  } else {
    frameBuffer = (uint16_t*)malloc(VIEW_W * VIEW_H * sizeof(uint16_t));
    Serial.println("PSRAM not found, using heap");
    useDualCore = true;  // Enable dual-core when no PSRAM
  }
  
  if (!frameBuffer) {
    Serial.println("Failed to allocate frame buffer!");
    while (1) delay(100);
  }
  
  // Create mutex only if using dual-core
  if (useDualCore) {
    frameMutex = xSemaphoreCreateMutex();
    if (frameMutex == NULL) {
      Serial.println("Failed to create mutex!");
      while (1) delay(100);
    }
    Serial.println("Mutex created");
  }
  
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
  gfx->setCursor(10, 10);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->println("MLX90640 Thermal Cam");
  Serial.println("Display initialized");
  yield();  // Feed watchdog

  Wire.begin(41, 42);      // SDA, SCL
  Wire.setClock(400000);   // Start slower at 400kHz for stability with PSRAM
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
  Wire.setClock(1000000);  // 1MHz is safer with PSRAM than 1.5MHz
  Serial.println("I2C speed increased to 1MHz");
  
  // Start rendering task on Core 0 only if dual-core enabled
  if (useDualCore) {
    BaseType_t result = xTaskCreatePinnedToCore(
      renderTask,
      "RenderTask",
      8192,
      NULL,
      1,
      &renderTaskHandle,
      0
    );
    
    if (result != pdPASS) {
      Serial.println("Failed to create render task!");
      while (1) delay(100);
    }
    Serial.println("Setup complete! Dual-core rendering active.");
  } else {
    Serial.println("Setup complete! Single-core rendering (PSRAM mode).");
  }
}

void loop() {
  // Check for mode button press
  modesUpdate();
  
  if (useDualCore) {
    // Dual-core mode: sync with render task
    if (xSemaphoreTake(frameMutex, portMAX_DELAY)) {
      mlx.getFrame(mlxFrame);
      computeFrameRange();
      xSemaphoreGive(frameMutex);
    }
    
    // Notify render task that new frame is ready
    xTaskNotifyGive(renderTaskHandle);
    
    drawThermalFrame();
  } else {
    // Single-core mode: do everything inline
    mlx.getFrame(mlxFrame);
    computeFrameRange();
    
    // Render directly into frame buffer
    const float scaleX = (float)MLX_W / (float)VIEW_W;
    const float scaleY = (float)MLX_H / (float)VIEW_H;
    
    float minTemp, maxTemp;
    getTemperatureRange(minTemp, maxTemp, frameMin, frameMax);
    
    int bufIdx = 0;
    for (int y = 0; y < VIEW_H; ++y) {
      float srcY = (y + 0.5f) * scaleY - 0.5f;
      if (srcY < 0.0f) srcY = 0.0f;
      if (srcY >= MLX_H - 1) srcY = MLX_H - 1.001f;
      
      int y0 = (int)srcY;
      int y1 = y0 + 1;
      if (y1 >= MLX_H) y1 = MLX_H - 1;
      float dy = srcY - y0;
      float wy0 = 1.0f - dy;
      float wy1 = dy;

      for (int x = 0; x < VIEW_W; ++x) {
        float srcX = (x + 0.5f) * scaleX - 0.5f;
        if (srcX < 0.0f) srcX = 0.0f;
        if (srcX >= MLX_W - 1) srcX = MLX_W - 1.001f;
        
        int x0 = (int)srcX;
        int x1 = x0 + 1;
        if (x1 >= MLX_W) x1 = MLX_W - 1;
        float dx = srcX - x0;
        
        float t00 = mlxFrame[y0 * MLX_W + x0];
        float t10 = mlxFrame[y0 * MLX_W + x1];
        float t01 = mlxFrame[y1 * MLX_W + x0];
        float t11 = mlxFrame[y1 * MLX_W + x1];
        
        float t = (t00 * (1.0f - dx) + t10 * dx) * wy0 +
                  (t01 * (1.0f - dx) + t11 * dx) * wy1;
        
        frameBuffer[bufIdx++] = tempToColor(t, minTemp, maxTemp);
      }
    }
    
    // Display update
    const int screenW = 480;
    const int screenH = 320;
    const int startX = (screenW - VIEW_W) / 2;
    const int startY = (screenH - VIEW_H) / 2;
    gfx->draw16bitRGBBitmap(startX, startY, frameBuffer, VIEW_W, VIEW_H);
  }
  
  // Small delay to prevent tight loop
  delay(10);
}
