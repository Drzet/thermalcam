#ifndef THERMAL_MODES_H
#define THERMAL_MODES_H

#include <Arduino.h>

// Temperature mode definitions
enum ThermalMode {
  MODE_AUTO = 0,      // Auto-range based on frame min/max
  MODE_BODY = 1,      // Body temperature: 20-50°C
  MODE_HOME = 2       // Home/environment: -20 to 100°C
};

// Mode configuration structure
struct ModeConfig {
  const char* name;
  float tempMin;
  float tempMax;
  bool autoRange;  // true for auto-ranging, false for fixed range
};

// Mode configurations
const ModeConfig MODE_CONFIGS[] = {
  {"Auto", -40.0f, 300.0f, true},    // MODE_AUTO
  {"Body", 20.0f, 50.0f, false},     // MODE_BODY
  {"Home", -20.0f, 100.0f, false}    // MODE_HOME
};

const int NUM_MODES = 3;

// Global mode state
extern ThermalMode currentMode;

// Button pins
#define BUTTON_PIN_A 18  // INPUT_PULLUP - reads button state
#define BUTTON_PIN_B 3   // OUTPUT LOW - completes circuit when button pressed

// Button debounce
#define DEBOUNCE_DELAY 50

// Function declarations
void modesInit();
void modesUpdate();
ThermalMode getCurrentMode();
const char* getModeName();
void getTemperatureRange(float& minTemp, float& maxTemp, float frameMin, float frameMax);

#endif // THERMAL_MODES_H
