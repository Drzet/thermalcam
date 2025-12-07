#include "ThermalModes.h"

// Global mode state
ThermalMode currentMode = MODE_AUTO;

// Button state tracking (simple debounce)
static unsigned long lastDebounceTime = 0;
static bool lastButtonState = HIGH;
static bool buttonState     = HIGH;

void modesInit() {
  // Button wired between BUTTON_PIN_A and GND
  pinMode(BUTTON_PIN_A, INPUT_PULLUP);
  currentMode = MODE_AUTO;
}

void modesUpdate() {
  bool reading = digitalRead(BUTTON_PIN_A);  // active LOW

  // Edge detected → start debounce timer
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Stable long enough → accept new state
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;

      // On press (LOW) → advance mode
      if (buttonState == LOW) {
        currentMode = (ThermalMode)((currentMode + 1) % NUM_MODES);
      }
    }
  }

  lastButtonState = reading;
}

ThermalMode getCurrentMode() {
  return currentMode;
}

const char* getModeName() {
  return MODE_CONFIGS[currentMode].name;
}

void getTemperatureRange(float& minTemp, float& maxTemp, float frameMin, float frameMax) {
  const ModeConfig& config = MODE_CONFIGS[currentMode];

  if (config.autoRange) {
    // Auto mode: use frame-based range
    minTemp = frameMin;
    maxTemp = frameMax;

    // Ensure minimum span for color mapping
    if (maxTemp - minTemp < 0.5f) {
      maxTemp = minTemp + 0.5f;
    }
  } else {
    // Fixed range modes
    minTemp = config.tempMin;
    maxTemp = config.tempMax;
  }
}
