#include "ThermalModes.h"

// Global mode state
ThermalMode currentMode = MODE_AUTO;

// Button state tracking
static unsigned long lastDebounceTime = 0;
static bool lastButtonState = HIGH;
static bool buttonState = HIGH;

void modesInit() {
  // Configure button: Pin 3 as OUTPUT LOW, Pin 18 as INPUT with pull-up
  pinMode(BUTTON_PIN_B, OUTPUT);
  digitalWrite(BUTTON_PIN_B, LOW);
  pinMode(BUTTON_PIN_A, INPUT_PULLUP);
  
  currentMode = MODE_AUTO;
  
  Serial.println("Mode system initialized");
  Serial.printf("Starting mode: %s\n", MODE_CONFIGS[currentMode].name);
}

void modesUpdate() {
  // Read button state (active LOW with pull-up)
  bool reading = digitalRead(BUTTON_PIN_A);
  
  // Check if button state changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    Serial.printf("Button state change detected: %s\n", reading ? "HIGH" : "LOW");
  }
  
  // If stable for debounce delay
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // If button state has changed
    if (reading != buttonState) {
      buttonState = reading;
      Serial.printf("Button state confirmed after debounce: %s\n", buttonState ? "HIGH" : "LOW");
      
      // Button pressed (LOW)
      if (buttonState == LOW) {
        Serial.println("*** BUTTON PRESSED ***");
        // Cycle to next mode
        currentMode = (ThermalMode)((currentMode + 1) % NUM_MODES);
        
        Serial.printf("Mode changed to: %s (", MODE_CONFIGS[currentMode].name);
        if (MODE_CONFIGS[currentMode].autoRange) {
          Serial.println("Auto-range)");
        } else {
          Serial.printf("%.1f to %.1f°C)\n", 
                       MODE_CONFIGS[currentMode].tempMin,
                       MODE_CONFIGS[currentMode].tempMax);
        }
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
    
    // Ensure minimum span
    if (maxTemp - minTemp < 0.5f) {
      maxTemp = minTemp + 0.5f;
    }
  } else {
    // Fixed range modes
    minTemp = config.tempMin;
    maxTemp = config.tempMax;
  }
}
