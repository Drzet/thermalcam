#ifndef WIFIMAN_H
#define WIFIMAN_H

#include <Arduino.h>

// High-level WiFi modes (distinct from ESP-IDF WIFI_MODE_*)
enum WifiModeState {
    WIFI_STATE_STA_PRESET = 0,  // Connect to one of the saved APs
    WIFI_STATE_AP         = 1   // Run as an access point
};

// Initialize WiFi manager (call once from setup())
void wifiManInit();

// Get current WiFi mode
WifiModeState wifiManGetMode();

// True if WiFi is usable:
//  - STA_PRESET: WL_CONNECTED
//  - AP:         AP is running
bool wifiManIsConnected();

// Current IP address as String:
//  - STA_PRESET: WiFi.localIP()
//  - AP:         WiFi.softAPIP()
String wifiManGetIp();

// Toggle between STA_PRESET and AP modes.
//  - STA_PRESET -> AP
//  - AP -> STA_PRESET (cycle saved networks, fall back to AP on failure)
void wifiManToggleMode();

#endif // WIFIMAN_H

