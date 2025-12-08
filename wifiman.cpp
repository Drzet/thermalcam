#include "wifiman.h"
#include <WiFi.h>

struct WifiNetwork {
    const char* ssid;
    const char* password;
};

// TODO: move credentials to a separate file and include it here
//       once the structure is stable.
static const WifiNetwork WIFI_NETWORKS[] = {
    { "_/_", "lkji4_faaaG" },
    { "YOUR_SSID_2", "YOUR_PASSWORD_2" }
    // add more as needed
};

static const size_t WIFI_NETWORK_COUNT =
    sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// Current mode
static WifiModeState wifiCurrentMode = WIFI_STATE_STA_PRESET;

// Forward declarations (internal helpers)
static bool wifiConnectToSavedNetworks();
static void wifiStartApMode();

// Public API
void wifiManInit() {
    WiFi.persistent(false);
    WiFi.mode(WIFI_OFF);
}

WifiModeState wifiManGetMode() {
    return wifiCurrentMode;
}

bool wifiManIsConnected() {
    if (wifiCurrentMode == WIFI_STATE_STA_PRESET) {
        return WiFi.status() == WL_CONNECTED;
    } else {
        return WiFi.getMode() == WIFI_AP;
    }
}

String wifiManGetIp() {
    if (wifiCurrentMode == WIFI_STATE_STA_PRESET) {
        return WiFi.localIP().toString();
    } else {
        return WiFi.softAPIP().toString();
    }
}

void wifiManToggleMode() {
    if (wifiCurrentMode == WIFI_STATE_STA_PRESET) {
        // Go to AP mode
        Serial.println("WiFi: switching to AP mode");
        WiFi.disconnect(true, true);
        wifiStartApMode();
        wifiCurrentMode = WIFI_STATE_AP;
    } else {
        // Go to STA mode using preset networks
        Serial.println("WiFi: switching to STA preset mode");
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);

        if (wifiConnectToSavedNetworks()) {
            wifiCurrentMode = WIFI_STATE_STA_PRESET;
        } else {
            Serial.println("WiFi: STA preset failed, falling back to AP");
            wifiStartApMode();
            wifiCurrentMode = WIFI_STATE_AP;
        }
    }
}

// ---------- internal helpers ----------

static bool wifiConnectToSavedNetworks() {
    if (WIFI_NETWORK_COUNT == 0) {
        Serial.println("WiFi: no saved networks configured");
        return false;
    }

    for (size_t i = 0; i < WIFI_NETWORK_COUNT; ++i) {
        const WifiNetwork& net = WIFI_NETWORKS[i];

        Serial.print("WiFi: trying SSID ");
        Serial.println(net.ssid);

        WiFi.begin(net.ssid, net.password);

        const unsigned long timeoutMs = 10000; // 10s per network
        unsigned long start = millis();

        while (WiFi.status() != WL_CONNECTED &&
               (millis() - start) < timeoutMs) {
            delay(250);
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.print("WiFi: connected, IP=");
            Serial.println(WiFi.localIP());
            WiFi.setSleep(false);  // disable power save for better throughput
            return true;
        }

        Serial.println("WiFi: connect failed, trying next");
    }

    Serial.println("WiFi: all saved networks failed");
    return false;
}

static void wifiStartApMode() {
    const char* apSsid = "ThermalCam";   // change as needed
    const char* apPass = "12345678";     // minimum 8 chars for WPA2

    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(apSsid, apPass);

    if (ok) {
        Serial.print("WiFi: AP started, SSID=");
        Serial.print(apSsid);
        Serial.print(" IP=");
        Serial.println(WiFi.softAPIP());
    } else {
        Serial.println("WiFi: failed to start AP");
    }
}
