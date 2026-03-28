#pragma once

#include <Arduino.h>

class WifiManager {
public:
    WifiManager();

    // Initialize WiFi connection, using NVS credentials if present.
    void begin();

    // Save credentials in NVS. Returns true when value is written, false when unchanged.
    bool setCredentials(const String& ssid, const String& password);

    // Current connection status.
    bool isConnected() const;

    String getIP() const;

private:
    bool ensureNvsCredentials(String& ssid, String& password);
    bool isDefaultCredentials(const String& ssid, const String& password) const;
    bool _wifiOk;
};
