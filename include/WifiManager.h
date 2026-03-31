#pragma once

#include <Arduino.h>

class WifiManager {
public:
    WifiManager();

    // Initialize WiFi connection, using NVS credentials if present.
    void begin();

    // Save credentials in NVS. C-string API to avoid heap allocations.
    bool setCredentials(const char* ssid, const char* password);

    // Current connection status.
    bool isConnected() const;

    // Fill caller buffer with IP or N/A.
    void getIP(char* buf, size_t len) const;

private:
    bool ensureNvsCredentials(char* ssidBuf, size_t ssidBufLen, char* pwdBuf, size_t pwdBufLen);
    bool isDefaultCredentials(const char* ssid, const char* password) const;
    bool _wifiOk;
};
