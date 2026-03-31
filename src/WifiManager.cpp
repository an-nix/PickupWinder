#include "WifiManager.h"
#include <WiFi.h>
#include "Diag.h"
#include "Config.h"
#include "NvsCompat.h"

WifiManager::WifiManager() : _wifiOk(false) {}

bool WifiManager::ensureNvsCredentials(char* ssidBuf, size_t ssidBufLen, char* pwdBuf, size_t pwdBufLen) {
    // Use NVS compatibility API to read credentials without any Arduino
    // String usage or Preferences::getString fallbacks.
    size_t got = 0;
    bool ok = NvsCompat::readBlob("wifi", "ssid", ssidBuf, ssidBufLen - 1, got);
    if (ok && got > 0) ssidBuf[got < ssidBufLen ? got : ssidBufLen - 1] = '\0';
    else if (ssidBuf && ssidBufLen > 0) ssidBuf[0] = '\0';

    got = 0;
    ok = NvsCompat::readBlob("wifi", "pwd", pwdBuf, pwdBufLen - 1, got);
    if (ok && got > 0) pwdBuf[got < pwdBufLen ? got : pwdBufLen - 1] = '\0';
    else if (pwdBuf && pwdBufLen > 0) pwdBuf[0] = '\0';

    return (ssidBuf && ssidBuf[0] != '\0' && pwdBuf && pwdBuf[0] != '\0');
}

bool WifiManager::isDefaultCredentials(const char* ssid, const char* password) const {
    // Prevent using placeholder defaults for actual connection or NVS writes.
    if (!ssid || !password) return true;
    if (strcmp(ssid, "your_ssid_here") == 0 || strcmp(password, "your_password_here") == 0) return true;
    if (ssid[0] == '\0' || password[0] == '\0') return true;
    return false;
}

void WifiManager::begin() {
    // Prepare compile-time defaults into buffers.
    char ssidBuf[64];
    char pwdBuf[64];
    strncpy(ssidBuf, WIFI_SSID, sizeof(ssidBuf)); ssidBuf[sizeof(ssidBuf)-1] = '\0';
    strncpy(pwdBuf, WIFI_PASSWORD, sizeof(pwdBuf)); pwdBuf[sizeof(pwdBuf)-1] = '\0';

    // Attempt to read NVS credentials into buffers.
    char nvsSsid[64];
    char nvsPwd[64];
    bool hasNvs = ensureNvsCredentials(nvsSsid, sizeof(nvsSsid), nvsPwd, sizeof(nvsPwd));

    Diag::infof("[WiFi] compile-time credentials: SSID='%s' PWD='%s'", ssidBuf, pwdBuf);
    Diag::infof("[WiFi] NVS load status: hasNvs=%d nvsSsid='%s' nvsPwd='%s'", hasNvs, nvsSsid, nvsPwd);

    if (hasNvs) {
        // If runtime config is non-default and differs from NVS, persist new values.
        if (!isDefaultCredentials(ssidBuf, pwdBuf) && (strcmp(ssidBuf, nvsSsid) != 0 || strcmp(pwdBuf, nvsPwd) != 0)) {
            Diag::info("[WiFi] compile-time credentials are non-default and different from NVS; updating NVS.");
            setCredentials(ssidBuf, pwdBuf);
            strncpy(nvsSsid, ssidBuf, sizeof(nvsSsid));
            strncpy(nvsPwd, pwdBuf, sizeof(nvsPwd));
        }

        strncpy(ssidBuf, nvsSsid, sizeof(ssidBuf));
        strncpy(pwdBuf, nvsPwd, sizeof(pwdBuf));
        Diag::infof("[WiFi] Using credentials from NVS '%s'", ssidBuf);
    } else {
        // If no NVS data, only connect if non-default config values are supplied.
        if (isDefaultCredentials(ssidBuf, pwdBuf)) {
            Diag::error("[WiFi] No NVS credentials and config default values; WiFi disabled.");
            _wifiOk = false;
            return;
        }

        setCredentials(ssidBuf, pwdBuf);
        Diag::infof("[WiFi] Storing compile-time config values to NVS '%s'", ssidBuf);
    }

    WiFi.begin(ssidBuf, pwdBuf);
    Serial.print("[WiFi] Connecting");

    uint8_t tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        Serial.print(".");
        tries++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Diag::error("\n[WiFi] Failed — web interface disabled. Motor still works.");
        _wifiOk = false;
        return;
    }

    _wifiOk = true;
    Diag::infof("\n[WiFi] Connected — IP: %s", WiFi.localIP().toString().c_str());
}

bool WifiManager::setCredentials(const char* ssid, const char* password) {
    if (!ssid || !password) return false;
    if (isDefaultCredentials(ssid, password)) {
        Diag::error("[WiFi] Refuse default/empty credentials in NVS.");
        return false;
    }

    // Compare existing values using byte reads to avoid temporary Strings.
    char existingSsid[128] = {0};
    char existingPwd[128] = {0};
    // Use NVS-compatible byte reads to avoid Arduino String usage.
    size_t got = 0;
    NvsCompat::readBlob("wifi", "ssid", existingSsid, sizeof(existingSsid) - 1, got);
    NvsCompat::readBlob("wifi", "pwd", existingPwd, sizeof(existingPwd) - 1, got);

    if (strcmp(existingSsid, ssid) == 0 && strcmp(existingPwd, password) == 0) {
        Diag::info("[WiFi] Credentials unchanged; skip NVS update.");
        return false;
    }

    bool ok = NvsCompat::writeBlob("wifi", "ssid", ssid, strlen(ssid) + 1) &&
              NvsCompat::writeBlob("wifi", "pwd", password, strlen(password) + 1);

    Diag::info("[WiFi] Updated credentials in NVS.");
    return true;
}

void WifiManager::getIP(char* buf, size_t len) const {
    if (!buf || len == 0) return;
    if (!_wifiOk) {
        strncpy(buf, "N/A", len);
        buf[len-1] = '\0';
        return;
    }
    IPAddress ip = WiFi.localIP();
    // Format IP into caller buffer without constructing an Arduino String.
    snprintf(buf, len, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

bool WifiManager::isConnected() const {
    return _wifiOk;
}
