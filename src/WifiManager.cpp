#include "WifiManager.h"
#include <WiFi.h>
#include <Preferences.h>
#include "Diag.h"
#include "Config.h"

WifiManager::WifiManager() : _wifiOk(false) {}

bool WifiManager::ensureNvsCredentials(String& ssid, String& password) {
    Preferences prefs;
    prefs.begin("wifi", true);
    String storedSsid = prefs.getString("ssid", "");
    String storedPassword = prefs.getString("pwd", "");
    prefs.end();

    if (storedSsid.length() && storedPassword.length()) {
        ssid = storedSsid;
        password = storedPassword;
        return true;
    }

    return false;
}

bool WifiManager::isDefaultCredentials(const String& ssid, const String& password) const {
    // Prevent using placeholder defaults for actual connection or NVS writes.
    if (ssid == "your_ssid_here" || password == "your_password_here") return true;
    if (ssid.length() == 0 || password.length() == 0) return true;
    return false;
}

void WifiManager::begin() {
    String ssid = String(WIFI_SSID);
    String password = String(WIFI_PASSWORD);

    String nvsSsid;
    String nvsPassword;
    bool hasNvs = false;

    // Read NVS first, if present use that.
    {
        Preferences prefs;
        prefs.begin("wifi", true);
        nvsSsid = prefs.getString("ssid", "");
        nvsPassword = prefs.getString("pwd", "");
        prefs.end();
        hasNvs = nvsSsid.length() && nvsPassword.length();
    }

    if (hasNvs) {
        // If runtime config is non-default and differs from NVS, persist new values.
        if (!isDefaultCredentials(ssid, password) && (ssid != nvsSsid || password != nvsPassword)) {
            setCredentials(ssid, password);
            nvsSsid = ssid;
            nvsPassword = password;
        }

        ssid = nvsSsid;
        password = nvsPassword;
        Diag::infof("[WiFi] Using credentials from NVS '%s'", ssid.c_str());
    } else {
        // If no NVS data, only connect if non-default config values are supplied.
        if (isDefaultCredentials(ssid, password)) {
            Diag::error("[WiFi] No NVS credentials and config default values; WiFi disabled.");
            _wifiOk = false;
            return;
        }

        setCredentials(ssid, password);
        Diag::infof("[WiFi] Storing compile-time config values to NVS '%s'", ssid.c_str());
    }

    WiFi.begin(ssid.c_str(), password.c_str());
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

bool WifiManager::setCredentials(const String& ssid, const String& password) {
    if (isDefaultCredentials(ssid, password)) {
        Diag::error("[WiFi] Refuse default/empty credentials in NVS.");
        return false;
    }

    Preferences prefs;
    prefs.begin("wifi", false);

    String storedSsid = prefs.getString("ssid", "");
    String storedPassword = prefs.getString("pwd", "");

    if (storedSsid == ssid && storedPassword == password) {
        Diag::info("[WiFi] Credentials unchanged; skip NVS update.");
        prefs.end();
        return false;
    }

    prefs.putString("ssid", ssid);
    prefs.putString("pwd", password);
    prefs.end();

    Diag::info("[WiFi] Updated credentials in NVS.");
    return true;
}

bool WifiManager::isConnected() const {
    return _wifiOk;
}

String WifiManager::getIP() const {
    return _wifiOk ? WiFi.localIP().toString() : String("N/A");
}
