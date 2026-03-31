#include "NvsCompat.h"

#include <Preferences.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>

namespace NvsCompat {

bool readBlob(const char* ns, const char* key, void* buf, size_t bufLen, size_t& outLen) {
    outLen = 0;
#if defined(PREFERENCES_HAVE_GETBYTES)
    Preferences prefs;
    if (!prefs.begin(ns, true)) return false;
    size_t got = prefs.getBytes(key, buf, bufLen);
    prefs.end();
    outLen = got;
    return got > 0;
#else
    // Ensure NVS flash is initialized before using nvs_open.
    static bool nvsInitialized = false;
    if (!nvsInitialized) {
        esp_err_t rc = nvs_flash_init();
        if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // Erase and re-init to recover from partition changes.
            nvs_flash_erase();
            rc = nvs_flash_init();
        }
        if (rc != ESP_OK) {
            return false;
        }
        nvsInitialized = true;
    }

    // Use ESP-IDF NVS API directly to avoid Arduino String fallbacks.
    nvs_handle_t handle;
    esp_err_t rc = nvs_open(ns, NVS_READONLY, &handle);
    if (rc != ESP_OK) return false;
    size_t required = 0;
    rc = nvs_get_blob(handle, key, nullptr, &required);
    if (rc != ESP_OK || required == 0) {
        nvs_close(handle);
        return false;
    }
    size_t toRead = (required <= bufLen) ? required : bufLen;
    rc = nvs_get_blob(handle, key, buf, &toRead);
    nvs_close(handle);
    if (rc != ESP_OK) return false;
    outLen = toRead;
    return true;
#endif
}

bool writeBlob(const char* ns, const char* key, const void* data, size_t dataLen) {
#if defined(PREFERENCES_HAVE_PUTBYTES)
    Preferences prefs;
    if (!prefs.begin(ns, false)) return false;
    size_t written = prefs.putBytes(key, data, dataLen);
    prefs.end();
    return written > 0;
#else
    // Use ESP-IDF NVS API directly.
    nvs_handle_t handle;
    esp_err_t rc = nvs_open(ns, NVS_READWRITE, &handle);
    if (rc != ESP_OK) return false;
    rc = nvs_set_blob(handle, key, data, dataLen);
    if (rc != ESP_OK) { nvs_close(handle); return false; }
    rc = nvs_commit(handle);
    nvs_close(handle);
    return rc == ESP_OK;
#endif
}

} // namespace NvsCompat
