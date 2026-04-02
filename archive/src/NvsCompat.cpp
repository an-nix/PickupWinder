#include "NvsCompat.h"
#include "Diag.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <string.h>

static bool nvsInitializeOnce() {
    static bool initialized = false;
    static bool initResult = false;
    if (initialized) return initResult;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }

    initResult = (err == ESP_OK);
    if (!initResult) {
        Diag::errorf("[NVS] nvs_flash_init failed: 0x%08x", err);
    }

    initialized = true;
    return initResult;
}

bool NvsCompat::readBlob(const char* ns, const char* key, void* buf, size_t bufLen, size_t& outLen) {
    if (!ns || !key || !buf || bufLen == 0) {
        outLen = 0;
        return false;
    }
    if (!nvsInitializeOnce()) {
        outLen = 0;
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        outLen = 0;
        return false;
    }

    size_t required = 0;
    err = nvs_get_blob(handle, key, NULL, &required);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // No blob key
        nvs_close(handle);
        outLen = 0;
        return false;
    }

    if (err != ESP_OK && err != ESP_ERR_NVS_INVALID_LENGTH) {
        nvs_close(handle);
        outLen = 0;
        return false;
    }

    if (required == 0 || required > bufLen) {
        nvs_close(handle);
        outLen = 0;
        return false;
    }

    err = nvs_get_blob(handle, key, buf, &required);
    nvs_close(handle);

    if (err != ESP_OK) {
        outLen = 0;
        return false;
    }

    outLen = required;
    return true;
}

bool NvsCompat::writeBlob(const char* ns, const char* key, const void* data, size_t len) {
    if (!ns || !key || !data || len == 0) return false;
    if (!nvsInitializeOnce()) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return false;
    }

    err = nvs_set_blob(handle, key, data, len);
    if (err != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    return err == ESP_OK;
}

bool NvsCompat::eraseNamespace(const char* ns) {
    if (!ns) return false;
    if (!nvsInitializeOnce()) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err != ESP_OK) return false;

    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err == ESP_OK;
}
