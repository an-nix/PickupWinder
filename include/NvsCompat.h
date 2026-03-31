#pragma once

#include <cstddef>

namespace NvsCompat {
    // Read a blob from NVS/Preferences namespace `ns` with key `key` into
    // `buf` of length `bufLen`. On success `outLen` receives the number of
    // bytes read and the function returns true. No Arduino `String` is used.
    bool readBlob(const char* ns, const char* key, void* buf, size_t bufLen, size_t& outLen);

    // Write a blob to NVS/Preferences. Returns true on success.
    bool writeBlob(const char* ns, const char* key, const void* data, size_t dataLen);
}
