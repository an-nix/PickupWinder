#pragma once

#include <stddef.h>

class NvsCompat {
public:
    static bool readBlob(const char* ns, const char* key, void* buf, size_t bufLen, size_t& outLen);
    static bool writeBlob(const char* ns, const char* key, const void* data, size_t len);
    static bool eraseNamespace(const char* ns);
};
