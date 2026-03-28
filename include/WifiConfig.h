#pragma once

// Wi-Fi credential loading fallback chain:
// 1) Build flags: -D BUILD_WIFI_SSID="..." -D BUILD_WIFI_PASSWORD="..."
// 2) Optional include/Secrets.h (local and ignored by git)
// 3) Default placeholders (safe, non-secret)

#ifndef WIFI_SSID
  #ifdef BUILD_WIFI_SSID
    #define WIFI_SSID BUILD_WIFI_SSID
  #else
    #if defined(__has_include)
      #if __has_include("Secrets.h")
        #include "Secrets.h"
      #endif
    #endif
    #ifndef WIFI_SSID
      #define WIFI_SSID "<redacted>"
    #endif
  #endif
#endif

#ifndef WIFI_PASSWORD
  #ifdef BUILD_WIFI_PASSWORD
    #define WIFI_PASSWORD BUILD_WIFI_PASSWORD
  #else
    #if defined(__has_include)
      #if __has_include("Secrets.h")
        #include "Secrets.h"
      #endif
    #endif
    #ifndef WIFI_PASSWORD
      #define WIFI_PASSWORD "<redacted>"
    #endif
  #endif
#endif
