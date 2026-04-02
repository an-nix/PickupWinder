#include "Diag.h"
#include <Arduino.h>
#include <cstdarg>

Diag::Sink Diag::_sinks[Diag::MAX_SINKS] = { nullptr };
uint8_t Diag::_sinkCount = 0;

static const char* levelTag(Diag::Level l) {
    switch (l) {
        case Diag::Level::WARN:  return "[WARN] ";
        case Diag::Level::ERROR: return "[ERR]  ";
        default:                 return "";
    }
}

/**
 * @brief Dispatch message to Serial and registered sinks.
 */
void Diag::_dispatch(Level level, const char* msg) {
    // Serial is always the primary sink.
    Serial.print(levelTag(level));
    Serial.println(msg);

    // Forward to registered sinks (WebSocket, display, ...).
    if (_sinkCount > 0) {
        char line[544];
        snprintf(line, sizeof(line), "%s%s", levelTag(level), msg ? msg : "");
        for (uint8_t i = 0; i < _sinkCount; ++i) {
            if (_sinks[i]) _sinks[i](level, line);
        }
    }
}

/**
 * @brief Format a variadic log message and dispatch it.
 */
void Diag::_vlogf(Level level, const char* fmt, va_list args) {
    char buf[512];
    vsnprintf(buf, sizeof(buf), fmt, args);
    _dispatch(level, buf);
}

/** @brief Register one additional logging sink. */
void Diag::addSink(Sink sink) {
    if (!sink) return;
    if (_sinkCount >= MAX_SINKS) {
        warn("[Diag] sink registry full");
        return;
    }
    _sinks[_sinkCount++] = sink;
}

void Diag::info (const char* msg) { _dispatch(Level::INFO,  msg); }
void Diag::warn (const char* msg) { _dispatch(Level::WARN,  msg); }
void Diag::error(const char* msg) { _dispatch(Level::ERROR, msg); }

void Diag::infof(const char* fmt, ...)  { va_list a; va_start(a, fmt); _vlogf(Level::INFO,  fmt, a); va_end(a); }
void Diag::warnf(const char* fmt, ...)  { va_list a; va_start(a, fmt); _vlogf(Level::WARN,  fmt, a); va_end(a); }
void Diag::errorf(const char* fmt, ...) { va_list a; va_start(a, fmt); _vlogf(Level::ERROR, fmt, a); va_end(a); }
