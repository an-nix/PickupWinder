#include "Diag.h"
#include <Arduino.h>
#include <cstdarg>

std::vector<Diag::Sink> Diag::_sinks;

static const char* levelTag(Diag::Level l) {
    switch (l) {
        case Diag::Level::WARN:  return "[WARN] ";
        case Diag::Level::ERROR: return "[ERR]  ";
        default:                 return "";
    }
}

void Diag::_dispatch(Level level, const char* msg) {
    // Serial is always the primary sink.
    Serial.print(levelTag(level));
    Serial.println(msg);

    // Forward to registered sinks (WebSocket, display, ...).
    if (!_sinks.empty()) {
        String s = String(levelTag(level)) + msg;
        for (auto& sink : _sinks) sink(level, s);
    }
}

void Diag::_vlogf(Level level, const char* fmt, va_list args) {
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    _dispatch(level, buf);
}

void Diag::addSink(Sink sink) { _sinks.push_back(std::move(sink)); }

void Diag::info (const char* msg) { _dispatch(Level::INFO,  msg); }
void Diag::warn (const char* msg) { _dispatch(Level::WARN,  msg); }
void Diag::error(const char* msg) { _dispatch(Level::ERROR, msg); }

void Diag::infof(const char* fmt, ...)  { va_list a; va_start(a, fmt); _vlogf(Level::INFO,  fmt, a); va_end(a); }
void Diag::warnf(const char* fmt, ...)  { va_list a; va_start(a, fmt); _vlogf(Level::WARN,  fmt, a); va_end(a); }
void Diag::errorf(const char* fmt, ...) { va_list a; va_start(a, fmt); _vlogf(Level::ERROR, fmt, a); va_end(a); }
