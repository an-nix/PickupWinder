#pragma once
#include <Arduino.h>
#include <functional>
#include <vector>

// ── Diag ──────────────────────────────────────────────────────────────────────
// Lightweight diagnostic logger. Serial is always an active sink.
// Additional sinks (WebSocket, display, etc.) are registered once at startup
// via addSink() and receive every message with its severity level.
//
// Usage:
//   Diag::info("[Lateral] Homing started");
//   Diag::warnf("[Winder] %.2f mm out of range", pos);
//   Diag::error("[Stepper] Driver fault");
//
// Registering an extra sink in main.cpp:
//   Diag::addSink([](Diag::Level lvl, const String& msg) {
//       web.broadcast(msg);   // forward to WebSocket clients
//   });
class Diag {
public:
    enum class Level { INFO, WARN, ERROR };

    // Callback type: receives the level and the fully-formatted message.
    using Sink = std::function<void(Level, const String&)>;

    // Register an additional output sink (called after Serial each time).
    // Sinks are permanent; call only during setup() before any log calls from
    // other cores.
    static void addSink(Sink sink);

    // ── Logging API ───────────────────────────────────────────────────────────
    static void info (const char* msg);
    static void warn (const char* msg);
    static void error(const char* msg);

    static void infof (const char* fmt, ...);
    static void warnf (const char* fmt, ...);
    static void errorf(const char* fmt, ...);

private:
    static std::vector<Sink> _sinks;

    static void _dispatch(Level level, const char* msg);
    static void _vlogf(Level level, const char* fmt, va_list args);
};
