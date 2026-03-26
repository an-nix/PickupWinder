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
    /**
     * @brief Log severity level.
     */
    enum class Level { INFO, WARN, ERROR };

    /**
     * @brief Sink callback signature.
     *
     * Receives log level and fully-formatted message.
     */
    using Sink = std::function<void(Level, const String&)>;

    /**
     * @brief Register an additional output sink.
     * @param sink Sink callback to add.
     * @par Usage
     * Register during setup before multi-core activity starts.
     */
    static void addSink(Sink sink);

    // ── Logging API ───────────────────────────────────────────────────────────
    /** @brief Log INFO message. */
    static void info (const char* msg);
    /** @brief Log WARN message. */
    static void warn (const char* msg);
    /** @brief Log ERROR message. */
    static void error(const char* msg);

    /** @brief Log formatted INFO message. */
    static void infof (const char* fmt, ...);
    /** @brief Log formatted WARN message. */
    static void warnf (const char* fmt, ...);
    /** @brief Log formatted ERROR message. */
    static void errorf(const char* fmt, ...);

private:
    static std::vector<Sink> _sinks;

    /**
     * @brief Dispatch one formatted message to Serial + all sinks.
     */
    static void _dispatch(Level level, const char* msg);

    /**
     * @brief `va_list` formatter backend for `*f` logging methods.
     */
    static void _vlogf(Level level, const char* fmt, va_list args);
};
