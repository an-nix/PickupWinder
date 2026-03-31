#pragma once
// Centralized application constants and build-time knobs.
// Populate as refactor proceeds. Use `constexpr` only.

namespace AppConfig {
    constexpr uint16_t WEB_PORT = 80;
    // Default task configuration — review per-task as refactor continues
    constexpr size_t DEFAULT_TASK_STACK = 4096;
    constexpr unsigned DEFAULT_TASK_PRIO = 5;
    // Telemetry publish interval in ms (used by WebInterface tickers)
    constexpr unsigned TELEMETRY_MS = 200;
}
