#pragma once
#include <Arduino.h>
class WinderApp;

/**
 * @brief Thin command handling facade that decouples transports from domain logic.
 *
 * This class is intentionally small: it forwards high-level commands and
 * encoder deltas to `WinderApp`. It exists to make future testing and
 * separation easier without changing `WinderApp` internals.
 */
class CommandHandler {
public:
    explicit CommandHandler(WinderApp& w) : _winder(w) {}

    void handleCommand(const char* cmd, const char* val) { _winder.handleCommand(cmd, val); }
    void handleEncoderDelta(int32_t d) { _winder.handleEncoderDelta(d); }

private:
    WinderApp& _winder;
};
