#pragma once
#include <Arduino.h>
#include "Config.h"
#include "SpeedInput.h"
#include "SessionController.h"

class ControlHardware {
public:
    ControlHardware(SessionController& session, WinderApp& winder);
    void begin();
    void tick(uint32_t now, SessionController::TickInput& out);
    uint32_t readPotHz();
    int32_t getEncoderCount();
    void resetEncoderCount(int32_t v = 0);

private:
    // External controllers
    SessionController& _session;
    WinderApp& _winder;

    // Potentiometer input (speed sensor)
    SpeedInput _pot;

    // Encoder state (kept private, accessible via getEncoderCount)
    static volatile int32_t _encCount;
    static volatile uint8_t  _encLastAB;
    static volatile uint32_t _encLastUs;
    static const int8_t QEM[16];
    static void IRAM_ATTR _encISR();

    // Local timing/cache for periodic tasks
    uint32_t _lastEncMs = 0;
    int32_t  _lastPrinted = 1;
    int32_t  _lastEncConsumed = 0;
    uint32_t _lastPotMs = 0;
    uint32_t _lastPotHz = 0;
};
