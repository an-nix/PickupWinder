#pragma once
#include "StepperController.h"
#include "SpeedInput.h"
#include "WebInterface.h"
#include "WindingGeometry.h"
#include "LEDController.h"

enum class WinderMode { MANUAL, AUTO };
enum class Direction  { CW, CCW };

class WinderApp {
public:
    void begin();
    void run();

private:
    StepperController _stepper;
    SpeedInput        _pot;
    WebInterface      _web;
    LEDController     _led;
    WindingGeometry   _geom;

    // volatile : modifiés depuis le callback WebSocket (Core 0), lus depuis loop() (Core 1)
    volatile WinderMode _mode         = WinderMode::MANUAL;
    volatile Direction  _direction    = Direction::CW;
    volatile long       _targetTurns  = DEFAULT_TARGET_TURNS;
    volatile bool       _freerun      = false;
    volatile bool       _motorEnabled = true;

    bool     _potWasZero = true;

    uint32_t _lastWsMs  = 0;
    uint32_t _lastPotMs = 0;

    void _stop();
    void _handleCommand(const String& cmd, const String& value);
};
