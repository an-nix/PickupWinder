#pragma once

#include "WinderApp.h"

enum class SessionState { IDLE, RUNNING, PAUSED };

class SessionController {
public:
    enum class InputSource { None, Pot, IHM, Footswitch };

    explicit SessionController(WinderApp& winder);

    void setPotLevel(float level);   // 0.0 .. 1.0
    void setFootswitch(bool pressed);
    void requestStart();
    void requestPause();
    void requestStop();

    bool handleCommand(const String& cmd, const String& value);
    void tick();

private:
    WinderApp& _winder;
    SessionState _state = SessionState::IDLE;
    InputSource _lastInput = InputSource::None;
    float _potLevel = 0.0f;
    bool _footswitch = false;

    bool _reqStart = false;
    bool _reqPause = false;
    bool _reqStop = false;

    void applyPower();
};
