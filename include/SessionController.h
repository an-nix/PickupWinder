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

    struct TickInput {
        float potLevel = 0.0f;   // 0.0 .. 1.0
        bool hasPot = false;
        bool footswitch = false;
        bool hasFootswitch = false;
        uint32_t now = 0;
        // Bounded command list to avoid dynamic allocation (embedded-friendly)
        static constexpr int MAX_CMDS = 16;
        // Use shared CommandEntry defined in Types.h
        CommandEntry commands[MAX_CMDS];
        int cmdCount = 0;
    };

    bool handleCommand(const String& cmd, const String& value);
    void tick(const TickInput& in);
    // (status snapshot removed from SessionController)

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
    // (timers kept in main to avoid impacting send timing)
};
