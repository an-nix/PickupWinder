#pragma once
#include <Arduino.h>
#include <functional>

using CommandCallback = std::function<void(const String&, const String&)>;
using RecipeJsonProvider = std::function<String(void)>;

// ── WindingState ──────────────────────────────────────────────────────────────
// Simplified state machine. Verify is NOT a state — it uses flags during the
// normal PAUSED→WINDING transitions.
//
//  IDLE ──(start)──► PAUSED (positioning to low) ──(start)──► WINDING (verify high armed)
//                                                                │
//                                                          (pot→0)▼
//                                                           PAUSED
//                                                          (pot↑)──►┘
//                                                    (turns≥target)──► TARGET_REACHED
//
//  Any state ──(stop)──► IDLE
enum class WindingState {
    IDLE,
    CALIBRATING,
    WINDING,
    PAUSED,
    TARGET_REACHED,
    RODAGE,
};

inline const char* windingStateName(WindingState s) {
    switch (s) {
        case WindingState::IDLE:           return "IDLE";
        case WindingState::WINDING:        return "WINDING";
        case WindingState::PAUSED:         return "PAUSED";
        case WindingState::TARGET_REACHED: return "TARGET_REACHED";
        case WindingState::RODAGE:         return "RODAGE";
        default:                           return "UNKNOWN";
    }
}

// ── WinderStatus ──────────────────────────────────────────────────────────────
struct WinderStatus {
    float    rpm;
    uint32_t speedHz;
    long     turns;
    long     targetTurns;
    uint16_t maxRpm;
    bool     running;
    bool     motorEnabled;
    bool     startRequested;
    bool     carriageReady;
    bool     verifyLow;
    bool     verifyHigh;
    bool     rodageMode;
    int      rodagePassDone;
    int      rodagePasses;
    float    rodageDist_mm;
    bool     freerun;
    bool     directionCW;
    bool     burstEnabled;
    bool     burstActive;
    long     burstConfiguredTurns;
    long     burstTargetTurns;
    long     burstRemainingTurns;
    float    turnsPerPass;
    float    turnsPerPassCalc;
    float    turnsPerPassOffset;
    float    scatterFactor;
    int      currentPass;
    float    activeTurnsPerPass;
    float    activeSpeedScale;
    float    latProgress;
    float    latPosition_mm;
    float    windingStart_mm;
    float    windingEnd_mm;
    float    windingStartTrim_mm;
    float    windingEndTrim_mm;
    float    effectiveWidth_mm;
    float    geomTotal, geomBottom, geomTop, geomMargin, geomWire;
    float    latOffset;
    uint32_t seed;
    // Style/jitter fields removed from status (planner is straight-only).
    float    firstPassTraverseFactor;
    int      endPos;
    int      endPosTurns;
    bool     calibInProgress;
    int      calibCurrent;
    int      calibTotal;
    float    calibMeasuredTPP;
    long     calibSuggestedOffset;
    const char* stateName;
};

// Bounded command entry for embedded-friendly command passing (no dynamic alloc)
struct CommandEntry {
    static constexpr size_t CMD_SZ = 32;  // Long enough for window_shift_nudge, geom_*_nudge, etc.
    static constexpr size_t VAL_SZ = 48;
    char cmd[CMD_SZ];
    char val[VAL_SZ];
};
