#pragma once
#include <Arduino.h>
#include <functional>

// Callback invoqué quand une commande est reçue (WebSocket ou liaison série).
/** @brief Command callback signature `(command, value)`. */
using CommandCallback = std::function<void(const String&, const String&)>;

// Provider de recette JSON — retourne la recette courante sérialisée.
/** @brief Callback returning current recipe as JSON. */
using RecipeJsonProvider = std::function<String(void)>;

// ── WindingState ──────────────────────────────────────────────────────────────
// Single source of truth for the winding session lifecycle.
//
//  IDLE ──(start)──► VERIFY_LOW ──(confirm low)──► VERIFY_HIGH ──(confirm high)──► WINDING
//                        ▲               │                  ▲           │                 │
//                        │     (verify_low)◄────────────────┘           │           (pot→0)▼
//                        │               └───────────────(verify_high)──►          PAUSED
//                   (re-verify from                                             (pot↑)──►┘
//                   WINDING/PAUSED)                                         (turns≥target)──► TARGET_REACHED
//
//  Any state ──(stop/reset)──► IDLE
//  VERIFY_LOW ◄──► VERIFY_HIGH at any time via verify_low / verify_high commands.
//  When both low and high have been confirmed, auto-transitions to WINDING.
//  Any state ──(manual)──► MANUAL  (encoder = carriage jog, pot = motor, capture WS)
enum class WindingState {
    IDLE,           // No session — all parameters writable, carriage at home
    VERIFY_LOW,     // Verifying low (start) bound — carriage at start, pot runs motor at verify speed
    VERIFY_HIGH,    // Verifying high (end) bound  — carriage at end,   pot runs motor at verify speed
    WINDING,        // Active winding at user-set speed
    PAUSED,         // Mid-winding pause (pot→0) — resumes from current position
    TARGET_REACHED, // Auto-stop: target turns hit — blocked until reset or target raised
    MANUAL,         // Mode manuel : encodeur contrôle le chariot librement, moteur tourne via pot
    RODAGE,         // Rodage axe latéral : aller-retour N passes sur distance configurée
};

/**
 * @brief Convert `WindingState` to stable string label.
 * @param s State enum value.
 * @return Human-readable state name.
 */
inline const char* windingStateName(WindingState s) {
    switch (s) {
        case WindingState::IDLE:           return "IDLE";
        case WindingState::VERIFY_LOW:     return "VERIFY_LOW";
        case WindingState::VERIFY_HIGH:    return "VERIFY_HIGH";
        case WindingState::WINDING:        return "WINDING";
        case WindingState::PAUSED:         return "PAUSED";
        case WindingState::TARGET_REACHED: return "TARGET_REACHED";
        case WindingState::MANUAL:          return "MANUAL";
        case WindingState::RODAGE:          return "RODAGE";
        default:                            return "UNKNOWN";
    }
}

// ── WinderStatus ──────────────────────────────────────────────────────────────
// Full machine state snapshot sent to all connected WebSocket clients
// every WS_UPDATE_MS milliseconds, and to the screen over LinkSerial.
struct WinderStatus {
	/** Instantaneous spindle RPM. */
    float    rpm;
	/** Instantaneous spindle speed in Hz. */
    uint32_t speedHz;
	/** Current turns counter. */
    long     turns;
	/** Target turns. */
    long     targetTurns;
    bool     rewindMode;
    long     rewindBatchTurns;
    uint16_t rewindBatchRpm;
	/** true if spindle motor is currently running. */
    bool     running;
    bool     motorEnabled;        // True while motor is (or was) actively commanded this session
    bool     startRequested;      // True from Armed onwards (session active)
    bool     carriageReady;       // Lateral carriage positioned at winding start
    bool     verifyLow;           // True in VERIFY_LOW state
    bool     verifyHigh;          // True in VERIFY_HIGH state
    bool     manualMode;          // True in MANUAL state
    bool     rodageMode;          // True in RODAGE state
    int      rodagePassDone;      // Passes complétées
    int      rodagePasses;        // Total passes configurées
    float    rodageDistMm;        // Distance de traverse (mm)
    bool     freerun;
    bool     directionCW;
    bool     autoMode;            // Reserved
    long     turnsPerPass;
    long     turnsPerPassCalc;
    long     turnsPerPassOffset;
    float    scatterFactor;
    int      currentPass;
    long     activeTurnsPerPass;
    float    activeSpeedScale;
    float    latProgress;
    float    latPositionMm;
    float    windingStartMm;
    float    windingEndMm;
    float    windingStartTrimMm;
    float    windingEndTrimMm;
    float    effectiveWidth_mm;
    float    geomTotal, geomBottom, geomTop, geomMargin, geomWire;
    float    latOffset;
    const char* windingStyle;
    uint32_t seed;
    float    layerJitterPct;
    float    layerSpeedPct;
    float    humanTraversePct;
    float    humanSpeedPct;
    float    firstPassTraverseFactor;
    // Position finale de bobinage (0=NONE 1=HIGH 2=LOW) + nb tours de maintien.
    int      endPos;
    int      endPosTurns;
    const char* stateName;        // Human-readable WindingState label
};
