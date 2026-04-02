#pragma once
#include "Types.h"

// Simple forward-compatible telemetry DTO used for serialization.
// This decouples the wire format from internal runtime state.
struct Telemetry {
    static constexpr uint8_t TELEMETRY_VERSION = 1;
    uint8_t version = TELEMETRY_VERSION;

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
    float    firstPassTraverseFactor;
    int      endPos;
    int      endPosTurns;
    bool     calibInProgress;
    int      calibCurrent;
    int      calibTotal;
    float    calibMeasuredTPP;
    long     calibSuggestedOffset;
    const char* stateName;

    // Convert from internal WinderStatus to telemetry DTO.
    static Telemetry from(const WinderStatus& s) {
        Telemetry t;
        t.version = TELEMETRY_VERSION;
        t.rpm = s.rpm;
        t.speedHz = s.speedHz;
        t.turns = s.turns;
        t.targetTurns = s.targetTurns;
        t.maxRpm = s.maxRpm;
        t.running = s.running;
        t.motorEnabled = s.motorEnabled;
        t.startRequested = s.startRequested;
        t.carriageReady = s.carriageReady;
        t.verifyLow = s.verifyLow;
        t.verifyHigh = s.verifyHigh;
        t.rodageMode = s.rodageMode;
        t.rodagePassDone = s.rodagePassDone;
        t.rodagePasses = s.rodagePasses;
        t.rodageDist_mm = s.rodageDist_mm;
        t.freerun = s.freerun;
        t.directionCW = s.directionCW;
        t.turnsPerPass = s.turnsPerPass;
        t.turnsPerPassCalc = s.turnsPerPassCalc;
        t.turnsPerPassOffset = s.turnsPerPassOffset;
        t.scatterFactor = s.scatterFactor;
        t.currentPass = s.currentPass;
        t.activeTurnsPerPass = s.activeTurnsPerPass;
        t.activeSpeedScale = s.activeSpeedScale;
        t.latProgress = s.latProgress;
        t.latPosition_mm = s.latPosition_mm;
        t.windingStart_mm = s.windingStart_mm;
        t.windingEnd_mm = s.windingEnd_mm;
        t.windingStartTrim_mm = s.windingStartTrim_mm;
        t.windingEndTrim_mm = s.windingEndTrim_mm;
        t.effectiveWidth_mm = s.effectiveWidth_mm;
        t.geomTotal = s.geomTotal; t.geomBottom = s.geomBottom; t.geomTop = s.geomTop; t.geomMargin = s.geomMargin; t.geomWire = s.geomWire;
        t.latOffset = s.latOffset;
        t.seed = s.seed;
        t.firstPassTraverseFactor = s.firstPassTraverseFactor;
        t.endPos = s.endPos;
        t.endPosTurns = s.endPosTurns;
        t.calibInProgress = s.calibInProgress;
        t.calibCurrent = s.calibCurrent;
        t.calibTotal = s.calibTotal;
        t.calibMeasuredTPP = s.calibMeasuredTPP;
        t.calibSuggestedOffset = s.calibSuggestedOffset;
        t.stateName = s.stateName;
        return t;
    }
};
