#pragma once

#include <Arduino.h>

enum class CommandId : uint16_t {
    Unknown = 0,
    Start,
    Pause,
    Stop,
    Resume,
    Reset,
    Target,
    MaxRpm,
    Freerun,
    Direction,
    StopNextHigh,
    StopNextLow,
    EndPos,
    EndPosTurns,
    Rodage,
    RodageStop,
    RodageDist,
    RodagePasses,
    GeomStartTrim,
    GeomEndTrim,
    GeomStartTrimNudge,
    GeomEndTrimNudge,
    GeomPreset,
    GeomTotal,
    GeomBottom,
    GeomTop,
    GeomMargin,
    GeomWire,
    GeomTppOffset,
    GeomScatter,
    WindowShift,
    WindingStyle,
    WindingSeed,
    WindingLayerJitter,
    WindingLayerSpeed,
    WindingHumanTraverse,
    WindingHumanSpeed,
    WindingFirstPassTraverse,
    RecipeImport,
    LatOffset,
};

enum class CommandValueKind : uint8_t {
    None,
    Integer,
    Float,
    Boolean,
    Enum,
    Json,
};

struct CommandDefinition {
    CommandId id;
    const char* key;
    CommandValueKind kind;
    int32_t minInt;
    int32_t maxInt;
    float minFloat;
    float maxFloat;
    const char* enumValuesCsv;
    bool mutableDuringSession;
    const char* help;
};

class CommandRegistry {
public:
    static bool normalizeAndValidate(const char* cmdIn,
                                     const char* valueIn,
                                     char* canonicalCmdOut,
                                     size_t canonicalCmdOutLen,
                                     CommandId* idOut = nullptr);

    static void capabilitiesJson(char* outBuf, size_t outBufLen);

    static const CommandDefinition* findByKey(const char* key);
    static const CommandDefinition* findById(CommandId id);
    static const CommandDefinition* all(size_t& count);
};
