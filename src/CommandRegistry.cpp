#include "CommandRegistry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Protocol.h"

namespace {

static const CommandDefinition kCommands[] = {
    { CommandId::Start, "start", CommandValueKind::None, 0, 0, 0, 0, "", true, "Start winding session." },
    { CommandId::Pause, "pause", CommandValueKind::None, 0, 0, 0, 0, "", true, "Pause winding session." },
    { CommandId::Stop, "stop", CommandValueKind::None, 0, 0, 0, 0, "", true, "Stop winding and return to idle." },
    { CommandId::Resume, "resume", CommandValueKind::None, 0, 0, 0, 0, "", true, "Resume from paused state." },
    { CommandId::Reset, "reset", CommandValueKind::None, 0, 0, 0, 0, "", true, "Reset turns and return to idle." },
    { CommandId::Target, "target", CommandValueKind::Integer, 1, 200000, 0, 0, "", true, "Target turns count." },
    { CommandId::MaxRpm, "max_rpm", CommandValueKind::Integer, 10, 1500, 0, 0, "", true, "Max spindle RPM." },
    { CommandId::Freerun, "freerun", CommandValueKind::Boolean, 0, 0, 0, 0, "true,false", false, "Freerun mode." },
    { CommandId::Direction, "direction", CommandValueKind::Enum, 0, 0, 0, 0, "cw,ccw", false, "Spindle direction." },
    { CommandId::StopNextHigh, "stop_next_high", CommandValueKind::None, 0, 0, 0, 0, "", true, "Stop lateral at next high bound." },
    { CommandId::StopNextLow, "stop_next_low", CommandValueKind::None, 0, 0, 0, 0, "", true, "Stop lateral at next low bound." },
    { CommandId::EndPos, "end_pos", CommandValueKind::Enum, 0, 0, 0, 0, "none,high,low", false, "Final carriage position." },
    { CommandId::EndPosTurns, "end_pos_turns", CommandValueKind::Integer, 1, 20, 0, 0, "", false, "Hold turns on final position." },
    { CommandId::Rodage, "rodage", CommandValueKind::None, 0, 0, 0, 0, "", true, "Start rodage mode." },
    { CommandId::RodageStop, "rodage_stop", CommandValueKind::None, 0, 0, 0, 0, "", true, "Stop rodage mode." },
    { CommandId::RodageDist, "rodage_dist", CommandValueKind::Float, 0, 0, 5.0f, 100.0f, "", false, "Rodage shuttle distance (mm)." },
    { CommandId::RodagePasses, "rodage_passes", CommandValueKind::Integer, 1, 200, 0, 0, "", false, "Rodage pass count." },
    { CommandId::GeomStartTrim, "geom_start_trim", CommandValueKind::Float, 0, 0, -5.0f, 5.0f, "", false, "Low bound trim (mm)." },
    { CommandId::GeomEndTrim, "geom_end_trim", CommandValueKind::Float, 0, 0, -5.0f, 5.0f, "", false, "High bound trim (mm)." },
    { CommandId::GeomStartTrimNudge, "geom_start_trim_nudge", CommandValueKind::Float, 0, 0, -1.0f, 1.0f, "", false, "Nudge low bound trim." },
    { CommandId::GeomEndTrimNudge, "geom_end_trim_nudge", CommandValueKind::Float, 0, 0, -1.0f, 1.0f, "", false, "Nudge high bound trim." },
    { CommandId::GeomPreset, "geom_preset", CommandValueKind::Integer, 0, 20, 0, 0, "", false, "Apply geometry preset." },
    { CommandId::GeomTotal, "geom_total", CommandValueKind::Float, 0, 0, 0.0f, 200.0f, "", false, "Total bobbin width (mm)." },
    { CommandId::GeomBottom, "geom_bottom", CommandValueKind::Float, 0, 0, 0.0f, 50.0f, "", false, "Bottom flange width (mm)." },
    { CommandId::GeomTop, "geom_top", CommandValueKind::Float, 0, 0, 0.0f, 50.0f, "", false, "Top flange width (mm)." },
    { CommandId::GeomMargin, "geom_margin", CommandValueKind::Float, 0, 0, 0.0f, 20.0f, "", false, "Winding margin (mm)." },
    { CommandId::GeomWire, "geom_wire", CommandValueKind::Float, 0, 0, 0.01f, 1.0f, "", false, "Wire diameter (mm)." },
    { CommandId::GeomTppOffset, "geom_tpp_ofs", CommandValueKind::Integer, -2000, 2000, 0, 0, "", false, "Turns-per-pass offset." },
    { CommandId::GeomScatter, "geom_scatter", CommandValueKind::Float, 0, 0, 0.5f, 5.0f, "", false, "Scatter factor." },
    { CommandId::WindowShift, "window_shift", CommandValueKind::Float, 0, 0, -5.0f, 5.0f, "", true, "Shift both winding bounds (mm)." },
    { CommandId::WindingStyle, "winding_style", CommandValueKind::Enum, 0, 0, 0, 0, "straight,scatter,human", false, "Pattern style." },
    { CommandId::WindingSeed, "winding_seed", CommandValueKind::Integer, 1, 2147483647, 0, 0, "", false, "Pattern random seed." },
    { CommandId::WindingLayerJitter, "winding_layer_jitter", CommandValueKind::Float, 0, 0, 0.0f, 0.45f, "", false, "Per-layer TPP jitter." },
    { CommandId::WindingLayerSpeed, "winding_layer_speed", CommandValueKind::Float, 0, 0, 0.0f, 0.45f, "", false, "Per-layer speed jitter." },
    { CommandId::WindingHumanTraverse, "winding_human_traverse", CommandValueKind::Float, 0, 0, 0.0f, 0.45f, "", false, "Smooth traverse variation." },
    { CommandId::WindingHumanSpeed, "winding_human_speed", CommandValueKind::Float, 0, 0, 0.0f, 0.45f, "", false, "Smooth speed variation." },
    { CommandId::WindingFirstPassTraverse, "winding_first_pass_traverse", CommandValueKind::Float, 0, 0, 0.40f, 1.80f, "", false, "First pass traverse scale." },
    { CommandId::RecipeImport, "recipe_import", CommandValueKind::Json, 0, 0, 0, 0, "", false, "Import full recipe JSON." },
    { CommandId::LatOffset, "lat_offset", CommandValueKind::Float, 0, 0, 0.0f, 200.0f, "", false, "Lateral home offset (mm)." },
};

bool isBool(const char* s) {
    return strcmp(s, "true") == 0 || strcmp(s, "false") == 0;
}

bool csvContains(const char* csv, const char* value) {
    if (!csv || !value) return false;
    const size_t vlen = strlen(value);
    const char* p = csv;
    while (*p) {
        const char* c = strchr(p, ',');
        const size_t len = c ? (size_t)(c - p) : strlen(p);
        if (len == vlen && strncmp(p, value, len) == 0) return true;
        if (!c) break;
        p = c + 1;
    }
    return false;
}

const CommandDefinition* findWindowShiftDef() {
    for (size_t i = 0; i < (sizeof(kCommands) / sizeof(kCommands[0])); ++i) {
        if (strcmp(kCommands[i].key, "window_shift") == 0) return &kCommands[i];
    }
    return nullptr;
}

bool normalizeKeyImpl(const char* in, char* out, size_t outLen) {
    if (!in || !out || outLen == 0) return false;

    if (strcmp(in, "max-rpm") == 0) in = "max_rpm";
    if (strcmp(in, "windows_shift") == 0) in = "window_shift";

    if (strncmp(in, "window_shift", strlen("window_shift")) == 0) {
        in = "window_shift";
    }

    const size_t len = strlen(in);
    if (len + 1 > outLen) return false;
    memcpy(out, in, len + 1);
    return true;
}

bool validateValue(const CommandDefinition* def, const char* value) {
    if (!def || !value) return false;

    switch (def->kind) {
    case CommandValueKind::None:
        return value[0] == '\0';

    case CommandValueKind::Boolean:
        return isBool(value);

    case CommandValueKind::Enum:
        return csvContains(def->enumValuesCsv, value);

    case CommandValueKind::Integer: {
        char* end = nullptr;
        const long v = strtol(value, &end, 10);
        if (!end || *end != '\0') return false;
        return v >= def->minInt && v <= def->maxInt;
    }

    case CommandValueKind::Float: {
        char* end = nullptr;
        const float v = strtof(value, &end);
        if (!end || *end != '\0') return false;
        return v >= def->minFloat && v <= def->maxFloat;
    }

    case CommandValueKind::Json:
        return value[0] == '{' && strlen(value) >= 2;
    }

    return false;
}

const char* kindToString(CommandValueKind kind) {
    switch (kind) {
    case CommandValueKind::None: return "none";
    case CommandValueKind::Integer: return "integer";
    case CommandValueKind::Float: return "float";
    case CommandValueKind::Boolean: return "boolean";
    case CommandValueKind::Enum: return "enum";
    case CommandValueKind::Json: return "json";
    default: return "unknown";
    }
}

} // namespace

const CommandDefinition* CommandRegistry::all(size_t& count) {
    count = sizeof(kCommands) / sizeof(kCommands[0]);
    return kCommands;
}

const CommandDefinition* CommandRegistry::findByKey(const char* key) {
    if (!key) return nullptr;

    if (strncmp(key, "window_shift", strlen("window_shift")) == 0) {
        return findWindowShiftDef();
    }

    for (size_t i = 0; i < (sizeof(kCommands) / sizeof(kCommands[0])); ++i) {
        if (strcmp(kCommands[i].key, key) == 0) return &kCommands[i];
    }
    return nullptr;
}

const CommandDefinition* CommandRegistry::findById(CommandId id) {
    for (size_t i = 0; i < (sizeof(kCommands) / sizeof(kCommands[0])); ++i) {
        if (kCommands[i].id == id) return &kCommands[i];
    }
    return nullptr;
}

bool CommandRegistry::normalizeAndValidate(const char* cmdIn,
                                           const char* valueIn,
                                           char* canonicalCmdOut,
                                           size_t canonicalCmdOutLen,
                                           CommandId* idOut) {
    if (!cmdIn || !valueIn) return false;
    if (!normalizeKeyImpl(cmdIn, canonicalCmdOut, canonicalCmdOutLen)) return false;

    const CommandDefinition* def = findByKey(canonicalCmdOut);
    if (!def) return false;
    if (!validateValue(def, valueIn)) return false;

    if (idOut) *idOut = def->id;
    return true;
}

void CommandRegistry::capabilitiesJson(char* outBuf, size_t outBufLen) {
    if (!outBuf || outBufLen == 0) return;

    size_t used = 0;
    int n = snprintf(outBuf, outBufLen,
                     "{\"capabilitiesVersion\":\"%s\","
                     "\"protocol\":{\"ws\":\"%s\",\"uart\":\"%s\",\"recipe\":%u},"
                     "\"commands\":[",
                     PICKUP_CAPABILITIES_VERSION,
                     PICKUP_WS_PROTOCOL_VERSION,
                     PICKUP_UART_PROTOCOL_VERSION,
                     (unsigned)PICKUP_RECIPE_FORMAT_VERSION);
    if (n < 0) {
        outBuf[0] = '\0';
        return;
    }
    used = (size_t)n;

    size_t count = 0;
    const CommandDefinition* defs = all(count);
    for (size_t i = 0; i < count && used < outBufLen; ++i) {
        const CommandDefinition& d = defs[i];
        n = snprintf(outBuf + used, outBufLen - used,
                     "%s{\"key\":\"%s\",\"type\":\"%s\",\"mutableDuringSession\":%s,\"help\":\"%s\"}",
                     (i == 0) ? "" : ",",
                     d.key,
                     kindToString(d.kind),
                     d.mutableDuringSession ? "true" : "false",
                     d.help ? d.help : "");
        if (n < 0) break;
        used += (size_t)n;
    }

    if (used + 3 <= outBufLen) {
        snprintf(outBuf + used, outBufLen - used, "]}");
    } else {
        // Truncated: close the JSON array to keep the output parseable.
        if (outBufLen >= 3) {
            outBuf[outBufLen - 3] = ']';
            outBuf[outBufLen - 2] = '}';
        }
        outBuf[outBufLen - 1] = '\0';
    }
}
