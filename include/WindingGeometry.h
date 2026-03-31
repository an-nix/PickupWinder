#pragma once
#include <Arduino.h>

// ── Insulated wire diameters (mm) ────────────────────────────────────────────
namespace WireGauge {
    constexpr float AWG42 = 0.071f;   // Typical Stratocaster winding gauge
    constexpr float AWG43 = 0.064f;
    constexpr float AWG44 = 0.058f;
    constexpr float AWG46 = 0.047f;
}

// ── Common bobbin presets ────────────────────────────────────────────────────
struct BobbinPreset { const char* name; float total; float flangeBot; float flangeTop; float wire; };

static const BobbinPreset BOBBIN_PRESETS[] = {
    { "Strat",      17.0f, 1.5f, 1.5f, WireGauge::AWG42 },
    { "Telecaster", 18.5f, 1.5f, 1.5f, WireGauge::AWG42 },
    { "P90",        30.0f, 2.0f, 2.0f, WireGauge::AWG42 },
    { "Humbucker",  38.0f, 2.0f, 2.0f, WireGauge::AWG43 },
};
static constexpr uint8_t BOBBIN_PRESET_COUNT = sizeof(BOBBIN_PRESETS) / sizeof(BOBBIN_PRESETS[0]);

// ── Winding geometry ─────────────────────────────────────────────────────────
/**
 * @brief Geometric parameters used to compute winding bounds and spacing.
 */
struct WindingGeometry {
    float totalWidth_mm   = 17.0f;   // Total bobbin width, flange to flange.
    float flangeBottom_mm = 1.5f;    // Bottom flange thickness.
    float flangeTop_mm    = 1.5f;    // Top flange thickness.
    float margin_mm       = 0.5f;    // Safety margin applied on both sides.
    float windingStartTrim_mm = 0.0f; // Fine trim applied to the low/start bound.
    float windingEndTrim_mm = 0.0f;  // Fine trim applied to the high/end bound.
    float wireDiameter_mm = WireGauge::AWG42;  // Insulated wire diameter.
    long  turnsPerPassOffset = 0;    // Offset applied to auto-calc (-N to +N)
    float scatterFactor = 1.0f;      // Base spacing multiplier (1.0 = regular, >1.0 = looser)

    /**
     * @brief Compute usable winding width in millimeters.
     * @return Positive width between end and start bounds.
     */
    float effectiveWidth() const {
        return max(0.0f, windingEndMm() - windingStartMm());
    }

    /**
     * @brief Compute low/start bound in millimeters.
     * @return Start position measured from bobbin base.
     */
    float windingStartMm() const {
        float start = flangeBottom_mm + margin_mm + windingStartTrim_mm;
        return max(0.0f, start);
    }

    /**
     * @brief Compute high/end bound in millimeters.
     * @return End position measured from bobbin base.
     */
    float windingEndMm() const {
        float end = totalWidth_mm - flangeTop_mm - margin_mm + windingEndTrim_mm;
        return max(windingStartMm(), end);
    }

    /**
     * @brief Compute nominal turns-per-pass from geometry and wire spacing.
     * @return Calculated turns-per-pass before manual offset.
     */
    long turnsPerPassCalc() const {
        if (wireDiameter_mm <= 0.0f) return 1;
        float spacing = wireDiameter_mm * max(0.5f, scatterFactor);
        return max(1L, (long)(effectiveWidth() / spacing));
    }

    /**
     * @brief Compute effective turns-per-pass.
     * @return Calculated turns-per-pass plus offset, clamped to at least 1.
     */
    float turnsPerPassFloat() const {
        // Effective turns-per-pass as a float value with manual offset.
        float val = (float)turnsPerPassCalc() + (float)turnsPerPassOffset;
        return max(1.0f, val);
    }

    long turnsPerPass() const {
        return max(1L, (long)lroundf(turnsPerPassFloat()));
    }

    /**
     * @brief Apply a predefined bobbin geometry preset.
     * @param idx Preset index in `BOBBIN_PRESETS`.
     */
    void applyPreset(uint8_t idx) {
        if (idx >= BOBBIN_PRESET_COUNT) return;
        totalWidth_mm   = BOBBIN_PRESETS[idx].total;
        flangeBottom_mm = BOBBIN_PRESETS[idx].flangeBot;
        flangeTop_mm    = BOBBIN_PRESETS[idx].flangeTop;
        windingStartTrim_mm = 0.0f;
        windingEndTrim_mm = 0.0f;
        wireDiameter_mm = BOBBIN_PRESETS[idx].wire;
        turnsPerPassOffset = 0;  // Reset offset on preset change
    }
};
