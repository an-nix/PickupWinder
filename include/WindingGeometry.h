#pragma once
#include <Arduino.h>
#include <math.h>

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
    long turnsPerPass() const {
        return max(1L, turnsPerPassCalc() + turnsPerPassOffset);
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

    /**
     * @brief Compute the turns-per-mm traverse ratio.
     * @return Effective turns per millimeter (turns per pass / effective width).
     * 
     * @details
     * The traverse ratio is the number of wire turns deposited per millimeter
     * of carriage travel. For uniform distribution, this should be NON-INTEGER.
     * 
     * Example:
     *   - turnsPerPass = 37
     *   - effectiveWidth = 10 mm
     *   - ratio = 3.7 turns/mm (GOOD: creates cross-hatch pattern)
     * 
     * If ratio is 3, 4, or 5 (integer), wires stack vertically on repeating layers,
     * causing visible banding and uneven density. This is a winding quality defect.
     */
    float computeTraverseRatio() const {
        float width = effectiveWidth();
        if (width <= 0.0f) return 0.0f;
        return (float)turnsPerPass() / width;
    }

    /**
     * @brief Check if traverse ratio is dangerously close to an integer.
     * @param ratio Traverse ratio from computeTraverseRatio().
     * @param tolerance Acceptable deviation from integer (default 0.15 = ±15%).
     * @return true if ratio is within tolerance of any integer value.
     * 
     * @details
     * Ratios like 2.9, 3.0, 3.1, 4.0, 4.1 are flagged as problematic because
     * they cause wire stacking and banding.
     */
    static bool isTraverseRatioProblematic(float ratio, float tolerance = 0.15f) {
        if (ratio <= 0.0f) return false;
        float frac = ratio - floorf(ratio);
        return (frac <= tolerance) || (frac >= (1.0f - tolerance));
    }
};
