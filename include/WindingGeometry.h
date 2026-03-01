#pragma once
#include <Arduino.h>

// ── Diamètres de fil avec isolation (en mm) ──────────────────────────────────
namespace WireGauge {
    constexpr float AWG42 = 0.071f;   // ~8000 tours Strat
    constexpr float AWG43 = 0.064f;
    constexpr float AWG44 = 0.058f;
    constexpr float AWG46 = 0.047f;
}

// ── Préréglages de bobines courantes ─────────────────────────────────────────
struct BobbinPreset { const char* name; float total; float flangeBot; float flangeTop; float wire; };

static const BobbinPreset BOBBIN_PRESETS[] = {
    { "Strat",      17.0f, 1.5f, 1.5f, WireGauge::AWG42 },
    { "Telecaster", 18.5f, 1.5f, 1.5f, WireGauge::AWG42 },
    { "P90",        30.0f, 2.0f, 2.0f, WireGauge::AWG42 },
    { "Humbucker",  38.0f, 2.0f, 2.0f, WireGauge::AWG43 },
};
static constexpr uint8_t BOBBIN_PRESET_COUNT = sizeof(BOBBIN_PRESETS) / sizeof(BOBBIN_PRESETS[0]);

// ── Géométrie de bobinage ────────────────────────────────────────────────────
struct WindingGeometry {
    float totalWidth_mm   = 17.0f;   // Épaisseur totale du bobbin (bord à bord)
    float flangeBottom_mm = 1.5f;    // Épaisseur du tonework bas
    float flangeTop_mm    = 1.5f;    // Épaisseur du tonework haut
    float margin_mm       = 0.5f;    // Marge de sécurité de chaque côté
    float wireDiameter_mm = WireGauge::AWG42;  // Diamètre du fil avec isolation
    long  turnsPerPassOffset = 0;  // Offset applied to auto-calc (-N to +N)

    // Largeur de bobinage utile
    float effectiveWidth() const {
        float w = totalWidth_mm - flangeBottom_mm - flangeTop_mm - 2.0f * margin_mm;
        return max(0.0f, w);
    }

    // Nombre de tours calculé automatiquement (formule géométrique pure)
    long turnsPerPassCalc() const {
        if (wireDiameter_mm <= 0.0f) return 1;
        return max(1L, (long)(effectiveWidth() / wireDiameter_mm));
    }

    // Nombre de tours par aller (ou retour).
    // Applique l'offset au calcul automatique (clampé à minimum 1).
    long turnsPerPass() const {
        return max(1L, turnsPerPassCalc() + turnsPerPassOffset);
    }

    void applyPreset(uint8_t idx) {
        if (idx >= BOBBIN_PRESET_COUNT) return;
        totalWidth_mm   = BOBBIN_PRESETS[idx].total;
        flangeBottom_mm = BOBBIN_PRESETS[idx].flangeBot;
        flangeTop_mm    = BOBBIN_PRESETS[idx].flangeTop;
        wireDiameter_mm = BOBBIN_PRESETS[idx].wire;
        turnsPerPassOffset = 0;  // Reset offset on preset change
    }
};
