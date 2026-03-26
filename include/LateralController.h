#pragma once
#include <FastAccelStepper.h>
#include "Config.h"

// États de la machine d'état du homing latéral.
enum class LatState {
    FAULT,          // Capteur absent ou défaillant — tout mouvement latéral bloqué
    BACKOFF,        // Capteur actif au démarrage — recul jusqu'à libération du capteur
    HOMING,         // Déplacement vers la position initiale (home)
    HOMING_DECEL,   // Capteur atteint — décélération normale en cours avant arrêt
    HOMING_ALIGN,   // Alignement sur le pas complet le plus proche (supprime le grésillement)
    HOMING_OFFSET,  // Déplacement de l'offset entre le capteur et la vraie position 0
    HOMED,          // Position initiale atteinte — driver maintenu actif en permanence
    POSITIONING,    // Déplacement vers la position de départ avant autorisation du bobinage
    WINDING_FWD,    // Bobinage réel : aller (0 → effWidth), synchronisé sur la vitesse de bobinage
    WINDING_BWD,    // Bobinage réel : retour (effWidth → 0), synchronisé sur la vitesse de bobinage
};

// ── LateralController ─────────────────────────────────────────────────────────
// Gère le stepper de l'axe latéral (guide-fil) avec homing automatique.
//
// Séquence de démarrage :
//   1. Vérification de la présence du capteur (NO et NC complémentaires).
//      En cas d'absence → état FAULT, mouvement impossible.
//   2. Si le capteur est déjà actif (axe déjà en home) → recul jusqu'à libération,
//      puis homing normal.
//   3. Déplacement vers le home (LAT_HOME_DIR) à vitesse réduite (LAT_HOME_SPEED_HZ).
//   4. Dès que le capteur détecte la position, arrêt + position remise à zéro.
//      Le driver reste ACTIF en permanence après homing (ne jamais appeler disableDriver
//      sur l'axe latéral une fois homé).
//
// Protocole capteur (INPUT_PULLUP, capteur connecté à GND) :
//   Hors home : NO=HIGH (ouvert), NC=LOW  (fermé sur GND)
//   En home   : NO=LOW  (fermé sur GND),  NC=HIGH (ouvert)
//   Défaut    : NO=HIGH, NC=HIGH → capteur débranché (les deux tirés par pull-up)
class LateralController {
public:
    /**
     * @brief Initialize lateral axis and start homing sequence.
     * @param engine Shared FastAccelStepper engine.
     * @param homeOffsetMm Offset from switch trigger to logical position zero.
     */
    void begin(FastAccelStepperEngine& engine, float homeOffsetMm = LAT_HOME_OFFSET_DEFAULT_MM);

    /**
     * @brief Non-blocking state-machine update.
     * @par Usage
     * Call on every main-loop iteration.
     */
    void update();

    /** @brief Get current lateral-axis state. */
    LatState    getState()  const { return _state; }
    /** @brief Check if axis is homed or currently traversing. */
    bool        isHomed()   const {
        return _state == LatState::HOMED
            || _state == LatState::WINDING_FWD
            || _state == LatState::WINDING_BWD;
    }
    /** @brief Check whether axis is in fault state. */
    bool        isFault()   const { return _state == LatState::FAULT;  }
    /** @brief Human-readable state string. */
    const char* stateStr()  const;

    /**
     * @brief Restart homing sequence manually.
     * @par Usage
     * Used by service UI and recovery workflows.
     */
    void rehome();

    /**
     * @brief Move carriage to requested start position.
     * @param startMm Target position in mm from bobbin base.
     * @param speedHz Positioning speed in step-Hz.
     */
    void prepareStartPosition(float startMm, uint32_t speedHz = LAT_TRAVERSE_SPEED_HZ);
    /** @brief True when axis is homed exactly at configured start position. */
    bool isPositionedForStart() const { return _state == LatState::HOMED && _isAtStartPosition(); }
    /** @brief Convenience helper parking carriage at 0 mm. */
    void parkAtZero() { prepareStartPosition(0.0f); }
    /** @brief Check if carriage is at physical/electrical zero (home). */
    bool isAtZero() const { return _state == LatState::HOMED && _stepper && abs(_stepper->getCurrentPosition()) <= MICROSTEPPING; }
    /** @brief True when lateral motor currently runs. */
    bool isBusy() const { return _stepper && _stepper->isRunning(); }

    /**
     * @brief Relative jog move from current/target position.
     * @param deltaMm Relative displacement in mm.
     */
    void jog(float deltaMm);
    /** @brief Arm pause on next natural reversal. */
    void armPauseOnNextReversal() { _pauseOnNextReversal = true; _pausedAtReversal = false; }
    /** @brief Arm one-shot stop at next high bound. */
    void armStopAtNextHigh() { _stopOnNextHigh = true; _pausedAtReversal = false; }
    /** @brief Arm one-shot stop at next low bound. */
    void armStopAtNextLow()  { _stopOnNextLow  = true; _pausedAtReversal = false; }
    /** @brief Check whether stop-on-high one-shot is armed. */
    bool isStopOnNextHighArmed() const { return _stopOnNextHigh; }
    /** @brief Check whether stop-on-low one-shot is armed. */
    bool isStopOnNextLowArmed()  const { return _stopOnNextLow; }
    /** @brief Clear all one-shot stop/pause flags. */
    void clearOneShotStops();
    /** @brief True if any stop-at-next-bound or pause-at-reversal flag is armed. */
    bool hasStopAtNextBoundArmed() const { return _stopOnNextHigh || _stopOnNextLow || _pauseOnNextReversal; }
    /**
     * @brief Consume and reset paused-at-reversal event latch.
     * @return true if a reversal pause event was pending.
     */
    bool consumePausedAtReversal() {
        bool v = _pausedAtReversal;
        _pausedAtReversal = false;
        return v;
    }

    /**
     * @brief Start synchronized lateral traversal for winding.
     * @param windingHz Spindle step frequency.
     * @param tpp Active turns-per-pass.
     * @param startMm Low bound in mm.
     * @param endMm High bound in mm.
     * @param speedScale Traverse speed multiplier.
     */
    void startWinding(uint32_t windingHz, long tpp, float startMm, float endMm, float speedScale = 1.0f);

    /**
     * @brief Update synchronized traversal parameters while running.
     */
    void updateWinding(uint32_t windingHz, long tpp, float startMm, float endMm, float speedScale = 1.0f);

    /** @brief Stop synchronized traversal and return to HOMED state. */
    void stopWinding();

    /** @brief True while lateral axis is inside reversal transition window. */
    bool isReversing() const { return millis() < _reversingUntilMs; }
    /** @brief True while actively traversing forward/backward. */
    bool isTraversing() const {
        return _state == LatState::WINDING_FWD || _state == LatState::WINDING_BWD;
    }
    /** @brief Number of completed half-passes since traversal start. */
    uint32_t getPassCount() const { return _passCount; }
    /** @brief Normalized progress (0..1) in current traversal direction. */
    float    getTraversalProgress() const;

    /**
     * @brief Set home offset between switch trigger and logical zero.
     * @param mm Offset in mm.
     */
    void  setHomeOffset(float mm);
    /** @brief Get configured home offset in mm. */
    float getHomeOffset() const { return _homeOffsetMm; }
    /** @brief Get current carriage position in mm. */
    float getCurrentPositionMm() const;
    /**
     * @brief Get current target position in mm.
     *
     * In `POSITIONING`, returns in-flight target. In `HOMED`, returns current
     * physical position.
     */
    float getTargetPositionMm() const;

private:
    FastAccelStepper*      _stepper         = nullptr;
    LatState               _state           = LatState::FAULT;
    uint32_t               _lastCheckMs     = 0;
    float                  _homeOffsetMm    = LAT_HOME_OFFSET_DEFAULT_MM;  // Offset chargé depuis NVS
    volatile bool          _homeFlag        = false;

    // ── Bobinage réel : état de la traversée synchronisée ────────────────────
    uint32_t               _latHz           = 0;    // Vitesse latérale courante (steps/s)
    int32_t                _latStartSteps   = 0;    // Début réel de la fenêtre de bobinage
    int32_t                _latEndSteps     = 0;    // Fin réelle de la fenêtre de bobinage
    uint32_t               _reversingUntilMs = 0;  // Fin de la fenêtre de ralentissement
    uint32_t               _passCount       = 0;    // Nombre de demi-couches réellement effectuées
    bool                   _pauseOnNextReversal = false;
    bool                   _stopOnNextHigh      = false;
    bool                   _stopOnNextLow       = false;
    bool                   _pausedAtReversal    = false;
    bool                   _lastDirFwd          = true;   // Direction mémorisée au dernier stopWinding()

    // Calcule la vitesse latérale (steps/s) synchronisée sur la vitesse de bobinage.
    // lat_Hz = effWidth_steps × windingHz / (tpp × STEPS_PER_REV)
    uint32_t _calcLatHz(uint32_t windingHz, long tpp, float widthMm, float speedScale) const;
    bool     _isAtStartPosition() const;
    void     _setTraverseBounds(float startMm, float endMm);

    // Déclenche la fenêtre de ralentissement bobinage au demi-tour latéral.
    void _onReversal();

    // ── Lecture capteur (INPUT_PULLUP, contact à GND = LOW) ───────────────
    // NO actif (contact fermé = en home) : pin LOW
    bool _noActive()       const { return digitalRead(HOME_PIN_NO) == LOW; }
    // NC actif (contact fermé = hors home) : pin LOW
    bool _ncActive()       const { return digitalRead(HOME_PIN_NC) == LOW; }
    // Capteur présent : NO et NC doivent être complémentaires (l'un HIGH, l'autre LOW)
    bool _sensorPresent()  const { return _noActive() != _ncActive(); }
    // En home : NO fermé (LOW) ET NC ouvert (HIGH)
    bool _atHome()         const { return _noActive() && !_ncActive(); }

    // ── ISR GPIO pour détection immédiate du capteur ──────────────────────
    // Attachée sur FALLING de HOME_PIN_NO uniquement pendant la phase HOMING.
    // Utilise attachInterruptArg() pour accéder à l'instance sans variable globale.
    static void IRAM_ATTR _homePinISR(void* arg);
    void _attachHomeISR();
    void _detachHomeISR();

    void _startHoming();
    void _startBackoff();
    void _applyOffsetOrNext(); // Après alignement : déplacement offset ou HOMED direct
    void _gotoHomed();         // Transition finale : axe prêt pour le bobinage
    void _enableDriver();
    void _disableDriver();
};
