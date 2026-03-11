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
    // Initialise les GPIO, se connecte à l'engine partagée et démarre le homing.
    // L'engine doit avoir été initialisée (engine.init()) avant cet appel.
    void begin(FastAccelStepperEngine& engine, float homeOffsetMm = LAT_HOME_OFFSET_DEFAULT_MM);

    // Machine d'états non-bloquante — appeler à chaque itération de loop().
    void update();

    // État courant de l'axe latéral.
    LatState    getState()  const { return _state; }
    bool        isHomed()   const {
        return _state == LatState::HOMED
            || _state == LatState::WINDING_FWD
            || _state == LatState::WINDING_BWD;
    }
    bool        isFault()   const { return _state == LatState::FAULT;  }
    const char* stateStr()  const;

    // Relancer le homing manuellement (ex. depuis l'interface web).
    // Sans effet si le capteur est absent.
    void rehome();

    // Déplacer le chariot vers la position de départ de la fenêtre de bobinage.
    // startMm est exprimé depuis la base du tonework.
    void prepareStartPosition(float startMm);
    bool isPositionedForStart() const { return _state == LatState::HOMED && _isAtStartPosition(); }
    void parkAtZero() { prepareStartPosition(0.0f); }
    bool isAtZero() const { return _state == LatState::HOMED && _stepper && abs(_stepper->getCurrentPosition()) <= MICROSTEPPING; }
    bool isBusy() const { return _stepper && _stepper->isRunning(); }
    void armPauseOnNextReversal() { _pauseOnNextReversal = true; _pausedAtReversal = false; }
    bool consumePausedAtReversal() {
        bool v = _pausedAtReversal;
        _pausedAtReversal = false;
        return v;
    }

    // ── Bobinage synchronisé ──────────────────────────────────────
    // Démarrer la traversée latérale synchronisée avec le moteur de bobinage.
    //   windingHz : fréquence de pas du stepper bobinage (readHz() / SpeedInput)
    //   tpp       : tours par passe (WindingGeometry::turnsPerPass())
    //   effWidthMm: largeur utile de bobinage en mm (WindingGeometry::effectiveWidth())
    // Sans effet si l'axe n'est pas en état HOMED.
    void startWinding(uint32_t windingHz, long tpp, float startMm, float endMm, float speedScale = 1.0f);

    // Mettre à jour la vitesse latérale en temps réel (appeler à chaque lecture du pot).
    // Sans effet si l'axe n'est pas en état WINDING_FWD / WINDING_BWD.
    void updateWinding(uint32_t windingHz, long tpp, float startMm, float endMm, float speedScale = 1.0f);

    // Arrêter la traversée latérale (décélération douce, retour à HOMED).
    void stopWinding();

    // Vrai pendant la fenêtre de demi-tour (décél + accél du moteur latéral).
    // WinderApp réduit la vitesse de bobinage pendant ce temps (LAT_REVERSAL_SLOWDOWN).
    bool isReversing() const { return millis() < _reversingUntilMs; }
    bool isTraversing() const {
        return _state == LatState::WINDING_FWD || _state == LatState::WINDING_BWD;
    }
    uint32_t getPassCount() const { return _passCount; }
    float    getTraversalProgress() const;

    // Offset entre le hard-stop capteur et la vraie position 0.
    // Valeur positive en mm.
    void  setHomeOffset(float mm);
    float getHomeOffset() const { return _homeOffsetMm; }
    float getCurrentPositionMm() const;

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
