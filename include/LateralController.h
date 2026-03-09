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
    HOMED,          // Position initiale atteinte — driver maintenu actif en permanence
    TRAVERSE_FWD,   // Test/rodage : aller vers la position maximale
    TRAVERSE_BWD,   // Test/rodage : retour vers la position initiale (0)
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
    void begin(FastAccelStepperEngine& engine);

    // Machine d'états non-bloquante — appeler à chaque itération de loop().
    void update();

    // État courant de l'axe latéral.
    LatState    getState()  const { return _state; }
    bool        isHomed()   const { return _state == LatState::HOMED;  }
    bool        isFault()   const { return _state == LatState::FAULT;  }
    const char* stateStr()  const;

    // Relancer le homing manuellement (ex. depuis l'interface web).
    // Sans effet si le capteur est absent.
    void rehome();

private:
    FastAccelStepper*      _stepper         = nullptr;
    LatState               _state           = LatState::FAULT;
    uint32_t               _lastCheckMs     = 0;  // Timer pour les vérifications périodiques
    // Drapeau positionné par l'ISR GPIO dès que HOME_PIN_NO passe à LOW.
    // Traité dans update() pour arrêter le moteur sans dépendre de la cadence de la boucle.
    volatile bool          _homeFlag        = false;

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
    void _startTraverseFwd();  // Lance un aller vers LAT_TRAVERSE_MM (test/rodage)
    void _enableDriver();
    void _disableDriver();
};
