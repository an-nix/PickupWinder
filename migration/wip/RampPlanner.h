#pragma once
#include <stdint.h>
#include "StepperController.h"

/**
 * Calcule les MoveCommand à partir d'une consigne de position.
 * Tourne idéalement sur le core 1 (xTaskCreatePinnedToCore core=1).
 * N'a aucune dépendance sur RMT ou GPIO.
 */
class RampPlanner {
public:
    struct MotionProfile {
        uint32_t max_freq_hz;    // vitesse de pointe
        uint32_t start_freq_hz;  // vitesse de départ/fin
        uint32_t sample_points;  // points de la courbe smoothstep
        // Pour un profil triangulaire, posez accel_ratio = 0.5
        float    accel_ratio;    // fraction des pas en accélération [0..0.5]
        float    decel_ratio;    // fraction des pas en décélération [0..0.5]
    };

    explicit RampPlanner(StepperController& ctrl,
                         const MotionProfile& profile);

    /**
     * Calcule et enfile une commande pour atteindre `absolute_steps`
     * depuis la position actuelle du contrôleur.
     */
    esp_err_t moveTo(int32_t absolute_steps);

    /**
     * Déplacement relatif.
     */
    esp_err_t moveBy(int32_t delta_steps);

private:
    MoveCommand buildCommand(int32_t delta) const;

    StepperController& ctrl_;
    MotionProfile      profile_;
};