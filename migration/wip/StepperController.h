#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/rmt_tx.h"
#include "stepper_motor_encoder.h"
#include "command_queue.h"

/**
 * Segment de mouvement atomique passé dans la queue.
 * Le planificateur externe remplit cette structure ;
 * StepperController l'exécute sans rien calculer.
 */
struct StepSegment {
    enum class Type : uint8_t {
        CURVE,    // accélération ou décélération (encodeur courbe)
        UNIFORM,  // vitesse constante
        STOP,     // arrêt immédiat (poison pill pour quitter la tâche)
    };
    Type     type;
    uint32_t steps;          // nombre d'impulsions à émettre
    uint32_t freq_start_hz;  // utilisé par CURVE uniquement
    uint32_t freq_end_hz;    // utilisé par CURVE uniquement
    uint32_t freq_hz;        // utilisé par UNIFORM uniquement
};

/**
 * Commande de haut niveau envoyée par l'application.
 * StepperController la décompose en segments RMT.
 * NOTE : si vous préférez pré-calculer les segments côté planificateur,
 * remplacez MoveCommand par un tableau de StepSegment.
 */
struct MoveCommand {
    int32_t  target_steps;    // positif = avant, négatif = arrière
    uint32_t accel_steps;     // nombre de pas en phase d'accélération
    uint32_t decel_steps;     // nombre de pas en phase de décélération
    uint32_t start_freq_hz;   // fréquence de départ (accel/decel)
    uint32_t end_freq_hz;     // fréquence de pointe (vitesse max)
    uint32_t sample_points;   // points de la courbe smoothstep
};

struct StepperConfig {
    gpio_num_t  step_gpio;
    gpio_num_t  dir_gpio;
    uint32_t    resolution_hz;      // résolution RMT, ex. 1 000 000
    UBaseType_t task_priority;      // priorité de la tâche FreeRTOS
    BaseType_t  task_core;          // core cible (0 ou 1)
};

class StepperController {
public:
    explicit StepperController(const StepperConfig& cfg);
    ~StepperController();

    /** Démarre la tâche FreeRTOS dédiée. */
    esp_err_t start();

    /** Envoie une commande ; bloque si la queue est pleine. */
    esp_err_t enqueue(const MoveCommand& cmd,
                      TickType_t timeout = portMAX_DELAY);

    /** Arrête la tâche proprement (envoie un segment STOP). */
    void stop();

    /** Position courante en pas (mise à jour après chaque segment). */
    int32_t position() const { return position_; }

private:
    static void taskEntry(void* arg);
    void        run();                          // boucle de la tâche
    esp_err_t   executeSegment(const StepSegment& seg, bool forward);

    static constexpr size_t QUEUE_CAPACITY = 16;

    StepperConfig                     cfg_;
    CommandQueue<MoveCommand, QUEUE_CAPACITY> queue_;
    TaskHandle_t                      task_handle_ = nullptr;
    rmt_channel_handle_t   rmt_chan_    = nullptr;
    rmt_encoder_handle_t   uniform_enc_ = nullptr;
    volatile int32_t       position_   = 0;
};