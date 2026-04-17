#include "StepperController.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include <cstdlib>

static const char* TAG = "StepperController";

// ── Construction / destruction ────────────────────────────────────────────────

StepperController::StepperController(const StepperConfig& cfg)
    : cfg_(cfg) {}

StepperController::~StepperController() {
    stop();
    if (rmt_chan_)    rmt_del_channel(rmt_chan_);
    if (uniform_enc_) rmt_del_encoder(uniform_enc_);
}

// ── Démarrage ─────────────────────────────────────────────────────────────────

esp_err_t StepperController::start() {
    // Aucun allocation dynamique nécessaire : la queue est lock-free et embarquée.

    // Canal RMT
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num        = cfg_.step_gpio,
        .clk_src         = RMT_CLK_SRC_DEFAULT,
        .resolution_hz   = cfg_.resolution_hz,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&chan_cfg, &rmt_chan_),
                        TAG, "rmt channel failed");

    // Encodeur uniforme (partagé entre tous les segments UNIFORM)
    stepper_motor_uniform_encoder_config_t uni_cfg = {
        .resolution = cfg_.resolution_hz,
    };
    ESP_RETURN_ON_ERROR(rmt_new_stepper_motor_uniform_encoder(&uni_cfg, &uniform_enc_),
                        TAG, "uniform encoder failed");

    ESP_RETURN_ON_ERROR(rmt_enable(rmt_chan_), TAG, "rmt enable failed");

    // Direction GPIO
    gpio_set_direction(cfg_.dir_gpio, GPIO_MODE_OUTPUT);

    // Tâche dédiée sur le core voulu
    BaseType_t ok = xTaskCreatePinnedToCore(
        taskEntry, "stepper_ctrl",
        4096, this,
        cfg_.task_priority,
        &task_handle_,
        cfg_.task_core
    );
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "task create failed");
    return ESP_OK;
}

// ── Interface publique ────────────────────────────────────────────────────────

esp_err_t StepperController::enqueue(const MoveCommand& cmd, TickType_t timeout) {
    if (timeout == 0) {
        return queue_.push(cmd) ? ESP_OK : ESP_ERR_TIMEOUT;
    }

    if (timeout == portMAX_DELAY) {
        while (!queue_.push(cmd)) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return ESP_OK;
    }

    const TickType_t start = xTaskGetTickCount();
    while (!queue_.push(cmd)) {
        if ((xTaskGetTickCount() - start) >= timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_OK;
}

void StepperController::stop() {
    MoveCommand poison{};
    while (!queue_.push(poison)) {
        taskYIELD();
    }
}

// ── Tâche FreeRTOS ────────────────────────────────────────────────────────────

void StepperController::taskEntry(void* arg) {
    static_cast<StepperController*>(arg)->run();
}

void StepperController::run() {
    MoveCommand cmd;
    for (;;) {
        while (!queue_.pop(cmd)) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Commande vide = signal d'arrêt
        if (cmd.target_steps == 0 &&
            cmd.accel_steps  == 0 &&
            cmd.decel_steps  == 0) {
            break;
        }

        const bool forward = cmd.target_steps > 0;
        gpio_set_level(cfg_.dir_gpio, forward ? 1 : 0);
        const uint32_t total = static_cast<uint32_t>(
                                   forward ? cmd.target_steps : -cmd.target_steps);

        // Phase 1 : accélération (courbe smoothstep)
        if (cmd.accel_steps > 0) {
            StepSegment seg{
                .type         = StepSegment::Type::CURVE,
                .steps        = cmd.accel_steps,
                .freq_start_hz = cmd.start_freq_hz,
                .freq_end_hz   = cmd.end_freq_hz,
            };
            executeSegment(seg, forward);
        }

        // Phase 2 : vitesse uniforme
        const uint32_t uniform_steps =
            total - cmd.accel_steps - cmd.decel_steps;
        if (uniform_steps > 0) {
            StepSegment seg{
                .type    = StepSegment::Type::UNIFORM,
                .steps   = uniform_steps,
                .freq_hz = cmd.end_freq_hz,
            };
            executeSegment(seg, forward);
        }

        // Phase 3 : décélération
        if (cmd.decel_steps > 0) {
            StepSegment seg{
                .type          = StepSegment::Type::CURVE,
                .steps         = cmd.decel_steps,
                .freq_start_hz = cmd.end_freq_hz,
                .freq_end_hz   = cmd.start_freq_hz,
            };
            executeSegment(seg, forward);
        }
    }
    vTaskDelete(nullptr);
}

// ── Exécution d'un segment RMT ────────────────────────────────────────────────

esp_err_t StepperController::executeSegment(const StepSegment& seg, bool forward) {
    rmt_encoder_handle_t encoder = nullptr;
    bool curve_encoder_created   = false;

    if (seg.type == StepSegment::Type::CURVE) {
        stepper_motor_curve_encoder_config_t curve_cfg = {
            .resolution     = cfg_.resolution_hz,
            .sample_points  = seg.steps,  // 1 symbole = 1 pas
            .start_freq_hz  = seg.freq_start_hz,
            .end_freq_hz    = seg.freq_end_hz,
        };
        ESP_RETURN_ON_ERROR(
            rmt_new_stepper_motor_curve_encoder(&curve_cfg, &encoder),
            TAG, "curve encoder create failed");
        curve_encoder_created = true;
    } else {
        encoder = uniform_enc_;
    }

    rmt_transmit_config_t tx_cfg = {
        .loop_count = static_cast<int>(seg.steps),  // répète le symbole N fois
    };

    uint32_t primary = (seg.type == StepSegment::Type::UNIFORM)
                       ? seg.freq_hz
                       : seg.steps;   // pour la courbe : nb de points actifs

    esp_err_t ret = rmt_transmit(rmt_chan_, encoder, &primary,
                                 sizeof(primary), &tx_cfg);
    if (ret == ESP_OK) {
        ret = rmt_tx_wait_all_done(rmt_chan_, pdMS_TO_TICKS(5000));
    }

    if (curve_encoder_created) rmt_del_encoder(encoder);

    if (ret == ESP_OK) {
        position_ += forward
                     ? static_cast<int32_t>(seg.steps)
                     : -static_cast<int32_t>(seg.steps);
    }
    return ret;
}