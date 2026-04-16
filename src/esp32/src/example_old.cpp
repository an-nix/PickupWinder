// main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include "hardware_config.h"
#include "types.h"

static const char *TAG = "stepper_rmt";



/* RMT */
#define RMT_CHANNEL    RMT_CHANNEL_0
#define RMT_CLK_DIV    80   // 80 MHz / 80 = 1 MHz -> 1 tick = 1 us
#define RMT_MEM_BLOCKS 1

/* Queue / buffering */
#define QUEUE_LEN         8192
#define BLOCK_SIZE        64   // nombre max d'items rmt envoyés par bloc
#define PULSE_US          10   // largeur du front actif (µs)
#define MAX_RMT_DUR       32767U // 15 bits
#define IDLE_DISABLE_MS   100  // délai d'inactivité avant désactiver ENABLE



/* Double buffers pour RMT */
static rmt_item32_t bufA[BLOCK_SIZE];
static rmt_item32_t bufB[BLOCK_SIZE];

static QueueHandle_t step_queue;

/* Etat ENABLE */
static bool enable_state = false; // true = enabled (driver actif)
static TickType_t last_activity_tick = 0;

/* Init RMT + DIR + ENABLE pin */
static void rmt_init_tx(void)
{
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL,
        .gpio_num = STEP_GPIO,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = RMT_MEM_BLOCKS,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        }
    };
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);

    // DIR pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<DIR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // ENABLE pin (active LOW typiquement) -> on met HIGH = disabled par défaut
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL<<ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&en_conf);
    gpio_set_level(ENABLE_GPIO, 1); // disabled (active low)
    enable_state = false;
}

/* Fonctions utilitaires pour ENABLE */
static inline void enable_driver(bool en)
{
    // active LOW : en=true -> level 0
    gpio_set_level(ENABLE_GPIO, en ? 0 : 1);
    enable_state = en;
    if (en) {
        last_activity_tick = xTaskGetTickCount();
    }
}

/* Ajoute dans buffer[] les rmt_item32_t correspondant à period_us avec pulse_us.
   idx est mis à jour. Retourne 0 si ok, -1 si buffer plein. */
static int append_step_items(rmt_item32_t *buffer, int buffer_len, int *idx,
                             uint32_t period_us, uint32_t pulse_us)
{
    if (pulse_us < 1) pulse_us = 1;
    if (pulse_us > period_us) pulse_us = period_us;

    uint32_t low_us = period_us - pulse_us;

    // Premier item : HIGH pulse_us, LOW chunk (<= MAX_RMT_DUR)
    if (*idx >= buffer_len) return -1;
    uint32_t first_low = (low_us > MAX_RMT_DUR) ? MAX_RMT_DUR : low_us;

    buffer[*idx].level0 = 1;
    buffer[*idx].duration0 = pulse_us;
    buffer[*idx].level1 = 0;
    buffer[*idx].duration1 = first_low;
    (*idx)++;
    low_us -= first_low;

    // Si il reste de la LOW, on ajoute des items LOW-only (level0 = 0)
    while (low_us > 0) {
        if (*idx >= buffer_len) return -1;
        uint32_t chunk = (low_us > MAX_RMT_DUR) ? MAX_RMT_DUR : low_us;
        buffer[*idx].level0 = 0;
        buffer[*idx].duration0 = chunk;
        // mettre un filler court pour level1 (évite duration1=0)
        buffer[*idx].level1 = 0;
        buffer[*idx].duration1 = 1;
        (*idx)++;
        low_us -= chunk;
    }
    return 0;
}

/* Tâche simulant le host : génère une rampe et pousse StepEvent dans la queue */
static void host_sim_task(void *arg)
{
    const uint32_t total_steps = 2000;
    const uint32_t accel_steps = 500;
    const uint32_t decel_steps = 500;
    const uint32_t cruise_steps = total_steps - accel_steps - decel_steps;

    const uint32_t start_period = 2000; // µs par pas (lent)
    const uint32_t cruise_period = 500; // µs par pas (rapide)

    for (uint32_t i = 0; i < total_steps; ++i) {
        StepEvent evt;
        evt.flags = 0;
        evt.dir = 1;

        if (i < accel_steps) {
            float t = (float)i / (float)accel_steps;
            evt.duration_us = start_period - (uint32_t)((start_period - cruise_period) * t);
        } else if (i < accel_steps + cruise_steps) {
            evt.duration_us = cruise_period;
        } else {
            uint32_t j = i - (accel_steps + cruise_steps);
            float t = (float)j / (float)decel_steps;
            evt.duration_us = cruise_period + (uint32_t)((start_period - cruise_period) * t);
        }

        // Exemple : forcer ENABLE pour toute la séquence (optionnel)
        evt.flags |= 0x01; // bit0 = demander enable

        // push dans la queue (bloquant si pleine)
        xQueueSend(step_queue, &evt, portMAX_DELAY);
    }

    vTaskDelete(NULL);
}

/* Tâche scheduler : remplit les buffers et envoie au RMT en double buffering */
static void rmt_scheduler_task(void *arg)
{
    rmt_item32_t *cur = bufA;
    rmt_item32_t *next = bufB;

    while (1) {
        int idx = 0;
        uint8_t current_dir = 0xFF; // valeur invalide pour forcer écriture au premier evt
        bool local_enable_forced = false;

        // Remplir le buffer courant jusqu'à BLOCK_SIZE ou jusqu'à ce que queue soit vide
        while (idx < BLOCK_SIZE) {
            StepEvent evt;
            if (xQueueReceive(step_queue, &evt, pdMS_TO_TICKS(10)) != pdTRUE) {
                // pas d'evt disponible rapidement -> sortir pour envoyer ce qu'on a
                break;
            }

            // Gérer flags d'ENABLE forcé/désactivé
            if (evt.flags & 0x02) {
                // bit1 = force disable
                enable_driver(false);
                local_enable_forced = true;
            } else if (evt.flags & 0x01) {
                // bit0 = force enable
                enable_driver(true);
                local_enable_forced = true;
            } else {
                // si non forcé, on met à jour last_activity pour auto-disable
                last_activity_tick = xTaskGetTickCount();
            }

            // Si direction change, et si on a déjà des items dans le buffer,
            // on doit envoyer le buffer partiel pour garantir DIR stable avant steps.
            if (current_dir != 0xFF && evt.dir != current_dir && idx > 0) {
                // remettre evt en tête pour le prochain tour
                xQueueSendToFront(step_queue, &evt, 0);
                break;
            }

            // appliquer la direction avant d'ajouter les items
            if (current_dir != evt.dir) {
                current_dir = evt.dir;
                gpio_set_level(DIR_GPIO, current_dir);
                // petit délai hardware si nécessaire (optionnel)
                // ets_delay_us(1);
            }

            // ajouter les items correspondant à cet evt (peut ajouter plusieurs rmt items)
            if (append_step_items(cur, BLOCK_SIZE, &idx, evt.duration_us, PULSE_US) < 0) {
                // buffer plein : on remet evt non consommé en tête
                xQueueSendToFront(step_queue, &evt, 0);
                break;
            }

            // marquer activité
            last_activity_tick = xTaskGetTickCount();
        }

        if (idx == 0) {
            // pas d'items -> vérifier idle disable
            if (enable_state) {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_activity_tick) > pdMS_TO_TICKS(IDLE_DISABLE_MS)) {
                    enable_driver(false);
                    ESP_LOGI(TAG, "Driver disabled due to idle");
                }
            }
            // attendre un peu avant de retenter
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Avant d'envoyer, s'assurer que le driver est activé
        if (!enable_state) {
            enable_driver(true);
            // petit délai si nécessaire pour que le driver prenne en compte ENABLE
            ets_delay_us(5);
        }

        // Envoyer le buffer courant au RMT (one-shot)
        rmt_write_items(RMT_CHANNEL, cur, idx, false);
        // Attendre la fin de transmission du bloc
        rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);

        // swap buffers
        rmt_item32_t *tmp = cur;
        cur = next;
        next = tmp;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init");
    step_queue = xQueueCreate(QUEUE_LEN, sizeof(StepEvent));
    if (!step_queue) {
        ESP_LOGE(TAG, "Queue creation failed");
        return;
    }

    rmt_init_tx();

    xTaskCreate(rmt_scheduler_task, "rmt_scheduler", 4096, NULL, 10, NULL);
    xTaskCreate(host_sim_task, "host_sim", 4096, NULL, 5, NULL);
}
