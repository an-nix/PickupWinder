/* encoder.cpp — PCNT quadrature decoder implementation.
 *
 * Two PCNT channels on PCNT_UNIT_0 for full 4X decoding:
 *   Channel 0: pulse = A, control = B
 *   Channel 1: pulse = B, control = A  (reverses on B edges)
 *
 * Hardware filter: 100 APB ticks ≈ 1.25 µs at 80 MHz → rejects EMI.
 */

#include "encoder.h"
#include <driver/pcnt.h>
#include <esp_log.h>

static const char* TAG = "encoder";

static constexpr pcnt_unit_t ENC_UNIT = PCNT_UNIT_0;

// Last sampled count used by encoder_get_and_clear_delta().
static int16_t s_last_count = 0;

void encoder_init(const EncoderCfg& cfg) {
    // Channel 0: A is the pulse, B controls direction.
    //   B=HIGH → A rising  = count up,   A falling = count down
    //   B=LOW  → A rising  = count down, A falling = count up  (reversed)
    pcnt_config_t ch_a = {};
    ch_a.pulse_gpio_num = cfg.pin_a;
    ch_a.ctrl_gpio_num  = cfg.pin_b;
    ch_a.channel        = PCNT_CHANNEL_0;
    ch_a.unit           = ENC_UNIT;
    ch_a.pos_mode       = PCNT_COUNT_INC;
    ch_a.neg_mode       = PCNT_COUNT_DEC;
    ch_a.lctrl_mode     = PCNT_MODE_REVERSE;   // B=LOW  → reverse
    ch_a.hctrl_mode     = PCNT_MODE_KEEP;      // B=HIGH → keep
    ch_a.counter_h_lim  =  32767;
    ch_a.counter_l_lim  = -32767;
    pcnt_unit_config(&ch_a);

    // Channel 1: B is the pulse, A controls direction.
    //   A=HIGH → B rising  = count down, B falling = count up
    //   A=LOW  → B rising  = count up,   B falling = count down (reversed)
    pcnt_config_t ch_b = {};
    ch_b.pulse_gpio_num = cfg.pin_b;
    ch_b.ctrl_gpio_num  = cfg.pin_a;
    ch_b.channel        = PCNT_CHANNEL_1;
    ch_b.unit           = ENC_UNIT;
    ch_b.pos_mode       = PCNT_COUNT_DEC;
    ch_b.neg_mode       = PCNT_COUNT_INC;
    ch_b.lctrl_mode     = PCNT_MODE_REVERSE;
    ch_b.hctrl_mode     = PCNT_MODE_KEEP;
    ch_b.counter_h_lim  =  32767;
    ch_b.counter_l_lim  = -32767;
    pcnt_unit_config(&ch_b);

    // Apply hardware glitch filter (100 APB clocks ≈ 1.25 µs at 80 MHz).
    pcnt_set_filter_value(ENC_UNIT, 100);
    pcnt_filter_enable(ENC_UNIT);

    pcnt_counter_pause(ENC_UNIT);
    pcnt_counter_clear(ENC_UNIT);
    pcnt_counter_resume(ENC_UNIT);

    s_last_count = 0;

    ESP_LOGI(TAG, "Encoder PCNT unit %d: A=GPIO%d B=GPIO%d (4X quadrature, filter=1.25µs)",
             ENC_UNIT, cfg.pin_a, cfg.pin_b);
}

int16_t encoder_get_count() {
    int16_t count = 0;
    pcnt_get_counter_value(ENC_UNIT, &count);
    return count;
}

int16_t encoder_get_and_clear_delta() {
    int16_t now   = encoder_get_count();
    int16_t delta = static_cast<int16_t>(now - s_last_count);
    s_last_count  = now;
    return delta;
}
