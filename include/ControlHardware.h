#pragma once
#include <Arduino.h>
#include "Config.h"
#include "SessionController.h"

class WinderApp;

/**
 * @brief Hardware input aggregator for the control task.
 *
 * This class reads raw physical inputs, performs lightweight filtering or
 * debounce where required, and populates `SessionController::TickInput`.
 */
class ControlHardware {
public:
    /**
     * @brief Construct the hardware input helper.
     */
    ControlHardware();

    /** @brief Initialize potentiometer, footswitch and encoder IO. */
    void begin();

    /**
     * @brief Read hardware inputs and append fresh values to one tick packet.
     * @param now Current `millis()` timestamp.
     * @param out Tick packet populated in place.
     */
    void tick(uint32_t now, SessionController::TickInput& out);



    // --- Inlined SpeedInput logic ---
    /**
     * @brief Potentiometer smoothing buffer and transfer logic (was SpeedInput).
     */
    uint32_t _potSamples[POT_FILTER_SIZE] = {};
    uint8_t  _potIdx = 0;
    uint32_t _potLastHz = 0;

    /**
     * @brief Initialize ADC smoothing buffer for the potentiometer.
     * Pre-fills the moving-average window with current ADC value to avoid startup transients.
     */
    void _initPotSmoothing();

    /**
     * @brief Read and filter potentiometer input, returning filtered Hz value.
     * Reads one sample, updates moving average, then maps result to [SPEED_HZ_MIN, SPEED_HZ_MAX].
     */
    uint32_t _readPotHz();

    // Debounce state for the digital footswitch input.
    bool _lastFootswitchRaw = false;
    bool _footswitchStable = false;
    uint32_t _lastFootswitchChangeMs = 0;

    // Shared quadrature decoder state updated from the encoder ISR.
    static volatile int32_t _encCount;
    static volatile uint8_t  _encLastAB;
    static volatile uint32_t _encLastUs;
    static const int8_t QEM[16];

    /** @brief Quadrature encoder ISR used to accumulate detents. */
    static void IRAM_ATTR _encISR();

    // Per-task caches and rate-limiters.
    uint32_t _lastEncMs = 0;
    int32_t  _lastPrinted = 1;
    int32_t  _lastEncConsumed = 0;
    uint32_t _lastPotMs = 0;
    uint32_t _lastPotHz = 0;
};
