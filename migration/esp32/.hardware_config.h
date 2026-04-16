// hardware_config.h — Compile-time hardware pin configuration
#pragma once

#include "stepper_engine.h"   // AxisPins, RMT_NUM_AXES (FastAccelStepper wrapper)
#include "spi_slave.h"     // SpiPins

// ── Pin configuration ─────────────────────────────────────────────────────────

inline constexpr AxisPins AXIS_PINS[RMT_NUM_AXES] = {
    // Axis 0 — Bobbin rotation
    { .step = 26, .dir = 27, .enable = 14, .endstop_no = -1, .endstop_nc = -1 },
    // Axis 1 — Lateral carriage (2-contact home sensor)
    { .step = 32, .dir = 33, .enable = 25, .endstop_no = 21, .endstop_nc = 22 },
    // Axis 2 — Wire tensioner (no dedicated endstop)
    { .step = 16, .dir = 17, .enable =  4, .endstop_no = -1, .endstop_nc = -1 },
};

inline constexpr SpiPins SPI_PINS = {
    .mosi = 23, .miso = 19, .sclk = 18, .cs = 5
};