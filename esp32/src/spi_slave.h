/* spi_slave.h — Core 0 SPI slave driver.
 *
 * Interrupt-driven SPI slave using ESP-IDF spi_slave driver.
 * Receives CmdFrame (8 bytes) from RPi master, sends StatusFrame (32 bytes).
 *
 * The SPI transaction is full-duplex: while the master clocks in a command,
 * the slave simultaneously clocks out the last status frame.
 *
 * Flow:
 *   1. Master sends 8-byte CmdFrame + 24 bytes padding (total 32 bytes)
 *   2. Slave sends 32-byte StatusFrame simultaneously
 *   3. On transaction complete, the CmdFrame is validated (CRC) and pushed
 *      to the CmdQueue for Core 1.
 *   4. The StatusFrame is refreshed from StepperEngine.
 */

#pragma once

#include "protocol.h"
#include "command_queue.h"

// ── Pin assignments (match RPi SPI0 wiring) ─────────────────────────────────
// Adjustable via spi_slave_init() parameters.
struct SpiPins {
    int8_t mosi  = 23;   // MOSI (Master Out Slave In)
    int8_t miso  = 19;   // MISO (Master In Slave Out)
    int8_t sclk  = 18;   // SPI Clock
    int8_t cs    = 5;    // Chip Select
};

/// Initialize SPI slave on VSPI (SPI3) peripheral.
/// Must be called from setup() before starting the SPI task.
void spi_slave_init(const SpiPins& pins);

/// Start the SPI slave task on Core 0.
/// This task blocks waiting for SPI transactions, processes CmdFrames,
/// and pushes them to the CmdQueue.
void spi_slave_start(CmdQueue& cmd_queue);

/// Get SPI statistics (for debugging).
struct SpiStats {
    uint32_t rx_frames;       // total received frames
    uint32_t rx_crc_errors;   // CRC validation failures
    uint32_t rx_queue_full;   // frames dropped (queue full)
    uint32_t tx_frames;       // total sent status frames
};

SpiStats spi_get_stats();
