/**
 * @file comm_interface.h
 * @brief SPI slave transport for host → ESP32 motion messages.
 *
 * The communication layer is intentionally separated from the stepper runtime:
 *
 * - `messages.h` defines all wire-level structures and result codes
 * - `CommInterface` owns the SPI slave task and DMA buffers
 * - handler methods translate validated messages into queue/driver operations
 *
 * This structure keeps the code maintainable when adding new commands:
 * define a new payload in `messages.h`, then add one handler here.
 */

#pragma once

#include <memory>
#include <esp_err.h>

#include "comm_types.h"
#include "comm_runtime.h"
#include "motion_planner.h"

class CommRequestDispatcher;
class CommStatusBuilder;
class SpiSlaveTransport;
class MultiAxisExecutor;

class CommInterface {
public:
    // Construct the SPI communication interface, wiring up per-axis stepper queues.
    CommInterface(StepperQueue* queues[], uint8_t n_motors);
    ~CommInterface();

    // Initialize SPI slave and start both the SPI task (Core 0) and the executor (Core 1).
    esp_err_t init(const SpiBusPins& pins);

private:
    SpiBusPins      pins_ {};              // Copied pin configuration
    MotionPlanner   planner_;              // Planner: converts blocks -> planned segments
    CommRuntime     runtime_;

    std::unique_ptr<CommRequestDispatcher> request_dispatcher_;
    std::unique_ptr<CommStatusBuilder> status_builder_;
    std::unique_ptr<SpiSlaveTransport> spi_transport_;
    std::unique_ptr<MultiAxisExecutor> multi_axis_executor_;

    esp_err_t configureHomePins() const;
    esp_err_t createRuntimeQueues();
    esp_err_t registerEndstopIsr() const;
};
