/**
 * @file comm_interface.h
 * @brief Communication subsystem façade.
 */

#pragma once

#include <memory>
#include <esp_err.h>

#include "comm_types.h"
#include "comm_runtime.h"
#include "../motion/motion_planner.h"

class CommRequestDispatcher;
class CommStatusBuilder;
class SpiSlaveTransport;
class MultiAxisExecutor;

/**
 * @brief High-level communication orchestrator.
 *
 * This façade wires and initializes the communication components:
 * - runtime context
 * - request dispatcher
 * - status builder
 * - SPI transport task
 * - multi-axis executor task
 */
class CommInterface {
public:
    /** @brief Construct the communication façade for the configured motor set. */
    CommInterface(StepperQueue* queues[], uint8_t n_motors);
    ~CommInterface();

    /**
     * @brief Initialize queues, planner, executor and SPI transport.
     */
    esp_err_t init(const SpiBusPins& pins);

private:
    SpiBusPins pins_ {};
    MotionPlanner planner_;
    CommRuntime runtime_;

    std::unique_ptr<CommRequestDispatcher> request_dispatcher_;
    std::unique_ptr<CommStatusBuilder> status_builder_;
    std::unique_ptr<SpiSlaveTransport> spi_transport_;
    std::unique_ptr<MultiAxisExecutor> multi_axis_executor_;

    esp_err_t configureHomePins() const;
    esp_err_t createRuntimeQueues();
    esp_err_t registerEndstopIsr() const;
};
