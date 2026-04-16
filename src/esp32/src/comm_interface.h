/**
 * @file comm_interface.h
 * @brief Host communication interface — UART stub (Klipper-ready).
 *
 * In production this module receives comm_packet_t frames serialised by the
 * Linux host over UART (or SPI), deserialises them, routes each block to the
 * appropriate StepperQueue, and sends flow-control ACKs back to the host.
 *
 * This is currently a STUB implementation:
 *   • The UART receive path is present but does nothing useful until a real
 *     frame parser is wired in.
 *   • The ACK path logs to the console instead of writing to UART TX.
 *
 * ── Packet framing ─────────────────────────────────────────────────────────
 *   Each packet on the wire is:
 *     [SYNC 0xAA] [LENGTH u16-LE] [comm_packet_t payload] [CRC8]
 *
 *   The stub skips the framing layer.  Wire a real byte-stream parser here
 *   when integrating with the Linux host.
 *
 * ── Flow-control ACK ───────────────────────────────────────────────────────
 *   The ACK frame sent to the host is:
 *     [0xAC] [motor_id u8] [free_slots u8]
 *
 *   The host throttles injection when free_slots < FLOW_CONTROL_THRESHOLD.
 */

#pragma once

#include <esp_err.h>
#include "step_types.h"
#include "stepper_queue.h"

// Maximum number of motors supported by this interface
#define COMM_MAX_MOTORS 2

class CommInterface {
public:
    /**
     * @brief Construct the CommInterface.
     *
     * @param queues   Array of StepperQueue pointers, indexed by motor_id.
     * @param n_motors Number of entries in @p queues (≤ COMM_MAX_MOTORS).
     */
    CommInterface(StepperQueue* queues[], uint8_t n_motors);

    /**
     * @brief Initialise the UART peripheral and launch the RX task.
     *
     * @param uart_num  UART port number (e.g., UART_NUM_1).
     * @param tx_gpio   GPIO for UART TX (to host).
     * @param rx_gpio   GPIO for UART RX (from host).
     * @param baud_rate Baud rate (e.g., 921600).
     */
    esp_err_t init(int uart_num, int tx_gpio, int rx_gpio, int baud_rate);

    /**
     * @brief Inject a comm_packet_t directly (bypasses UART — for testing).
     *
     * Routes the packet to the appropriate queue and sends a flow-control ACK
     * if the free-slot count drops below FLOW_CONTROL_THRESHOLD.
     *
     * @return ESP_OK, ESP_ERR_INVALID_ARG (bad motor_id), or ESP_ERR_TIMEOUT
     *         (target queue full).
     */
    esp_err_t injectPacket(const comm_packet_t& pkt);

private:
    StepperQueue* queues_[COMM_MAX_MOTORS];
    uint8_t       n_motors_;
    int           uart_num_ {-1};

    /** Send a flow-control ACK to the host for the given motor. */
    void sendFlowAck(uint8_t motor_id, uint32_t free_slots);

    /** UART RX task — listens for incoming comm_packet_t frames. */
    static void rxTask(void* arg);
};
