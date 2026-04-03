/* IpcChannel.h — Linux-side access to PRU Shared RAM.
 *
 * Opens and maps the PRU Shared RAM region via /dev/mem.
 * Provides:
 *   - sendCommand(): lifecycle control (enable, stop, home, flush, reset)
 *   - sendMove():    push one Klipper-style step move into a per-axis ring
 *   - readSpindleTelem() / readLateralTelem(): snapshot telemetry
 *   - moveQueueFreeSlots(): free slots in a move ring
 *
 * Thread safety: NOT thread-safe. Call only from the control loop thread.
 */

#pragma once
#include <cstdint>
#include "../../pru/include/pru_ipc.h"

class IpcChannel {
public:
    IpcChannel();
    ~IpcChannel();

    /** Open /dev/mem and map PRU Shared RAM. Returns false on failure. */
    bool begin(const char *devmem = "/dev/mem");
    void close();
    bool isOpen() const { return _mapped != nullptr; }

    /** Enqueue one command into the ring buffer.
     *  Returns false if ring is full (caller must drop + log). */
    bool sendCommand(uint8_t cmd, uint8_t axis,
                     uint32_t value_a = 0, uint32_t value_b = 0);

    /** Push one Klipper-style step move into a per-axis move ring.
     *  axis: AXIS_SPINDLE or AXIS_LATERAL.
     *  Returns false if ring is full. */
    bool sendMove(uint8_t axis, uint32_t interval, uint32_t count,
                  int32_t add, uint8_t direction);

    /** Free slots in a move ring. */
    uint32_t moveQueueFreeSlots(uint8_t axis) const;

    /** Copy latest spindle telemetry snapshot. Returns false if not open. */
    bool readSpindleTelem(pru_axis_telem_t &out) const;

    /** Copy latest lateral telemetry snapshot. Returns false if not open. */
    bool readLateralTelem(pru_axis_telem_t &out) const;

    /** Read inter-PRU sync area snapshot. */
    bool readSync(pru_sync_t &out) const;

    /** Reset step counters on both axes via command ring. */
    void resetPositions();

    /** TMC UART mailbox helpers (PRU <-> host). */
    bool pushTmcuartRequest(uint8_t type, uint8_t chip, uint8_t reg,
                            const uint8_t *data, uint8_t len, uint8_t flags = 0);
    bool popTmcuartResponse(pru_tmcuart_msg_t &out) const;

private:
    void   *_mapped       = nullptr;
    int     _fd           = -1;
    size_t  _map_size     = 0;

    volatile pru_cmd_t        *_cmdRing      = nullptr;
    volatile uint32_t         *_cmdWhead     = nullptr;
    volatile uint32_t         *_cmdRhead     = nullptr;
    volatile pru_axis_telem_t *_spTelem      = nullptr;
    volatile pru_axis_telem_t *_latTelem     = nullptr;
    volatile pru_sync_t       *_sync         = nullptr;

    /* Spindle move ring */
    volatile pru_move_t       *_spMoveRing   = nullptr;
    volatile uint32_t         *_spMoveWhead  = nullptr;
    volatile uint32_t         *_spMoveRhead  = nullptr;

    /* Lateral move ring */
    volatile pru_move_t       *_latMoveRing  = nullptr;
    volatile uint32_t         *_latMoveWhead = nullptr;
    volatile uint32_t         *_latMoveRhead = nullptr;

    /* TMC UART request/response rings */
    volatile pru_tmcuart_msg_t *_tmcReqRing   = nullptr;
    volatile uint32_t         *_tmcReqWhead  = nullptr;
    volatile uint32_t         *_tmcReqRhead  = nullptr;

    volatile pru_tmcuart_msg_t *_tmcRespRing  = nullptr;
    volatile uint32_t         *_tmcRespWhead = nullptr;
    volatile uint32_t         *_tmcRespRhead = nullptr;
};

