/* IpcChannel.h — Linux-side access to PRU Shared RAM.
 *
 * Opens and maps the PRU Shared RAM region using /dev/mem.
 * Provides atomic ring-buffer command sending and telemetry reading.
 *
 * Thread safety: sendCommand() is NOT thread-safe. Call only from the
 * control loop thread (≤ 10 ms tick).
 */

#pragma once
#include <cstdint>
#include <cstring>
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
     *  Returns false if the ring buffer is full (caller must drop + log). */
    bool sendCommand(uint8_t cmd, uint8_t axis,
                     uint32_t value_a = 0, uint32_t value_b = 0);

    /** Copy latest spindle telemetry snapshot. Returns false if not open. */
    bool readSpindleTelem(pru_axis_telem_t &out) const;

    /** Copy latest lateral telemetry snapshot. Returns false if not open. */
    bool readLateralTelem(pru_axis_telem_t &out) const;

    /** Read inter-PRU sync area. */
    bool readSync(pru_sync_t &out) const;

    /** Set compensation enable flag in sync area. */
    void setCompensationEnabled(bool en);

    /** Write nominal lateral Hz for compensation numerics. */
    void setNominalLateralHz(uint32_t hz);

    /** Reset PRU step counters on both axes. */
    void resetPositions();

private:
    void   *_mapped          = nullptr;
    int     _fd              = -1;
    size_t  _map_size        = 0;

    volatile pru_cmd_t        *_cmdRing   = nullptr;
    volatile uint32_t         *_cmdWhead  = nullptr;
    volatile uint32_t         *_cmdRhead  = nullptr;
    volatile pru_axis_telem_t *_spTelem   = nullptr;
    volatile pru_axis_telem_t *_latTelem  = nullptr;
    volatile pru_sync_t       *_sync      = nullptr;
};
