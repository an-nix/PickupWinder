/* IpcChannel.cpp — Linux-side PRU Shared RAM access via /dev/mem. */

#include "IpcChannel.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <cstdio>
#include <cstring>

/* Page-align the mapping: /dev/mem requires page-aligned offsets */
#define MAP_PAGE_SIZE   4096u
#define MAP_OFFSET_MASK (MAP_PAGE_SIZE - 1u)

IpcChannel::IpcChannel() = default;

IpcChannel::~IpcChannel() {
    close();
}

bool IpcChannel::begin(const char *devmem) {
    _fd = ::open(devmem, O_RDWR | O_SYNC);
    if (_fd < 0) {
        perror("IpcChannel: open /dev/mem");
        return false;
    }

    /* Map a full page aligned region covering PRU_SRAM_SIZE */
    const uint32_t phys_base  = PRU_SRAM_PHYS_BASE & ~MAP_OFFSET_MASK;
    const uint32_t phys_off   = PRU_SRAM_PHYS_BASE &  MAP_OFFSET_MASK;
    _map_size = PRU_SRAM_SIZE + phys_off;

    _mapped = mmap(nullptr, _map_size,
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED, _fd,
                   (off_t)phys_base);

    if (_mapped == MAP_FAILED) {
        perror("IpcChannel: mmap");
        ::close(_fd);
        _fd     = -1;
        _mapped = nullptr;
        return false;
    }

    /* Compute volatile pointers into mapped region */
    auto base = static_cast<uint8_t *>(_mapped) + phys_off;

    _cmdRing  = reinterpret_cast<volatile pru_cmd_t *>
                    (base + IPC_CMD_RING_OFFSET);
    _cmdWhead = reinterpret_cast<volatile uint32_t *>
                    (base + IPC_CMD_WHEAD_OFFSET);
    _cmdRhead = reinterpret_cast<volatile uint32_t *>
                    (base + IPC_CMD_RHEAD_OFFSET);
    _spTelem  = reinterpret_cast<volatile pru_axis_telem_t *>
                    (base + IPC_SPINDLE_TELEM_OFFSET);
    _latTelem = reinterpret_cast<volatile pru_axis_telem_t *>
                    (base + IPC_LATERAL_TELEM_OFFSET);
    _sync     = reinterpret_cast<volatile pru_sync_t *>
                    (base + IPC_SYNC_OFFSET);

    /* Init ring indices to 0 */
    *_cmdWhead = 0u;
    *_cmdRhead = 0u;

    return true;
}

void IpcChannel::close() {
    if (_mapped && _mapped != MAP_FAILED) {
        munmap(_mapped, _map_size);
        _mapped = nullptr;
    }
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
    _cmdRing = nullptr;
    _spTelem = nullptr;
    _latTelem= nullptr;
    _sync    = nullptr;
}

bool IpcChannel::sendCommand(uint8_t cmd, uint8_t axis,
                              uint32_t value_a, uint32_t value_b) {
    if (!_mapped) return false;

    uint32_t wh = *_cmdWhead % IPC_CMD_RING_SLOTS;
    uint32_t rh = *_cmdRhead % IPC_CMD_RING_SLOTS;

    /* Ring full check */
    if (((wh + 1u) % IPC_CMD_RING_SLOTS) == rh)
        return false;   /* caller must drop + log FAULT_CMD_OVERFLOW */

    pru_cmd_t c;
    c.cmd     = cmd;
    c.axis    = axis;
    c.flags   = 0u;
    c._pad0   = 0u;
    c.value_a = value_a;
    c.value_b = value_b;
    c._reserved = 0u;

    /* Write struct then advance head — volatile ensures ordering */
    _cmdRing[wh] = c;
    *_cmdWhead   = (wh + 1u) % IPC_CMD_RING_SLOTS;

    return true;
}

bool IpcChannel::readSpindleTelem(pru_axis_telem_t &out) const {
    if (!_spTelem) return false;
    /* Snapshot — no lock needed; PRU writes atomically per field */
    out = *_spTelem;
    return true;
}

bool IpcChannel::readLateralTelem(pru_axis_telem_t &out) const {
    if (!_latTelem) return false;
    out = *_latTelem;
    return true;
}

bool IpcChannel::readSync(pru_sync_t &out) const {
    if (!_sync) return false;
    out.spindle_hz      = _sync->spindle_hz;
    out.lateral_hz      = _sync->lateral_hz;
    out.nominal_lat_hz  = _sync->nominal_lat_hz;
    out.control_flags   = _sync->control_flags;
    return true;
}

void IpcChannel::setCompensationEnabled(bool en) {
    if (!_sync) return;
    if (en)
        _sync->control_flags |=  1u;
    else
        _sync->control_flags &= ~1u;
}

void IpcChannel::setNominalLateralHz(uint32_t hz) {
    if (!_sync) return;
    _sync->nominal_lat_hz = hz;
}

void IpcChannel::resetPositions() {
    sendCommand(CMD_RESET_POSITION, AXIS_SPINDLE, 0u, 0u);
    sendCommand(CMD_RESET_POSITION, AXIS_LATERAL, 0u, 0u);
}
