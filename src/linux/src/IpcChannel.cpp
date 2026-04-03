/* IpcChannel.cpp — Linux-side PRU Shared RAM access via /dev/mem. */

#include "IpcChannel.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <cstdio>
#include <cstring>

#define MAP_PAGE_SIZE   4096u
#define MAP_OFFSET_MASK (MAP_PAGE_SIZE - 1u)

IpcChannel::IpcChannel() = default;

IpcChannel::~IpcChannel() {
    close();
}

bool IpcChannel::begin(const char *devmem) {
    _fd = ::open(devmem, O_RDWR | O_SYNC);
    if (_fd < 0) { perror("IpcChannel: open /dev/mem"); return false; }

    const uint32_t phys_base = PRU_SRAM_PHYS_BASE & ~MAP_OFFSET_MASK;
    const uint32_t phys_off  = PRU_SRAM_PHYS_BASE &  MAP_OFFSET_MASK;
    _map_size = PRU_SRAM_SIZE + phys_off;

    _mapped = mmap(nullptr, _map_size,
                   PROT_READ | PROT_WRITE, MAP_SHARED, _fd, (off_t)phys_base);

    if (_mapped == MAP_FAILED) {
        perror("IpcChannel: mmap");
        ::close(_fd); _fd = -1; _mapped = nullptr;
        return false;
    }

    auto *base = static_cast<uint8_t *>(_mapped) + phys_off;

    _cmdRing  = reinterpret_cast<volatile pru_cmd_t *>       (base + IPC_CMD_RING_OFFSET);
    _cmdWhead = reinterpret_cast<volatile uint32_t *>        (base + IPC_CMD_WHEAD_OFFSET);
    _cmdRhead = reinterpret_cast<volatile uint32_t *>        (base + IPC_CMD_RHEAD_OFFSET);
    _spTelem  = reinterpret_cast<volatile pru_axis_telem_t *>(base + IPC_SPINDLE_TELEM_OFFSET);
    _latTelem = reinterpret_cast<volatile pru_axis_telem_t *>(base + IPC_LATERAL_TELEM_OFFSET);
    _sync     = reinterpret_cast<volatile pru_sync_t *>      (base + IPC_SYNC_OFFSET);

    _spMoveWhead  = reinterpret_cast<volatile uint32_t *>   (base + IPC_SP_MOVE_WHEAD_OFFSET);
    _spMoveRhead  = reinterpret_cast<volatile uint32_t *>   (base + IPC_SP_MOVE_RHEAD_OFFSET);
    _spMoveRing   = reinterpret_cast<volatile pru_move_t *> (base + IPC_SP_MOVE_RING_OFFSET);

    _latMoveWhead = reinterpret_cast<volatile uint32_t *>   (base + IPC_LAT_MOVE_WHEAD_OFFSET);
    _latMoveRhead = reinterpret_cast<volatile uint32_t *>   (base + IPC_LAT_MOVE_RHEAD_OFFSET);
    _latMoveRing  = reinterpret_cast<volatile pru_move_t *> (base + IPC_LAT_MOVE_RING_OFFSET);

    /* TMC UART rings */
    _tmcReqWhead  = reinterpret_cast<volatile uint32_t *>        (base + IPC_TMCUART_REQ_WHEAD_OFFSET);
    _tmcReqRhead  = reinterpret_cast<volatile uint32_t *>        (base + IPC_TMCUART_REQ_RHEAD_OFFSET);
    _tmcReqRing   = reinterpret_cast<volatile pru_tmcuart_msg_t*>(base + IPC_TMCUART_REQ_RING_OFFSET);

    _tmcRespWhead = reinterpret_cast<volatile uint32_t *>        (base + IPC_TMCUART_RESP_WHEAD_OFFSET);
    _tmcRespRhead = reinterpret_cast<volatile uint32_t *>        (base + IPC_TMCUART_RESP_RHEAD_OFFSET);
    _tmcRespRing  = reinterpret_cast<volatile pru_tmcuart_msg_t*>(base + IPC_TMCUART_RESP_RING_OFFSET);

    *_cmdWhead     = 0u;
    *_cmdRhead     = 0u;
    *_spMoveWhead  = 0u;
    *_spMoveRhead  = 0u;
    *_latMoveWhead = 0u;
    *_latMoveRhead = 0u;
    if (_tmcReqWhead)  *_tmcReqWhead  = 0u;
    if (_tmcReqRhead)  *_tmcReqRhead  = 0u;
    if (_tmcRespWhead) *_tmcRespWhead = 0u;
    if (_tmcRespRhead) *_tmcRespRhead = 0u;

    return true;
}

void IpcChannel::close() {
    if (_mapped && _mapped != MAP_FAILED) {
        munmap(_mapped, _map_size);
        _mapped = nullptr;
    }
    if (_fd >= 0) { ::close(_fd); _fd = -1; }
    _cmdRing = nullptr; _spTelem = nullptr; _latTelem = nullptr; _sync = nullptr;
    _spMoveRing = nullptr; _latMoveRing = nullptr;
    _tmcReqRing = nullptr; _tmcReqWhead = nullptr; _tmcReqRhead = nullptr;
    _tmcRespRing = nullptr; _tmcRespWhead = nullptr; _tmcRespRhead = nullptr;
}

bool IpcChannel::pushTmcuartRequest(uint8_t type, uint8_t chip, uint8_t reg,
                                    const uint8_t *data, uint8_t len, uint8_t flags) {
    if (!_mapped || !_tmcReqRing) return false;
    uint32_t wh = *_tmcReqWhead % IPC_TMCUART_REQ_SLOTS;
    uint32_t rh = *_tmcReqRhead % IPC_TMCUART_REQ_SLOTS;
    if (((wh + 1u) % IPC_TMCUART_REQ_SLOTS) == rh) return false; /* full */

    pru_tmcuart_msg_t m{};
    m.len = len; m.type = type; m.flags = flags; m.chip = chip; m.reg = reg;
    for (uint8_t i = 0; i < len && i < 8u; ++i) m.data[i] = data[i];
    _tmcReqRing[wh] = m;
    *_tmcReqWhead = (wh + 1u) % IPC_TMCUART_REQ_SLOTS;
    return true;
}

bool IpcChannel::popTmcuartResponse(pru_tmcuart_msg_t &out) const {
    if (!_mapped || !_tmcRespRing) return false;
    uint32_t wh = *_tmcRespWhead % IPC_TMCUART_RESP_SLOTS;
    uint32_t rh = *_tmcRespRhead % IPC_TMCUART_RESP_SLOTS;
    if (rh == wh) return false; /* empty */
    const volatile pru_tmcuart_msg_t *src = &_tmcRespRing[rh];
    out = *reinterpret_cast<const pru_tmcuart_msg_t *>(src);
    const_cast<volatile uint32_t *>(_tmcRespRhead)[0] = (rh + 1u) % IPC_TMCUART_RESP_SLOTS;
    return true;
}

bool IpcChannel::sendCommand(uint8_t cmd, uint8_t axis,
                              uint32_t value_a, uint32_t value_b) {
    if (!_mapped) return false;
    uint32_t wh = *_cmdWhead % IPC_CMD_RING_SLOTS;
    uint32_t rh = *_cmdRhead % IPC_CMD_RING_SLOTS;
    if (((wh + 1u) % IPC_CMD_RING_SLOTS) == rh) return false;

    pru_cmd_t c{};
    c.cmd = cmd; c.axis = axis; c.value_a = value_a; c.value_b = value_b;
    _cmdRing[wh] = c;
    *_cmdWhead   = (wh + 1u) % IPC_CMD_RING_SLOTS;
    return true;
}

bool IpcChannel::sendMove(uint8_t axis, uint32_t interval, uint32_t count,
                          int32_t add, uint8_t direction) {
    if (!_mapped) return false;

    volatile pru_move_t  *ring;
    volatile uint32_t    *whead;
    volatile uint32_t    *rhead;
    uint32_t slots;

    if (axis == AXIS_SPINDLE) {
        ring = _spMoveRing; whead = _spMoveWhead; rhead = _spMoveRhead;
        slots = IPC_SP_MOVE_SLOTS;
    } else if (axis == AXIS_LATERAL) {
        ring = _latMoveRing; whead = _latMoveWhead; rhead = _latMoveRhead;
        slots = IPC_LAT_MOVE_SLOTS;
    } else {
        return false;
    }

    uint32_t wh = *whead % slots;
    uint32_t rh = *rhead % slots;
    if (((wh + 1u) % slots) == rh) return false;   /* ring full */

    pru_move_t m{};
    m.interval  = interval;
    m.count     = count;
    m.add       = add;
    m.direction = direction;
    ring[wh] = m;
    *whead   = (wh + 1u) % slots;
    return true;
}

uint32_t IpcChannel::moveQueueFreeSlots(uint8_t axis) const {
    if (!_mapped) return 0u;
    const volatile uint32_t *whead, *rhead;
    uint32_t slots;
    if (axis == AXIS_SPINDLE) {
        whead = _spMoveWhead; rhead = _spMoveRhead; slots = IPC_SP_MOVE_SLOTS;
    } else if (axis == AXIS_LATERAL) {
        whead = _latMoveWhead; rhead = _latMoveRhead; slots = IPC_LAT_MOVE_SLOTS;
    } else { return 0u; }
    uint32_t used = (*whead % slots + slots - *rhead % slots) % slots;
    return slots - used - 1u;
}

bool IpcChannel::readSpindleTelem(pru_axis_telem_t &out) const {
    if (!_spTelem) return false;
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
    out.spindle_interval = _sync->spindle_interval;
    out.lateral_interval = _sync->lateral_interval;
    out.control_flags    = _sync->control_flags;
    return true;
}

void IpcChannel::resetPositions() {
    sendCommand(CMD_RESET_POSITION, AXIS_SPINDLE);
    sendCommand(CMD_RESET_POSITION, AXIS_LATERAL);
}
