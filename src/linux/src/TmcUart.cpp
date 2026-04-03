/*
 * TmcUart.cpp — Preparatory implementation of TMC UART manager.
 *
 * Notes:
 * - This implementation provides the high-level flow used in Klipper-like
 *   drivers: a serial worker thread, a request queue, retries/timeouts,
 *   and a small register cache. It intentionally uses a simplified frame
 *   format as a placeholder — replace framing/CRC with the target TMC
 *   family's specification before using on hardware.
 */

#include "TmcUart.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <chrono>
#include <iostream>

namespace {
    // Simple XOR checksum over bytes (placeholder only)
    uint8_t checksum_xor(const uint8_t *b, size_t n) {
        uint8_t c = 0;
        for (size_t i = 0; i < n; ++i) c ^= b[i];
        return c;
    }

    // Map baud to termios speed_t (supports common rates)
    speed_t baud_to_termios(int baud) {
        switch (baud) {
        case 115200: return B115200;
        case 57600:  return B57600;
        case 38400:  return B38400;
        case 19200:  return B19200;
        case 9600:   return B9600;
        default:     return B115200;
        }
    }
}

TmcUart::TmcUart(const std::string &device, int baud)
    : _device(device), _baud(baud) {}

TmcUart::~TmcUart() { stop(); }

bool TmcUart::begin() {
    if (_ipc) {
        _running = true;
        _worker = std::thread(&TmcUart::workerLoop, this);
        return true;
    }
    if (!openSerial()) return false;
    _running = true;
    _worker = std::thread(&TmcUart::workerLoop, this);
    return true;
}

void TmcUart::stop() {
    _running = false;
    _queueCv.notify_all();
    if (_worker.joinable()) _worker.join();
    closeSerial();
}

bool TmcUart::isOpen() const { return _fd >= 0; }

bool TmcUart::writeRegister(uint8_t chip, uint8_t reg, uint32_t value) {
    Request r; r.type = Request::WRITE; r.chip = chip; r.reg = reg; r.value = value;
    std::lock_guard<std::mutex> lk(_queueMutex);
    // For blocking convenience we bypass queue and do direct IO
    bool ok = sendFrame(r);
    if (!ok) return false;
    uint8_t r_chip, r_reg; uint32_t r_val;
    if (!recvFrame(r_chip, r_reg, r_val, 200)) return false;
    return (r_chip == chip && r_reg == reg);
}

bool TmcUart::readRegister(uint8_t chip, uint8_t reg, uint32_t &out_value, int timeout_ms) {
    Request r; r.type = Request::READ; r.chip = chip; r.reg = reg; r.value = 0;
    // direct blocking request for preparation
    bool ok = sendFrame(r);
    if (!ok) return false;
    uint8_t r_chip, r_reg; uint32_t r_val;
    if (!recvFrame(r_chip, r_reg, r_val, timeout_ms)) return false;
    if (r_chip != chip || r_reg != reg) return false;
    out_value = r_val;
    {
        std::lock_guard<std::mutex> lk(_cacheMutex);
        _regCache[chip][reg] = r_val;
    }
    return true;
}

void TmcUart::enqueueWrite(uint8_t chip, uint8_t reg, uint32_t value) {
    Request r; r.type = Request::WRITE; r.chip = chip; r.reg = reg; r.value = value;
    {
        std::lock_guard<std::mutex> lk(_queueMutex);
        _queue.push_back(r);
    }
    _queueCv.notify_one();
}

void TmcUart::enqueueRead(uint8_t chip, uint8_t reg, ReadCallback cb) {
    Request r; r.type = Request::READ; r.chip = chip; r.reg = reg; r.callback = cb;
    {
        std::lock_guard<std::mutex> lk(_queueMutex);
        _queue.push_back(r);
    }
    _queueCv.notify_one();
}

bool TmcUart::openSerial() {
    if (_fd >= 0) return true;
    _fd = ::open(_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) return false;

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CLOCAL | CREAD | CS8;
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0; // raw
    cfsetispeed(&tio, baud_to_termios(_baud));
    cfsetospeed(&tio, baud_to_termios(_baud));
    tcsetattr(_fd, TCSANOW, &tio);
    return true;
}

void TmcUart::closeSerial() {
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

// Simplified placeholder frame format:
// [ADDR][TYPE][REG][B3][B2][B1][B0][CHK]
bool TmcUart::sendFrame(const Request &r) {
    uint8_t buf[8];
    buf[0] = r.chip;
    buf[1] = (r.type == Request::READ) ? 0x01 : 0x00;
    buf[2] = r.reg;
    buf[3] = (uint8_t)((r.value >> 24) & 0xFF);
    buf[4] = (uint8_t)((r.value >> 16) & 0xFF);
    buf[5] = (uint8_t)((r.value >> 8) & 0xFF);
    buf[6] = (uint8_t)((r.value) & 0xFF);
    buf[7] = checksum_xor(buf, 7);

    if (_ipc) {
        /* Send via PRU mailbox for bit-banged UART */
        uint8_t flags = _singleWire ? TMC_FLAG_SINGLE_WIRE : 0u;
        return _ipc->pushTmcuartRequest(buf[1], buf[0], buf[2], buf, sizeof(buf), flags);
    }

    if (_fd < 0) return false;
    ssize_t wrote = ::write(_fd, buf, sizeof(buf));
    if (wrote != (ssize_t)sizeof(buf)) return false;
    return true;
}

bool TmcUart::recvFrame(uint8_t &chip, uint8_t &reg, uint32_t &value, int timeout_ms) {
    if (_ipc) {
        auto start = std::chrono::steady_clock::now();
        pru_tmcuart_msg_t out;
        while (true) {
            if (_ipc->popTmcuartResponse(out)) {
                chip = out.chip;
                reg = out.reg;
                value = 0u;
                uint8_t blen = out.len;
                if (blen >= 4) {
                    value = ((uint32_t)out.data[0] << 24) | ((uint32_t)out.data[1] << 16)
                          | ((uint32_t)out.data[2] << 8) | (uint32_t)out.data[3];
                } else {
                    for (uint8_t i = 0; i < blen && i < 4; ++i) value = (value << 8) | out.data[i];
                }
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms)
                return false;
        }
    }

    if (_fd < 0) return false;
    auto start = std::chrono::steady_clock::now();
    uint8_t buf[8];
    size_t got = 0;

    while (got < sizeof(buf)) {
        ssize_t r = ::read(_fd, buf + got, sizeof(buf) - got);
        if (r > 0) { got += (size_t)r; continue; }
        if (r == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else return false;
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms)
            return false;
    }

    uint8_t chk = checksum_xor(buf, 7);
    if (chk != buf[7]) return false;

    chip = buf[0];
    // buf[1] = type
    reg = buf[2];
    value = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
            ((uint32_t)buf[5] << 8) | (uint32_t)buf[6];
    return true;
}

void TmcUart::workerLoop() {
    while (_running) {
        Request r;
        {
            std::unique_lock<std::mutex> lk(_queueMutex);
            _queueCv.wait_for(lk, std::chrono::milliseconds(50), [this]{ return !_queue.empty() || !_running; });
            if (!_running) break;
            if (_queue.empty()) continue;
            r = _queue.front(); _queue.pop_front();
        }

        // Best-effort processing: send and optionally read response
        if (!sendFrame(r)) {
            // retry logic could be added here; for preparation we log
            continue;
        }

        if (r.type == Request::READ) {
            uint8_t chip; uint8_t reg; uint32_t val;
            if (recvFrame(chip, reg, val, 200)) {
                {
                    std::lock_guard<std::mutex> lk(_cacheMutex);
                    _regCache[chip][reg] = val;
                }
                if (r.callback) r.callback(true, val);
            } else {
                if (r.callback) r.callback(false, 0u);
            }
        }
    }
}
