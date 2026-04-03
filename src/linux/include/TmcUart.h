/*
 * TmcUart.h — Preparatory TMC UART manager (Klipper-inspired logic)
 *
 * Purpose: provide a preparatory implementation of the control logic used
 * by Klipper to drive Trinamic (TMC) drivers over a UART link. This is a
 * lightweight, self-contained helper intended for preparation and design
 * work — adapt the low-level frame format and error handling to the exact
 * TMC family (2130/2209/5160...) before production use.
 */

#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <deque>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "IpcChannel.h"

class TmcUart {
public:
    using ReadCallback = std::function<void(bool success, uint32_t value)>;

    explicit TmcUart(const std::string &device = "/dev/ttyS1", int baud = 115200);
    ~TmcUart();

    /** Use PRU-side tmcuart mailbox via `IpcChannel`. If set, requests
     *  are sent to PRU for bit-banged UART; otherwise uses native serial. */
    void setIpcChannel(IpcChannel *ipc) { _ipc = ipc; }

    // Open serial device and start worker thread
    bool begin();
    void stop();
    
        /** Configure PRU-backed single-wire mode when using IpcChannel. */
        void setSingleWire(bool enable) { _singleWire = enable; }

    // Blocking access (synchronous) — convenience for preparation
    bool writeRegister(uint8_t chip, uint8_t reg, uint32_t value);
    bool readRegister(uint8_t chip, uint8_t reg, uint32_t &out_value, int timeout_ms = 200);

    // Non-blocking queue: background worker will process and call callback
    void enqueueWrite(uint8_t chip, uint8_t reg, uint32_t value);
    void enqueueRead(uint8_t chip, uint8_t reg, ReadCallback cb);

    bool isOpen() const;

private:
    struct Request {
        enum Type { READ, WRITE } type;
        uint8_t chip;
        uint8_t reg;
        uint32_t value;           // write payload or placeholder for read
        ReadCallback callback;    // for async read
        int retries = 0;
    };

    // Low-level helpers (implement simplified framing here; adapt later)
    bool openSerial();
    void closeSerial();
    bool sendFrame(const Request &r);
    bool recvFrame(uint8_t &chip, uint8_t &reg, uint32_t &value, int timeout_ms);
    void workerLoop();

    std::string _device;
    int _baud;
    int _fd = -1; // POSIX file descriptor for serial (Linux)
    IpcChannel *_ipc = nullptr;

    // Worker queue + synchronisation
    std::deque<Request> _queue;
    std::mutex _queueMutex;
    std::condition_variable _queueCv;
    std::thread _worker;
    std::atomic<bool> _running{false};

    // Simple cache of recently-read registers per chip (preparatory)
    std::map<uint8_t, std::map<uint8_t, uint32_t>> _regCache;
    std::mutex _cacheMutex;
        bool _singleWire = false;
};
