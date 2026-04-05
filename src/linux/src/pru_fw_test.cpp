/* pru_fw_test.cpp — Real firmware spindle test app (Linux userspace).
 *
 * Uses the real IPC path:
 *   /dev/mem -> IpcChannel -> command/move rings -> am335x-pru1-fw
 *
 * This is NOT a synthetic PRU test firmware. It validates the production
 * PRU motion path by sending real move segments to AXIS_SPINDLE.
 *
 * Usage (on BBB, as root):
 *   ./pru_fw_test
 *   ./pru_fw_test --rpm 20 --seconds 8 --forward 1
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <thread>
#include <chrono>

#include "../include/IpcChannel.h"
#include "../include/StepperPRU.h"

static constexpr uint32_t STEPS_PER_REV = 6400u;

static void print_usage(const char *prog) {
    std::printf("Usage: %s [--rpm N] [--seconds N] [--forward 0|1]\n", prog);
    std::printf("Defaults: --rpm 20 --seconds 10 --forward 1\n");
}

static bool parse_u32(const char *s, uint32_t &out) {
    if (!s || !*s) return false;
    char *end = nullptr;
    unsigned long v = std::strtoul(s, &end, 10);
    if (!end || *end != '\0') return false;
    out = static_cast<uint32_t>(v);
    return true;
}

int main(int argc, char **argv) {
    uint32_t rpm = 20u;
    uint32_t seconds = 10u;
    bool forward = true;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--rpm") == 0 && (i + 1) < argc) {
            uint32_t v = 0;
            if (!parse_u32(argv[++i], v)) { print_usage(argv[0]); return 2; }
            rpm = v;
        } else if (std::strcmp(argv[i], "--seconds") == 0 && (i + 1) < argc) {
            uint32_t v = 0;
            if (!parse_u32(argv[++i], v)) { print_usage(argv[0]); return 2; }
            seconds = v;
        } else if (std::strcmp(argv[i], "--forward") == 0 && (i + 1) < argc) {
            uint32_t v = 0;
            if (!parse_u32(argv[++i], v)) { print_usage(argv[0]); return 2; }
            forward = (v != 0u);
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            print_usage(argv[0]);
            return 2;
        }
    }

    if (rpm == 0u) rpm = 1u;
    if (seconds == 0u) seconds = 1u;

    /* Convert RPM -> step Hz for StepperPRU::setSpeedHz() */
    uint64_t hz64 = (static_cast<uint64_t>(rpm) * STEPS_PER_REV) / 60u;
    uint32_t hz = static_cast<uint32_t>(hz64);

    std::printf("[pru_fw_test] rpm=%u  seconds=%u  forward=%u  (hz=%u)\n",
                rpm, seconds, forward ? 1u : 0u, hz);

    IpcChannel channel;
    if (!channel.begin("/dev/mem")) {
        std::fprintf(stderr, "[pru_fw_test] Failed to open IPC (/dev/mem). Run as root?\n");
        return 1;
    }

    StepperPRU stepper(channel);
    stepper.begin(1000u);

    /* Clean start: spindle queue empty + disabled, reset counters. */
    channel.sendCommand(CMD_QUEUE_FLUSH, AXIS_SPINDLE);
    channel.sendCommand(CMD_ENABLE, AXIS_SPINDLE, 0u);
    channel.sendCommand(CMD_RESET_POSITION, AXIS_SPINDLE);

    stepper.setSpeedHz(hz);
    stepper.start(forward);

    auto t0 = std::chrono::steady_clock::now();
    auto nextPrint = t0;

    while (true) {
        stepper.tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        auto now = std::chrono::steady_clock::now();
        if (now >= nextPrint) {
            float rpmNow = stepper.getRPM();
            long turns = stepper.getTurns();
            std::printf("[run] rpm_now=%.1f turns=%ld running=%u\n",
                        rpmNow, turns, stepper.isRunning() ? 1u : 0u);
            std::fflush(stdout);
            nextPrint = now + std::chrono::milliseconds(500);
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - t0).count();
        if (elapsed >= static_cast<long long>(seconds)) break;
    }

    std::printf("[pru_fw_test] Request stop...\n");
    stepper.stop();

    /* Keep ticking while decelerating */
    auto stopStart = std::chrono::steady_clock::now();
    while (true) {
        stepper.tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        if (!stepper.isRunning()) break;

        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - stopStart).count();
        if (ms > 5000) {
            std::fprintf(stderr, "[pru_fw_test] Stop timeout, forcing stop.\n");
            stepper.forceStop();
            break;
        }
    }

    stepper.disableDriver();

    pru_axis_telem_t telem{};
    if (channel.readSpindleTelem(telem)) {
        std::printf("[done] step_count=%u interval=%u state=0x%04x faults=0x%04x\n",
                    telem.step_count, telem.current_interval, telem.state, telem.faults);
    }

    return 0;
}
