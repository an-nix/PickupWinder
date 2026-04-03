/*
 * tmc_uart_example.cpp — small example of using TmcUart (preparatory)
 *
 * This example demonstrates the synchronous convenience APIs. The framing
 * used by TmcUart is a placeholder and must be adapted to the target
 * TMC family before running on hardware.
 */

#include "TmcUart.h"
#include "IpcChannel.h"
#include <iostream>

int main() {
    IpcChannel ipc;
    TmcUart tmc("/dev/ttyS1", 115200);

    if (ipc.begin("/dev/mem")) {
        std::cerr << "Using PRU mailbox for TMC UART via /dev/mem\n";
        tmc.setIpcChannel(&ipc);
        tmc.setSingleWire(true);
    } else {
        std::cerr << "Falling back to native serial device\n";
    }

    if (!tmc.begin()) {
        std::cerr << "TmcUart: failed to start\n";
        return 1;
    }

    // Example: read register 0x00 from chip 0
    uint32_t val = 0;
    if (tmc.readRegister(0, 0x00, val)) {
        std::cout << "Read reg 0x00 = 0x" << std::hex << val << std::dec << "\n";
    } else {
        std::cout << "Read failed (placeholder implementation)\n";
    }

    tmc.stop();
    ipc.close();
    return 0;
}
