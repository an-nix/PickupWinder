#pragma once
#include "Types.h"
#include "Config.h"

// UART2 link between the winding controller ESP32 and the display ESP32.
//
// Protocol:
//   TX (winder -> display): compact status line every LINK_UPDATE_MS.
//     Format: "S|<turns>|<targetTurns>|<rpm>|<speedHz>|<running>|<motorEnabled>|<freerun>|<dirCW>\n"
//
//   RX (display -> winder): commands formatted as "cmd:value\n".
//     The payload is intentionally aligned with the WebSocket command format.
//     Examples: "target:8000\n", "freerun:true\n", "direction:cw\n",
//               "geom_wire:0.0650\n", "reset:\n"
class LinkSerial {
public:
    /**
     * @brief Initialize UART2 link with display controller.
     * @par Usage
     * Called once at startup before any status exchange.
     */
    void begin();

    /**
     * @brief Send one compact status line to the screen ESP.
     *
     * @param rpm Current spindle RPM.
     * @param speedHz Current spindle speed in Hz.
     * @param turns Current turns count.
     * @param targetTurns Current target turns.
     * @param running true if spindle stepper is running.
     * @param motorEnabled true if motor command path is enabled.
     * @param freerun true if freerun mode is active.
     * @param dirCW true for clockwise winding direction.
     */
    void sendStatus(float    rpm,
                    uint32_t speedHz,
                    long     turns,
                    long     targetTurns,
                    bool     running,
                    bool     motorEnabled,
                    bool     freerun,
                    bool     dirCW);

    /**
     * @brief Read UART input and dispatch complete commands.
     *
     * @param cb Callback invoked for each parsed command line.
     * @par Usage
     * Called in each main-loop iteration.
     */
    void poll(CommandCallback cb);

    /**
     * @brief Store a command callback to be used by `poll()` without args.
     */
    void setCommandCallback(CommandCallback cb);

    /**
     * @brief Poll UART and dispatch using previously registered callback.
     *
     * This overload allows calling `poll()` without passing the callback each time.
     */
    void poll();

private:
    String _rxBuf;  // Receive buffer for the current in-flight line.
    CommandCallback _callback = nullptr;
};
