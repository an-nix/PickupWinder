#include "LinkSerial.h"
#include <Arduino.h>
#include "Diag.h"

/**
 * @brief Initialize UART bridge to the display ESP.
 */
void LinkSerial::begin() {
    // SERIAL_8N1 means 8 data bits, no parity and 1 stop bit.
    // TX/RX pins are configured in Config.h.
    LINK_UART.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    Diag::info("[Link] UART2 OK");
}

/**
 * @brief Send one compact status frame over UART.
 */
void LinkSerial::sendStatus(float    rpm,
                             uint32_t speedHz,
                             long     turns,
                             long     targetTurns,
                             bool     running,
                             bool     motorEnabled,
                             bool     freerun,
                             bool     dirCW) {
    // Compact pipe-separated frame terminated by '\n'.
    // The display ESP parses the frame by splitting on '|' after checking the
    // leading 'S' marker.
    LINK_UART.printf("S|%ld|%ld|%.1f|%u|%d|%d|%d|%d\n",
                     turns, targetTurns, rpm, speedHz,
                     (int)running, (int)motorEnabled, (int)freerun, (int)dirCW);
}

/**
 * @brief Poll UART and dispatch full command lines.
 * @param cb Callback receiving parsed command/value pairs.
 */
void LinkSerial::poll(CommandCallback cb) {
    // Consume all available bytes without blocking the control flow.
    while (LINK_UART.available()) {
        char c = (char)LINK_UART.read();

        if (c == '\n') {
            // A full line was received; split it around the ':' separator.
            int sep = _rxBuf.indexOf(':');
            if (sep > 0 && cb) {
                String cmd = _rxBuf.substring(0, sep);
                String val = _rxBuf.substring(sep + 1);
                cmd.trim();
                val.trim();
                cb(cmd, val);
            }
            _rxBuf = "";
        } else if (c != '\r') {
            // Protect against corrupted or runaway lines.
            if (_rxBuf.length() < 128) _rxBuf += c;
            else                        _rxBuf = "";  // Silent reset on overflow.
        }
    }
}

void LinkSerial::setCommandCallback(CommandCallback cb) {
    _callback = cb;
}

void LinkSerial::poll() {
    poll(_callback);
}
