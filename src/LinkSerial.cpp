#include "LinkSerial.h"
#include <Arduino.h>
#include "Diag.h"
#include "Protocol.h"

/**
 * @brief Initialize UART bridge to the display ESP.
 */
void LinkSerial::begin() {
    // SERIAL_8N1 means 8 data bits, no parity and 1 stop bit.
    // TX/RX pins are configured in Config.h.
    LINK_UART.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    LINK_UART.printf("P|%s\n", PICKUP_UART_PROTOCOL_VERSION);
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
static void trimChars(char* s) {
    if (!s) return;
    // Trim leading spaces
    char* p = s;
    while (*p != '\0' && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
    if (p != s) memmove(s, p, strlen(p) + 1);
    // Trim trailing spaces
    size_t len = strlen(s);
    while (len > 0 && (s[len-1] == ' ' || s[len-1] == '\t' || s[len-1] == '\r' || s[len-1] == '\n')) {
        s[--len] = '\0';
    }
}

void LinkSerial::poll(CommandCallback cb) {
    // Consume all available bytes without blocking the control flow.
    while (LINK_UART.available()) {
        char c = (char)LINK_UART.read();

        if (c == '\n') {
            if (_rxLen == 0) continue;
            _rxBuf[_rxLen] = '\0';
            char* sep = strchr(_rxBuf, ':');
            if (sep && cb) {
                *sep = '\0';
                char* val = sep + 1;
                trimChars(_rxBuf);
                trimChars(val);
                cb(_rxBuf, val);
            }
            _rxLen = 0;
        } else if (c != '\r') {
            if (_rxLen < RX_BUF_SIZE - 1) {
                _rxBuf[_rxLen++] = c;
            } else {
                _rxLen = 0;
                Diag::warn("[UART] RX buffer overflow — line dropped");
            }
        }
    }
}

void LinkSerial::setCommandCallback(CommandCallback cb) {
    _callback = cb;
}

void LinkSerial::poll() {
    poll(_callback);
}
