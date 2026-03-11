#include "LinkSerial.h"
#include <Arduino.h>
#include "Diag.h"

void LinkSerial::begin() {
    // SERIAL_8N1 : 8 bits de données, pas de parité, 1 bit de stop.
    // Les pins TX/RX sont définis dans Config.h.
    LINK_UART.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    Diag::info("[Link] UART2 OK");
}

void LinkSerial::sendStatus(float    rpm,
                             uint32_t speedHz,
                             long     turns,
                             long     targetTurns,
                             bool     running,
                             bool     motorEnabled,
                             bool     freerun,
                             bool     dirCW) {
    // Format compact pipe-separated, terminé par '\n'.
    // L'ESP écran parse en splitant sur '|' et en vérifiant le préfixe 'S'.
    // Champs : S | turns | targetTurns | rpm | speedHz | running | motorEnabled | freerun | dirCW
    LINK_UART.printf("S|%ld|%ld|%.1f|%u|%d|%d|%d|%d\n",
                     turns, targetTurns, rpm, speedHz,
                     (int)running, (int)motorEnabled, (int)freerun, (int)dirCW);
}

void LinkSerial::poll(CommandCallback cb) {
    // Lire tous les octets disponibles sans bloquer.
    while (LINK_UART.available()) {
        char c = (char)LINK_UART.read();

        if (c == '\n') {
            // Ligne complète reçue — chercher le séparateur ':'
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
            // Protéger contre les buffers trop longs (ligne corrompue).
            if (_rxBuf.length() < 128) _rxBuf += c;
            else                        _rxBuf = "";  // reset silencieux
        }
    }
}
