#pragma once
#include "Types.h"
#include "Config.h"

// ── LinkSerial ────────────────────────────────────────────────────────────────
// Liaison UART2 entre le winder ESP32 et l'ESP32 écran.
//
// Protocole :
//   TX (winder → écran)  : ligne de statut compacte toutes les LINK_UPDATE_MS ms
//     Format : "S|<turns>|<targetTurns>|<rpm>|<speedHz>|<running>|<motorEnabled>|<freerun>|<dirCW>\n"
//
//   RX (écran → winder)  : commandes au format "cmd:valeur\n"
//     Identique au protocole WebSocket — réutilise directement _handleCommand().
//     Exemples : "target:8000\n", "freerun:true\n", "direction:cw\n",
//                "geom_wire:0.0650\n", "reset:\n"
class LinkSerial {
public:
    // Initialise UART2 sur LINK_TX_PIN / LINK_RX_PIN à LINK_BAUD bauds.
    void begin();

    // Envoyer une ligne de statut vers l'ESP écran.
    // Appelé périodiquement depuis WinderApp::run() (toutes les LINK_UPDATE_MS ms).
    void sendStatus(float    rpm,
                    uint32_t speedHz,
                    long     turns,
                    long     targetTurns,
                    bool     running,
                    bool     motorEnabled,
                    bool     freerun,
                    bool     dirCW);

    // Lire les octets disponibles sur UART2 et dispatcher les commandes complètes
    // vers le callback (même signature que WebSocket → _handleCommand réutilisé tel quel).
    // Appelé à chaque itération de WinderApp::run().
    void poll(CommandCallback cb);

private:
    String _rxBuf;  // Buffer d'accumulation pour la ligne en cours de réception
};
