#pragma once
#include <Arduino.h>
#include <functional>

// Callback invoqué quand une commande est reçue (WebSocket ou liaison série).
// Paramètres : cmd (nom de la commande), val (valeur optionnelle).
// Ce type est partagé par WebInterface et LinkSerial pour que WinderApp
// puisse passer le même _handleCommand aux deux.
using CommandCallback = std::function<void(const String&, const String&)>;
