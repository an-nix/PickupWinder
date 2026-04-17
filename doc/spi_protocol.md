# Protocole SPI — messages (PickupWinder)

Ce document décrit le format des trames SPI utilisées entre le Raspberry Pi (host) et l'ESP32 (slave), les règles de sérialisation, le calcul de CRC, et la procédure concrète pour ajouter un nouveau message côté host (Python) et côté firmware (ESP32/C++).

---

## Aperçu général

- Taille de trame fixe : 512 octets.
- Une trame = en‑tête (12 octets) + payload (variable) + remplissage. Le CRC couvre l'en‑tête + le payload.
- Endianness : little‑endian.
- Intégrité : CRC16‑CCITT‑FALSE (init 0xFFFF, poly 0x1021).

Diagramme :

```
+----------------------+--------------------------------------+----------+
| Offset 0..11 (12B)   | Offset 12..(12+payload-1) (payload)  | Padding  |
| SpiMessageHeader     | Payload spécifique au message       | zeros    |
+----------------------+--------------------------------------+----------+
```

### `SpiMessageHeader` (12 bytes, packed)

- `uint16_t magic`         — 0x5057 ("PW").
- `uint8_t  version`       — protocole (actuellement 1).
- `uint8_t  msg_type`      — valeur `SpiMessageType`.
- `uint16_t sequence`      — numéro séquentiel (host → MCU) pour détecter pertes/retransmissions.
- `uint16_t payload_length`— longueur utile du payload en octets.
- `uint16_t flags`         — réservé/ou usage spécifique.
- `uint16_t crc16`         — CRC calculé sur `header (avec crc16=0) + payload`.

> Remarque : la taille d'en‑tête est statique (12 octets). Le MCU valide : magic, version, payload_length puis le CRC.

---

## Payloads

- Les payloads request et response sont des structs `packed` en C/C++ côté ESP32 (voir `src/esp32/src/messages.h`).
- Côté host, utiliser les `struct.Struct` et dataclasses dans `src/rpi/messages.py` pour empaqueter/dépaqueter.
- Toujours maintenir la correspondance stricte noms/ordres/types/taille entre le struct C et le format Python.

Exemples existants :

- `FlushPayload` (4 bytes)
- `MultiAxisSegmentBlockHeader` + records (format variable)
- `StatusPayload` (48 bytes) — réponse MCU contenant `last_executed_sequence` utile au host.

---

## CRC et validation

- Calcul : CRC16‑CCITT‑FALSE. Sur ESP32, utilisez `spi_crc16_ccitt()` et `spi_message_finalize()` (définitions dans `src/esp32/src/messages.h`).
- Sur le host Python, `crc16_ccitt()` et `build_frame()` dans `src/rpi/messages.py` s'occupent du calcul et de la construction complète de la trame.
- Lors de la génération du CRC, positionnez d'abord `header.crc16 = 0` puis calculez le CRC sur `header + payload`.

---

## `StatusPayload` et feedback d'exécution

Le MCU renvoie périodiquement une `StatusPayload` (dans la trame RX) qui contient, entre autres :

- `queue_free_slots[]` et `ring_free_slots[]` pour chaque axe (controle de flux host).
- `underrun_count[]` pour diagnostiquer pertes.
- `last_rx_sequence`, `last_result` pour interpréter le traitement du dernier message reçu.
- `last_executed_sequence` : numéro de séquence du dernier segment multi‑axe **complètement exécuté** — utilisé par le host pour savoir combien de temps de mouvement reste en file.

---

## Bonnes pratiques

- Ajoutez toujours `reserved[]` (padding) si vous pensez étendre le payload plus tard.
- Ne changez pas l'alignement/ordre des champs existants sans incrémenter `SPI_MSG_VERSION` et fournir une migration.
- Utilisez `static_assert(sizeof(MyPayload) == N)` côté firmware pour éviter des incompatibilités binaires.
- Traitez `payload_length` et `crc` comme des validations obligatoires côté MCU.

---

## Procédure pas‑à‑pas pour ajouter un nouveau message

1. **Choisir une valeur de message**
   - Ajoutez l'identifiant à l'énum `SpiMessageType` dans `src/esp32/src/messages.h` (e.g. `MY_NEW_CMD = 0x20`) ET dans `src/rpi/messages.py` (même valeur dans `SpiMessageType`).

2. **Définir le payload côté ESP32 (C/C++)**
   - Déclarez un `struct __attribute__((packed)) MyNewPayload { ... }` dans `src/esp32/src/messages.h`.
   - Ajoutez `static_assert(sizeof(MyNewPayload) == X, "...")`.

3. **Définir l'équivalent Python**
   - Créez un `_MYNEW_STRUCT = struct.Struct("<...")` et une `dataclass` (avec `pack()` et/ou `unpack()`) dans `src/rpi/messages.py`.
   - Vérifiez la taille : `_MYNEW_STRUCT.size` doit être égale à la taille C.

4. **Implémenter le handler côté firmware**
   - Déclarez `esp_err_t CommInterface::handleMyNew(const MyNewPayload& p)` dans `src/esp32/src/comm_interface.h`.
   - Implémentez le handler dans `src/esp32/src/comm_interface.cpp` : vérifiez `header.payload_length`, contrôlez les champs, puis exécutez l'action (enqueue, config, toggle GPIO, …).
   - Retournez des codes `ESP_ERR_*` appropriés (ex. `ESP_ERR_INVALID_ARG`, `ESP_ERR_TIMEOUT`). Le dispatch translate en `SpiMessageResult`.

5. **Ajouter le case dans `handleFrame()`**
   - Dans `CommInterface::handleFrame()`, ajoutez un `case SpiMessageType::MY_NEW_CMD:` qui appelle votre handler.

6. **Ajouter les helpers Python d'envoi**
   - Ajoutez `make_my_new(payload, sequence=0)` dans `src/rpi/messages.py` (utilise `build_frame()` pour CRC et framing).
   - Si nécessaire, ajoutez une API de transport dans `src/rpi/spi_transport.py` pour l'envoyer depuis le streamer ou les outils de test.

7. **Gestion d'exécution et de concurrence**
   - Si le message provoque du mouvement, adaptez `stepper_queue` / `stepper_queue.cpp` ou le nouvel exécuteur multi‑axe pour consommer en sécurité.
   - Si le message doit être prioritaire (ex. FLUSH/ESTOP), implémentez une file séparée ou un mécanisme de signal prioritaire dans l'executor (tel que le pattern flush utilisé pour MULTI_AXIS_SEGMENT_BLOCK).

8. **Tests**
   - Vérifier Python :

```powershell
python -m py_compile src\rpi\messages.py
python -c "from src.rpi import messages; print(messages._MYNEW_STRUCT.size)"
```

   - Compiler firmware et vérifier `static_assert` :

```powershell
cd src/esp32
# PlatformIO
platformio run
# ou ESP-IDF
idf.py build
```

   - Test runtime : envoyer la trame via `demo_spi.py` ou un petit script envoyant `build_frame(MyNewPayload)` et observer `StatusPayload.last_result` / logs MCU.

9. **Versioning et rétro‑compatibilité**
   - Si la nouvelle structure casse le format existant, incrémentez `SPI_MSG_VERSION` et gérez les deux versions côté host ou MCU.

---

## Exemple minimal (SET_LED)

C (ajout dans `src/esp32/src/messages.h`):

```c
struct __attribute__((packed)) SetLedPayload {
    uint8_t led_id;
    uint8_t on;       // 0 = off, 1 = on
    uint8_t reserved[2];
};
static_assert(sizeof(SetLedPayload) == 4, "SetLedPayload must be 4 bytes");
```

Python (ajout dans `src/rpi/messages.py`):

```py
_SET_LED_STRUCT = struct.Struct("<BB2x")

@dataclass(slots=True)
class SetLedPayload:
    led_id: int
    on: bool

    def pack(self) -> bytes:
        return _SET_LED_STRUCT.pack(self.led_id, 1 if self.on else 0)

# helper
def make_set_led(led_id: int, on: bool, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType(0x20), SetLedPayload(led_id, on).pack(), sequence=sequence)
```

ESP32 handler (squelette dans `comm_interface.cpp`):

```cpp
esp_err_t CommInterface::handleSetLed(const SetLedPayload& p) {
    // validate led_id, appliquer l'état sur un GPIO ou via un driver
    return ESP_OK;
}
// et dans handleFrame(): case SpiMessageType::SET_LED: return handleSetLed(*reinterpret_cast<const SetLedPayload*>(payload));
```

---

## Ressources dans le dépôt

- Déclarations des messages (C) : `src/esp32/src/messages.h`
- Mirror Python : `src/rpi/messages.py`
- Transport / envoi : `src/rpi/spi_transport.py`
- Dispatcher SPI / handlers : `src/esp32/src/comm_interface.cpp`
- Executor / files queues : `src/esp32/src/stepper_queue.h` et `.cpp`

---

Si vous voulez, je peux :

- ajouter un message d'exemple complet (C handler + Python helper + test) au dépôt et ouvrir un patch ; ou
- générer un petit script de test qui envoie la trame et affiche `StatusPayload` reçu.

Dites ce que vous préférez et je l'implémente directement.
