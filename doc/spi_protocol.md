# Protocole SPI — PickupWinder

Ce document décrit le format des trames SPI, la sémantique d'ACK pipelinée, et les règles de séquencement/dédoublonnage entre le Raspberry Pi et l'ESP32.

## Vue d'ensemble

- Taille de trame fixe : `512` octets.
- En-tête fixe : `12` octets (`SpiMessageHeader`).
- Endianness : little-endian.
- Intégrité : `CRC16-CCITT-FALSE`.
- Chemins de référence :
  - firmware : `src/esp32/src/messages.h`
  - host : `src/rpi/transport/messages.py`
  - transport host : `src/rpi/transport/spi_transport.py`

Une transaction SPI échange toujours une trame complète dans chaque sens.

Mode électrique actif : SPI mode `1` (`CPOL=0`, `CPHA=1`) sur le Raspberry Pi et sur l'ESP32. Cette symétrie est requise ; mélanger les modes ou revenir en mode `0` réintroduit une zone de timing déjà observée comme fragile avec le slave DMA ESP32.

```text
Host TX request N   ---> ESP32 parses request N
Host RX status N   <--- ESP32 returns status built after request N-1
```

Le retour est donc pipeliné d'une transaction.

## En-tête de trame

`SpiMessageHeader` contient :

- `magic = 0x5057`
- `version = 3`
- `msg_type`
- `sequence`
- `payload_length`
- `flags`
- `crc16`

Le CRC est calculé sur `header (crc16=0) + payload`.

## Messages actifs

- `GET_STATUS (0x06)` : télémétrie pure, sans publication d'un nouvel ACK de contrôle.
- `FLUSH (0x12)` : abandon des segments au-delà d'un seuil de séquence.
- `MULTI_AXIS_SEGMENT_BLOCK (0x13)` : chemin de production.
- `ENABLE_ENDSTOP (0x14)` : armement/désarmement matériel.
- `STEP_BLOCK (0x10)` et `SEGMENT_BLOCK (0x11)` : legacy/debug.

## `StatusPayload`

Le firmware renvoie notamment :

- `queue_free_slots[]`
- `ring_free_slots[]`
- `underrun_count[]`
- `multi_axis_queue_free`
- `last_rx_sequence`
- `last_rx_type`
- `last_result`
- `last_executed_sequence`
- `planner_queue_free`
- `last_planned_sequence`
- `segments_dropped`
- `enabled_mask`, `running_mask`
- `lateral_endstop_state`, `endstop_armed_mask`, `endstop_hit_mask`

`endstop_hit_mask` publie le signal canonique des arrêts endstop : le bit `N`
reste à `1` après un déclenchement sur l'axe `N` jusqu'au prochain
réarmement via `ENABLE_ENDSTOP`. Le host l'utilise en priorité pour détecter
un homing réussi, puis retombe sur des heuristiques plus faibles seulement en
compatibilité.

Le host doit considérer `last_result` comme l'ACK réel d'une requête seulement après avoir attendu la confirmation du `last_rx_sequence` correspondant.
Le triplet publié `last_rx_sequence` / `last_rx_type` / `last_result` désigne toujours la
dernière requête de contrôle non télémétrique effectivement prise en compte par le firmware.
Les polls `GET_STATUS`, `PING`, `NOP` et les parse errors SPI transitoires (`BAD_MAGIC`,
`BAD_VERSION`, `BAD_LENGTH`, `BAD_CRC`) ne doivent pas écraser cet ACK publié.

## Sémantique d'ACK

Le code actif côté host est :

- construction de trame : `src/rpi/transport/messages.py`
- émission : `src/rpi/transport/spi_transport.py`
- confirmation : `wait_for_request_result()`

Règle impérative :

- ne jamais interpréter la réponse full-duplex immédiate comme l'ACK certain de la requête courante ;
- attendre que `StatusPayload.last_rx_sequence == request_sequence`.

Conséquence pratique :

- `GET_STATUS` sert à lire l'état courant,
- `wait_for_request_result()` sert à confirmer une requête de contrôle,
- un poll de statut ne doit jamais faire "disparaître" l'ACK de la requête précédente.

## Séquences utilisées

Le protocole emploie trois séquences 16 bits distinctes :

1. `SpiMessageHeader.sequence`
   - corrélation transport requête / ACK.
2. `MultiAxisSegmentBlockHeader.block_seq`
   - détection des retries de bloc déjà acceptés.
3. `multi_axis_segment_t.motion_sequence`
   - ordre d'exécution du mouvement.

Les helpers de comparaison sont wrap-aware :

- Python : `sequence_signed_distance()` dans `src/rpi/transport/messages.py`
- firmware : `sequence_signed_distance_u16()` dans `src/esp32/src/step_types.h`

La comparaison est sûre tant que l'écart entre deux valeurs comparées reste strictement inférieur à `32768`, ce qui est largement vrai ici vu la profondeur des files et de l'inflight.

## Dédoublonnage et protection d'ordre

Le pipeline actif est désormais :

- transport exact retry dedupe dans `CommInterface`
  - clé : `sequence`, `type`, `length`, `crc`
- dédoublonnage des blocs déjà acceptés par `block_seq`
- rejet planner des `motion_sequence` stale ou hors ordre
- `flush` qui pose un plancher de séquence et vide les files amont avant d'injecter le sentinel

Les détails complets sont dans `doc/sequencing.md`.

## Ajouter un nouveau message

1. Ajouter le type dans `src/esp32/src/messages.h` et `src/rpi/transport/messages.py`.
2. Définir le payload packed côté firmware.
3. Définir le miroir Python (`struct.Struct` + dataclass).
4. Ajouter le handler dans `src/esp32/src/comm_interface.cpp`.
5. Ajouter le helper d'émission côté host dans `src/rpi/transport/messages.py` puis l'API correspondante dans `src/rpi/transport/spi_transport.py` si nécessaire.
6. Tester la taille binaire, le CRC, puis le comportement runtime.

## Validation minimale

Python :

```powershell
python -m py_compile src\rpi\transport\messages.py src\rpi\transport\spi_transport.py
```

Firmware :

```powershell
cd src/esp32
platformio run
```
