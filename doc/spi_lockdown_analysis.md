# SPI Corruption Diagnostic

## Objectif

Documenter le problème de corruption SPI observé dans `bad_magic` / `zero_rx` et expliquer la cause probable avec les extraits de code pertinents. Ce document doit pouvoir être transmis à une IA pour analyse approfondie.

---

## Contexte

- Application host : `src/rpi/...`
- Firmware ESP32 : `src/esp32/...`
- Problème observé : `bad_magic`, `bad_crc`, `zero_rx` et `host/firmware buffer desync suspected`
- Conséquence : arrêt correct sur endstop pendant `home_lateral`, mais le moteur ne repart pas pour la phase suivante.

---

## Synthèse du problème

1. Le signal de fin de course est détecté correctement.
2. Le streamer host arrête le flux et demande un flush.
3. Lors de la transaction SPI suivante, le host reçoit une trame invalide ou nulle.
4. Cela fait perdre la synchronisation entre le host et le firmware.
5. En pratique, la reprise du mouvement ne peut pas se produire.

---

## Log pertinent

Extraits de `bad_magic.txt` :

```text
transport.spi_transport WARNING spi_transport: attempt 1 parse error: bad magic: 0x2DF5
transport.spi_transport WARNING spi_transport: attempt 1 parse error: bad magic: 0x0000
transport.spi_transport WARNING spi diag host: xfers=190 bad_magic=3 bad_crc=0 zero_rx=2 reopens=0
transport.streamer WARNING host/firmware buffer desync suspected: inflight=60 buffered=321.7ms multi_axis_free=63 planner_free=128 ring_free=(4096, 4019, 0, 0) last_executed=674 last_confirmed=674 last_sent=734
```

---

## Cause racine probable

La corruption vient de la couche SPI elle-même, côté firmware ESP32 :

- `comm_interface.cpp` utilise un loop `spi_slave_transmit()` simple.
- Entre deux transactions, il existe une fenêtre pendant laquelle aucun transfert n’est prêt.
- Si le maître Pi envoie pendant cette fenêtre, le slave renvoie `MISO` à zéro et ignore `MOSI`.
- Le code host tente de compenser avec un délai fixe de 700 µs sur `xfer3()`.
- Ce délai est insuffisant dans la configuration actuelle.

---

## Extraits de code importants

### 1. Côté host : transport SPI

Fichier : `src/rpi/transport/spi_transport.py`

```python
    def _xfer(self, frame: bytes) -> bytes:
        """Perform one full-duplex SPI frame transfer with explicit params.

        Uses explicit speed/mode-compatible arguments on every call to avoid
        hidden defaults. Prefers xfer3 when available.
        """
        tx = list(frame)
        # delay_usecs=700: the ESP32 needs ~0.5 ms to process a received frame
        # (handleFrame + buildStatusFrame) and re-arm spi_slave_transmit().
        # 15 µs was too short — the DMA TX buffer was not armed when the Pi
        # sent the next frame, causing a 1-byte shift (bad magic: 0x0150).
        # 700 µs gives comfortable margin with negligible throughput impact
        # (700 µs vs 4096 µs transfer time = <15% overhead at 1 MHz).
        if hasattr(self._spi, "xfer3"):
            return bytes(self._spi.xfer3(tx, self._speed_hz, 700, 8))
        return bytes(self._spi.xfer2(tx, self._speed_hz, 700, 8))
```

Cette méthode est utilisée pour tous les échanges SPI et doit recevoir une réponse de taille fixe.

---

### 2. Côté firmware : boucle SPI du slave

Fichier : `src/esp32/src/comm_interface.cpp`

```cpp
void CommInterface::spiTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);
    ESP_LOGI(TAG, "SPI task started on core %d", xPortGetCoreID());

    spi_slave_transaction_t txn = {};

    // Pre-build the very first status frame before entering the loop so that
    // the top of the loop can call spi_slave_transmit() immediately with
    // minimal gap.
    self->buildStatusFrame(s_tx_frame_a);

    for (;;) {
        // ── Step 1: transmit immediately (status already pre-built) ───────────
        txn.length    = SPI_FRAME_SIZE * 8;
        txn.tx_buffer = s_tx_frame_a;
        txn.rx_buffer = s_rx_frame;
        esp_err_t err = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        ++diag_cycles;

        // ── Step 3: parse and handle the received frame ----------------------
        SpiMessageHeader header {};
        memcpy(&header, s_rx_frame, sizeof(SpiMessageHeader));

        if (header.magic != SPI_MSG_MAGIC) {
            ++diag_bad_magic;
            if (s_rx_frame[0] == 0x00 && s_rx_frame[1] == 0x00) {
                ++diag_bad_magic_zero;
            }
        } else if (header.version != SPI_MSG_VERSION) {
        } else if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
        } else if (!spi_message_validate(s_rx_frame, header)) {
            ++diag_bad_crc;
        } else {
            ++diag_ok;
            ...
        }

        // Pre-build the next status frame to minimize the window where MISO is all-zero.
        self->buildStatusFrame(s_tx_frame_a);

        // Tiny delay to ensure CPU caches flush into DMA-capable RAM before the next transaction.
        // Prevents a 1-byte FIFO alignment glitch on rapid back-to-back transfers.
        esp_rom_delay_us(2);
    }
}
```

### 3. Comment le firmware détecte la corruption

À l’intérieur de la boucle :

- `header.magic != SPI_MSG_MAGIC` incrémente `diag_bad_magic`
- `s_rx_frame[0]==0 && s_rx_frame[1]==0` incrémente `diag_bad_magic_zero`
- `!spi_message_validate()` incrémente `diag_bad_crc`

Ces diagnostics sont la source des erreurs remontées sur le host.

---

## Recommandation de verrouillage

### Correctif recommandé

- Remplacer le pattern `spi_slave_transmit()` par un pattern de type `spi_slave_queue_transmit()` / `spi_slave_get_trans_result()`.
- Maintenir toujours une transaction SPI en attente côté firmware.
- Utiliser au minimum deux buffers DMA : un pour la transaction en attente, un pour la transaction en cours.
- Éliminer la fenêtre morte entre deux transactions.

### Résultat attendu

- plus de réponse `zero_rx`
- plus de `bad_magic`
- plus de désynchronisation entre host et firmware
- le homing devrait pouvoir passer de `approach` à `backoff`

---

## Fichiers à analyser

- `src/rpi/transport/spi_transport.py`
- `src/rpi/transport/streamer.py`
- `src/esp32/src/comm_interface.cpp`
- `src/esp32/src/messages.h`

---

## Notes complémentaires

- Le host utilise `xfer3()` avec `delay_usecs=700` comme contremesure.
- Le firmware utilise `spi_slave_transmit()` et préconstruit le status frame.
- Ce document vise à donner un contexte complet à une IA d’analyse.
