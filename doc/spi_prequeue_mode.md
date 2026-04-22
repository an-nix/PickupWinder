# Mode SPI — avec et sans `SPI_EXPERIMENTAL_PREQUEUE`

Ce document décrit le fonctionnement du transport SPI côté firmware dans `src/esp32/src/comm_interface.cpp`, selon que `SPI_EXPERIMENTAL_PREQUEUE` est `false` (mode de production actuel) ou `true` (mode expérimental pré-queue).

## Contexte

Dans le code actuel, le firmware ESP32 est configuré comme esclave SPI avec :

- `slave_cfg.mode = 0`
- `slave_cfg.queue_size = SPI_EXPERIMENTAL_PREQUEUE ? 2 : 1`
- un buffer fixe de `SPI_FRAME_SIZE = 512` octets
- DMA-capable buffers alloués via `heap_caps_malloc(..., MALLOC_CAP_DMA | MALLOC_CAP_32BIT)`

Le host Python utilise `spidev` et établit le même mode SPI via `Esp32SpiTransport(..., mode=0)`.

## 1. Sans `SPI_EXPERIMENTAL_PREQUEUE` (mode de production)

### Pipeline

1. Construire `S(Rn-1)` dans `s_tx_frame_a` avec `buildStatusFrame(s_tx_frame_a)`.
2. Appeler `spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY)`, où `txn.tx_buffer = s_tx_frame_a` et `txn.rx_buffer = s_rx_frame`.
3. Quand le transfert se termine, traiter `s_rx_frame` dans `process_rx_frame(s_rx_frame)`.
4. Reconstruire le statut suivant avec `buildStatusFrame(s_tx_frame_a)`.
5. Appeler `esp_rom_delay_us(2)` pour s'assurer que le cache est flushé vers la DRAM DMA.
6. Retour à l'étape 2.

### Propriétés

- `queue_size` = 1
- synchronisation stricte : un échange SPI à la fois
- statut visible au transfer suivant (`S(Rn)` envoyé dans le transfer `N+1`)
- latence ACK : **N+1**
- petite fenêtre morte entre transferts pendant laquelle aucun transfert n'est armé
- mécanisme de déduplication et d'ACK conservé

### Avantages

- simplicité
- comportement déterministe
- pas de logique supplémentaire de ping-pong
- facile à comprendre et à diagnostiquer

### Inconvénients

- présence d'une petite fenêtre morte entre deux `spi_slave_transmit`
- possible pertes si le maître envoie pendant cette fenêtre
- nécessite un délai actif (`esp_rom_delay_us(2)`) pour garantir la cohérence du buffer DMA

## 2. Avec `SPI_EXPERIMENTAL_PREQUEUE` (mode pré-queue expérimental)

### Pipeline

1. Allouer deux transactions `txn_a` et `txn_b`.
2. Pré-construire `S(R-2)` et `S(R-1)` dans `s_tx_frame_a` et `s_tx_frame_b`.
3. Appeler `spi_slave_queue_trans(SPI3_HOST, &txn_a, portMAX_DELAY)` puis `spi_slave_queue_trans(SPI3_HOST, &txn_b, portMAX_DELAY)`.
4. Boucle :
   - appeler `spi_slave_get_trans_result(SPI3_HOST, &done_txn, portMAX_DELAY)` pour récupérer la transaction terminée.
   - identifier si `done_txn == &txn_a` ou `done_txn == &txn_b`.
   - traiter le RX correspondant (`s_rx_frame` ou `s_rx_frame_b`).
   - reconstruire le statut dans le TX du transaction revenu (`completed_tx`).
   - appel `esp_rom_delay_us(2)` pour la cohérence cache DMA.
   - `spi_slave_queue_trans(SPI3_HOST, recycle_txn, portMAX_DELAY)` pour remettre la transaction en file.

### Propriétés

- `queue_size` = 2
- deux transactions SPI peuvent être armées en même temps
- statut visible avec un décalage supplémentaire : **N+2**
- suppression quasi-totale de la fenêtre morte entre transferts, si le traitement est suffisamment rapide

### Avantages

- réduit la fenêtre morte entre transactions SPI
- peut diminuer les `zero_rx` ou `bad_magic` si ces erreurs viennent d’une transaction manquante
- le firmware est toujours prêt à recevoir le prochain transfert lorsque l’actuel se termine

### Inconvénients

- complexité logicielle accrue
- latence ACK plus élevée : chaque `S(Rn)` arrive un cycle de plus
- dépendance à `spi_slave_get_trans_result()` et `spi_slave_queue_trans()`
- le code est expérimental et doit être validé sur banc
- toujours besoin de `esp_rom_delay_us(2)` dans l’implémentation actuelle pour la cohérence DMA

## 3. Comparaison directe

| Comportement | Sans pré-queue | Avec pré-queue |
|---|---|---|
| `queue_size` | 1 | 2 |
| transactions armées simultanément | non | oui |
| statut visible | N+1 | N+2 |
| fenêtre morte | oui | quasi-nulle |
| complexité | faible | élevée |
| risque de timing | plus faible | plus élevé |
| utilité | production stable | expérimental, timing critique |

## 4. Recommandation

- Rester en `SPI_EXPERIMENTAL_PREQUEUE = false` pour la production, sauf si tu as une raison convaincante de tester le pré-queue.
- Si tu utilises le pré-queue, fais des tests de robustesse sur le matériel réel et observe les ACK, `bad_magic` et `zero_rx`.
- Le pré-queue peut aider uniquement si la source du problème est la fenêtre morte entre transferts.
- Si le problème est lié au mode SPI (`mode 0` vs `mode 1/3`) ou au bus partagé, le pré-queue ne résoudra pas ce problème.

## 5. Conclusion

- Le mode sans `SPI_EXPERIMENTAL_PREQUEUE` est le chemin de production recommandé.
- Le mode avec `SPI_EXPERIMENTAL_PREQUEUE` est un essai utile pour réduire la dead-time entre transactions, mais il déplace le problème vers une logique plus complexe et une latence ACK plus longue.
