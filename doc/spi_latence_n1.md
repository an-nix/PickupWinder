# Réduction de la latence ACK à N+1 dans le mode expérimental pré-queue

## Contexte : pourquoi le mode actuel donne N+2

### Pipeline de production (`spi_slave_transmit`, latence N+1)

```
Transfer N   : Pi envoie Rn,   ESP32 envoie S(Rn-1)   ← S pré-construit avant le transfer
               ── DMA complète ──
               process_rx_frame(Rn)     → met à jour last_rx_sequence_, last_result_
               buildStatusFrame(tx_a)  → S(Rn) prêt dans tx_a
               esp_rom_delay_us(2)
               *** fenêtre morte (~10–50 µs) ***
Transfer N+1 : Pi envoie Rn+1, ESP32 envoie S(Rn)     ← Pi voit l'ACK de Rn
```

S(Rn) est visible au transfer **N+1**. En contrepartie il y a une fenêtre morte.

---

### Pipeline expérimental (`queue_trans/get_trans_result`, latence N+2)

```
Avant transfer N :
  file DMA = [ txn_a(tx=S(Rn-2)), txn_b(tx=S(Rn-1)) ]   ← txn_b déjà en hardware

Transfer N   : Pi envoie Rn,   ESP32 envoie S(Rn-2)  (txn_a exécuté)
               ── DMA complète ──
               get_trans_result() → txn_a rendu
               process_rx_frame(Rn)
               buildStatusFrame(tx_a)  → S(Rn) dans tx_a
               esp_rom_delay_us(2)
               queue_trans(txn_a)      → S(Rn) entre en file DERRIÈRE txn_b

Transfer N+1 : Pi envoie Rn+1, ESP32 envoie S(Rn-1)  (txn_b était déjà en hardware)
               ── DMA complète ──
               ...

Transfer N+2 : Pi envoie Rn+2, ESP32 envoie S(Rn)    ← Pi voit l'ACK de Rn, 2 transfers plus tard
```

Le TX de txn_b était figé avant que Rn arrive. S(Rn) ne peut physiquement entrer qu'en troisième position.

---

## La contrainte causale fondamentale

> **S(Rn) ne peut être construit qu'après la fin du transfer N.**  
> **Pour que S(Rn) soit dans le TX du transfer N+1, ce buffer doit être prêt avant que N+1 démarre.**  
> **Éliminer la fenêtre morte impose d'avoir ce buffer prêt avant même que N se termine.**

Ces trois contraintes sont mutuellement exclusives avec une approche purement « task loop ».

Conséquence directe :

| Approche | Fenêtre morte | Latence ACK |
|---|---|---|
| `spi_slave_transmit` (prod) | ~10–50 µs | **N+1** |
| `queue_trans` 2-in-flight (actuel) | Quasi-nulle | **N+2** |
| `queue_trans` 1-in-flight seulement | ~10–50 µs identique | **N+1** mais aucun avantage |
| `post_trans_cb` + snapshot ISR-safe | **< 1 µs** (ISR immédiat) | **N+1** ✅ |

---

## Seule option viable : `post_trans_cb` + snapshot ISR-safe

### Principe

1. Le callback `post_trans_cb` est appelé depuis l'ISR du DMA dès la fin physique du transfer.
2. Il lit un **snapshot atomique** pré-calculé par la tâche, construit le frame suivant et appelle `spi_slave_queue_trans()` immédiatement — avant même que le scheduler réveille la tâche.
3. La tâche, elle, traite Rn et met à jour ce snapshot pour le transfer N+2.

```
Transfer N   : Pi envoie Rn,   ESP32 envoie S(Rn-1)   ← snapshot pré-calculé par la tâche
               ── DMA ISR ──
               post_trans_cb() :
                 buildFastStatusFrame(snapshot → tx_ready)   ← lit atomics SEULEMENT
                 esp_rom_delay_us(2)
                 spi_slave_queue_trans(txn_ready)  ← < 1 µs après fin du DMA
                 notify_task()

               Tâche réveillée :
                 process_rx_frame(Rn)
                 update_snapshot(Rn)   ← met à jour le snapshot pour N+2

Transfer N+1 : Pi envoie Rn+1, ESP32 envoie S(Rn)     ← Pi voit l'ACK de Rn ✅
               ── DMA ISR ──
               post_trans_cb() : même logique avec snapshot mis à jour
```

---

## Ce qu'il faut modifier concrètement

### 1. Ajouter un snapshot ISR-safe dans `CommInterface`

Dans `comm_interface.h`, ajouter une struct atomique lisible depuis un ISR :

```cpp
struct SpiStatusSnapshot {
    std::atomic<uint16_t> last_rx_sequence   {0};
    std::atomic<uint8_t>  last_rx_type       {static_cast<uint8_t>(SpiMessageType::NOP)};
    std::atomic<uint8_t>  last_result        {static_cast<uint8_t>(SpiMessageResult::OK)};
    std::atomic<uint16_t> last_exec_seq      {0xFFFFu};
    // Champs lents (queue counts) intentionnellement absents : ils ont 1 transfer de lag,
    // ce qui est acceptable et déjà le cas aujourd'hui avec le chemin de prod.
};
SpiStatusSnapshot snapshot_ {};
```

Les champs absents du snapshot (niveaux de queue, underrun counts, etc.) sont fournis par `buildStatusFrame()` complet appelé depuis la tâche, mais l'ISR construit un frame "réduit" avec uniquement les champs atomiques.

> **Alternative plus simple** : laisser `buildStatusFrame` inchangé et déplacer son appel dans un troisième buffer pré-calculé par la tâche, signalé comme « prêt » via un flag atomique. L'ISR ne fait alors que copier ce buffer prêt et appeler `queue_trans`. C'est plus sûr car `buildStatusFrame` n'entre jamais dans l'ISR.

### 2. Troisième buffer DMA `s_tx_frame_c`

Allouer un troisième buffer :

```cpp
static uint8_t* s_tx_frame_c = nullptr; // "ready" — pré-construit par la tâche, copié par l'ISR
```

Séquence des buffers :
- `s_tx_frame_a` : en DMA hardware (transfer en cours)
- `s_tx_frame_b` : en DMA hardware (transfer suivant, déjà queued)
- `s_tx_frame_c` : pré-construit par la tâche, contient S(Rn), protégé par un `atomic_bool ready_c`

### 3. Modifier `spi_slave_interface_config_t`

```cpp
slave_cfg.queue_size    = 2;
slave_cfg.post_trans_cb = &CommInterface::spiPostTransCb;
```

La signature d'un `post_trans_cb` ESP-IDF :

```cpp
static void IRAM_ATTR spiPostTransCb(spi_slave_transaction_t* t);
```

Elle doit être `IRAM_ATTR` pour ne pas pagefaulter depuis l'ISR.

### 4. Implémenter `spiPostTransCb`

```cpp
// IRAM_ATTR obligatoire — appelé depuis ISR DMA
void IRAM_ATTR CommInterface::spiPostTransCb(spi_slave_transaction_t* t)
{
    // Identifier quel txn vient de se terminer et quel buffer TX il utilisait.
    // Récupérer s_self (pointeur statique vers l'instance CommInterface).
    CommInterface* self = s_self;  // pointeur statique à stocker dans init()

    // Si le buffer "ready_c" est prêt (la tâche a fini de construire S(Rn)),
    // copier c → tx_buffer du txn recyclé et le queue immédiatement.
    if (s_ready_c.load(std::memory_order_acquire)) {
        // Identifier le txn recyclable (celui qui vient de finir).
        spi_slave_transaction_t* recycle = /* txn_a ou txn_b selon t */ ;
        memcpy(recycle->tx_buffer, s_tx_frame_c, SPI_FRAME_SIZE);
        esp_rom_delay_us(2);
        // Si `spi_slave_trans_isr` est disponible sur la version d'ESP-IDF,
        // l'utiliser ici à la place de `spi_slave_queue_trans` pour un
        // chemin ISR-safe explicite.
        spi_slave_queue_trans(SPI3_HOST, recycle, 0);  // timeout=0 depuis ISR
        s_ready_c.store(false, std::memory_order_release);
    }
    // Si ready_c n'est pas encore prêt, la fenêtre morte réapparaît.
    // C'est acceptable si buildStatusFrame() est plus rapide qu'un transfer SPI (~1 ms).

    // Notifier la tâche SPI (BaseType_t* pxHigherPriorityTaskWoken).
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_spi_task_handle, &woken);
    portYIELD_FROM_ISR(woken);
}
```

### 5. Modifier la boucle tâche pour le mode `post_trans_cb`

```cpp
for (;;) {
    // Attendre la notification de l'ISR (au lieu de get_trans_result bloquant).
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Identifier quelle transaction vient de se terminer (flag atomique depuis ISR).
    uint8_t* completed_rx = /* rx du txn qui vient de finir */ ;

    process_rx_frame(completed_rx);
    maybe_log_diag();

    // Construire le prochain frame et le marquer ready pour l'ISR.
    self->buildStatusFrame(s_tx_frame_c);
    // Si s_tx_frame_c est alloué avec MALLOC_CAP_DMA, aucun délai actif n'est
    // nécessaire ici : l'accès DMA lit directement la mémoire cohérente.
    s_ready_c.store(true, std::memory_order_release);
    // L'ISR se chargera de le queue_trans au prochain post_trans_cb.
}
```

---

## Résumé des modifications

| Fichier | Modification |
|---|---|
| `comm_interface.h` | + `SpiStatusSnapshot snapshot_` (optionnel si approche triple-buffer) ; + déclaration `spiPostTransCb` ; + `s_spi_task_handle` statique |
| `comm_interface.cpp` `init()` | + allocation `s_tx_frame_c` ; `slave_cfg.post_trans_cb = spiPostTransCb` ; mémoriser `s_self` et `s_spi_task_handle` |
| `comm_interface.cpp` boucle expérimentale | Remplacer `get_trans_result` par `ulTaskNotifyTake` ; écrire `s_tx_frame_c` + `s_ready_c` flag |
| Nouveau `spiPostTransCb` | `IRAM_ATTR` ; `memcpy` + `queue_trans` immédiat depuis ISR ; `vTaskNotifyGiveFromISR` |

---

## Risques spécifiques à cette approche

1. **`spi_slave_queue_trans()` depuis ISR** : c'est le point le plus fragile. Dans ESP-IDF, l'appel peut passer par une queue FreeRTOS interne, et il faut donc absolument vérifier le code source de la version cible : si le driver n'utilise pas `xQueueSendFromISR` quand `timeout = 0`, la solution est invalide. Si la version d'ESP-IDF propose `spi_slave_trans_isr` ou un équivalent ISR-safe, c'est une alternative préférable, car elle évite la dépendance à la sémantique interne d'une queue.

2. **`esp_rom_delay_us(2)` dans un ISR** : ce n'est pas seulement un petit délai ; 2 µs avec interruptions masquées sur Core 0 est significatif. Il faut se demander pourquoi ce délai existe. Si c'est pour garantir la cohérence d'un buffer DMA, la bonne solution est de s'assurer que `s_tx_frame_c` est déjà alloué en mémoire DMA-capable et correctement alignée, ou d'utiliser un flush ciblé (`Cache_WriteBack_Addr`) plutôt que d'attendre en boucle.

3. **`memcpy(512 bytes)` dans un ISR** : ~64 cycles XTENSA à 240 MHz ≈ 270 ns. C'est court, mais c'est un travail non nul dans l'ISR. Il faut le mesurer et préférer une copie intelligente ou un buffer déjà prêt si possible.

4. **Identification du txn complété** : le document actuel suggère de comparer `t == &txn_a` ou `t == &txn_b`, mais si `txn_a` et `txn_b` sont des variables locales à la boucle, ce n'est pas valide. Ils doivent être stockés dans une zone accessible par l'ISR (variables statiques, membres de classe, ou via `s_self`) avant de pouvoir être examinés en callback.

5. **Alternative réelle à explorer** : plutôt que de reconstruire le status dans l'ISR, construire un buffer `s_tx_frame_c` dans la tâche et rendre simplement ce buffer disponible à l'ISR via un flag atomique. L'ISR ferait alors uniquement une copie finale ou un swap de pointeur et un `queue_trans`, ce qui limite fortement la surface d'erreur.

---

## Contraintes ESP-IDF SPI Slave DMA

Cette documentation d'Espressif confirme que le mode DMA du SPI slave impose des contraintes strictes :

- Le buffer RX doit être aligné sur 32 bits et la longueur doit être un multiple de 4 octets. Sinon, le driver peut renvoyer une erreur et les données peuvent être écrites incorrectement.
- Le master doit également envoyer des longueurs multiples de 4 octets. Des trames de longueur inappropriée sont susceptibles d'être rejetées ou ignorées.
- Le DMA SPI slave nécessite préférentiellement les modes SPI 1 et 3. Pour les modes SPI 0 et 2, la sortie MISO est lancée une demi-période d'horloge plus tôt pour satisfaire le timing DMA, ce qui modifie la fenêtre de setup/hold :
  - si la matrice GPIO est contournée, le hold time devient 68,75 ns ;
  - si la matrice GPIO est utilisée, le hold time passe à 93,75 ns.
- Le host doit échantillonner immédiatement sur le front de latch réel, ou bien initialiser le périphérique sans DMA si son timing ne peut pas l’assurer.
- Le SPI slave ESP32 peut maintenir MISO actif même quand CS est désactivé, ce qui peut perturber d'autres appareils sur le bus. La solution recommandée est :
  - utiliser un bus SPI séparé pour l'ESP32 slave ;
  - ou intercaler une puce tampon sur la ligne MISO, par exemple un 74HC125.

Ces contraintes renforcent le fait que le prototype DMA doit être testé avec soin, et qu'un bus dédié ou une isolation matérielle est plus sûr que le partage du même bus SPI avec d'autres dispositifs.

---

## Conclusion

N+1 avec fenêtre morte quasi-nulle est faisable, mais requiert de déplacer le `queue_trans` dans un `post_trans_cb` ISR, ce qui introduit des contraintes sur les appels API depuis ISR et sur la durée du callback. L'approche la plus sûre est le **triple buffer avec flag atomique** : la tâche construit `s_tx_frame_c` librement, l'ISR ne fait que le copier et le queue, sans jamais appeler `buildStatusFrame` lui-même.
