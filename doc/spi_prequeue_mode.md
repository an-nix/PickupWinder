# Mode SPI — avec et sans `SPI_EXPERIMENTAL_PREQUEUE`

Ce document décrit le fonctionnement du transport SPI côté firmware dans `src/esp32/src/comm_interface.cpp`, selon que `SPI_EXPERIMENTAL_PREQUEUE` est `false` (mode de production actuel) ou `true` (mode expérimental pré-queue).

## Contexte

Dans le code actuel, le firmware ESP32 est configuré comme esclave SPI avec :

- `slave_cfg.mode = 1`
- `slave_cfg.queue_size = SPI_EXPERIMENTAL_PREQUEUE ? 2 : 1`
- un buffer fixe de `SPI_FRAME_SIZE = 512` octets
- DMA-capable buffers alloués via `heap_caps_malloc(..., MALLOC_CAP_DMA | MALLOC_CAP_32BIT)`

Le host Python utilise `spidev` et établit le même mode SPI via `Esp32SpiTransport(..., mode=1)`.

Le choix entre chemin simple et pré-queue est orthogonal au mode SPI : le document compare ici la stratégie de queue, pas le réglage électrique du bus.

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
- Si le problème est lié au mode SPI ou au bus partagé, le pré-queue ne résoudra pas ce problème.

## 5. Note sur le mode SPI actif

- Le chemin documenté ici tourne désormais en SPI mode `1` côté host et firmware.
- La bascule en mode 1 ne change pas la sémantique N+1 du chemin simple ni la sémantique N+2 du pré-queue.
- Elle vise uniquement à retirer une fragilité électrique/timing observée avec le slave DMA en mode 0.

## 6. Conclusion

- Le mode sans `SPI_EXPERIMENTAL_PREQUEUE` est le chemin de production recommandé.
- Le mode avec `SPI_EXPERIMENTAL_PREQUEUE` est un essai utile pour réduire la dead-time entre transactions, mais il déplace le problème vers une logique plus complexe et une latence ACK plus longue.
*** Add File: c:\temp\pw\doc\spi_mode1_migration_summary.md
# Migration SPI mode 1 et revue des correctifs hérités du mode 0

## Résumé exécutif

La pile SPI active a été basculée en mode `1` des deux côtés :

- firmware ESP32 : `slave_cfg.mode = 1` dans `src/esp32/src/comm_interface.cpp`
- host Python : `Esp32SpiTransport(..., mode=1)` et valeur par défaut `mode=1`

Cette migration retire la dépendance au mode `0`, qui était devenu un point de fragilité avec le slave SPI DMA de l'ESP32. Je n'ai pas supprimé en bloc les garde-fous ajoutés pendant l'enquête précédente : certains restent utiles même en mode `1`, d'autres doivent simplement être revalidés au banc avant d'être allégés.

## Fichiers modifiés

- `src/esp32/src/comm_interface.cpp`
- `src/esp32/src/motion_planner.cpp`
- `src/rpi/transport/spi_transport.py`
- `src/rpi/app/runtime.py`
- `src/rpi/examples/debug_spindle.py`
- `src/rpi/examples/demo_spi.py`
- `src/rpi/examples/segment_json.py`
- `src/rpi/transport/streamer.py`
- `README.md`
- `doc/architecture.md`
- `doc/spi_protocol.md`
- `doc/spi_prequeue_mode.md`

## Inventaire des correctifs en place

### 1. `delay_usecs=700` dans `src/rpi/transport/spi_transport.py`

**But**

Laisser une marge entre deux transferts back-to-back pendant que l'ESP32 :

1. parse la requête reçue,
2. reconstruit la prochaine trame STATUS,
3. réarme la transaction slave DMA.

**Verdict**

- **À garder pour l'instant**.
- Ce garde-fou a été introduit comme palliatif pendant l'enquête mode `0`, mais il protège aussi un comportement réel du chemin simple : la trame suivante n'est pas armée instantanément.
- En mode `1`, il est probablement plus conservateur que nécessaire, mais le supprimer maintenant reviendrait à mélanger migration électrique et retuning de timing.

**Commentaire**

Le correctif est coûteux en latence mais simple et robuste. La bonne suite n'est pas de le supprimer à l'aveugle, mais de le réduire progressivement après mesures matérielles.

### 2. `esp_rom_delay_us(2)` après `buildStatusFrame()` dans `src/esp32/src/comm_interface.cpp`

**But**

Ajouter une mini-garde entre la reconstruction du buffer TX et le réarmement DMA suivant.

**Verdict**

- **À revalider, mais à garder provisoirement**.
- Ce délai est très faible et son coût est négligeable.
- En mode `1`, il se peut qu'il ne soit plus déterminant, mais il reste un garde-fou peu intrusif sur un chemin sensible.

**Commentaire**

Ce n'est pas une preuve de correction architecturale. C'est un stabilisateur local qu'il vaut mieux mesurer avant retrait.

### 3. Boucle firmware simple `spi_slave_transmit()` avec `SPI_EXPERIMENTAL_PREQUEUE = false`

**But**

Rester sur un seul transfert armé à la fois, avec une latence ACK `N+1`, au lieu d'introduire un ping-pong à `queue_size=2`.

**Verdict**

- **À garder**.
- Ce n'est pas un correctif spécifique au mode `0`, mais c'est le chemin de production le plus compréhensible et le plus déterministe.

**Commentaire**

Le pré-queue traite surtout la dead-time entre transactions. Il ne corrige pas un mauvais mode SPI et ajoute une latence ACK supplémentaire.

### 4. Préconstruction de la première trame STATUS puis reconstruction immédiate de la suivante

**But**

Réduire la fenêtre pendant laquelle aucun TX valable n'est prêt côté slave, donc limiter les réponses tout-zéro ou corrompues si le Pi enchaîne très vite.

**Verdict**

- **À garder**.
- Ce comportement reste utile en mode `1` car il traite la structure du pipeline `spi_slave_transmit()`, pas uniquement le choix du mode électrique.

**Commentaire**

Ce point est structurellement sain et ne devrait pas être retiré.

### 5. Retries host sur `bad magic`, `bad response CRC` et `zero_rx`

**But**

Absorber les corruptions ou trous transitoires sans casser la session applicative.

**Verdict**

- **À garder**.
- Le lien SPI restera soumis à des aléas physiques, même en mode `1`.
- Ces retries rendent le host résilient sans casser les invariants de protocole.

**Commentaire**

Ce correctif ne doit pas être confondu avec une rustine pour mode `0`. C'est une vraie stratégie de robustesse transport.

### 6. `wait_for_request_result()` et confirmation par `last_rx_sequence`

**But**

Tenir compte du pipeline d'ACK `N+1` et éviter d'interpréter la réponse full-duplex immédiate comme l'ACK certain de la requête courante.

**Verdict**

- **À garder absolument**.
- Ce n'est pas un palliatif au mode `0` ; c'est une règle fondamentale du protocole.

**Commentaire**

La migration en mode `1` ne change rien à cet invariant.

### 7. Fallback de statut stale dans `poll_status()`

**But**

Continuer à faire vivre la logique host pendant une panne télémétrique brève au lieu de casser immédiatement le streaming.

**Verdict**

- **À garder avec prudence**.
- Ce comportement est utile opérationnellement, mais il peut masquer une dégradation transport si on s'en contente trop longtemps.

**Commentaire**

Ce n'est pas une correction électrique. C'est un compromis applicatif entre continuité de service et visibilité des erreurs.

### 8. Compteurs et logs `bad_magic`, `bad_crc`, `zero_rx`, `reopens`

**But**

Rendre la qualité du lien observable pendant les essais et en exploitation.

**Verdict**

- **À garder**.
- Ils sont encore plus utiles maintenant, car ils permettront de vérifier objectivement le gain après passage en mode `1`.

**Commentaire**

Sans ces métriques, il serait impossible de dire si la migration a réellement amélioré le lien ou si la stabilité perçue vient d'autres garde-fous.

### 9. `vTaskDelay(1)` dans `src/esp32/src/motion_planner.cpp` quand le planner est idle

**But**

Éviter qu'une boucle vide ne monopolise inutilement le CPU et ne dégrade le comportement du cœur 0 qui porte la tâche SPI.

**Verdict**

- **À garder**.
- Le commentaire a été assaini : il ne faut plus présenter ce sleep comme une preuve causale unique du bug `0x0150`, mais comme une mesure saine d'équité CPU.

**Commentaire**

Ce correctif relève plus de l'hygiène temps réel que du mode SPI lui-même.

### 10. Note dans `src/rpi/transport/streamer.py` sur le pacing intra-boucle

**But**

Éviter d'ajouter des sleeps ad hoc supplémentaires dans la boucle serrée d'envoi sans re-mesurer le lien.

**Verdict**

- **Commentaire clarifié**.
- Le code ne dormait déjà pas à cet endroit ; le commentaire précédent laissait croire le contraire.

**Commentaire**

Ce n'était pas un correctif actif, mais une explication ambiguë. Je l'ai remise en cohérence avec le comportement réel.

## Ce que je n'ai volontairement pas retiré

- le `delay_usecs=700`
- les deux `esp_rom_delay_us(2)`
- les retries et logs host
- le chemin simple sans pré-queue

La raison est simple : changer le mode SPI et supprimer en même temps plusieurs garde-fous rendrait l'analyse impossible si la liaison se dégrade encore. La migration actuelle isole le changement principal : le passage de `mode 0` à `mode 1`.

## Ce qui mérite une revalidation matérielle

1. mesurer `bad_magic`, `bad_crc` et `zero_rx` avant/après migration ;
2. valider à la fréquence nominale actuelle ;
3. réduire ensuite `delay_usecs` par paliers, par exemple `700 -> 300 -> 100`, seulement si les compteurs restent propres ;
4. tester ensuite la suppression des `esp_rom_delay_us(2)` si les captures restent stables.

## Conclusion

Le vrai correctif de fond est la cohérence host/firmware en SPI mode `1`. Les autres correctifs présents dans le code ne doivent pas être vus comme des erreurs à supprimer d'urgence : plusieurs sont des protections de pipeline ou de robustesse transport qui restent défendables même après la migration.
