# Synthèse — Diagnostic SPI et niveau de preuve

## Statut d'implémentation

- Appliqué côté host : ordonnancement du `flush` dans `src/rpi/transport/streamer.py`.
- Implémenté côté firmware en mode expérimental : chemin `spi_slave_queue_trans()` / `spi_slave_get_trans_result()` prêt dans `src/esp32/src/comm_interface.cpp`, mais désactivé par défaut via `SPI_EXPERIMENTAL_PREQUEUE = false`.
- Non appliqué côté host transport : réduction `delay_usecs` 700 µs → 50 µs, car elle dépend d'une refonte firmware qui n'est pas validée.

### Risque projet

L'activation de la refonte firmware expérimentale reste risquée pour l'intégralité du projet en l'état.

Raisons :

- le dépôt ne contient aucune implémentation historisée de `spi_slave_queue_trans()` / `spi_slave_get_trans_result()` dans `src/esp32/src/comm_interface.cpp` ;
- l'affirmation "1-byte DMA shift" et la signature `0x0150` ne sont supportées que par des commentaires ajoutés dans le commit `d4f6b41`, pas par des logs fournis dans ce dépôt ;
- l'exemple de refonte n'est pas compatible tel quel avec l'API actuelle (`handleFrame` ne prend pas ces paramètres) ;
- l'exemple n'emploie pas la stratégie DMA actuellement utilisée (`heap_caps_malloc(..., MALLOC_CAP_DMA | MALLOC_CAP_32BIT)`) et n'est donc pas transposable mécaniquement.

Le code présent dans le dépôt a été adapté pour contourner ces points : il réutilise l'API actuelle, conserve les buffers DMA existants, ajoute un second jeu de buffers DMA, et laisse le chemin expérimental explicitement désactivé par défaut.

**Projet :** RPi host ↔ ESP32 firmware via SPI  
**Problème :** `bad_magic` / `zero_rx` / `host-firmware buffer desync` → blocage du homing après endstop

---

## 1. Constats et hypothèses

### 1.1 Constat confirmé : corruption SPI réelle

Les symptômes suivants sont bien observés dans les logs :

- `bad magic: 0x0000`
- `bad magic: 0x0350`
- `bad magic: 0x24F5`
- `bad magic: 0x2DF5`
- `zero_rx`
- `host/firmware buffer desync suspected`

Ils suffisent à établir qu'il existe une corruption ou désynchronisation SPI réelle sur certains échanges.

### 1.2 Hypothèse plausible : fenêtre côté firmware entre deux `spi_slave_transmit()`

Le pattern actuel côté firmware reste :

```cpp
// Each iteration:
//   1. buildStatusFrame(tx)
//   2. spi_slave_transmit()
//   3. handleFrame(rx)
//
// There is a brief window between transmit() returning and the next call
// where no transaction is queued.
```

Cette fenêtre explique bien au moins une partie des erreurs observées :

- `0x0000` et `zero_rx` sont cohérents avec un esclave non prêt ;
- un `flush` ou un `get_status` envoyé pendant cette fenêtre peut être perdu.

Cette hypothèse est compatible avec le code et avec les symptômes observés.

### 1.3 Hypothèse non prouvée : "erreur DMA" ou décalage d'un octet

Le code actuel contient des commentaires qui attribuent une ancienne corruption à :

- un pattern `queue_trans/get_trans_result`,
- un décalage DMA d'un octet,
- une signature `bad magic: 0x0150`.

Après vérification de l'historique :

- ces commentaires ont été introduits dans le commit `d4f6b41` ;
- le dépôt ne contient pas de version historisée de `comm_interface.cpp` utilisant réellement `spi_slave_queue_trans()` ;
- les logs disponibles dans ce projet ne montrent pas `0x0150`, mais `0x0000`, `0x0350`, `0x24F5`, `0x2DF5`.

Conclusion : la corruption SPI est avérée, mais l'explication précise "bug DMA avec décalage d'un octet" n'est pas démontrée par ce dépôt.

### 1.4 Constat confirmé : le chemin de `flush` host augmentait la pression SPI au mauvais moment

Avant correction, `stream_all()` pouvait émettre le `flush` demandé sur le même cycle de streaming que l'arrêt sur endstop, y compris sur le chemin d'arrêt.

Ce point était bien un problème logiciel côté host : il augmentait la charge SPI exactement au moment où la transition endstop rendait le lien plus fragile.

---

## 2. Correctif sûr appliqué

### 2.1 Host streamer — flush ordonnancé sur le chemin d'arrêt

**Fichier :** `src/rpi/transport/streamer.py`

Le correctif appliqué ne prétend pas supprimer toute corruption SPI. Il réduit la pression au moment critique en sérialisant le `flush` via un helper unique sur les chemins d'arrêt.

```python
def _flush_requested_stop(self) -> None:
    if self._flush_sequence_requested is None:
        return
    # Let the firmware finish publishing the stop/endstop status before
    # sending the explicit flush request on the same SPI link.
    time.sleep(self._poll_interval_s * 2.0)
    self.flush_until(self._flush_sequence_requested)
    self._flush_sequence_requested = None
```

Et dans `stream_all()` :

```python
while True:
    if self._stop_requested:
        self._flush_requested_stop()
        break

    ...

    if self._check_stall(status):
        self._flush_requested_stop()
        break

    if self._check_endstop(status):
        self._flush_requested_stop()
        break
```

Ce correctif est déjà intégré côté host.

### 2.2 Ce que ce correctif prouve, et ce qu'il ne prouve pas

Ce correctif prouve qu'il existait au moins un problème d'ordonnancement côté host autour du `flush` post-endstop.

Il ne prouve pas à lui seul :

- que la cause restante est exclusivement côté firmware ;
- ni que le problème restant est spécifiquement un bug DMA.

---

## 3. Pistes non appliquées

### 3.1 Refonte firmware avec transaction pré-queue

Piste envisagée : réarmer le slave avant de traiter la frame reçue afin de réduire la fenêtre sans transaction prête.

Cette piste reste expérimentale. Elle peut être pertinente, mais elle n'est pas validée par l'historique du dépôt ni par des captures matérielles.

À ce stade, elle doit être traitée comme un prototype à tester sur banc, pas comme un correctif certain.

État actuel : le prototype existe désormais dans le code firmware, derrière un drapeau explicite.

Points d'attention du prototype :

- il met `slave_cfg.queue_size = 2` ;
- il introduit un second couple de buffers DMA RX/TX ;
- il peut retarder la visibilité du `STATUS` ou de l'ACK d'une transaction d'un transfert supplémentaire par rapport au chemin de production ;
- il n'élimine pas formellement toute fenêtre vide si le traitement d'une frame prend plus longtemps qu'un transfert SPI complet.

### 3.2 Réduction du délai inter-frame côté host

Piste envisagée : réduire `delay_usecs` de 700 µs à 50 µs dans `src/rpi/transport/spi_transport.py`.

Cette modification n'est pas retenue tant que la stratégie firmware n'est pas stabilisée, car :

- les 700 µs actuels sont une contremesure empirique ;
- réduire ce délai sans supprimer la fenêtre côté firmware peut empirer les `zero_rx`.

---

## 4. Lecture correcte des logs

```
bad magic: 0x0000
bad magic: 0x0350
bad magic: 0x24F5
bad magic: 0x2DF5
host/firmware buffer desync suspected
```

Interprétation défendable :

- `0x0000` suggère fortement une transaction lue alors que l'esclave n'avait rien de prêt ;
- `0x0350` peut être compatible avec un header décalé ou tronqué, mais ne prouve pas à lui seul un bug DMA ;
- `0x24F5` et `0x2DF5` ressemblent davantage à une corruption ou un mauvais alignement générique qu'à une signature unique ;
- `buffer desync` signifie que le host croit encore avoir des segments en vol alors que l'état firmware ne reflète plus ce flux.

---

## 5. Conclusion

Position techniquement défendable après revue du code et de l'historique :

- la corruption SPI est réelle ;
- une fenêtre de réarmement côté firmware est une hypothèse plausible et cohérente avec `zero_rx` ;
- le chemin de `flush` côté host était aussi un facteur aggravant, et ce point a été corrigé ;
- l'explication "bug DMA avéré avec décalage d'un octet" n'est pas suffisamment prouvée par ce dépôt.

En conséquence, la prochaine étape prudente n'est pas de déployer une grosse refonte firmware, mais de :

1. valider le correctif host sur le matériel ;
2. instrumenter le lien SPI ;
3. capturer une preuve matérielle ou logicielle avant de conclure à un bug DMA précis.