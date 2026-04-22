# Migration SPI mode 1 et revue des correctifs herites du mode 0

## Resume executif

La pile SPI active a ete basculee en mode `1` des deux cotes :

- firmware ESP32 : `slave_cfg.mode = 1` dans `src/esp32/src/comm_interface.cpp`
- host Python : `Esp32SpiTransport(..., mode=1)` et valeur par defaut `mode=1`

Cette migration retire la dependance au mode `0`, qui etait devenu un point de fragilite avec le slave SPI DMA de l'ESP32. Je n'ai pas supprime en bloc les garde-fous ajoutes pendant l'enquete precedente : certains restent utiles meme en mode `1`, d'autres doivent simplement etre revalides au banc avant d'etre alleges.

## Fichiers modifies

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

1. parse la requete recue,
2. reconstruit la prochaine trame STATUS,
3. rearme la transaction slave DMA.

**Verdict**

- **A garder pour l'instant**.
- Ce garde-fou a ete introduit comme palliatif pendant l'enquete mode `0`, mais il protege aussi un comportement reel du chemin simple : la trame suivante n'est pas armee instantanement.
- En mode `1`, il est probablement plus conservateur que necessaire, mais le supprimer maintenant reviendrait a melanger migration electrique et retuning de timing.

**Commentaire**

Le correctif est couteux en latence mais simple et robuste. La bonne suite n'est pas de le supprimer a l'aveugle, mais de le reduire progressivement apres mesures materielles.

### 2. `esp_rom_delay_us(2)` apres `buildStatusFrame()` dans `src/esp32/src/comm_interface.cpp`

**But**

Ajouter une mini-garde entre la reconstruction du buffer TX et le rearmement DMA suivant.

**Verdict**

- **A revalider, mais a garder provisoirement**.
- Ce delai est tres faible et son cout est negligeable.
- En mode `1`, il se peut qu'il ne soit plus determinant, mais il reste un garde-fou peu intrusif sur un chemin sensible.

**Commentaire**

Ce n'est pas une preuve de correction architecturale. C'est un stabilisateur local qu'il vaut mieux mesurer avant retrait.

### 3. Boucle firmware simple `spi_slave_transmit()` avec `SPI_EXPERIMENTAL_PREQUEUE = false`

**But**

Rester sur un seul transfert arme a la fois, avec une latence ACK `N+1`, au lieu d'introduire un ping-pong a `queue_size=2`.

**Verdict**

- **A garder**.
- Ce n'est pas un correctif specifique au mode `0`, mais c'est le chemin de production le plus comprehensible et le plus deterministe.

**Commentaire**

Le pre-queue traite surtout la dead-time entre transactions. Il ne corrige pas un mauvais mode SPI et ajoute une latence ACK supplementaire.

### 4. Preconstruction de la premiere trame STATUS puis reconstruction immediate de la suivante

**But**

Reduire la fenetre pendant laquelle aucun TX valable n'est pret cote slave, donc limiter les reponses tout-zero ou corrompues si le Pi enchaine tres vite.

**Verdict**

- **A garder**.
- Ce comportement reste utile en mode `1` car il traite la structure du pipeline `spi_slave_transmit()`, pas uniquement le choix du mode electrique.

**Commentaire**

Ce point est structurellement sain et ne devrait pas etre retire.

### 5. Retries host sur `bad magic`, `bad response CRC` et `zero_rx`

**But**

Absorber les corruptions ou trous transitoires sans casser la session applicative.

**Verdict**

- **A garder**.
- Le lien SPI restera soumis a des aleas physiques, meme en mode `1`.
- Ces retries rendent le host resilient sans casser les invariants de protocole.

**Commentaire**

Ce correctif ne doit pas etre confondu avec une rustine pour mode `0`. C'est une vraie strategie de robustesse transport.

### 6. `wait_for_request_result()` et confirmation par `last_rx_sequence`

**But**

Tenir compte du pipeline d'ACK `N+1` et eviter d'interpreter la reponse full-duplex immediate comme l'ACK certain de la requete courante.

**Verdict**

- **A garder absolument**.
- Ce n'est pas un palliatif au mode `0` ; c'est une regle fondamentale du protocole.

**Commentaire**

La migration en mode `1` ne change rien a cet invariant.

### 7. Fallback de statut stale dans `poll_status()`

**But**

Continuer a faire vivre la logique host pendant une panne telemetrique breve au lieu de casser immediatement le streaming.

**Verdict**

- **A garder avec prudence**.
- Ce comportement est utile operationnellement, mais il peut masquer une degradation transport si on s'en contente trop longtemps.

**Commentaire**

Ce n'est pas une correction electrique. C'est un compromis applicatif entre continuite de service et visibilite des erreurs.

### 8. Compteurs et logs `bad_magic`, `bad_crc`, `zero_rx`, `reopens`

**But**

Rendre la qualite du lien observable pendant les essais et en exploitation.

**Verdict**

- **A garder**.
- Ils sont encore plus utiles maintenant, car ils permettront de verifier objectivement le gain apres passage en mode `1`.

**Commentaire**

Sans ces metriques, il serait impossible de dire si la migration a reellement ameliore le lien ou si la stabilite percue vient d'autres garde-fous.

### 9. `vTaskDelay(1)` dans `src/esp32/src/motion_planner.cpp` quand le planner est idle

**But**

Eviter qu'une boucle vide ne monopolise inutilement le CPU et ne degrade le comportement du coeur 0 qui porte la tache SPI.

**Verdict**

- **A garder**.
- Le commentaire a ete assaini : il ne faut plus presenter ce sleep comme une preuve causale unique du bug `0x0150`, mais comme une mesure saine d'equite CPU.

**Commentaire**

Ce correctif releve plus de l'hygiene temps reel que du mode SPI lui-meme.

### 10. Note dans `src/rpi/transport/streamer.py` sur le pacing intra-boucle

**But**

Eviter d'ajouter des sleeps ad hoc supplementaires dans la boucle serree d'envoi sans re-mesurer le lien.

**Verdict**

- **Commentaire clarifie**.
- Le code ne dormait deja pas a cet endroit ; le commentaire precedent laissait croire le contraire.

**Commentaire**

Ce n'etait pas un correctif actif, mais une explication ambigue. Je l'ai remise en coherence avec le comportement reel.

## Ce que je n'ai volontairement pas retire

- le `delay_usecs=700`
- les deux `esp_rom_delay_us(2)`
- les retries et logs host
- le chemin simple sans pre-queue

La raison est simple : changer le mode SPI et supprimer en meme temps plusieurs garde-fous rendrait l'analyse impossible si la liaison se degrade encore. La migration actuelle isole le changement principal : le passage de `mode 0` a `mode 1`.

## Ce qui merite une revalidation materielle

1. mesurer `bad_magic`, `bad_crc` et `zero_rx` avant/apres migration ;
2. valider a la frequence nominale actuelle ;
3. reduire ensuite `delay_usecs` par paliers, par exemple `700 -> 300 -> 100`, seulement si les compteurs restent propres ;
4. tester ensuite la suppression des `esp_rom_delay_us(2)` si les captures restent stables.

## Conclusion

Le vrai correctif de fond est la coherence host/firmware en SPI mode `1`. Les autres correctifs presents dans le code ne doivent pas etre vus comme des erreurs a supprimer d'urgence : plusieurs sont des protections de pipeline ou de robustesse transport qui restent defendables meme apres la migration.