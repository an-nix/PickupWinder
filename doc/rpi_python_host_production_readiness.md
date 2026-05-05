# Audit Production Readiness - Python Host PickupWinder

**Perimetre** : `src/rpi/`  
**Date** : Avril 2026  
**Objectif** : evaluer les ecarts restants pour faire passer le host Python d'un niveau prototype avance / pre-production a un niveau production / industrial ready.

---

## Resume executif

Le host Python a deja plusieurs fondations solides pour un systeme de motion temps reel : une barriere de serialisation unique via `MoveQueue`, un protocole SPI robuste avec ACK explicite, une couverture de tests correcte sur la generation/consommation de segments, et des ameliorations recentes sur les diagnostics transport, les transitions d'etat atomiques et la centralisation partielle du stop.

En l'etat, le systeme n'est toutefois **pas encore industrial ready**. Le principal ecart n'est plus la logique de mouvement elle-meme ; il se situe maintenant sur la **surete d'exploitation**, la **robustesse du plan de controle RPC**, la **formalisation des modes d'arret**, la **visibilite operateur**, et la **capacite de recuperation apres incident**.

### Evaluation synthese

- **Niveau actuel estime** : prototype avance / pre-production
- **Ce qui est deja bon** : motion streaming, protocole SPI, homing, validations de configuration de base, tests unitaires sur le coeur motion
- **Ce qui bloque encore la prod** : contrat d'arret incomplet, arret non prouve safe de bout en bout, plan de controle RPC trop fragile, manque de watchdog/health model, config non operationalisee, pas de journal d'audit ni de reprise

---

## Points forts deja presents

### 1. Serialisation claire du mouvement

Le mouvement passe par une unique barriere de serialisation via `MoveQueue`, ce qui simplifie la raison sur l'ordre des commandes et limite les races sur la couche motion.

Fichiers principaux :

- `src/rpi/motion/move_queue.py`
- `src/rpi/core/engine.py`
- `src/rpi/winding/service.py`

### 2. Transport SPI deja plus robuste qu'un simple prototype

Le transport gere :

- la taille fixe de frame
- les retries sur CRC/magic invalides
- l'usage de status stale sous certaines conditions transitoires
- l'exposition de diagnostics cumules

Fichier principal :

- `src/rpi/transport/spi_transport.py`

### 3. Diagnostics motion riches cote streamer

Le streamer maintient deja de nombreux signaux utiles : planner pressure, underrun, flush, endstop, stall timeout, segments dropped.

Fichier principal :

- `src/rpi/transport/streamer.py`

### 4. Base de tests utile sur le coeur motion

La suite de tests couvre deja une bonne partie du comportement de `MultiAxisRampStreamer`, du homing, des conversions de config et de plusieurs regressions critiques recemment corrigees.

Fichiers principaux :

- `tests/test_motion_v3.py`
- `tests/test_adaptive_winding.py`
- `tests/test_critique_v10_regressions.py`

---

## Resultats de l'analyse

### 1. Le contrat d'arret n'est pas encore formalise au bon niveau

Le point le plus important pour un grade industriel n'est pas seulement de "stopper", mais de **definir explicitement plusieurs politiques d'arret** et de garantir leur semantique.

Etat actuel observe :

- il existe un chemin `pause_session()` cote winding adaptatif
- il existe un chemin `request_stop()` / `stop()` pour l'arret operationnel
- le code sait deja conserver certains axes actives via `keep_enabled_axes`
- la politique exacte de conservation / invalidation des axes n'est pas encore un contrat systeme explicite

Fichiers concernes :

- `src/rpi/winding/service.py`
- `src/rpi/core/coordinator.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/transport/streamer.py`
- `src/rpi/motion/axis_state.py`

### Conclusion

Il faut formaliser au niveau architecture une **matrice de modes d'arret**. Sans cela, il sera toujours difficile de prouver qu'un arret preserve bien l'autorite de position et l'etat des axes comme attendu.

---

### 2. L'arret du process n'est pas encore materiellement "fail-safe"

Le chemin d'arret du process est encore trop permissif pour un usage industriel.

Etat actuel observe :

- `WinderApplication.stop()` arrete les services puis ferme le transport
- `engine.stop()` et `move_queue.stop()` font des `join(timeout=...)` mais ne remontent pas d'echec si le thread reste vivant
- `MultiAxisRampStreamer.stream_all()` journalise une erreur si `_disable_axes()` echoue, mais ne force pas une escalation d'etat
- aucun chemin final ne prouve qu'un `disable_all()` ou une mise en securite equivalente a bien ete obtenue avant retour au caller

Fichiers concernes :

- `src/rpi/app/runtime.py`
- `src/rpi/core/engine.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/transport/streamer.py`
- `src/rpi/transport/spi_transport.py`

### Conclusion

Le systeme peut aujourd'hui "s'arreter" du point de vue logiciel sans qu'un **etat mecanique final prouve** soit garanti. Pour de la prod, le chemin d'arret doit etre verifiable, escaladable et explicite.

---

### 3. Le moteur principal peut mourir sans frontiere d'exception top-level

La boucle `_run()` de `WindingEngine` n'encadre pas `_execute_program()` dans un `try/except` top-level.

Impact potentiel :

- mort silencieuse du thread moteur
- process toujours vivant mais plus d'orchestration motion
- etat RPC ambigu : l'application repond encore, mais le coeur du controle ne tourne plus

Fichier concerne :

- `src/rpi/core/engine.py`

### Conclusion

Tout thread critique doit avoir une frontiere d'exception de dernier niveau, avec :

- log complet
- transition vers un etat `FAULT`
- publication d'un evenement critique
- exposition du statut de vie du worker

---

### 4. Le plan de controle JSON-RPC reste trop fragile pour l'exploitation

Le serveur RPC est encore adapte a un pilotage simple, pas a un environnement de supervision robuste.

Problemes observes :

- serveur mono-client (`listen(1)`)
- handlers executes dans le thread de connexion
- pas de timeout d'execution par appel metier
- buffer de reception sans limite avant le prochain `\n`
- `RpcHandler.dispatch()` utilise un fallback sur `TypeError` qui peut masquer une vraie erreur interne

Fichiers concernes :

- `src/rpi/jsonrpc/rpc_server.py`
- `src/rpi/jsonrpc/handlers.py`
- `src/rpi/jsonrpc/protocol.py`

### Conclusion

La couche RPC doit etre consideree comme un **plan de controle critique**, pas seulement comme une interface pratique. Elle doit etre defensive, time-bound et observable.

---

### 5. Le health model est encore incomplet

Le code expose deja `winder.status`, `winding.status` et des diagnostics transport, mais il manque encore une vue claire de la sante systeme.

Manques notables :

- pas d'etat `healthy / degraded / faulted`
- pas de liveness explicite des threads critiques
- pas de watchdog host qui escalade un `last_status_age_s` anormal
- `EventBus.dropped_count` n'est pas remonte dans le status operateur
- perte de homing invalidee localement, mais sans veritable politique de supervision continue

Fichiers concernes :

- `src/rpi/core/status.py`
- `src/rpi/core/events.py`
- `src/rpi/core/lateral.py`
- `src/rpi/transport/spi_transport.py`
- `src/rpi/app/runtime.py`

### Conclusion

Il manque encore une couche de **sante systeme exploitable**, utile autant pour l'operateur que pour un superviseur externe.

---

### 6. La configuration n'est pas encore une vraie source de verite operationnelle

`AppConfiguration` est mieux validee qu'avant, mais le mode d'exploitation reste faible.

Problemes observes :

- le runtime part directement sur `AppConfiguration()` si rien n'est injecte
- `ConfigurationManager` n'est pas le point d'entree standard du runtime
- les cles inconnues du JSON sont ignorees silencieusement au chargement
- pas de version de schema ni de controle strict des profils de configuration
- pas de reload encadre ou de diff appliquee proprement

Fichiers concernes :

- `src/rpi/core/config.py`
- `src/rpi/app/runtime.py`

### Conclusion

Pour la prod, la configuration doit etre :

- persistante
- stricte
- versionnee
- chargee explicitement
- tracable

---

### 7. Il manque un vrai audit trail et un logging d'exploitation

Le logging actuel est suffisant pour du debug manuel, mais pas pour de l'exploitation industrielle.

Problemes observes :

- `logging.basicConfig(...)` simple dans `winding_main.py`
- pas de correlation d'une requete RPC a ses effets
- pas de journal d'audit immutable des commandes operateur
- pas de journal structure pour ingestion dans un superviseur

Fichiers concernes :

- `src/rpi/winding_main.py`
- `src/rpi/jsonrpc/rpc_server.py`
- `src/rpi/jsonrpc/winding_handler.py`
- `src/rpi/core/engine.py`
- `src/rpi/winding/service.py`

### Conclusion

Le systeme manque encore de **tracabilite d'exploitation**.

---

### 8. La recuperation apres incident n'est pas couverte

Les longues sessions de bobinage restent vulnerables a toute interruption process ou alimentation.

Problemes observes :

- pas de checkpoint de session
- pas de reprise de programme classique apres crash
- pas de journal d'intention de commande
- pas de "cold start recovery" qui detecte un arret precedent incomplet

Fichiers concernes :

- `src/rpi/core/shared_state.py`
- `src/rpi/winding/adaptive.py`
- `src/rpi/winding/service.py`
- `src/rpi/core/engine.py`

### Conclusion

L'absence de reprise est acceptable en prototype, beaucoup moins en environnement de production.

---

### 9. Les tests sont bons sur le motion, moins bons sur le cycle de vie applicatif

La qualite de test est honorable sur le coeur motion, mais il manque encore plusieurs couches de validation indispensables avant prod.

Manques principaux :

- tests du serveur RPC sous charge / clients concurrents / trames invalides
- tests du cycle de vie `WinderApplication.start()/stop()`
- tests de worker qui meurt en cours de run
- tests de faute SPI prolongee
- tests de coupure/reprise sur chemin `pause` et chemin `stop`
- tests HIL / soak tests longue duree

Fichiers concernes :

- `tests/test_motion_v3.py`
- `tests/test_critique_v10_regressions.py`
- absence de suites equivalentes pour `rpc_server.py`, `runtime.py`, `winding_main.py`

### Conclusion

La prochaine marche qualitative n'est pas seulement du code : c'est aussi une **strategie de validation d'exploitation**.

---

## Cible de semantique d'arret

Le point d'attention fourni doit devenir une exigence d'architecture explicite.

### Regle de base

Le systeme doit distinguer au minimum :

1. **Pause**
2. **Stop**

Un **E-stop** materiel/transport peut exister en plus, mais il ne doit pas etre confondu avec les deux modes ci-dessus.

### Contrat cible recommande

| Mode | But | Comportement motion | Politique d'axes | Autorite de position | Reprise |
|---|---|---|---|---|---|
| `pause` | interruption controlee et reversible | deceleration controlee jusqu'a vitesse nulle, sans casser l'etat de reprise | **les axes homed / autoritaires restent alimentes** ; seuls certains axes sans homing ou sans position fiable peuvent etre desactives si c'est explicitement autorise | conservee pour les axes gardes sous controle | oui |
| `stop` | arret operationnel complet | arret du mouvement, purge/clear de queue, fin de session en cours | politique plus stricte ; les axes peuvent etre desactives selon la strategie surete | invalider toute position qui n'est plus prouvable | non, sauf reprise explicite outillee |
| `e-stop` (optionnel mais recommande) | securite / incident | coupure immediate ou commande firmware dediee | selon politique surete machine | non garantie | non |

### Consequence directe sur le backlog

Toute evolution du stop doit partir d'une **matrice axe x mode**. Exemple de questions a trancher explicitement :

- quels axes restent alimentes en `pause` ?
- quels axes gardent une position autoritaire apres `pause` ?
- quels evenements invalident la position meme en `pause` ?
- quand un `stop` degrade-t-il automatiquement en `e-stop` ?
- quel est le comportement si un axe perd son enable pendant une pause ?

---

## Backlog priorise

### Suivi d'implementation

**Sprint 1 / Vague 1** : implemente en code dans cette passe.

- `BG-01` : modele explicite `pause` / `stop` / `emergency_stop`, propagation des stop plans, retention des axes homed en pause.
- `BG-02` : shutdown applicatif verifiable, joins non silencieux, `safe_shutdown()` transport avec verification `enabled_mask` / `running_mask`.
- `BG-03` : frontieres d'exception top-level et health reporting sur engine, move queue, adaptive winding et serveur RPC.
- `BG-04` : durcissement JSON-RPC avec binding strict, taille de requete bornee, timeout d'appel et hygiene du socket path.

**Validation** : des tests unitaires cibles ont ete ajoutes, mais leur execution n'a pas ete lancee ici car l'appel terminal a ete explicitement ignore pendant cette session.

### P0 - Bloquants production

### ~~BG-01 - Formaliser et implementer la matrice `pause` / `stop`~~

**Statut** : implemente dans cette passe.

**Objectif** : rendre l'arret deterministic et prouvable.

**Code cible** :

- `src/rpi/core/coordinator.py`
- `src/rpi/core/shared_state.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/transport/streamer.py`
- `src/rpi/winding/service.py`
- `src/rpi/core/engine.py`

**Travail attendu** :

- introduire un modele explicite des modes d'arret
- definir une politique `keep_enabled_axes` par mode
- separer clairement `pause`, `stop`, et si besoin `emergency_stop`
- documenter les transitions d'etat associees
- verifier que `pause` preserve les axes homed qui doivent le rester

**Critere d'acceptation** :

- la semantique `pause` vs `stop` est documentee et testee
- un axe homed conserve selon la politique ne perd pas sa position sur `pause`
- un axe non autoritaire peut etre desactive sans casser la reprise si et seulement si la politique le permet
- les tests couvrent au minimum `pause`, `resume`, `stop`, `fault`, perte d'enable pendant pause

### ~~BG-02 - Rendre le shutdown du process materiellement verifiable~~

**Statut** : implemente dans cette passe.

**Objectif** : garantir qu'un `stop()` d'application laisse la machine dans un etat final connu.

**Code cible** :

- `src/rpi/app/runtime.py`
- `src/rpi/core/engine.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/transport/streamer.py`
- `src/rpi/transport/spi_transport.py`

**Travail attendu** :

- verifier tous les `join(timeout)` et remonter un echec si un worker reste vivant
- definir une sequence finale explicite de mise en securite transport
- escalader en `FAULT` ou `E-STOP REQUIRED` si le cleanup final n'est pas confirme
- exposer le resultat du shutdown au caller et aux logs

**Critere d'acceptation** :

- `WinderApplication.stop()` ne retourne pas silencieusement si un thread critique reste vivant
- un cleanup transport final est execute ou un echec explicite est remonte
- les tests de shutdown sous charge et avec streamer bloque existent

### ~~BG-03 - Mettre des frontieres d'exception top-level sur tous les workers critiques~~

**Statut** : implemente dans cette passe.

**Objectif** : interdire la mort silencieuse d'un thread critique.

**Code cible** :

- `src/rpi/core/engine.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/jsonrpc/rpc_server.py`
- `src/rpi/winding/service.py`

**Travail attendu** :

- encadrer chaque boucle worker par un `try/except` de dernier niveau
- basculer dans un etat critique exploitable
- publier un evenement critique
- remonter la sante de chaque worker dans le status

**Critere d'acceptation** :

- aucune exception inattendue ne peut tuer silencieusement un worker critique
- le status expose `thread_alive` / `thread_faulted` pour les composants critiques

### ~~BG-04 - Durcir le plan de controle JSON-RPC~~

**Statut** : implemente dans cette passe.

**Objectif** : faire du RPC un canal de controle robuste.

**Code cible** :

- `src/rpi/jsonrpc/rpc_server.py`
- `src/rpi/jsonrpc/handlers.py`
- `src/rpi/jsonrpc/protocol.py`
- `src/rpi/jsonrpc/winding_handler.py`

**Travail attendu** :

- limite de taille de requete
- timeout metier par appel RPC
- binding strict des signatures sans fallback ambigu sur `TypeError`
- meilleure gestion des sockets stale et du path socket
- separation eventuelle des appels longs et du thread de connexion

**Critere d'acceptation** :

- une requete malformee ou trop grosse ne peut pas faire grossir la memoire indefiniment
- un appel bloquant ne monopolise pas indefiniment le plan de controle
- une vraie erreur interne n'est plus reinterpretee comme un simple probleme de params

---

### P1 - Durcissement exploitation

### BG-05 - Introduire un vrai health model et un watchdog host

**Objectif** : rendre l'etat systeme interpretable par un operateur ou un superviseur.

**Code cible** :

- `src/rpi/core/status.py`
- `src/rpi/transport/spi_transport.py`
- `src/rpi/core/events.py`
- `src/rpi/core/lateral.py`
- `src/rpi/app/runtime.py`

**Travail attendu** :

- ajouter `healthy`, `degraded`, `faulted`
- exposer la liveness des threads et l'age des derniers status
- remonter `EventBus.dropped_count`
- declencher une escalation si la telemetrie SPI reste stale trop longtemps

**Critere d'acceptation** :

- un superviseur externe peut distinguer un systeme sain, degrade, faulted
- un blocage SPI prolonge est visible et escalade en etat critique

### BG-06 - Faire de la configuration une source de verite stricte

**Objectif** : eviter toute derive de configuration silencieuse.

**Code cible** :

- `src/rpi/core/config.py`
- `src/rpi/app/runtime.py`

**Travail attendu** :

- imposer un chargement explicite depuis un fichier de config
- refuser les cles inconnues au lieu de les ignorer
- versionner le schema
- journaliser la config effective au demarrage

**Critere d'acceptation** :

- une faute de frappe dans le JSON empeche le demarrage
- le runtime sait afficher la version de schema et la config active

### BG-07 - Ajouter audit log et logging structure

**Objectif** : tracer qui a demande quoi, quand, et avec quel effet.

**Code cible** :

- `src/rpi/winding_main.py`
- `src/rpi/jsonrpc/rpc_server.py`
- `src/rpi/jsonrpc/winding_handler.py`
- `src/rpi/core/engine.py`
- `src/rpi/winding/service.py`

**Travail attendu** :

- correlation ID par commande RPC
- journal d'audit des ordres operateur
- logs structures pour ingestion externe
- journaux d'etat critiques et de transitions majeures

**Critere d'acceptation** :

- chaque ordre operateur peut etre retrace de l'entree RPC au resultat motion

### BG-08 - Construire une campagne de validation d'exploitation

**Objectif** : couvrir les zones qui restent peu testees.

**Code cible** :

- `tests/`

**Travail attendu** :

- tests RPC sous charge et input invalide
- tests lifecycle `start()/stop()`
- tests crash de worker
- tests telemetrie SPI stale / timeout / retry prolonge
- tests `pause` vs `stop`
- soak tests longue duree et, si possible, HIL

**Critere d'acceptation** :

- un lot minimal de tests d'exploitation tourne en CI
- les chemins de shutdown, pause, stop et fault sont couverts

---

### P2 - Recuperation et exploitation avancee

### BG-09 - Ajouter checkpoint et reprise outillee

**Objectif** : reduire la perte operationnelle sur longues sessions.

**Code cible** :

- `src/rpi/core/shared_state.py`
- `src/rpi/core/engine.py`
- `src/rpi/winding/service.py`
- `src/rpi/winding/adaptive.py`

**Travail attendu** :

- snapshots periodiques de progression
- reprise explicite et verifiee
- validation de compatibilite config/programme avant resume

**Critere d'acceptation** :

- reprise impossible sans verification de compatibilite
- reprise possible sur les cas explicitement supportes

### BG-10 - Separer supervision et controle, ou supporter plusieurs clients proprement

**Objectif** : eviter qu'un client unique soit un SPOF operateur.

**Code cible** :

- `src/rpi/jsonrpc/rpc_server.py`
- `src/rpi/core/status.py`

**Travail attendu** :

- multi-client lecture seule, ou
- canal de supervision separe, ou
- multiplexage propre entre telemetrie et commande

**Critere d'acceptation** :

- la perte du client principal n'empeche pas une autre console de supervision de se connecter

### BG-11 - Emballer le runtime comme un service exploitable

**Objectif** : rendre le host deployable et operable comme un vrai service machine.

**Code cible** :

- `src/rpi/winding_main.py`
- scripts / packaging / documentation d'exploitation

**Travail attendu** :

- mode service avec restart policy claire
- hooks startup/shutdown propres
- readiness / liveness checks
- documentation d'exploitation et de recuperation

**Critere d'acceptation** :

- le process peut etre gere par un superviseur type `systemd` avec semantique de sante claire

---

## Ordre recommande de realisation

### ~~Vague 1 - Bloquants de surete~~

1. ~~BG-01 - matrice `pause` / `stop`~~
2. ~~BG-02 - shutdown verifiable~~
3. ~~BG-03 - frontieres d'exception workers~~
4. ~~BG-04 - durcissement JSON-RPC~~

### Vague 2 - Mise sous controle exploitation

5. BG-05 - health model + watchdog
6. BG-06 - configuration stricte
7. BG-07 - audit log + logging structure
8. BG-08 - validation d'exploitation

### Vague 3 - Recuperation et exploitation longue duree

9. BG-09 - checkpoint / reprise
10. BG-10 - supervision multi-client
11. BG-11 - packaging service / exploitation

---

## Definition pratique de "production / industrial ready"

Le host Python pourra raisonnablement etre considere comme pret pour un usage industriel quand les conditions suivantes seront reunies :

1. les modes `pause` et `stop` sont specifiques, testes et documentes
2. un arret du process laisse la machine dans un etat final prouve ou remonte un echec explicite
3. aucun worker critique ne peut mourir silencieusement
4. la sante systeme est visible et monitorable
5. la configuration est stricte, persistante et tracable
6. chaque commande importante est journalisee et correlable
7. les scenarii de panne realistes sont testes
8. la machine peut etre exploitee via un vrai mode service

---

## Conclusion

Le code Python du host PickupWinder est aujourd'hui sur une base technique saine pour la motion et le streaming, mais il lui manque encore plusieurs briques de **surete d'exploitation** pour passer au niveau production / industriel.

Le prochain palier n'est plus un sujet de geometrie de bobinage ou de generation de segments. C'est un sujet de :

- contrat d'arret
- supervision
- tolerance aux fautes
- discipline de configuration
- auditabilite
- validation d'exploitation

En pratique, **les items P0 doivent etre consideres comme bloquants avant toute mise en service industrielle**.