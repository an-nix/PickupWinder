# PickupWinder Python Host Architecture

Ce document décrit l’architecture globale du code hôte Python de PickupWinder, les responsabilités des principaux packages et classes, ainsi que des points critiques sur la conception actuelle.

## 1. Structure générale

Le code `src/rpi` se répartit en trois grandes familles de responsabilités :

- `jsonrpc/` : transport et dispatch JSON-RPC sur socket Unix
- `transport/` : communication SPI vers l’ESP32 et streaming des segments
- `motion/` : génération de mouvement et exécution des mouvements
- `core/` : configuration, état partagé, et cibler l’application
- `winding/` : définition des programmes de bobinage haut niveau
- `examples/` : scripts de démonstration / clients RPC

Le point d’entrée de l’application de contrôle est `src/rpi/winding_main.py`.

## 2. Point d’entrée : `src/rpi/winding_main.py`

### Rôle

- instancie `AppConfiguration`
- ouvre le transport SPI (`Esp32SpiTransport`)
- initialise l’état d’axe (`AxisState`)
- démarre le moteur de mouvement (`WindingEngine`)
- enregistre les méthodes RPC de la couche winding
- démarre le serveur `JsonRpcServer`

### Composants clés

- `AppConfiguration` : lecture des paramètres de la machine et des limites
- `Esp32SpiTransport` : objet SPI physique vers l’ESP32
- `WindingEngine` : exécutant des commandes motion
- `AppRpcHandler` + `_register_winding_rpc_methods` : pont entre RPC et engine
- `JsonRpcServer` : socket JSON-RPC et notifications d’événements

### Observations

- `winding_main.py` reste le composant de composition principal.
- Il connaît directement `WindingEngine`, `Esp32SpiTransport` et la structure des méthodes RPC.
- L’enregistrement RPC est centralisé ici, mais la logique métier reste dans `motion/engine.py`.

## 3. RPC : `jsonrpc/`

### `jsonrpc/rpc_server.py`

Rôle : serveur JSON-RPC minimaliste sur socket Unix.

- accepte un seul client à la fois
- lit les requêtes JSON ligne par ligne
- appelle `RpcHandler.dispatch()`
- renvoie les réponses JSON-RPC
- pousse des notifications `winding.event` via l’`EventBus`

Critique :

- la logique est bien séparée du métier. `RpcServer` ne connaît pas les méthodes.
- le design est simple et déterministe.
- l’absence de multiplexage de clients est acceptable pour cette application, mais limite l’interopérabilité.

### `jsonrpc/handlers.py`

- `RpcHandler` est un dispatcher simple par nom de méthode.
- `AppRpcHandler` expose une base minimale (`ping`, `status`, `shutdown`, `config`).

Observations :

- `AppRpcHandler` est conçu comme un conteneur de méthodes, pas comme une couche métier.
- la gestion des paramètres est basique, mais adaptée à un service interne.

## 4. Transport SPI : `transport/spi_transport.py`

### Rôle

- encapsule `spidev` et la construction/lecture de trames SPI fixes
- fournit les primitives de haut niveau : `send_segment_block`, `get_status`, `arm_endstop`, etc.

### Remarques

- la classe reste un wrapper « thin » sur la couche matériel.
- elle gère la numérotation de séquence et la conversion de trames.
- c’est la bonne place pour isoler la logique SPI du reste du host.

## 5. Streaming des segments : `transport/streamer.py`

### Rôle

- exécute la boucle d’envoi de segments multi-axes vers l’ESP32
- influence le débit selon la consommation du planner firmware
- applique des seuils de lookahead et de pression planner
- gère la détection d’underrun et de fin prématurée

### Points forts

- supporte un modèle de streaming poussé similaire à Klipper
- utilise un générateur `MultiAxisSegmentGenerator` pour produire les segments
- permet d’équilibrer la profondeur de buffer et le débit SPI

### Critique

- la logique de `MultiAxisRampStreamer` est dense et dispose de beaucoup d’état interne.
- la configuration des seuils est codée en dur dans la classe.
- l’idée de conserver un seul flux SPI/segment est bonne, mais l’intégration très spécifique au firmware demande une attention de test accrue.

## 6. Mouvement et génération : `motion/`

### 6.1 `motion/engine.py`

`WindingEngine` est le cœur d’exécution.

Responsabilités :

- gérer le `MoveQueue`
- exposer les API de commande (`submit_program`, `request_stop`, `wound_run`, `jog`, `run_axis`, etc.)
- porter l’état `EngineState`
- utiliser `EventBus` pour notifier les événements

Observations importantes :

- `WindingEngine` est la façade métier principale pour le contrôleur.
- il orchestre l’envoi vers le firmware mais ne produit pas directement les segments.
- depuis la dernière modification, il contient également la logique de calcul des temps de rampe pour `run_axis`.

Critique :

- le calcul de rampe est ici implémenté comme helper privé `_compute_ramp_times`.
- on pourrait extraire ce calcul dans un module de configuration / utilitaire commun.
- l’ajout de `run_axis` est cohérent mais le fait qu’il soit dans le moteur montre que l’`engine` porte à la fois l’orchestration et une partie de la planification.

### 6.2 `motion/move.py`

Définition de l’arborescence `Move` :

- `Move` : abstraction de base
- `RampMove` : move multi-axes avec rampes trapézoïdales
- `JogMove` : déplacement fixe en pas à vitesse donnée
- `HomingMove` : séquence de phase d’homing
- `WoundMove` : mouvement de bobinage synchronisé

Rôle :

- fournir l’interface `segments()` pour la production de `MultiAxisSegment`
- encapsuler le type de mouvement et ses paramètres

Critique :

- l’approche est propre et extensible.
- la séparation entre `JogMove` et `RampMove` est pertinente.
- `WoundMove` mélange la logique de géométrie de bobine avec la génération de segments, ce qui peut rendre la validation plus complexe.

### 6.3 `motion/move_queue.py`

Rôle :

- exécuter les `Move` en FIFO
- démarrer/arrêter proprement l’exécution
- créer une `MultiAxisRampStreamer` par mouvement
- mettre à jour l’état des axes (`AxisState`)
- traiter l’homing comme un cas spécial

Points clés :

- `MoveQueue` est la vraie boucle d’exécution
- il est responsable de la transition entre `Move` et `Stream`.

Critique :

- le dispatch `_execute_move` est simple et propre.
- l’état de queue et l’historique sont gérés de façon cohérente.
- cependant, le lien entre `Move` et `streamer` reste étroit, ce qui rend le test de ces composants dépendant du transport.

### 6.4 `motion/ramp_config.py`

Rôle :

- décrire une rampe d’axe (`target_rpm`, `accel_s`, `cruise_s`, `decel_s`, etc.)
- exposer un profil `TrapezoidalMotionProfile`
- calculer des conversions `hz_at_time`, `steps_at`, `step_delta`

C’est le bon endroit pour représenter une rampe prête à être générée.

### 6.5 `motion/__init__.py`

Rôle :

- exposer un package de façade
- différer l’import des classes pour éviter les boucles de dépendance

## 7. Configuration et état partagé : `core/`

### `core/config.py`

Rôle :

- stocker les paramètres de la machine
- convertir les limites de vitesse et d’accélération en unités utiles

### `core/shared_state.py`

Rôle :

- représenter l’état partagé de l’axe et de l’engine
- fournir un emplacement central pour les snapshots d’état

### `core/app.py`

Rôle apparent :

- définir `WinderApp` comme une application host alternative

Critique :

- `core/app.py` existe mais n’est pas utilisé par `winding_main.py`.
- cela suggère un fragment de conception non intégré ou une architecture parallèle qui n’est pas consolidée.
- c’est un point de dette technique à clarifier.

## 8. Flux d’exécution

1. `winding_main.py` démarre l’application
2. `Esp32SpiTransport` ouvre le bus SPI
3. `WindingEngine` démarre son thread et le `MoveQueue`
4. `JsonRpcServer` attend les connexions client
5. un client appelle `winding.run_axis`, `winding.jog`, `winding.wound_run`, etc.
6. `AppRpcHandler` mappe l’appel à une méthode d’`engine`
7. `engine` crée un `Move` et l’enqueue dans `MoveQueue`
8. `MoveQueue` crée un `MultiAxisRampStreamer`
9. `streamer` produit et envoie des segments via `Esp32SpiTransport`
10. l’ESP32 exécute les segments et renvoie le statut

## 9. Critique architecturale

### Forces

- bonne séparation du transport SPI et de la logique de mouvement
- architecture par commande (`Move`) claire et extensible
- JSON-RPC est bien isolé du métier
- le streamer applique un modèle de backpressure réaliste

### Faiblesses

- il existe encore des doublons conceptuels entre `core/app.py` et `motion/engine.py`
- `run_axis` et son calcul de rampe sont implémentés dans le moteur alors que la logique devrait être dans un module de planification dédié
- `winding_main.py` reste le point de composition : c’est normal, mais il centralise trop d’assemblage
- la méthode `RpcHandler.dispatch` est flexible, mais ne valide pas les types au-delà des erreurs `TypeError`
- `AppRpcHandler` expose des méthodes RPC via fonctions internes plutôt qu’une couche objet dédiée, ce qui complexifie la lecture

### Recommandations

- consolider l’architecture en choisissant un seul modèle d’application hôte : soit `WinderApp`, soit `WindingEngine`
- extraire le calcul de `accel_s/cruise_s/decel_s` dans un utilitaire commun ou dans `ramp_config.py`
- documenter clairement quelles méthodes RPC sont supportées et quelles sont celles du controller vs. celles du winding program
- limiter les responsabilités de `MoveQueue` à l’exécution et déplacer la logique de création de streamers vers un builder distinct si besoin
- envisager un `RpcMethodRegistry` plus typé pour éviter des erreurs de paramètres silencieuses

## 10. Conclusion

L’architecture hôte est globalement saine et découple correctement :

- transport bas niveau (SPI)
- streaming et bufferisation
- génération de segments
- orchestration de mouvements
- exposition RPC

La principale critique est la présence de zones de chevauchement (`core/app.py`, calcul de rampe) et un manque d’unification claire entre la configuration machine et la génération de profil.

Une refactorisation future pourrait viser à :

- rendre `WindingEngine` plus passif en déléguant la construction des `Move`
- rendre `core/` le véritable point de configuration partagée
- simplifier `winding_main.py` en lui donnant uniquement un rôle de bootstrap
