# Revue architecturale — Python host PickupWinder

**Périmètre** : `src/rpi/` (44 fichiers, ~6 000 lignes)  
**Date** : Avril 2026  
**Statut des sources** : Code complet analysé

---

## Sommaire

1. [Vue d'ensemble](#1-vue-densemble)
2. [Problèmes critiques](#2-problèmes-critiques)
3. [Problèmes importants](#3-problèmes-importants)
4. [Améliorations structurelles](#4-améliorations-structurelles)
5. [Durcissement défensif](#5-durcissement-défensif)
6. [Dette technique mineure](#6-dette-technique-mineure)
7. [Récapitulatif par fichier](#7-récapitulatif-par-fichier)

---

## Statut de traitement

| Section | Statut | Décision appliquée |
|---|---|---|
| 2.1 | Fait | `winding.stop` demande maintenant systématiquement l'arrêt du service adaptatif et du moteur classique. |
| 2.2 | Fait | `_wait_for_move_queue()` pose toujours un fault puis appelle `MoveQueue.clear()` pour stopper le streamer actif sur timeout. |
| 2.3 | Fait | Quatre méthodes de transition atomiques ajoutées à `SharedState` (`transition_to_paused`, `transition_to_running`, `transition_to_idle_session`, `transition_to_stopping`). `AdaptiveWindingService._run_session()` et `request_stop()` les utilisent à la place des paires `set_engine_state` + `set_winding_session` séparées. |
| 2.4 | Fait partiellement | Le refresh existait déjà via `winding.status` et `require_homed()`. Cette passe ajoute une revalidation avant chaque layer classique et avant chaque chunk adaptatif. |
| 3.1 | Fait | `ConfigurationManager` a reçu une implémentation JSON minimale, la faute `save_configration` reste disponible comme alias de compatibilité. |
| 3.2 | Fait | `AppConfiguration.__post_init__()` valide désormais les invariants critiques. |
| 3.3 | Fait | `MotionCommandService.wound_run()` utilise maintenant `lateral_steps_per_mm` pour l'axe traverse. |
| 3.4 | Fait | `EventBus` est documenté comme multi-producteurs, compte les drops et les journalise. |
| 3.5 | Fait | `resume_session()` refuse désormais de reprendre une session si le worker n'est plus vivant. |
| 3.6 | Ignoré | L'analyse du code confirme qu'il ne s'agit pas d'un bug fonctionnel; aucun changement utile à faire. |
| 4.1 | Fait | `core/coordinator.py` créé avec `MotionCoordinator` (`request_stop`, `clear_fault`). `WindingRpcHandler` reçoit le coordinateur et l'utilise pour `winding.stop` et `winding.clear_fault`. `WinderApplication` l'instancie et le passe au handler. |
| 4.2 | Ignoré | Le flag `wait` proposé ne remonte pas le vrai résultat du move et exposerait une API potentiellement trompeuse sans handle de résultat. |
| 4.3 | Fait | Les diagnostics SPI sont désormais exposés via `RuntimeStatusService.engine_status()`. |
| 4.4 | Fait | Le timeout de stall est maintenant configurable et dimensionné à partir de la durée du move synchronisé. |
| 4.5 | Fait | `load_segments()` déstructure correctement le triplet retourné par `load_segment_json()`. |
| 5.1 | Fait | Les événements `EventBus` portent maintenant une `version`, relayée dans `winding.event`. |
| 5.2 | Ignoré | La prémisse de la critique est incorrecte: `_accept_loop()` traite déjà les clients de manière sérielle et n'accepte pas un second client pendant `_handle_client()`. |
| 5.3 | Couvert par 4.5 | Aucun correctif séparé nécessaire une fois `load_segments()` réparé. |
| 5.4 | Fait | `AxisState.check_move()` documente explicitement le cas position inconnue et accepte désormais `strict=True`. |
| 5.5 | Fait | `WindingProgram.snapshot()` repose maintenant sur `dataclasses.asdict()` puis ajoute les champs dérivés. |
| 6.1 | Ignoré | Dette de conception mineure, sans bug immédiat ni gain opérationnel court terme. |
| 6.2 | Ignoré | Sujet d'outillage statique, pas un problème runtime. |
| 6.3 | Ignoré | Nettoyage d'API non prioritaire par rapport aux bugs réels. |
| 6.4 | Fait | Les écritures socket du serveur RPC sont désormais sérialisées par un verrou dédié. |

### Validation de cette passe

- Des tests ciblés ont été ajoutés dans `tests/test_critique_v10_regressions.py`.
- L'exécution de `pytest` n'a pas été réalisée dans cette session car l'appel terminal a été explicitement ignoré côté utilisateur.

---

## 1. Vue d'ensemble

Le runtime Python est un orchestrateur de mouvement temps-réel structuré autour de trois contraintes dures : la sérialisation du mouvement via `MoveQueue`, la cohérence d'état multi-thread entre plusieurs sources de vérité, et le streaming déterministe vers un ESP32 sur un protocole SPI pipeliné à frame fixe (512 octets).

### Points forts

- La séparation entre `WindingEngine` (programme classique) et `AdaptiveWindingService` (session live) est bien pensée : les deux paths convergent vers `MoveQueue` sans duplication de logique transport.
- `MoveQueue` comme barrière de sérialisation unique est le bon choix architectural. Il linéarise le flux de mouvement avant SPI.
- La hiérarchie `BaseMove` → `Move` → `CompositeMove` est propre et conforme à LSP. `HomingMove` n'hérite pas de `Move` et ne prétend jamais produire directement des segments.
- `TrapezoidalMotionProfile` a une validation d'entrée et des tests arithmétiques corrects pour `turns_at` et `steps_at`.
- Le streamer `MultiAxisRampStreamer` expose un diagnostic riche (underrun, segments dropped, stall, planner pressure) qui est un atout majeur pour le debug en production.

### Zones de risque

Quatre zones concentrent la quasi-totalité du risque opérationnel : la fragmentation de l'état machine, la sémantique du stop, la gestion du timeout de `_wait_for_move_queue`, et l'absence de tests unitaires sur les chemins critiques.

---

## 2. Problèmes critiques

### ~~2.1 Race condition dans `WindingRpcHandler.stop()`~~

**Fichier** : `src/rpi/jsonrpc/winding_handler.py`, méthode `stop()`

**Code actuel** :
```python
def stop(self, _params: Any | None = None) -> dict[str, str]:
    adaptive_status = self._adaptive_winding.session_status()
    if adaptive_status.get("active") and adaptive_status.get("session") is not None:
        self._adaptive_winding.request_stop()
        return {"status": "stopping"}
    self._engine.request_stop()
    return {"status": "stopping"}
```

**Problème** : Il n'y a pas de verrou entre l'appel à `session_status()` et `request_stop()`. Si une session adaptive se termine entre ces deux appels (fenêtre de quelques microsecondes), ni le service adaptive ni le moteur classique ne reçoivent le stop. Un `MotionCommandService` actif avec un jog en cours n'est également jamais annulé par cette méthode.

**Correction** :
```python
def stop(self, _params: Any | None = None) -> dict[str, str]:
    # Toujours demander un stop aux deux services. Chacun ignore l'appel
    # s'il n'est pas actif. Cela supprime la race condition de branchement.
    self._adaptive_winding.request_stop()
    self._engine.request_stop()
    return {"status": "stopping"}
```

La méthode `request_stop()` d'`AdaptiveWindingService` retourne déjà proprement quand aucune session n'est active (ligne `if runtime is None: return {"active": False, ...}`). Il n'y a donc aucun risque à l'appeler inconditionnellement.

---

### ~~2.2 `_wait_for_move_queue` ne stoppe pas le streamer actif~~

**Fichier** : `src/rpi/core/engine.py`, méthode `_wait_for_move_queue()`

**Code actuel** :
```python
def _wait_for_move_queue(self, poll_s: float = 0.05, timeout_s: float = 60.0) -> None:
    deadline = time.monotonic() + timeout_s
    while (
        not self._stop_requested()
        and (self._move_queue.pending_count > 0 or self._move_queue.current_move is not None)
    ):
        if time.monotonic() >= deadline:
            msg = "..."
            self._state.set_fault(msg)
            break  # ← sort sans annuler le streamer
        time.sleep(poll_s)
```

**Problème** : Sur timeout, la méthode pose un fault dans `SharedState` et sort de la boucle, mais le `MultiAxisRampStreamer` en cours dans `MoveQueue` continue à tourner. Le move actuel sera marqué `COMPLETED` (pas `ABORTED`) quand le streamer termine normalement. `_run_layer` lira alors `move.state.name == "COMPLETED"` et retournera `True`, ce qui provoque l'exécution du layer suivant malgré le fault posé.

**Correction** :
```python
def _wait_for_move_queue(self, poll_s: float = 0.05, timeout_s: float = 60.0) -> None:
    deadline = time.monotonic() + timeout_s
    while (
        not self._stop_requested()
        and (self._move_queue.pending_count > 0 or self._move_queue.current_move is not None)
    ):
        if time.monotonic() >= deadline:
            msg = (
                f"_wait_for_move_queue timed out after {timeout_s:.1f} s — "
                "firmware may have stopped responding"
            )
            self._state.set_fault(msg)
            # Annuler le mouvement actif pour que son état devienne ABORTED.
            self._move_queue.clear()
            break
        time.sleep(poll_s)
```

`MoveQueue.clear()` appelle déjà `_request_current_streamer_stop()` en interne.

---

### ~~2.3 Fragmentation de la source de vérité sur l'état machine~~

**Fichiers** : `core/shared_state.py`, `motion/axis_state.py`, `winding/adaptive.py` (`AdaptiveWindingRuntime`)

**Problème** : Trois objets maintiennent indépendamment l'état du système :

- `SharedState._engine_state` (verrou `RLock`)
- `AdaptiveWindingRuntime._state` (verrou `RLock` séparé)
- `AxisState._homed` / `_position_steps` (verrou `Lock` par axe)

Aucune transaction atomique ne couvre plusieurs de ces objets. Des séquences comme `runtime.mark_paused()` + `self._state.set_engine_state(EngineState.PAUSED)` dans `service.py` (deux écritures non atomiques) peuvent exposer un état incohérent à un client RPC qui lirait entre les deux.

**Correction recommandée** : Introduire un `MotionStateAuthority` qui encapsule les transitions d'état composite. À minima, envelopper les paires de mutations dans une seule méthode verrouillée :

```python
# Dans SharedState — ajouter :
def transition_to_paused(self, session_snapshot: dict) -> None:
    with self._lock:
        self._engine_state = EngineState.PAUSED
        self._winding_session = session_snapshot
```

Puis dans `AdaptiveWindingService._run_session()` :
```python
# Remplacer :
runtime.mark_paused()
self._state.set_engine_state(EngineState.PAUSED)
self._publish_status(runtime)

# Par :
runtime.mark_paused()
self._state.transition_to_paused(runtime.snapshot())
self._events.publish(EventKind.STATUS_UPDATE, winding_session=runtime.snapshot())
```

---

### 2.4 `refresh_home_state()` appelée à la demande, pas en continu

**Fichier** : `src/rpi/core/lateral.py`, méthode `refresh_home_state()`

**Problème** : `refresh_home_state()` est appelée uniquement quand un client appelle `winding.status` ou `require_homed()`. Si l'axe perd son enable entre deux appels (coupure d'alimentation, reset ESP32), le host continue à croire l'axe homé. Le prochain move lateral sera envoyé avec une position de départ incorrecte.

**Correction** : Appeler `refresh_home_state()` dans `MoveQueue._execute_wound_move()` et `_execute_ramp_move()` avant chaque move sur l'axe lateral, ou la déclencher depuis le thread `rpc_notify` qui poll les statuts firmware régulièrement.

---

## 3. Problèmes importants

### ~~3.1 `ConfigurationManager` est un squelette non fonctionnel~~

**Fichier** : `src/rpi/core/config.py`

**Code actuel** :
```python
class ConfigurationManager:
    def load_configuration(self):
        pass

    def save_configration(self):  # ← faute de frappe
        pass

    def get_saved_configuration(self):
        pass

    def get_activate_configuration(self):
        pass
```

**Problème** : La classe est importée dans `core/__init__.py` et donc exposée comme API publique. Elle ne fait rien. Les utilisateurs qui l'instancient croient charger une configuration depuis un fichier alors que `active_configuration` reste toujours l'`AppConfiguration` par défaut.

**Correction** : Soit l'implémenter (chargement JSON/TOML depuis `config_file_path`), soit la supprimer de `core/__init__.py` et la marquer `# TODO` explicitement. La faute de frappe `save_configration` doit être corrigée en `save_configuration`.

---

### ~~3.2 `AppConfiguration` sans validation d'invariants~~

**Fichier** : `src/rpi/core/config.py`

**Problème** : Des valeurs absurdes passent silencieusement et causent des erreurs cryptiques dans la stack motion (division par zéro, dépassements de limites physiques).

Exemples de configurations invalides non détectées :
- `spindle_max_speed_rpm = 0` → division par zéro dans `compute_ramp_times`
- `lateral_soft_limit_max_mm < lateral_soft_limit_min_mm` → les limites soft bloquent tout mouvement
- `lateral_traverse_pitch_mm = 0` → division par zéro dans `lateral_steps_per_mm`
- `spindle_steps_per_revolution = 0` → division par zéro dans `spindle_max_acceleration_steps_per_s2`

**Correction** : Ajouter un `__post_init__` :
```python
def __post_init__(self) -> None:
    if self.spindle_steps_per_revolution <= 0:
        raise ValueError("spindle_steps_per_revolution must be positive")
    if self.spindle_microstepping <= 0:
        raise ValueError("spindle_microstepping must be positive")
    if self.spindle_max_speed_rpm <= 0:
        raise ValueError("spindle_max_speed_rpm must be positive")
    if self.lateral_steps_per_revolution <= 0:
        raise ValueError("lateral_steps_per_revolution must be positive")
    if self.lateral_microstepping <= 0:
        raise ValueError("lateral_microstepping must be positive")
    if self.lateral_traverse_pitch_mm <= 0.0:
        raise ValueError("lateral_traverse_pitch_mm must be positive")
    if not re.fullmatch(r"/dev/spidev\d+\.\d+", self.spi_device):
        raise ValueError("spi_device must be in the form /dev/spidev<bus>.<device>")
    if (
        self.lateral_soft_limit_min_mm is not None
        and self.lateral_soft_limit_max_mm is not None
        and self.lateral_soft_limit_max_mm <= self.lateral_soft_limit_min_mm
    ):
        raise ValueError(
            "lateral_soft_limit_max_mm must be greater than lateral_soft_limit_min_mm"
        )
```

---

###  ~~3.3 `wound_run` dans `MotionCommandService` utilise les mauvais `steps_per_unit`~~

**Fichier** : `src/rpi/motion/command_service.py`, méthode `wound_run()`

**Code actuel** :
```python
traverse_cfg=SyncAxisConfig(
    axis_index=traverse_axis_id,
    steps_per_unit=(
        self._config.lateral_steps_per_revolution
        * self._config.lateral_microstepping
    ),
    reverse_direction=traverse_reverse,
),
```

**Problème** : `steps_per_unit` pour l'axe traverse devrait être `lateral_steps_per_mm` (steps par millimètre), pas `steps_per_revolution`. Le générateur de segments (`SynchronizedSegmentGenerator`) multiplie `traverse_steps_at(t)` par `steps_per_unit` où `t` donne une position en mm. Utiliser `steps_per_revolution` au lieu de `steps_per_mm` produit des déplacements erronés, typiquement 8× trop grands avec un filetage de 1 mm/tour et 32 micropas.

Comparer avec `WindingEngine._run_layer()` qui utilise correctement :
```python
traverse_cfg=SyncAxisConfig(
    axis_index=program.lateral_axis_id,
    steps_per_unit=program.lateral_steps_per_mm,  # ← correct
    ...
),
```

**Correction** :
```python
traverse_cfg=SyncAxisConfig(
    axis_index=traverse_axis_id,
    steps_per_unit=self._config.lateral_steps_per_mm,  # ← corriger ici
    reverse_direction=traverse_reverse,
),
```

---

### ~~3.4 `EventBus` est mono-consommateur par design mais multi-producteurs en pratique~~

**Fichier** : `src/rpi/core/events.py`

**Problème** : Le commentaire de classe indique "Single-producer (WindingEngine), single-consumer (JsonRpcServer)" mais en pratique `AdaptiveWindingService` publie aussi des événements (`STATUS_UPDATE`), et `LateralAxisController` publie `HOMING_STARTED/COMPLETED/FAILED`. Si la queue atteint `MAX_EVENTS = 256`, les événements sont silencieusement droppés (`except queue.Full: pass`). Il n'y a pas de compteur de drops, pas d'alerte.

**Correction** :
```python
def publish(self, kind: EventKind, **data: Any) -> None:
    try:
        self._q.put_nowait(Event(kind=kind, data=data))
    except queue.Full:
        self._dropped_count += 1
        if self._dropped_count % 10 == 1:
            import logging
            logging.getLogger(__name__).warning(
                "EventBus full: dropped event %s (total drops: %d)",
                kind.name,
                self._dropped_count,
            )
```

Mettre à jour le commentaire pour refléter la réalité multi-producteurs.

---

### ~~3.5 `AdaptiveWindingService.resume_session()` peut relancer une session en FAULT~~

**Fichier** : `src/rpi/winding/service.py`, méthode `resume_session()`

**Code actuel** :
```python
def resume_session(self) -> dict[str, Any]:
    runtime = self._require_session()
    if runtime.snapshot()["target_rpm"] <= 0.0:
        raise RuntimeError("Set a positive target_rpm before resuming...")
    runtime.resume()
    ...
```

**Problème** : `_require_session()` avec `allow_terminal=False` lève une exception si l'état est `completed/stopped/fault`. Mais si `runtime.state == "paused"` et que le worker thread a planté entre-temps (exception non catchée), `resume()` réveille la boucle principale qui ne tournera plus. Le `_worker` est `None` (nettoyé dans `finally`), mais `_active_session` est toujours non-`None`. `resume_session()` réussit côté RPC mais rien ne se produit physiquement.

**Correction** : Vérifier que le worker est toujours vivant avant de reprendre :
```python
def resume_session(self) -> dict[str, Any]:
    runtime = self._require_session()
    with self._lock:
        worker_alive = self._worker is not None and self._worker.is_alive()
    if not worker_alive:
        raise RuntimeError(
            "Adaptive winding worker is no longer running; "
            "the session may have failed. Check session_status() for details."
        )
    if runtime.snapshot()["target_rpm"] <= 0.0:
        raise RuntimeError("Set a positive target_rpm before resuming...")
    runtime.resume()
    self._publish_status(runtime)
    self._wake_event.set()
    return runtime.snapshot()
```

---

### 3.6 `home_lateral` dans `MotionCommandService` peut laisser `engine_state` en HOMING sur exception

**Fichier** : `src/rpi/motion/command_service.py`, méthode `home_lateral()`

**Code actuel** :
```python
def home_lateral(self, ...) -> dict[str, Any]:
    ...
    self._state.set_engine_state(EngineState.HOMING)
    try:
        success, reason = self._lateral.home(...)
        if not success:
            raise RuntimeError(...)
        return self._lateral.require_axis_state(...).snapshot()
    finally:
        if self._state.engine_state == EngineState.HOMING:
            self._state.set_engine_state(EngineState.IDLE)
```

**Problème** : Le `finally` remet bien l'état en IDLE si on est en HOMING. Mais si `self._lateral.home()` appelle `self._state.set_fault()` en interne (cas d'échec de homing dans `LateralAxisController.home()`), `engine_state` sera `FAULT` dans le `finally`, la condition `== HOMING` sera fausse, et l'état `FAULT` sera correctement préservé. Ce chemin est donc correct.

Cependant, si `require_axis_state()` lève une `RuntimeError` après un homing réussi (axis_id inconnu), le `finally` repositionne en IDLE alors que l'axe vient d'être homé. C'est le bon comportement, mais il faut s'assurer que `require_axis_state` ne peut pas lever dans ce contexte — et effectivement l'axe `lateral_axis_id` est toujours dans `axis_states` puisqu'il est créé dans `_build_axis_states`. Ce chemin est donc sûr.

**Aucune correction requise** — documenter explicitement ce raisonnement dans un commentaire inline pour les futurs mainteneurs.

---

## 4. Améliorations structurelles

### ~~4.1 Introduire un `MotionCoordinator` pour centraliser les transitions d'état~~

**Problème** : La logique de "quel service stopper en premier" est actuellement dupliquée entre `WindingRpcHandler.stop()`, `WinderApplication.stop()`, et `AdaptiveWindingService.request_stop()`. Les trois implémentations ne sont pas équivalentes.

**Amélioration** : Créer `core/coordinator.py` :

```python
class MotionCoordinator:
    """Unique point d'entrée pour les transitions d'état globales du système."""

    def __init__(self, engine, adaptive_winding, move_queue, shared_state):
        self._engine = engine
        self._adaptive = adaptive_winding
        self._queue = move_queue
        self._state = shared_state

    def request_stop(self) -> None:
        """Arrêt propre : stoppe les deux services puis vide la queue."""
        self._adaptive.request_stop()
        self._engine.request_stop()
        # request_stop() dans engine appelle déjà queue.clear()
        # mais appel explicite pour garantir la sémantique quelle que
        # soit l'implémentation future de l'un ou l'autre.
        self._queue.clear()

    def clear_fault(self) -> None:
        """Acquitte une FAULT si aucun mouvement n'est en cours."""
        if self._queue.current_move is not None or self._queue.pending_count > 0:
            raise RuntimeError("Cannot clear fault while motion is in progress")
        self._engine.clear_fault()
        # SharedState.clear_fault() est appelé par engine.clear_fault().
```

`WindingRpcHandler` reçoit une référence au `MotionCoordinator` en lieu et place des deux services séparés pour les opérations transversales.

---

### 4.2 Ajouter `wait: bool = False` aux méthodes RPC de motion

**Problème** : `jog`, `move_lateral_mm`, `wound_run`, `run_axis` retournent `{"status": "queued"}` immédiatement sans information sur le résultat réel. Un client qui enchaîne des appels peut saturer la queue silencieusement.

**Amélioration** : Ajouter un paramètre optionnel `wait` :

```python
# Dans WindingRpcHandler :
def jog(self, axis_id: int, steps: int, rpm: float,
        reverse: bool = False, wait: bool = False) -> dict[str, Any]:
    self._commands.jog(axis_id=axis_id, steps=steps, rpm=rpm, reverse=reverse)
    if wait:
        try:
            self._commands._move_queue.wait_until_idle(timeout_s=30.0)
        except TimeoutError as exc:
            raise JsonRpcError(-32000, str(exc)) from exc
    queue_depth = self._commands._move_queue.pending_count
    return {"status": "queued", "queue_depth": queue_depth}
```

---

### ~~4.3 Exposer les diagnostics du streamer via RPC~~

**Problème** : `MultiAxisRampStreamer` accumule des compteurs précieux (`_diag_bad_magic`, `_diag_bad_crc`, `_diag_zero_rx`, underrun deltas, segments dropped) mais ces données ne sont jamais exposées dans la réponse `winding.status`.

**Amélioration** : Ajouter une méthode `transport_diagnostics()` à `Esp32SpiTransport` :

```python
def transport_diagnostics(self) -> dict:
    return {
        "total_xfers": self._diag_total_xfers,
        "bad_magic": self._diag_bad_magic,
        "bad_crc": self._diag_bad_crc,
        "zero_rx": self._diag_zero_rx,
        "reopens": self._diag_reopens,
        "last_status_age_s": round(time.monotonic() - self._last_status_ts, 3)
        if self._last_status is not None else None,
    }
```

Et l'intégrer dans `RuntimeStatusService.engine_status()`.

---

### ~~4.4 Timeout de stall adaptatif dans `MultiAxisRampStreamer`~~

**Fichier** : `src/rpi/transport/streamer.py`

**Problème** : `_stall_timeout_s = 5.0` est une constante globale. Lors d'une phase de décelération planifiée longue (fin de bobine dans le path adaptatif, où le spindle ralentit sur plusieurs secondes), la détection de stall peut se déclencher faussement si `last_executed_sequence` ne progresse pas assez vite.

**Amélioration** : Passer le timeout en paramètre à la construction du streamer, calculé depuis la durée du move :

```python
# Dans MoveQueue._make_wound_streamer() :
stall_timeout_s = max(5.0, move.kinematics.total_duration * 2.0)

streamer = MultiAxisRampStreamer.from_axis_ids(
    ...,
    stall_timeout_s=stall_timeout_s,
)
```

```python
# Dans MultiAxisRampStreamer.__init__() :
def __init__(self, ..., stall_timeout_s: float = 5.0):
    ...
    self._stall_timeout_s = max(1.0, stall_timeout_s)
```

---

### ~~4.5 `segment_json.py` — `load_segments()` retourne un `list` mais a une signature incohérente~~

**Fichier** : `src/rpi/motion/segment_json.py`

**Problème** : `load_segment_json()` retourne `(axis_ids, metadata, Iterator[MultiAxisSegment])` mais `load_segments()` ignore `axis_ids` et `metadata` :

```python
def load_segments(path: Path | str) -> list[MultiAxisSegment]:
    _, iterator = load_segment_json(path)  # ← déstructuration à 2 éléments mais retour à 3
    return list(iterator)
```

Ce code lève une `ValueError` au runtime (`too many values to unpack`).

**Correction** :
```python
def load_segments(path: Path | str) -> list[MultiAxisSegment]:
    _axis_ids, _metadata, iterator = load_segment_json(path)
    return list(iterator)
```

---

## 5. Durcissement défensif

### ~~5.1 Versioning des événements `EventBus`~~

**Fichier** : `src/rpi/core/events.py`

Les notifications JSON-RPC `winding.event` n'ont pas de version. Ajouter `version: int = 1` à la dataclass `Event` et l'inclure dans la notification `rpc_server.py` :

```python
notification = {
    "jsonrpc": "2.0",
    "method": "winding.event",
    "params": {
        "version": event.version,  # ← ajouter
        "kind": event.kind.name,
        "data": event.data,
    },
}
```

---

### 5.2 `JsonRpcServer` : gestion des clients multiples concurrents

**Fichier** : `src/rpi/jsonrpc/rpc_server.py`

Le serveur accepte une seule connexion à la fois (`listen(1)`). Si un client se connecte pendant qu'un autre est actif, la nouvelle connexion sera acceptée mais le traitement de l'ancien client sera interrompu (le `with self._client_lock` dans `_accept_loop` écrase `_current_client`). Ce comportement est intentionnel mais non documenté.

**Amélioration** : Ajouter un commentaire explicite dans `_accept_loop()` et logger un warning quand une connexion est écrasée :

```python
with self._client_lock:
    if self._current_client is not None:
        logger.warning(
            "New client connection while previous client still active — "
            "previous client will be disconnected"
        )
    self._current_client = conn
```

---

### 5.3 `MockSpiTransport` : `load_segment_json` — déstructuration incorrecte (voir 4.5)

Même bug que `load_segments()` : toute utilisation de `MockSpiTransport` dans des tests qui appellent `load_segments()` lèvera une exception silencieuse au moment de l'import si le fichier de segments est chargé.

---

### ~~5.4 `AxisState.check_move()` retourne `True` si position inconnue~~

**Fichier** : `src/rpi/motion/axis_state.py`

**Code actuel** :
```python
def check_move(self, delta_steps: int) -> bool:
    with self._lock:
        if self._position_steps is None:
            return True  # ← autorise tout mouvement si position inconnue
```

**Problème** : Retourner `True` quand la position est inconnue est une décision de design documentée ("limit not checkable"). Cependant, les soft limits ne peuvent pas protéger un axe non homé. La logique de `ensure_delta_allowed()` dans `LateralAxisController` appelle `require_homed()` avant `check_move()`, ce qui bloque ce chemin. Mais un appel direct à `AxisState.check_move()` depuis un futur code contournerait la protection.

**Amélioration** : Documenter explicitement ce comportement dans le docstring, et ajouter un paramètre `strict=False` pour les appelants qui veulent un comportement strict :

```python
def check_move(self, delta_steps: int, *, strict: bool = False) -> bool:
    """
    Returns True if the move is within soft limits.

    If position is unknown (not homed):
      - strict=False (default): returns True (limit not checkable)
      - strict=True: returns False (move refused when position unknown)
    """
    with self._lock:
        if self._position_steps is None:
            return not strict
        ...
```

---

### ~~5.5 Absence de `__slots__` sur `WindingProgram` snapshot~~

**Fichier** : `src/rpi/winding/program.py`

`WindingProgram` utilise `@dataclass(slots=True)` correctement. Mais `snapshot()` reconstruit un dict à la main plutôt que `dataclasses.asdict()`. Si un champ est ajouté à `WindingProgram` sans mise à jour de `snapshot()`, le champ sera silencieusement absent du snapshot RPC.

**Correction** :
```python
import dataclasses

def snapshot(self) -> dict[str, Any]:
    d = dataclasses.asdict(self)
    d["turns_per_mm"] = self.turns_per_mm
    d["layer_duration_s"] = self.layer_duration_s()
    return d
```

---

## 6. Dette technique mineure

### 6.1 `SpindleKinematics` hérite de `TrapezoidalMotionProfile` via `@dataclass(slots=True)`

**Fichier** : `src/rpi/motion/spindle_kinematics.py`

`SpindleKinematics` est un `@dataclass(slots=True)` qui hérite de `TrapezoidalMotionProfile` (classe normale, pas dataclass). Le `__post_init__` appelle `super().__init__()` avec les bons arguments. Ce pattern fonctionne mais est fragile : si `TrapezoidalMotionProfile` ajoute un paramètre à `__init__`, `SpindleKinematics.__post_init__` doit être mis à jour manuellement.

**Recommandation** : Faire de `TrapezoidalMotionProfile` un `@dataclass` également, et `SpindleKinematics` devient une simple instance de configuration sans héritage.

---

### 6.2 `motion/__init__.py` utilise `__getattr__` pour les imports lazy

**Fichier** : `src/rpi/motion/__init__.py`

Le mécanisme `__getattr__` au niveau module est correct pour éviter les imports circulaires, mais les outils d'analyse statique (mypy, pylance) ne le comprennent pas. Les imports `from motion import RampConfig` apparaîtront comme des erreurs dans les IDEs.

**Recommandation** : Utiliser des `TYPE_CHECKING` guards pour les outils statiques, ou documenter explicitement que ce module requiert une configuration mypy avec `ignore_missing_imports`.

---

### 6.3 `MultiAxisRampStreamer` — deux constructeurs (`__init__` et `from_axis_ids`)

**Fichier** : `src/rpi/transport/streamer.py`

Les deux constructeurs partagent `_initialize_streamer_state()` mais la distinction entre les deux est subtile : `__init__` dérive `target_hz` depuis des `RampConfig`, `from_axis_ids` le reçoit explicitement. Cette dualité est bien documentée dans le code mais crée une surface de test double.

**Recommandation** : Fusionner en un seul constructeur avec `target_hz: float | None = None` où `None` déclenche la dérivation depuis les `RampConfig`. Réduirait la surface d'API externe.

---

### ~~6.4 `rpc_server.py` — `_handle_client` et `_notify_loop` partagent le même socket sans coordination d'écriture~~

**Fichier** : `src/rpi/jsonrpc/rpc_server.py`

`_handle_client` (thread `rpc_accept`) et `_notify_loop` (thread `rpc_notify`) écrivent tous les deux sur le socket client, protégés seulement par `_client_lock` pour la lecture de `_current_client`. Les deux threads peuvent appeler `conn.sendall()` concurremment.

`socket.sendall()` sur un `AF_UNIX SOCK_STREAM` est thread-safe au niveau kernel (les writes atomiques pour des buffers < PIPE_BUF), mais il n'y a aucune garantie d'ordre entre les messages RPC response et les notifications d'événements. Sur un flux lent, des réponses et des notifications peuvent s'entrelacer au niveau applicatif.

**Recommandation** : Ajouter un `threading.Lock` dédié aux writes sur le socket client :

```python
self._write_lock = threading.Lock()

# Dans _handle_client et _notify_loop :
with self._write_lock:
    conn.sendall(...)
```

---

## 7. Récapitulatif par fichier

| Fichier | Sévérité | Problème | Section |
|---|---|---|---|
| `jsonrpc/winding_handler.py` | **Critique** | Race condition dans `stop()` | 2.1 |
| `core/engine.py` | **Critique** | Timeout sans annulation du streamer | 2.2 |
| `core/shared_state.py` | **Critique** | Mutations d'état non atomiques | 2.3 |
| `core/lateral.py` | **Critique** | `refresh_home_state()` non proactive | 2.4 |
| `core/config.py` | **Important** | `ConfigurationManager` non implémenté | 3.1 |
| `core/config.py` | **Important** | `AppConfiguration` sans validation | 3.2 |
| `motion/command_service.py` | **Important** | `steps_per_unit` erroné dans `wound_run` | 3.3 |
| `core/events.py` | **Important** | Drop silencieux d'événements | 3.4 |
| `winding/service.py` | **Important** | `resume_session()` avec worker mort | 3.5 |
| `motion/command_service.py` | **Important** | État HOMING sur exception (non-bug documenté) | 3.6 |
| `core/coordinator.py` | **Structurel** | Absent — centraliser les stops | 4.1 |
| `jsonrpc/winding_handler.py` | **Structurel** | Pas de backpressure RPC | 4.2 |
| `transport/spi_transport.py` | **Structurel** | Diagnostics non exposés en RPC | 4.3 |
| `transport/streamer.py` | **Structurel** | Stall timeout fixe | 4.4 |
| `motion/segment_json.py` | **Important** | `load_segments()` lève au runtime | 4.5 |
| `core/events.py` | **Défensif** | Pas de versioning des événements | 5.1 |
| `jsonrpc/rpc_server.py` | **Défensif** | Coupure silencieuse d'un client actif | 5.2 |
| `motion/axis_state.py` | **Défensif** | `check_move()` permissif si non-homé | 5.4 |
| `winding/program.py` | **Défensif** | `snapshot()` manuel, pas `asdict()` | 5.5 |
| `motion/spindle_kinematics.py` | **Mineur** | Héritage fragile sur dataclass | 6.1 |
| `motion/__init__.py` | **Mineur** | Imports lazy opaques aux outils statiques | 6.2 |
| `transport/streamer.py` | **Mineur** | Deux constructeurs à unifier | 6.3 |
| `jsonrpc/rpc_server.py` | **Mineur** | Writes concurrents sur socket client | 6.4 |

---

## Ordre de priorité pour l'implémentation

**~~Sprint 1 — Corrections de bugs~~**
1. ~~`load_segments()` déstructuration incorrecte (§4.5) — 5 min, risque de crash~~
2. ~~`wound_run` steps_per_unit erroné (§3.3) — 2 min, bug physique silencieux~~
3. ~~Race condition `stop()` (§2.1) — 15 min, correctif simple~~
4. ~~Timeout `_wait_for_move_queue` sans annulation (§2.2) — 20 min~~

**Sprint 2 — Robustesse**
5. ~~Validation `AppConfiguration.__post_init__` (§3.2)~~
6. ~~`resume_session()` avec worker mort (§3.5)~~
7. ~~Drop silencieux `EventBus` avec compteur (§3.4)~~
8. `refresh_home_state()` proactive (§2.4)

**Sprint 3 — Architecture**
9. ~~`MotionCoordinator` pour centraliser les stops (§4.1)~~
10. ~~Mutations d'état atomiques dans `SharedState` (§2.3)~~
11. ~~Diagnostics transport en RPC (§4.3)~~
12. ~~Stall timeout adaptatif (§4.4)~~

**Sprint 4 — Qualité**
13. ~~`ConfigurationManager` implémenté ou supprimé (§3.1)~~
14. ~~Versioning des événements (§5.1)~~
15. ~~`WindingProgram.snapshot()` via `asdict()` (§5.5)~~
16. ~~Writes socket protégés (§6.4)~~