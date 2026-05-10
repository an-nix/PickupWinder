# PickupWinder — Analyse et plan de modifications

> Analyse du code source `rpi_python_src_full_dump.txt`  
> Périmètre : `rpi/` (backend) + `wendy/` (passerelle HTTP/WS)

---

## 1. Bug : session.state affiche `"queued"` (= idle) pendant tout le homing

### Cause racine (corrigée)

La première analyse pointait vers une race condition de quelques millisecondes. Ce n'est **pas** ça. Le vrai coupable est dans `_run_session()` (`winding/service.py`).

Voici la séquence exacte lors d'un `start_session()` avec `home_before_start=True` :

```
_run_session():
  1.  engine_state  ← HOMING         (SharedState)
  2.  _publish_status(runtime)        → session.state = "queued"  ← PROBLÈME
  3.  self._lateral.home(...)         → 30 à 120 secondes de homing physique
  4.  self._move_lateral_to(window.low_mm)  → encore quelques secondes
  5.  runtime.mark_running()          → session.state = "running"  ← ENFIN correct
  6.  engine_state  ← RUNNING
  7.  _publish_status(runtime)
```

Pendant toute la durée des étapes 3–4 (potentiellement **plusieurs minutes**) :
- `engine_state` = `"HOMING"` → correct au niveau moteur
- `session.state` = `"queued"` → la session apparaît comme non démarrée

Tout client qui lit `winding.session_status` ou le champ `winding_session` dans `winding.status` voit :

```json
{
  "engine_state": "HOMING",
  "winding_session": {
    "state": "queued"      ← lu comme "idle/pas commencé"
  }
}
```

Même problème avec `home_before_start=False` : `engine_state` passe à `RUNNING` mais `session.state` reste `"queued"` pendant tout le repositionnement latéral vers `window.low_mm`.

---

### Correction — deux niveaux

**Niveau 1 : ajouter l'état `"homing"` à `AdaptiveWindingRuntime`** (`winding/adaptive.py`)

```python
# Dans AdaptiveWindingRuntime :

def mark_homing(self) -> None:
    """Transition vers l'état de homing pré-bobinage."""
    with self._lock:
        if self._state == "queued":
            self._state = "homing"

def mark_running(self) -> None:
    with self._lock:
        # Accepte la transition depuis "queued", "homing" ou "paused"
        self._state = "running"
```

Mettre à jour `session_status()` pour que `active=True` inclue aussi `"homing"` :
```python
# Déjà correct : "homing" n'est pas dans {"completed", "stopped", "fault"}
"active": snapshot["state"] not in {"completed", "stopped", "fault"},
```

**Niveau 2 : appeler `mark_homing()` AVANT le homing physique** (`winding/service.py`)

```python
def _run_session(self, runtime: AdaptiveWindingRuntime) -> None:
    ...
    try:
        session_config = runtime.session_config()
        if session_config.target_rpm <= 0.0:
            raise ValueError("Adaptive winding session must start with target_rpm > 0")
        if runtime.current_window.low_mm < -_EPSILON:
            raise ValueError("window_low_mm must be >= 0 relative to home")

        if session_config.home_before_start:
            # ── AVANT : set_engine_state seulement, session.state restait "queued"
            # ── APRÈS : les deux sont mis à jour immédiatement
            runtime.mark_homing()                              # NOUVEAU
            self._state.set_engine_state(EngineState.HOMING)
        else:
            runtime.mark_running()                             # avancé ici
            self._state.set_engine_state(EngineState.RUNNING)

        self._publish_status(runtime)   # maintenant "homing" ou "running" dès le départ

        if session_config.home_before_start:
            success, reason = self._lateral.home(...)
            if not success:
                raise RuntimeError(reason or "lateral homing failed")

        # Repositionnement vers window.low_mm
        self._move_lateral_to(runtime, runtime.current_window.low_mm)

        # Passage à "running" (depuis "homing" ou conservé "running")
        runtime.mark_running()
        self._state.set_engine_state(EngineState.RUNNING)
        self._publish_status(runtime)

        while True:
            ...
```

### Ce que voit l'utilisateur après correction

| Phase | `engine_state` | `session.state` |
|-------|---------------|-----------------|
| Avant `start_session()` | `IDLE` | — |
| Homing physique | `HOMING` | **`homing`** ← nouveau |
| Repositionnement vers window | `HOMING` | **`homing`** |
| Bobinage actif | `RUNNING` | `running` |
| Pause | `PAUSED` | `paused` |
| Terminé | `IDLE` | `completed` |
| Arrêté | `IDLE` | `stopped` |

---

## 2. Contrôle pendant le bobinage : pause, vitesse, paramètres

### 2.1 Ce qui fonctionne côté RPC

Les méthodes JSON-RPC existent et sont correctement implémentées :

| Méthode RPC | Action |
|-------------|--------|
| `winding.pause_session` | Pause contrôlée (ralentit jusqu'à 0 RPM) |
| `winding.resume_session` | Reprend après pause |
| `winding.update_session` | Change `target_rpm`, fenêtre, diamètre fil, etc. en vol |
| `winding.session_status` | Snapshot complet de la session |
| `winding.stop` | Arrêt propre ou d'urgence |

### 2.2 Ce qui manque côté HTTP (Wendy)

`wendy/handlers.py` n'expose **aucun endpoint REST** pour contrôler une session adaptative. À ajouter :

```python
class SessionHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """GET /api/session  — statut de la session active.
       POST /api/session — démarrer une nouvelle session.
       DELETE /api/session — arrêter la session."""

    def get(self) -> None:
        self.rpc_result("winding.session_status")

    def post(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return
        self.rpc_result("winding.start_session",
                        params={"session": body}, success_status=202)

    def delete(self) -> None:
        mode = self.get_query_argument("mode", default="stop")
        self.rpc_result("winding.stop", params={"mode": mode})


class SessionPauseHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/session/pause"""
    def post(self) -> None:
        body = {}
        if self.request.body:
            try:
                body = json.loads(self.request.body.decode("utf-8"))
            except json.JSONDecodeError:
                pass
        self.rpc_result("winding.pause_session",
                        params={"pause_at_turn": body.get("pause_at_turn")})


class SessionResumeHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/session/resume"""
    def post(self) -> None:
        self.rpc_result("winding.resume_session")


class SessionUpdateHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """PATCH /api/session — modifier target_rpm, fenêtre, fil en vol"""
    def patch(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return
        if not isinstance(body, dict) or not body:
            self.write_json({"error": "Body must be a non-empty JSON object"}, status=400)
            return
        self.rpc_result("winding.update_session", params=body)
```

Routes dans `make_application()` :

```python
(r"/api/session",          SessionHandler),
(r"/api/session/pause",    SessionPauseHandler),
(r"/api/session/resume",   SessionResumeHandler),
(r"/api/session/update",   SessionUpdateHandler),
```

### 2.3 Comportement de `update_session` pendant une pause

Lors d'un `update_session` avec modification de fenêtre en pause, `update_controls()` :
1. Calcule un `_pending_reposition_mm`
2. Positionne l'axe latéral **au resume** via `consume_pending_reposition()`

Ce comportement est correct mais invisible. Rendre explicite dans le snapshot :

```python
# Dans AdaptiveWindingRuntime.snapshot() — déjà présent :
"pending_reposition_mm": self._pending_reposition_mm,

# Ajouter :
"pending_actions": (
    ["reposition_lateral"] if self._pending_reposition_mm is not None else []
),
```

### 2.4 Absence de pause/reprise pour les programmes classiques

`MotionCoordinator.request_pause()` **ne supporte pas les programmes classiques** (le commentaire du code le dit). Plan pour le futur :

1. Ajouter `_pause_event` / `_resume_event` dans `WindingEngine`
2. Dans `_wait_for_move_queue()`, vérifier `_pause_event` et basculer `engine_state → PAUSED`
3. Bloquer jusqu'à `_resume_event` ou timeout
4. Étendre `MotionCoordinator.request_pause()` pour déléguer selon l'état actif

---

## 3. Gestion des programmes et sauvegardes

### 3.1 Absence de backup avant modification

`ProgramStore._write_program()` écrase le fichier sans conserver l'ancienne version.

**Correction dans `program_store.py` :**

```python
import shutil

_BACKUP_KEEP = 5


def _write_program(self, program: WindingProgram) -> None:
    self._storage_dir.mkdir(parents=True, exist_ok=True)
    path = self._program_path(
        program.program_id
        or self._normalize_new_id(None, fallback_name=program.name)
    )

    # Backup avant écrasement
    if path.exists():
        try:
            existing = self._read_program(path)
            backup_dir = self._storage_dir / ".backup"
            backup_dir.mkdir(exist_ok=True)
            backup_name = f"{program.program_id}_rev{existing.revision:04d}.json"
            shutil.copy2(path, backup_dir / backup_name)
            self._prune_backups(backup_dir, program.program_id)
        except Exception as exc:
            logger.warning("Could not create backup for %s: %s", program.program_id, exc)

    temp_path = path.with_suffix(".json.tmp")
    with temp_path.open("w", encoding="utf-8") as handle:
        json.dump(program.to_dict(), handle, indent=2, sort_keys=True)
        handle.write("\n")
    temp_path.replace(path)


def _prune_backups(self, backup_dir: Path, program_id: str) -> None:
    """Conserver seulement les N dernières révisions."""
    backups = sorted(backup_dir.glob(f"{program_id}_rev*.json"))
    for old in backups[:-_BACKUP_KEEP]:
        try:
            old.unlink()
        except OSError as exc:
            logger.warning("Could not prune backup %s: %s", old, exc)
```

### 3.2 Lister et restaurer les révisions

Ajouter dans `ProgramStore` :

```python
def list_revisions(self, program_id: str) -> list[dict[str, Any]]:
    normalized_id = self._normalize_existing_id(program_id)
    backup_dir = self._storage_dir / ".backup"
    if not backup_dir.exists():
        return []
    revisions = []
    for path in sorted(backup_dir.glob(f"{normalized_id}_rev*.json"), reverse=True):
        try:
            revisions.append(self._summary(self._read_program(path)))
        except Exception as exc:
            logger.warning("Skipping unreadable backup %s: %s", path, exc)
    return revisions


def restore_revision(self, program_id: str, revision: int) -> WindingProgram:
    normalized_id = self._normalize_existing_id(program_id)
    backup_path = self._storage_dir / ".backup" / f"{normalized_id}_rev{revision:04d}.json"
    if not backup_path.exists():
        raise ProgramNotFoundError(
            f"Backup revision {revision} not found for program {program_id!r}"
        )
    with self._lock:
        return self.save_program(self._read_program(backup_path), program_id=normalized_id)
```

Enregistrer en RPC dans `WindingRpcHandler` :

```python
# register_all()
handler.register_method("program.list_revisions", self.list_revisions)
handler.register_method("program.restore_revision", self.restore_revision)

# méthodes
def list_revisions(self, program_id: str) -> dict[str, Any]:
    try:
        return {"program_id": program_id,
                "revisions": self._program_store.list_revisions(program_id)}
    except ProgramNotFoundError as exc:
        raise JsonRpcError(-32004, str(exc)) from exc

def restore_revision(self, program_id: str, revision: int) -> dict[str, Any]:
    try:
        restored = self._program_store.restore_revision(program_id, int(revision))
    except ProgramNotFoundError as exc:
        raise JsonRpcError(-32004, str(exc)) from exc
    return {"status": "restored", "program": restored.snapshot()}
```

Handlers HTTP Wendy :

```python
class ProgramRevisionsHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """GET /api/programs/{id}/revisions"""
    def get(self, program_id: str) -> None:
        self.rpc_result("program.list_revisions",
                        params={"program_id": unquote(program_id)})


class ProgramRestoreHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/programs/{id}/restore/{revision}"""
    def post(self, program_id: str, revision: str) -> None:
        self.rpc_result("program.restore_revision",
                        params={"program_id": unquote(program_id), "revision": int(revision)})
```

Routes :

```python
(r"/api/programs/([^/]+)/revisions",      ProgramRevisionsHandler),
(r"/api/programs/([^/]+)/restore/(\d+)",  ProgramRestoreHandler),
```

### 3.3 `_normalize_existing_id` ne vérifie pas l'existence

```python
def _normalize_existing_id(self, program_id: str) -> str:
    normalized = self._normalize_new_id(program_id, fallback_name="program")
    if not self._program_path(normalized).exists():
        raise ProgramNotFoundError(
            f"Unknown program_id: {program_id!r} (normalized: {normalized!r})"
        )
    return normalized
```

### 3.4 Suppression définitive sans filet de sécurité

```python
def delete_program(self, program_id: str) -> None:
    normalized_id = self._normalize_existing_id(program_id)
    with self._lock:
        path = self._program_path(normalized_id)
        # Backup avant suppression
        try:
            existing = self._read_program(path)
            backup_dir = self._storage_dir / ".backup"
            backup_dir.mkdir(exist_ok=True)
            backup_name = f"{normalized_id}_rev{existing.revision:04d}_deleted.json"
            path.rename(backup_dir / backup_name)
        except Exception as exc:
            logger.warning("Could not backup before delete %s: %s", normalized_id, exc)
            path.unlink()
```

---

## 4. Amélioration du status unifié

### 4.1 Ajouter `winding_mode` dans `engine_status()`

```python
def engine_status(self) -> dict[str, Any]:
    self._lateral.refresh_home_state()
    snap = self._shared_state.snapshot()
    winding_session = snap.get("winding_session")
    session_active = (
        winding_session is not None
        and winding_session.get("state") not in {"completed", "stopped", "fault"}
    )

    status = {
        "shared_state": snap,
        "move_queue": self._move_queue_status_provider(),
        "workers": self._workers_status(),
        "winding_mode": (
            "adaptive" if session_active
            else "classic" if snap.get("program") is not None
            else "idle"
        ),
    }
    # Inclure le snapshot de session directement pour éviter un deuxième appel
    if session_active:
        status["session"] = winding_session
    if self._transport_diagnostics_provider is not None:
        status["transport"] = self._transport_diagnostics_provider()
    return status
```

---

## 5. Notifications WebSocket — robustesse

### Problème actuel

`JsonRpcServer._notify_loop()` envoie les events à `_current_client`. Si le client est déconnecté au moment de l'event, celui-ci est perdu silencieusement. À la reconnexion, Wendy ne reçoit pas les events manqués.

### Correction : ring-buffer des events récents

```python
# Dans JsonRpcServer.__init__() :
from collections import deque
self._recent_events: deque[str] = deque(maxlen=50)

# Dans _notify_loop(), après avoir construit notification :
serialized = json.dumps(notification)
self._recent_events.append(serialized)   # NOUVEAU
with self._client_lock:
    client = self._current_client
if client is not None:
    self._send_text(client, serialized)

# Dans _handle_client(), au début :
for event_json in list(self._recent_events):
    if not self._send_text(conn, event_json):
        return
```

---

## 6. Résumé des fichiers à modifier

| Fichier | Modifications |
|---------|--------------|
| `rpi/winding/adaptive.py` | Ajouter `mark_homing()`, `pending_actions` dans `snapshot()` |
| `rpi/winding/service.py` | Appel de `mark_homing()`/`mark_running()` AVANT les moves préparatoires |
| `rpi/winding/program_store.py` | Backup avant écriture/suppression, `list_revisions()`, `restore_revision()`, fix `_normalize_existing_id` |
| `rpi/jsonrpc/winding_handler.py` | Enregistrer `program.list_revisions`, `program.restore_revision` |
| `rpi/core/status.py` | Ajouter `winding_mode` et `session` dans `engine_status()` |
| `rpi/jsonrpc/rpc_server.py` | Ring-buffer events, replay à la reconnexion |
| `wendy/handlers.py` | Handlers session (GET/POST/DELETE, pause, resume, update), revisions/restore |

---

## 7. Priorité suggérée

| Priorité | Ticket | Impact |
|----------|--------|--------|
| 🔴 Critique | Ajouter état `"homing"` au runtime + appel avant le homing physique | Status lisible "bien longtemps" — bug principal signalé |
| 🔴 Critique | Endpoints HTTP session (pause/resume/update/start) | Impossible à contrôler depuis l'UI sans passer par RPC brut |
| 🟠 Haute | Backup avant modification/suppression de programme | Risque de perte de données irréversible |
| 🟠 Haute | `winding_mode` + `session` dans `engine_status()` | Un seul appel status suffit pour tout savoir |
| 🟡 Moyenne | `_normalize_existing_id` avec vérification d'existence | Erreurs trompeuses |
| 🟡 Moyenne | `list_revisions` / `restore_revision` | Nécessite le backup (dépendance §3.1) |
| 🟢 Basse | Ring-buffer events WebSocket | Reconnexion propre sans perte d'events |
| 🟢 Basse | Pause/reprise programmes classiques | Chantier important, non bloquant si adaptatif utilisé |