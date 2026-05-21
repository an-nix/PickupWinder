# Pistes de refactoring — PickupWinder

> Analyse complémentaire après application des refactors de `usage.md` et `winding_program.md`.
> Mis à jour mai 2026 — intègre la clarification architecturale Programme / Session.

---

## 0. Vision cible — trois couches

### Principe

| Couche | Rôle | Persisté ? | Mutable en live ? |
|---|---|---|---|
| **MachineConfig** | Configuration physique de la machine | ✅ (fichier config) | ✗ |
| **WindingProgram** | *Quoi* — recette de bobinage | ✅ `ProgramStore` | ✗ (versionné) |
| **SessionParams** | *À quelle vitesse* — contexte d'exécution | ✗ transient | ✅ |

**Un programme ne peut être lancé qu'au travers d'une session.**  
La session démarre avec les valeurs du programme pour `total_turns` et la fenêtre ; l'opérateur peut les ajuster en cours d'exécution via `update_session()`.

### Répartition des champs

```
MachineConfig              WindingProgram              SessionParams
──────────────             ──────────────              ─────────────
spindle_axis_id            name, program_id            spindle_rpm
lateral_axis_id            num_layers
accel_s                    bobbin_width_mm             ── live overrides ──
decel_s                    layer_pitch_mm              total_turns   (opt)
home_before_start          wire_diameter_mm            window_low_mm (opt)
home_approach_rpm          total_turns                 window_high_mm (opt)
home_search_rpm            scatter_*                   chunk_time_s
home_backoff_steps
```

- **`total_turns`** et **`window_*`** appartiennent au programme : ils définissent *quoi* bobiner. La session peut les overrider si l'opérateur ajuste en cours d'exécution.
- **`accel_s` / `decel_s`** sont des constantes mécaniques de la machine : ils ne varient pas d'un programme à l'autre ni d'une session à l'autre.
- **IDs d'axes** et **homing** : idem, liés à la machine, pas à la recette.

### État actuel — le mélange

```
WindingProgram (actuel)               Couche cible
─────────────────────────             ──────────────────────────────────
name, program_id, revision    →       ✅ reste dans WindingProgram
num_layers, bobbin_width_mm   →       ✅ reste dans WindingProgram
layer_pitch_mm, wire_diameter →       ✅ reste dans WindingProgram
scatter_amplitude_mm/freq…    →       ✅ reste dans WindingProgram
total_turns                   →       ✅ reste dans WindingProgram
spindle_rpm           ❌       →       SessionParams.spindle_rpm
accel_s, decel_s      ❌       →       MachineConfig.accel_s / .decel_s
spindle_axis_id       ❌       →       MachineConfig.spindle_axis_id
lateral_axis_id       ❌       →       MachineConfig.lateral_axis_id
home_before_start     ❌       →       MachineConfig.home_before_start
home_approach_rpm     ❌       →       MachineConfig.home_approach_rpm
home_search_rpm       ❌       →       MachineConfig.home_search_rpm
home_backoff_steps    ❌       →       MachineConfig.home_backoff_steps
```

```
AdaptiveWindingSessionConfig (actuel)  Couche cible
───────────────────────────────────   ────────────────────────────────────
scatter_*, spindle_axis_id…   ❌ →     MachineConfig ou WindingProgram
home_*                        ❌ →     MachineConfig
target_rpm                    →       ✅ SessionParams.spindle_rpm
chunk_time_s                  →       ✅ SessionParams.chunk_time_s
total_turns                   →       ✅ SessionParams (override du programme)
window_low_mm, window_high_mm →       ✅ SessionParams (override du programme)
```

### Modèle cible

```
               MachineConfig
               ─────────────
               spindle_axis_id
               lateral_axis_id
               accel_s, decel_s
               home_*
                      │
                      ▼
WindingProgram ──► WindingSession (transient)
──────────────     ─────────────────────────
num_layers         spindle_rpm
bobbin_width_mm    total_turns   ← override du programme, ajustable live
layer_pitch_mm     window_low_mm ← override du programme, ajustable live
wire_diameter_mm   window_high_mm ← override du programme, ajustable live
scatter_*          chunk_time_s
total_turns ──────►(valeur par défaut si non overridé)
```

### Interface RPC cible

```
winding.start_session(program_id="...", spindle_rpm=1200)
   → démarre une session depuis le programme
   → total_turns et window_* initialisés depuis WindingProgram

winding.update_session(spindle_rpm=1400)
winding.update_session(total_turns=6000)
winding.update_session(window_low_mm=2.5, window_high_mm=28.5)
   → ajustements live en cours d'exécution
```

---

## 1. 🔴 A0 — Extraire `MachineConfig`

**Fichiers :** `rpi/winding/program.py` · `rpi/winding/adaptive.py` · `rpi/core/config.py` (ou nouveau `rpi/machine_config.py`)

### Problème

Les paramètres qui appartiennent à la machine physique sont actuellement éparpillés dans `WindingProgram` et dupliqués dans `AdaptiveWindingSessionConfig`. Ils ne changent pas d'un programme à l'autre ni d'une session à l'autre :
- `spindle_axis_id`, `lateral_axis_id`
- `accel_s`, `decel_s`
- `home_before_start`, `home_approach_rpm`, `home_search_rpm`, `home_backoff_steps`

### Correction

**Étape 1 :** Créer `MachineConfig` (dans `rpi/core/config.py` ou un nouveau `rpi/machine_config.py`) :

```python
@dataclass(frozen=True)
class MachineConfig:
    spindle_axis_id: int
    lateral_axis_id: int
    accel_s: float
    decel_s: float
    home_before_start: bool = True
    home_approach_rpm: float = 200.0
    home_search_rpm: float = 50.0
    home_backoff_steps: int = 200

    @classmethod
    def from_config(cls, cfg: dict) -> MachineConfig:
        ...
```

**Étape 2 :** Supprimer ces champs de `WindingProgram` et de `AdaptiveWindingSessionConfig`.

**Étape 3 :** Injecter `MachineConfig` dans `WindingEngine`, `AdaptiveWindingService`, et `MotionCommandService` depuis la composition root (`app/runtime.py`).

**Rétrocompatibilité :** Les programmes stockés avec ces champs continuent à se charger via `from_payload()` — les champs inconnus sont ignorés. Les clients qui passaient `home_approach_rpm` dans la session recevront un avertissement ou une erreur `-32602` explicite.

**Risque :** moyen — change la composition root et le contrat RPC des sessions. À faire avant A1/A3 pour nettoyer la base.

---

## 2. 🔴 A1 — Sortir `spindle_rpm` de `WindingProgram`

**Fichiers :** `rpi/winding/program.py` · `rpi/core/engine.py` · `rpi/jsonrpc/winding_handler.py`

### Problème

`WindingProgram` porte `spindle_rpm`. Deux programmes identiques en géométrie mais bobinés à des vitesses différentes sont aujourd'hui deux objets `WindingProgram` distincts persistés séparément. La vitesse est un choix de l'opérateur au moment de lancer la session.

`WindingEngine._run_layer()` lit `program.spindle_rpm` directement :

```python
# core/engine.py — actuel
move = build_wound_move(
    spindle_rpm=program.spindle_rpm,     # ← appartient à la session
    accel_s=machine_config.accel_s,      # ← après A0 : vient de MachineConfig
    bobbin_width_mm=program.bobbin_width_mm,
    ...
)
```

### Correction

**Étape 1 :** Créer `SessionParams` dans `winding/session.py` :

```python
@dataclass(slots=True)
class SessionParams:
    spindle_rpm: float
    # overrides live des valeurs du programme (None = utiliser la valeur du programme)
    total_turns: float | None = None
    window_low_mm: float | None = None
    window_high_mm: float | None = None
    chunk_time_s: float = 0.25

    def validate(self) -> None:
        if self.spindle_rpm <= 0.0:
            raise ValueError("spindle_rpm must be positive")
```

**Étape 2 :** Supprimer `spindle_rpm` (et `accel_s`/`decel_s` si A0 n'est pas encore fait) de `WindingProgram`.

**Étape 3 :** `WindingEngine.submit_program()` → `WindingEngine.start_session()` :

```python
def start_session(
    self,
    program: WindingProgram,
    params: SessionParams,
    machine: MachineConfig,
) -> None:
    ...
```

**Étape 4 :** `winding.submit_program` RPC → `winding.start_session` :

```python
def start_session(self, program_id: str, spindle_rpm: float, **overrides) -> dict:
    program = self._store.load(program_id)
    params = SessionParams(
        spindle_rpm=spindle_rpm,
        total_turns=overrides.get("total_turns"),
        window_low_mm=overrides.get("window_low_mm"),
        window_high_mm=overrides.get("window_high_mm"),
    )
    self._engine.start_session(program, params, self._machine)
    return {"status": "queued"}
```

**Rétrocompatibilité :** `winding.submit_program` reste comme alias déprécié. Les programmes stockés avec `spindle_rpm` continuent à se charger — le champ est ignoré par `from_payload()`.

**Risque :** moyen — change le contrat RPC principal de lancement. Migrer wendy en même temps.

---

## 3. 🔴 A2 — Supprimer `to_adaptive_session()` de `WindingProgram`

**Fichier :** `rpi/winding/program.py`

### Problème

`WindingProgram.to_adaptive_session()` importe et instancie `AdaptiveWindingSessionConfig` depuis `winding.adaptive` : une recette persistée dépend d'un moteur d'exécution live. Après A0 et A1, la méthode n'a plus de sens — la session se construit depuis `SessionParams` + `WindingProgram` + `MachineConfig`, pas depuis le programme seul.

### Correction

Supprimer `to_adaptive_session()` de `WindingProgram`. La construction de la session se fait dans le handler via `SessionParams` :

```python
# winding_handler.py — après A0/A1
def start_session(self, program_id: str, spindle_rpm: float, **overrides) -> dict:
    program = self._store.load(program_id)
    params = SessionParams(
        spindle_rpm=spindle_rpm,
        total_turns=overrides.get("total_turns"),          # None → programme.total_turns()
        window_low_mm=overrides.get("window_low_mm"),      # None → position courante
        window_high_mm=overrides.get("window_high_mm"),    # None → position + bobbin_width
    )
    self._engine.start_session(program, params, self._machine)
    return {"status": "queued"}
```

Le runtime résout les valeurs nulles au démarrage : `total_turns` vient de `program.total_turns()`, la fenêtre vient de la position courante + `program.bobbin_width_mm`.

**Risque :** faible — refactoring pur, comportement identique.

---

## 4. 🔴 A3 — `AdaptiveWindingSessionConfig` → `SessionParams`

**Fichier :** `rpi/winding/adaptive.py`

### Problème

Après A0, `AdaptiveWindingSessionConfig` n'a plus de champs machine ni de champs programme. Il ne reste que les paramètres propres à la session. La classe peut être remplacée par `SessionParams` (défini en A1), et `AdaptiveWindingRuntime` reçoit `(program, params, machine)` au lieu d'un objet monolithique.

### Correction

**Étape 1 :** Supprimer `AdaptiveWindingSessionConfig` et utiliser `SessionParams` (de A1).

**Étape 2 :** Mettre à jour `AdaptiveWindingRuntime.__init__()` :

```python
# Avant
def __init__(self, config: AdaptiveWindingSessionConfig):
    self._cfg = config

# Après
def __init__(
    self,
    program: WindingProgram,
    params: SessionParams,
    machine: MachineConfig,
) -> None:
    self._program = program
    self._params = params
    self._machine = machine
    # valeurs résolues — programme comme source de vérité, params comme override
    self.spindle_rpm = params.spindle_rpm
    self.total_turns = params.total_turns if params.total_turns is not None else program.total_turns()
    self.window_low_mm = params.window_low_mm   # résolu par le caller depuis la position courante
    self.window_high_mm = params.window_high_mm # résolu par le caller : window_low + bobbin_width
    self.wire_diameter_mm = program.wire_diameter_mm
    self.pitch_factor = program.layer_pitch_mm / program.wire_diameter_mm
    self.scatter = program  # lecture directe depuis le programme
    self.axis = machine     # lecture directe depuis la machine
```

**Étape 3 :** Mettre à jour `AdaptiveWindingService.start_session()` pour accepter `(program, params, machine)` au lieu de `AdaptiveWindingSessionConfig`.

`AdaptiveWindingSessionConfig` est supprimée — le renommage B1 est rendu sans objet.

**Risque :** élevé — change l'interface de `AdaptiveWindingService`. À planifier après A0 et A1.

---

## 5. 🟠 B1 — `AdaptiveWindingSessionConfig` supprimée par A3 *(sans objet)*

Après A3, `AdaptiveWindingSessionConfig` n'existe plus — remplacée par `SessionParams`. Aucune étape séparée nécessaire.

---

## 6. 🟠 B2 — `winding.start_session_from_program` devient redondant après A2

**Fichier :** `rpi/jsonrpc/winding_handler.py`

Une fois A2 appliqué, `start_session_from_program` est juste `start_session` avec `program_id` + paramètres d'exécution. Le endpoint devient redondant.

**Cible :** un seul endpoint `winding.start_session` qui accepte soit :
- `program_id` + `spindle_rpm` → session adaptative depuis programme stocké
- `program` inline + `spindle_rpm` → session adaptative depuis payload
- `window_low_mm` / `window_high_mm` + géométrie inline → session free-form (dev/debug)

Garder `start_session_from_program` comme alias déprecié.

**Risque :** moyen — change le contrat RPC. À coordonner avec les clients (wendy).

---

## 7. 🟠 B3 — `WindingRpcHandler` gère trois domaines — violation SRP

**Fichier :** `rpi/jsonrpc/winding_handler.py` (~500 lignes)

La classe fusionne : CRUD programmes + contrôle session adaptative + contrôle machine + soumission programme classique. Après A1/A2, la frontière programme/session sera plus nette, ce qui rendra le découpage plus naturel :

| Nouvelle classe | Périmètre | Dépendances |
|---|---|---|
| `ProgramRpcHandler` | `program.*` (8 méthodes CRUD) | `ProgramStore`, `SharedState`, `EventBus` |
| `SessionRpcHandler` | `winding.start_session`, `update_session`, `pause`, `resume`, `session_status` | `AdaptiveWindingService`, `MotionCoordinator` |
| `MachineRpcHandler` | `winding.jog`, `run_axis`, `home_lateral`, `clear_fault`, `stop`, `arm/disarm_endstop` | `MotionCommandService` |
| `ExecutionRpcHandler` | `winding.start_classic_session` (ex-`submit_program`), `wound_run`, `status`, `flush_until` | `WindingEngine` |

Chaque handler garde `register_all()` → pas de changement de contrat RPC.

**Risque :** moyen — refactoring structurel. À faire après A1/A2.

---

## 8. 🟠 C1 — `AdaptiveWindingSessionConfig` sans `from_payload()` — TypeError non descriptive

**Fichiers :** `rpi/winding/adaptive.py` · `rpi/jsonrpc/winding_handler.py`

```python
# Actuel — lève TypeError sur champ inconnu :
config = AdaptiveWindingSessionConfig(**session)
```

Ajouter `from_payload()` (même pattern que `WindingProgram`) :

```python
@classmethod
def from_payload(cls, payload: dict[str, Any]) -> AdaptiveWindingSessionConfig:
    if not isinstance(payload, dict):
        raise ValueError("session payload must be an object")
    known = {f.name for f in dataclasses.fields(cls)}
    return cls(**{k: v for k, v in payload.items() if k in known})
```

**Risque :** faible — quickwin indépendant des refactors A*.

---

## 9. 🟠 C2 — `update_session(**params)` — signature opaque

**Fichiers :** `rpi/jsonrpc/winding_handler.py` · `rpi/winding/service.py`

Les champs modifiables en live ne sont pas documentés dans la signature. Un champ inconnu déclenche une `TypeError` profonde au lieu d'un `JsonRpcError(-32602)`.

Après A0/A1/A3, les seuls champs live-adjustables sont ceux de `SessionParams` :

```python
_ALLOWED_UPDATE_FIELDS = frozenset({
    "spindle_rpm",
    "total_turns",
    "window_low_mm", "window_high_mm",
    # accel_s / decel_s : machine config, non modifiables en live
})

def update_session(self, **params: Any) -> dict[str, Any]:
    unknown = set(params) - _ALLOWED_UPDATE_FIELDS
    if unknown:
        raise JsonRpcError(-32602, f"Unknown session update fields: {sorted(unknown)}")
    ...
```

**Risque :** faible — quickwin.

---

## 10. 🟠 C3 — `rpc_result()` — exceptions socket non capturées

**Fichier :** `wendy/handlers.py`

`send_raw()` lève `RuntimeError` → HTTP 500 brut au lieu d'un corps JSON propre.

```python
try:
    response = self.application.rpc_client.send_raw(request_payload)
except RuntimeError as exc:
    self.write_json({"error": f"RPC transport error: {exc}"}, status=503)
    return False
```

**Risque :** faible — catch ciblé.

---

## 11. 🟡 D1 — `MultiAxisRampStreamer` — constantes de classe non injectables

**Fichier :** `rpi/transport/streamer.py`

Extraire les constantes de comportement en `StreamerTuning` dataclass injectée en `__init__`. Rend les tests d'intégration possibles sans sous-classer.

**Risque :** faible — rétrocompatible, `MoveQueue` est l'unique créateur.

---

## 12. 🟡 D2 — HTTP 502 pour erreurs client JSON-RPC (-32602)

**Fichier :** `wendy/handlers.py`

Retourner 400 pour les codes JSON-RPC `-32700`, `-32600`, `-32601`, `-32602`. Retourner 502 uniquement pour les erreurs serveur (`-32000` à `-32099`).

**Risque :** faible — peut casser des clients qui testent 502.

---

## 13. 🟡 D3 — Absence de tests unitaires sur les chemins critiques

Priorité minimale recommandée une fois A0/A1/A3 appliqués :

| Test | Valeur |
|---|---|
| `SessionParams.validate()` | spindle_rpm > 0 |
| `AdaptiveWindingRuntime` résolution total_turns/window | None → valeur du programme |
| `WindingProgram.from_payload()` avec champs inconnus | Non-régression |
| `plan_next_chunk()` | Rebond sur bord de fenêtre |
| `update_session()` avec champ inconnu | Vérifier -32602 au lieu de TypeError |
| `MachineConfig.from_config()` avec valeurs manquantes | Erreur explicite au démarrage |

---

## Synthèse et séquencement

```
Phase 1 — Introduire les trois couches (breaking)
  A0 : Extraire MachineConfig (axis IDs, accel/decel, homing)                 ✅ DONE
  A1 : Sortir spindle_rpm de WindingProgram → SessionParams                   ✅ DONE
  A2 : Supprimer WindingProgram.to_adaptive_session()                         ✅ DONE
  A3 : AdaptiveWindingSessionConfig → SessionParams + runtime (program,       ✅ DONE
       params, machine) — B1 absorbé                                          

Phase 2 — Nettoyer les interfaces (non-breaking une fois Phase 1 faite)
  B1 : Sans objet — absorbé par A3                                            ✓
  B2 : Fusionner start_session_from_program → start_session                   🟠
  B3 : Découper WindingRpcHandler en 4 handlers thématiques                  🟠

Phase 3 — Quickwins indépendants (applicables maintenant)
  C1 : from_payload() sur SessionParams (absorbé par A1/A3)                  ✅ DONE
  C2 : Valider les champs dans update_session()                               ✅ DONE
  C3 : Capturer RuntimeError dans rpc_result()                          🟠

Phase 4 — Améliorations de qualité
  D1 : StreamerTuning dataclass injectable                              🟡
  D2 : Codes HTTP corrects (400 vs 502) dans rpc_result()               🟡
  D3 : Tests unitaires Phase 1 + plan_next_chunk()                      🟡
```


---

## 1. 🔴 P1 — `AdaptiveWindingSessionConfig` sans `from_payload()` — TypeError non descriptive

**Fichier :** `rpi/jsonrpc/winding_handler.py` — `start_session()`

```python
# Actuel — lève TypeError si le payload contient un champ inconnu :
config = AdaptiveWindingSessionConfig(**session)
```

Contrairement à `WindingProgram.from_payload()` qui filtre les champs par `known_fields` et lève des erreurs métier claires, `AdaptiveWindingSessionConfig` n'a pas d'équivalent. Un client qui envoie un champ superflu (ex : `"source": "ui"`) reçoit :

```
TypeError: __init__() got an unexpected keyword argument 'source'
```

au lieu de `JsonRpcError(-32602, "Invalid params: unexpected field 'source'")`.

**Correction :**

Ajouter `from_payload()` sur `AdaptiveWindingSessionConfig` (même pattern que `WindingProgram`) :

```python
# winding/adaptive.py
@classmethod
def from_payload(cls, payload: dict[str, Any]) -> AdaptiveWindingSessionConfig:
    if not isinstance(payload, dict):
        raise ValueError("session payload must be an object")
    known = {f.name for f in dataclasses.fields(cls)}
    return cls(**{k: v for k, v in payload.items() if k in known})
```

Et dans `winding_handler.py` :
```python
config = AdaptiveWindingSessionConfig.from_payload(session)
```

**Risque :** faible — changement isolé, rend le comportement plus strict (erreurs clients deviennent 502 `Invalid params` au lieu de 500 `TypeError`).

---

## 2. 🟠 P2 — `WindingRpcHandler` gère deux domaines — violation SRP

**Fichier :** `rpi/jsonrpc/winding_handler.py` (~500 lignes)

La classe gère :
- CRUD des programmes (`program.list`, `program.get`, `program.save`, `program.update`, `program.delete`, `program.load`, `program.list_revisions`, `program.restore_revision`)
- Exécution du bobinage (`winding.submit_program`, `winding.start_session`, `winding.wound_run`, …)
- Contrôle machine (`winding.jog`, `winding.run_axis`, `winding.home_lateral`, …)

**Découpage recommandé :**

| Nouvelle classe | Méthodes | Dépendances |
|---|---|---|
| `ProgramRpcHandler` | `program.*` (8 méthodes) | `ProgramStore`, `SharedState`, `EventBus` |
| `SessionRpcHandler` | `winding.start_session`, `winding.update_session`, `winding.pause`, `winding.resume_session`, `winding.session_status`, `winding.start_session_from_program` | `AdaptiveWindingService`, `MotionCoordinator` |
| `MachineRpcHandler` | `winding.jog`, `winding.run_axis`, `winding.home_lateral`, `winding.move_lateral_mm`, `winding.clear_fault`, `winding.stop`, `winding.arm_endstop`, `winding.disarm_endstop` | `MotionCommandService` |
| `WindingRpcHandler` | `winding.submit_program`, `winding.wound_run`, `winding.status`, `winding.flush_until` | `WindingEngine`, reste |

Chaque handler appelle `register_all()` sur le même `RpcHandler`, donc pas de changement de contrat RPC.

**Risque :** moyen — refactoring structurel, nécessite de redistribuer `_resolve_program()` et `_coerce_program()` dans `ProgramRpcHandler`.

---

## 3. 🟠 P3 — `rpc_result()` dans `JsonRpcHandlerMixin` — exceptions socket non capturées

**Fichier :** `wendy/handlers.py` — `JsonRpcHandlerMixin.rpc_result()`

`send_raw()` dans `wendy/rpc.py` lève `RuntimeError` si le socket est mort ou en timeout. `rpc_result()` ne catch pas ces exceptions → le handler Tornado laisse remonter une exception Python → HTTP 500 sans corps JSON.

```python
# Actuel — exception non catchée :
response = self.application.rpc_client.send_raw(request_payload)
```

**Correction :**

```python
def rpc_result(self, method, *, params=None, success_status=200) -> bool:
    request_id = int(time.time() * 1000)
    request_payload = make_request(method, params=params, request_id=request_id)
    try:
        response = self.application.rpc_client.send_raw(request_payload)
    except RuntimeError as exc:
        self.write_json({"error": f"RPC transport error: {exc}"}, status=503)
        return False
    ...
```

HTTP 503 (Service Unavailable) est sémantiquement juste quand le socket RPC est mort.

**Risque :** faible — catch ciblé, améliore la résilience.

---

## 4. 🟠 P4 — `update_session(**params)` — signature opaque, validation impossible statiquement

**Fichier :** `rpi/jsonrpc/winding_handler.py` — `update_session()` + `rpi/winding/service.py` — `AdaptiveWindingService.update_session()`

```python
# Actuel — aucun champ documenté dans la signature :
def update_session(self, **params: Any) -> dict[str, Any]:
    if not params:
        raise JsonRpcError(-32602, ...)
    snapshot = self._adaptive_winding.update_session(**params)
```

Les champs acceptés (ex : `target_rpm`, `window_low_mm`, `window_high_mm`, `wire_diameter_mm`) ne sont pas vérifiables statiquement. Un champ inconnu déclenche une `TypeError` profonde.

**Correction :** Créer une `AdaptiveWindingSessionUpdate` dataclass ou TypedDict avec les champs optionnels connus, et valider explicitement dans `update_session()`.

```python
_ALLOWED_UPDATE_FIELDS = frozenset({
    "target_rpm", "window_low_mm", "window_high_mm",
    "wire_diameter_mm", "wire_awg", "turns_per_mm",
    "scatter_amplitude_mm", "scatter_damping_margin_mm",
    "scatter_freq1", "scatter_freq2",
})

def update_session(self, **params: Any) -> dict[str, Any]:
    unknown = set(params) - _ALLOWED_UPDATE_FIELDS
    if unknown:
        raise JsonRpcError(-32602, f"Unknown session update fields: {sorted(unknown)}")
    ...
```

**Risque :** faible — validation additionnelle, sans changement de comportement pour les clients valides.

---

## 5. 🟠 P5 — `to_adaptive_session()` dans `WindingProgram` — dépendance inversée

**Fichier :** `rpi/winding/program.py` — `WindingProgram.to_adaptive_session()`

Un modèle de données (`WindingProgram`) contient de la logique de conversion vers une couche service (`AdaptiveWindingSessionConfig`). Cela introduit une dépendance `program.py → adaptive.py` (couche modèle → couche service).

**Conséquence :** `program.py` importe `AdaptiveWindingSessionConfig` depuis `winding.adaptive`, ce qui empêche d'utiliser `WindingProgram` dans des contextes sans la couche adaptive (tests unitaires, CLI, etc.).

**Correction :**

Déplacer la fonction de conversion dans `winding_handler.py` ou dans un module utilitaire `winding/program_adapter.py` :

```python
# winding/program_adapter.py
def program_to_adaptive_session(
    program: WindingProgram,
    *,
    start_position_mm: float = 0.0,
    total_turns: float | None = None,
    chunk_time_s: float | None = None,
) -> AdaptiveWindingSessionConfig:
    ...
```

Et supprimer `to_adaptive_session()` de `WindingProgram`.

**Risque :** moyen — nécessite de mettre à jour les appelants (`winding_handler.py`, éventuels tests).

---

## 6. 🟡 P6 — `MultiAxisRampStreamer` — constantes de classe non configurables

**Fichier :** `rpi/transport/streamer.py`

Les paramètres de comportement du streamer (timeouts, seuils de buffer, profondeur de queue) sont des constantes de classe :

```python
class MultiAxisRampStreamer:
    TARGET_BUFFER_TIME_S = 0.10
    PREFILL_MAX_BUFFER_TIME_S = 0.50
    MIN_SEGMENT_TIME_S = 0.002
    MAX_SEGMENT_TIME_S = 0.005
    SEGMENT_QUEUE_DEPTH = 128
    ...
```

Pour les tests d'intégration ou pour adapter le comportement à une machine plus lente, il faut soit sous-classer, soit monkey-patcher les constantes — les deux sont fragiles.

**Correction :** Extraire en `StreamerTuning` dataclass injectée en `__init__` avec des valeurs par défaut correspondant aux constantes actuelles. Les tests peuvent passer une config différente.

```python
@dataclass
class StreamerTuning:
    target_buffer_time_s: float = 0.10
    prefill_max_buffer_time_s: float = 0.50
    min_segment_time_s: float = 0.002
    max_segment_time_s: float = 0.005
    segment_queue_depth: int = 128
    ...
```

**Risque :** faible — rétro-compatible si toutes les instanciations existantes passent par `MoveQueue` (seul créateur de `MultiAxisRampStreamer`).

---

## 7. 🟡 P7 — `JsonRpcHandlerMixin.rpc_result()` — code HTTP 502 pour erreur client (-32602)

**Fichier :** `wendy/handlers.py` — `rpc_result()`

```python
if "error" in response:
    self.write_json(response, status=502)
    return False
```

Tous les codes d'erreur JSON-RPC retournent 502, y compris `-32602` (Invalid params), qui est une erreur du client HTTP et devrait produire un 400.

**Correction :**

```python
if "error" in response:
    error_code = response.get("error", {}).get("code", 0)
    http_status = 400 if error_code in (-32700, -32600, -32601, -32602) else 502
    self.write_json(response, status=http_status)
    return False
```

| JSON-RPC code | Signification | HTTP suggéré |
|---|---|---|
| -32700 | Parse error | 400 |
| -32600 | Invalid request | 400 |
| -32601 | Method not found | 404 |
| -32602 | Invalid params | 400 |
| -32000 à -32099 | Server error | 502 |

**Risque :** faible (comportement plus correct), mais peut casser des clients qui testent le code 502 pour les erreurs de paramètres.

---

## 8. 🟡 P8 — `UnixJsonRpcClient` — timeout non configurable par appel depuis les handlers

**Fichier :** `wendy/rpc.py` — `UnixJsonRpcClient`

Le `timeout_s` est fixé à la construction du client (par défaut `10.0s`). Les handlers wendy appellent `send_raw()` sans `timeout_s` individuel. Pour des opérations longues comme `winding.home_lateral` (homing peut durer 30s), le timeout par défaut coupe la requête HTTP avant la fin de l'opération.

**Correction :**

- Utiliser `success_status=202` (déjà fait pour `MachineHomeHandler`) pour les endpoints à exécution longue — la réponse 202 confirme que l'opération est lancée, pas terminée.
- Documenter dans `rpc_result()` que les appels bloquants (qui attendent la fin de l'opération) ne doivent pas être utilisés depuis les handlers HTTP sans timeout explicite.
- Optionnellement, ajouter un paramètre `timeout_s` à `rpc_result()` pour les cas où le handler a besoin d'un timeout différent.

**Risque :** faible — principalement documentaire + optionnel.

---

## 9. 🟡 P9 — Absence de tests unitaires sur les chemins critiques

**Fichiers :** `rpi/winding/`, `rpi/core/`, `rpi/motion/`

Aucun dossier `tests/` ne contient de tests pour :
- `build_wound_move()` (factory, facile à tester)
- `WindingProgram.from_payload()` (filtrage, alias `id→program_id`)
- `AdaptiveWindingSessionConfig.validate()`
- `WindingProgram.to_adaptive_session()` (conversion)
- `plan_next_chunk()` (logique de planification des chunks)
- `MoveQueue` (machine d'état des moves)

La priorité minimale recommandée :

| Test | Valeur | Risque |
|---|---|---|
| `build_wound_move()` | Validation que les paramètres sont correctement mappés | Faible |
| `WindingProgram.from_payload()` avec champs inconnus | Non-régression | Faible |
| `plan_next_chunk()` | Vérification de la géométrie (rebond sur bord de fenêtre) | Moyen |
| `WindingRpcHandler.stop()` avec mode invalide | Vérification que 400 est renvoyé | Faible |

---

## Synthèse

| # | Fichier(s) | Problème | Priorité | Risque |
|---|---|---|---|---|
| P1 | `winding/adaptive.py`, `jsonrpc/winding_handler.py` | `AdaptiveWindingSessionConfig(**session)` lève `TypeError` sur champ inconnu | 🔴 | Faible |
| P2 | `jsonrpc/winding_handler.py` | Classe ~500 lignes, 3 domaines | 🟠 | Moyen |
| P3 | `wendy/handlers.py` | `rpc_result()` ne capture pas `RuntimeError` socket | 🟠 | Faible |
| P4 | `jsonrpc/winding_handler.py`, `winding/service.py` | `update_session(**params)` sans validation des champs | 🟠 | Faible |
| P5 | `winding/program.py` | `to_adaptive_session()` crée une dépendance inversée | 🟠 | Moyen |
| P6 | `transport/streamer.py` | Constantes de classe non injectables → tests difficiles | 🟡 | Faible |
| P7 | `wendy/handlers.py` | Code HTTP 502 pour erreurs client (Invalid params) | 🟡 | Faible |
| P8 | `wendy/rpc.py`, `wendy/handlers.py` | Timeout unique pour tous les appels, dont homing | 🟡 | Faible |
| P9 | `tests/` | Absence de tests sur les chemins critiques | 🟡 | Moyen |
