# Pistes de refactoring — PickupWinder

> Analyse complémentaire après application des refactors de `usage.md` et `winding_program.md`.
> Mis à jour mai 2026 — intègre la clarification architecturale Programme / Session.

---

## 0. Vision cible — Programme vs Session

### Principe

| Concept | Rôle | Persisté ? | Mutable en live ? |
|---|---|---|---|
| **WindingProgram** | *Quoi* — recette de bobinage | ✅ `ProgramStore` | ✗ (versionné par révisions) |
| **Session d'exécution** | *Comment / à quelle vitesse* — contexte d'exécution | ✗ transient | ✅ (mode adaptatif) |

**Un programme ne peut être lancé qu'au travers d'une session.**  
La session porte les paramètres qui relèvent de *l'acte d'exécution*, pas de la recette :
- **Vitesse** (`spindle_rpm` / `target_rpm`) — à quelle vitesse on bob → **session**
- **Profil de rampe** (`accel_s`, `decel_s`) — comment on accélère → **session**
- **Paramètres temps-réel** (`chunk_time_s`, `window_low_mm/high_mm`) → **session adaptative**

Le programme ne contient que la géométrie de la bobine et la configuration physique :
- Dimensions : `bobbin_width_mm`, `num_layers`, `layer_pitch_mm`, `wire_diameter_mm`
- Technique : scatter (`amplitude`, `freq1/2`, `damping`), IDs d'axes
- Setup homing : `home_before_start`, `home_approach_rpm`, `home_search_rpm`, `home_backoff_steps`

### État actuel — le mélange

```
WindingProgram (actuel)               Session cible
─────────────────────────             ──────────────────
name, program_id, revision    →       (reste dans le programme)
num_layers, bobbin_width_mm   →       (reste dans le programme)
layer_pitch_mm, wire_diameter →       (reste dans le programme)
scatter_amplitude_mm/freq…    →       (reste dans le programme)
spindle_axis_id, lateral_id   →       (reste dans le programme)
home_before_start, home_rpm…  →       (reste dans le programme)
spindle_rpm           ❌       →       WindingExecutionParams.spindle_rpm
accel_s, decel_s      ❌       →       WindingExecutionParams.accel_s/decel_s
```

```
AdaptiveWindingSessionConfig (actuel)  Rôle réel
───────────────────────────────────   ─────────────────────────────────────
name                          →       hérité du programme
scatter_*, spindle_axis_id…   →       dupliqués depuis WindingProgram ❌
target_rpm, chunk_time_s      →       paramètres de session ✅
window_low_mm, window_high_mm →       paramètres de session ✅
total_turns                   →       dérivé du programme ou override ✅
```

### Modèle cible

```
WindingProgram               WindingExecutionParams         AdaptiveExecutionControls
─────────────────            ──────────────────────         ─────────────────────────
name, program_id             spindle_rpm                    target_rpm  (live)
num_layers                   accel_s                        window_low_mm  (live)
bobbin_width_mm              decel_s                        window_high_mm  (live)
layer_pitch_mm                                              wire_diameter_mm  (live)
wire_diameter_mm                                            chunk_time_s
scatter_*
spindle_axis_id                     ┌───────────────────┐
lateral_axis_id                     │  WindingSession    │
home_before_start                   │  (transient)       │
home_approach_rpm     ─────────────►│  program + params  │
home_search_rpm                     │  mode: classic     │
home_backoff_steps                  │       | adaptive   │
                                    └───────────────────┘
```

### Deux modes de session — même objet racine

```
winding.start_session(program_id="...", rpm=1200, accel_s=0.5, decel_s=0.5)
   → Classic session : exécute N couches déterministes depuis le programme

winding.start_session(program_id="...", rpm=1200, mode="adaptive", total_turns=5000)
   → Adaptive session : live-controllable, chunks, fenêtre adjustable
```

---

## 1. 🔴 A1 — Sortir `spindle_rpm`, `accel_s`, `decel_s` de `WindingProgram`

**Fichiers :** `rpi/winding/program.py` · `rpi/core/engine.py` · `rpi/jsonrpc/winding_handler.py`

### Problème

`WindingProgram` porte `spindle_rpm`, `accel_s`, `decel_s`. Ces valeurs contrôlent la dynamique d'exécution, pas la géométrie de la bobine. Deux programmes identiques en géométrie mais bobinés à des vitesses différentes sont aujourd'hui deux objets `WindingProgram` distincts persistés séparément.

`WindingEngine._run_layer()` lit `program.spindle_rpm` directement :

```python
# core/engine.py — actuel : la vitesse vient du programme
move = build_wound_move(
    spindle_rpm=program.spindle_rpm,     # ← appartient à la session
    accel_s=program.accel_s,             # ← appartient à la session
    decel_s=program.decel_s,             # ← appartient à la session
    bobbin_width_mm=program.bobbin_width_mm,  # ← OK, géométrie
    ...
)
```

### Correction

**Étape 1 :** Créer `WindingExecutionParams` dans `winding/program.py` (ou `winding/session.py`) :

```python
@dataclass(slots=True)
class WindingExecutionParams:
    spindle_rpm: float
    accel_s: float = 0.5
    decel_s: float = 0.5

    def validate(self) -> None:
        if self.spindle_rpm <= 0.0:
            raise ValueError("spindle_rpm must be positive")
        if self.accel_s < 0.0 or self.decel_s < 0.0:
            raise ValueError("accel_s and decel_s must be >= 0")
```

**Étape 2 :** Supprimer `spindle_rpm`, `accel_s`, `decel_s` de `WindingProgram`.

**Étape 3 :** `WindingEngine.submit_program()` → `WindingEngine.start_classic_session()` :

```python
def start_classic_session(
    self,
    program: WindingProgram,
    execution: WindingExecutionParams,
) -> None:
    # ...
```

**Étape 4 :** `winding.submit_program` RPC accepte les params d'exécution :

```python
# winding_handler.py
def submit_program(
    self,
    program_id: str | None = None,
    program: dict | None = None,
    spindle_rpm: float | None = None,
    accel_s: float = 0.5,
    decel_s: float = 0.5,
    ...
) -> dict:
    p = self._resolve_program(program=program, program_id=program_id)
    if spindle_rpm is None:
        raise JsonRpcError(-32602, "spindle_rpm is required")
    execution = WindingExecutionParams(spindle_rpm=spindle_rpm, accel_s=accel_s, decel_s=decel_s)
    self._engine.start_classic_session(p, execution)
    return {"status": "queued"}
```

**Rétrocompatibilité :** Les programmes sauvegardés avec `spindle_rpm` dans leur JSON continueront à se charger via `from_payload()` — le champ sera ignoré (pattern déjà en place). Les clients qui ne passent pas encore `spindle_rpm` via RPC recevront un `-32602`.

**Risque :** moyen — change le contrat RPC de `winding.submit_program`. Pas de changement binaire ESP32. Migrer les clients (wendy handlers) en même temps.

---

## 2. 🔴 A2 — Supprimer `to_adaptive_session()` de `WindingProgram`

**Fichier :** `rpi/winding/program.py`

### Problème

`WindingProgram.to_adaptive_session()` (méthode de 30 lignes) importe et instancie `AdaptiveWindingSessionConfig` depuis `winding.adaptive`. Conséquences :

1. `program.py` dépend de `adaptive.py` — une recette persistée dépend d'un moteur d'exécution live.
2. La logique qui décide de `window_low_mm = start_position_mm` et `window_high_mm = start_position_mm + bobbin_width_mm` est cachée dans le modèle de données au lieu d'être visible dans la couche RPC.
3. Depuis A1, `WindingProgram` n'a plus de vitesse — mais `to_adaptive_session()` reçoit les `execution_params` en dehors, ce qui renforce l'idée que cette conversion n'appartient pas au programme.

### Correction

Déplacer la conversion dans `winding_handler.py` ou un module `winding/session_factory.py` :

```python
# winding/session_factory.py
def program_to_adaptive_session(
    program: WindingProgram,
    execution: WindingExecutionParams,
    *,
    start_position_mm: float,
    total_turns: float | None = None,
    chunk_time_s: float = 0.25,
) -> AdaptiveWindingSessionConfig:
    program.validate()
    return AdaptiveWindingSessionConfig(
        name=program.name,
        total_turns=program.total_turns() if total_turns is None else float(total_turns),
        target_rpm=execution.spindle_rpm,        # ← vient de la session, pas du programme
        window_low_mm=float(start_position_mm),
        window_high_mm=float(start_position_mm) + program.bobbin_width_mm,
        wire_diameter_mm=program.wire_diameter_mm,
        pitch_factor=program.layer_pitch_mm / program.wire_diameter_mm,
        scatter_amplitude_mm=program.scatter_amplitude_mm,
        scatter_damping_margin_mm=program.scatter_damping_margin_mm,
        scatter_freq1=program.scatter_freq1,
        scatter_freq2=program.scatter_freq2,
        spindle_axis_id=program.spindle_axis_id,
        lateral_axis_id=program.lateral_axis_id,
        home_before_start=program.home_before_start,
        home_approach_rpm=program.home_approach_rpm,
        home_search_rpm=program.home_search_rpm,
        home_backoff_steps=program.home_backoff_steps,
        chunk_time_s=chunk_time_s,
    )
```

Mettre à jour `winding_handler.start_session_from_program()` pour appeler cette fonction.

**Risque :** faible — refactoring pur, comportement identique.

---

## 3. 🔴 A3 — `AdaptiveWindingSessionConfig` duplique la géométrie du programme

**Fichier :** `rpi/winding/adaptive.py`

### Problème

`AdaptiveWindingSessionConfig` répète les champs de géométrie qui viennent du programme : `scatter_*`, `spindle_axis_id`, `lateral_axis_id`, `home_before_start`, `home_approach_rpm`, `home_search_rpm`, `home_backoff_steps`. Quand ces valeurs changent dans un programme, elles ne se propagent pas automatiquement dans les sessions futures.

Ce sont des *paramètres programme* copiés dans une session. La session devrait juste référencer le programme et n'apporter que ce qui lui est propre.

### Correction

**Cible finale :** `AdaptiveWindingRuntime` reçoit `(program, params)` — la géométrie reste dans `WindingProgram`, les paramètres d'exécution dans `AdaptiveSessionParams`.

**Étape 1 :** Créer `AdaptiveSessionParams` qui ne contient que les paramètres propres à la session :

```python
@dataclass(slots=True)
class AdaptiveSessionParams:
    target_rpm: float
    window_low_mm: float
    window_high_mm: float
    total_turns: float | None = None       # None → dérivé de WindingProgram.total_turns()
    chunk_time_s: float = 0.25
    # overrides optionnels — le programme reste la source de vérité
    wire_diameter_mm: float | None = None
    wire_awg: float | None = None
    turns_per_mm_override: float | None = None
    pitch_factor: float | None = None
```

**Étape 2 :** Mettre à jour `AdaptiveWindingRuntime.__init__()` :

```python
# Avant
def __init__(self, config: AdaptiveWindingSessionConfig):
    self._cfg = config

# Après
def __init__(self, program: WindingProgram, params: AdaptiveSessionParams):
    self._program = program
    self._params = params
    # géométrie résolue depuis le programme, overridable par params
    self.wire_diameter_mm = params.wire_diameter_mm or program.wire_diameter_mm
    self.pitch_factor = params.pitch_factor or (program.layer_pitch_mm / program.wire_diameter_mm)
    self.total_turns = params.total_turns if params.total_turns is not None else program.total_turns()
```

**Étape 3 :** Supprimer de `AdaptiveWindingSessionConfig` les champs qui appartiennent au programme :
- `scatter_amplitude_mm`, `scatter_damping_margin_mm`, `scatter_freq1`, `scatter_freq2`
- `spindle_axis_id`, `lateral_axis_id`
- `home_before_start`, `home_approach_rpm`, `home_search_rpm`, `home_backoff_steps`
- `wire_diameter_mm`, `pitch_factor`, `name` (portés par le programme ou dans `AdaptiveSessionParams`)

`AdaptiveWindingSessionConfig` devient `AdaptiveSessionParams` — le renommage B1 est absorbé par A3.

**Articulation avec A2 :** la factory `program_to_adaptive_session()` de A2 devient `build_adaptive_session_params()` — elle n'extrait plus que les overrides de session depuis le payload RPC. `AdaptiveWindingRuntime` lit la géométrie directement depuis `WindingProgram`.

**Risque :** élevé — change l'interface de `AdaptiveWindingRuntime.__init__()` et de `AdaptiveWindingService.start_session()`. À planifier après A1 et A2.

---

## 4. 🟠 B1 — Renommer `AdaptiveWindingSessionConfig` → `AdaptiveSessionParams` *(absorbé par A3)*

**Fichier :** `rpi/winding/adaptive.py` + tous les importeurs

Ce renommage fait partie intégrante de A3 : une fois les champs programme retirés, la classe restante s'appelle `AdaptiveSessionParams` (paramètres purs de session). Aucune étape supplémentaire n'est nécessaire ici — appliquer via `vscode_renameSymbol` dans le même PR que A3.

---

## 5. 🟠 B2 — `winding.start_session_from_program` devient redondant après A2

**Fichier :** `rpi/jsonrpc/winding_handler.py`

Une fois A2 appliqué, `start_session_from_program` est juste `start_session` avec `program_id` + paramètres d'exécution. Le endpoint devient redondant.

**Cible :** un seul endpoint `winding.start_session` qui accepte soit :
- `program_id` + `spindle_rpm` → session adaptative depuis programme stocké
- `program` inline + `spindle_rpm` → session adaptative depuis payload
- `window_low_mm` / `window_high_mm` + géométrie inline → session free-form (dev/debug)

Garder `start_session_from_program` comme alias déprecié.

**Risque :** moyen — change le contrat RPC. À coordonner avec les clients (wendy).

---

## 6. 🟠 B3 — `WindingRpcHandler` gère trois domaines — violation SRP

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

## 7. 🟠 C1 — `AdaptiveWindingSessionConfig` sans `from_payload()` — TypeError non descriptive

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

## 8. 🟠 C2 — `update_session(**params)` — signature opaque

**Fichiers :** `rpi/jsonrpc/winding_handler.py` · `rpi/winding/service.py`

Les champs modifiables en live ne sont pas documentés dans la signature. Un champ inconnu déclenche une `TypeError` profonde au lieu d'un `JsonRpcError(-32602)`.

```python
_ALLOWED_UPDATE_FIELDS = frozenset({
    "target_rpm", "window_low_mm", "window_high_mm",
    "wire_diameter_mm", "wire_awg", "turns_per_mm",
    "pitch_factor", "scatter_amplitude_mm", "scatter_damping_margin_mm",
    "scatter_freq1", "scatter_freq2",
})

def update_session(self, **params: Any) -> dict[str, Any]:
    unknown = set(params) - _ALLOWED_UPDATE_FIELDS
    if unknown:
        raise JsonRpcError(-32602, f"Unknown session update fields: {sorted(unknown)}")
    ...
```

**Risque :** faible — quickwin.

---

## 9. 🟠 C3 — `rpc_result()` — exceptions socket non capturées

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

## 10. 🟡 D1 — `MultiAxisRampStreamer` — constantes de classe non injectables

**Fichier :** `rpi/transport/streamer.py`

Extraire les constantes de comportement en `StreamerTuning` dataclass injectée en `__init__`. Rend les tests d'intégration possibles sans sous-classer.

**Risque :** faible — rétrocompatible, `MoveQueue` est l'unique créateur.

---

## 11. 🟡 D2 — HTTP 502 pour erreurs client JSON-RPC (-32602)

**Fichier :** `wendy/handlers.py`

Retourner 400 pour les codes JSON-RPC `-32700`, `-32600`, `-32601`, `-32602`. Retourner 502 uniquement pour les erreurs serveur (`-32000` à `-32099`).

**Risque :** faible — peut casser des clients qui testent 502.

---

## 12. 🟡 D3 — Absence de tests unitaires sur les chemins critiques

Priorité minimale recommandée une fois A1/A2 appliqués :

| Test | Valeur |
|---|---|
| `WindingExecutionParams.validate()` | Validation vitesse/rampes |
| `program_to_adaptive_session()` | Mapping géométrie + vitesse |
| `WindingProgram.from_payload()` avec champs inconnus | Non-régression |
| `plan_next_chunk()` | Rebond sur bord de fenêtre |
| `update_session()` avec champ inconnu | Vérifier -32602 au lieu de TypeError |

---

## Synthèse et séquencement

```
Phase 1 — Séparer Programme et Session (breaking)
  A1 : Sortir spindle_rpm/accel_s/decel_s de WindingProgram                    🔴
  A2 : Déplacer to_adaptive_session() → session_factory.py                    🔴
  A3 : AdaptiveWindingSessionConfig → AdaptiveSessionParams (+ runtime split)  🔴
       (inclut le renommage B1)

Phase 2 — Nettoyer les interfaces (non-breaking une fois Phase 1 faite)
  B1 : Renommage — absorbé par A3                                              ✓
  B2 : Fusionner start_session_from_program → start_session                    🟠
  B3 : Découper WindingRpcHandler en 4 handlers thématiques                   🟠

Phase 3 — Quickwins indépendants (applicables maintenant)
  C1 : Ajouter from_payload() sur AdaptiveWindingSessionConfig          🟠
  C2 : Valider les champs dans update_session()                         🟠
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
