# Refactors restants — post Phase 1

> État au 21 mai 2026. Phase 1 (A0–A3) entièrement appliquée. Phase 2 (B2, B3) et Phase 3 (C3) appliquées.

---

## Stockage des fichiers

### Config machine — `AppConfiguration`

**Chemin :** `~/pickupwinder_data/config.json`

Défini dans `src/rpi/app/runtime.py` → `_default_config_file_path()` (via `_default_data_dir()`).
Format : JSON plat (`dataclasses.asdict(config)`), lu/écrit par `ConfigurationManager.load_configuration()` / `save_configuration()`.
Si le fichier est absent au démarrage, les valeurs par défaut du dataclass sont utilisées silencieusement.

### Programmes — `WindingProgram`

**Chemin :** `~/pickupwinder_data/programs/`

Défini dans `src/rpi/app/runtime.py` → `_default_program_store_dir()` (via `_default_data_dir()`).
Un fichier JSON par programme, nommé `{program_id}.json`.
Sous-dossier `.backup/` pour les révisions archivées à chaque `save_program()` (5 dernières conservées) et les suppressions (`{id}_rev{NNNN}_deleted.json`).

---

## Refactors restants

### Phase 2 — Nettoyer les interfaces *(non-breaking)*

#### ✅ B2 — Supprimer `start_session_from_program` — DONE

**Fichier :** `src/rpi/jsonrpc/winding_handler.py`

La méthode est déjà un thin wrapper sur `start_session` depuis A2/A3.
Il reste à supprimer la méthode et à coordonner avec les clients wendy qui l'appellent encore.

**Risque :** faible.

---

#### ✅ B3 — Découper `WindingRpcHandler` en 4 handlers thématiques — DONE

**Fichier :** `src/rpi/jsonrpc/winding_handler.py` (~540 lignes, 22 méthodes)

La classe fusionne trois domaines distincts. Après A0–A3 la frontière programme/session est nette — le découpage est naturel.

| Nouvelle classe | Méthodes | Dépendances |
|---|---|---|
| `ProgramRpcHandler` | `list_programs`, `get_program`, `save_program`, `update_program`, `delete_program`, `list_revisions`, `restore_revision` | `ProgramStore`, `SharedState`, `EventBus` |
| `SessionRpcHandler` | `start_session`, `start_session_from_program`¹, `update_session`, `pause`, `pause_session`, `resume_session` | `AdaptiveWindingService`, `MotionCoordinator` |
| `MachineRpcHandler` | `jog`, `run_axis`, `home_lateral`, `clear_fault`, `arm_endstop`, `disarm_endstop` | `MotionCommandService` |
| `ExecutionRpcHandler` | `submit_program`, `wound_run`, `flush_until`, `status`, `stop` | `WindingEngine` |

¹ À supprimer après B2.

Chaque classe garde `register_all()` — le contrat RPC ne change pas.

**Risque :** moyen — refactoring structurel pur, aucun changement de comportement.

---

### Phase 3 — Quickwins indépendants

#### ✅ C3 — `rpc_result()` — `RuntimeError` non capturée — DONE

**Fichier :** `src/wendy/handlers.py` (ligne ~48)

`send_raw()` peut lever `RuntimeError` (ex : socket inaccessible) → HTTP 500 brut sans corps JSON au lieu d'une réponse structurée.

```python
# Actuel
response = self.application.rpc_client.send_raw(request_payload)

# Corrigé
try:
    response = self.application.rpc_client.send_raw(request_payload)
except RuntimeError as exc:
    self.write_json({"error": f"RPC transport error: {exc}"}, status=503)
    return False
```

**Risque :** faible — catch ciblé, 3 lignes.

---

### Phase 4 — Améliorations de qualité

#### D1 — `MultiAxisRampStreamer` — constantes non injectables

**Fichier :** `src/rpi/transport/streamer.py`

Extraire les constantes de comportement en `StreamerTuning` dataclass injectée en `__init__`.
Rend les tests d'intégration possibles sans sous-classer.

**Risque :** faible — rétrocompatible, `MoveQueue` est l'unique créateur.

---

#### D2 — Codes HTTP incorrects dans `rpc_result()`

**Fichier :** `src/wendy/handlers.py`

Retourner 400 pour les codes JSON-RPC côté client (`-32700`, `-32600`, `-32601`, `-32602`).
Retourner 502/503 uniquement pour les erreurs serveur (`-32000` à `-32099`).

**Risque :** faible — peut casser des clients qui testent le code 502 en dur.

---

#### D3 — Tests unitaires Phase 1

Priorité minimale une fois A0/A1/A3 appliqués :

| Test | Valeur |
|---|---|
| `SessionParams.validate()` | `spindle_rpm > 0` |
| `AdaptiveWindingRuntime` résolution `total_turns`/`window` | `None` → valeur du programme |
| `WindingProgram.from_payload()` avec champs inconnus | Non-régression |
| `plan_next_chunk()` | Rebond sur bord de fenêtre |
| `update_session()` avec champ inconnu | Vérifie `-32602` au lieu de `TypeError` |

---

## Ordre recommandé

```
✅ C3  — catch RuntimeError dans wendy/handlers.py
✅ B2  — supprimer start_session_from_program
✅ B3  — découper WindingRpcHandler en 4 classes
1. D2  — codes HTTP corrects dans wendy                     (30 min)
2. D1  — StreamerTuning injectable                          (1 h)
3. D3  — tests unitaires                                    (ongoing)
```
