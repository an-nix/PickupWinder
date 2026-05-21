# Refactors Phase 4 — Améliorations de qualité

> État au 21 mai 2026. Phases 1–3 (A0–A3, B2, B3, C3) entièrement appliquées.
> Ce fichier liste les travaux restants.

---

## D1 — `MultiAxisRampStreamer` — constantes non injectables

**Fichier :** `src/rpi/transport/streamer.py`

**Problème :** les constantes de comportement du streamer (fill initial, taille de lot,
seuil de starvation, etc.) sont des littéraux éparpillés dans la classe. Impossible à
surcharger en test sans sous-classer.

**Approche :**

1. Créer un dataclass `StreamerTuning` dans le même module :

```python
@dataclass
class StreamerTuning:
    start_fill: int = 128          # STEP_STREAM_START_FILL
    part_size: int = 8             # PART_SIZE
    low_water_mark: int = 32       # seuil coast → resume
    batch_size: int = 16           # blocs envoyés par cycle de streaming
```

2. Injecter en `__init__` avec `tuning: StreamerTuning | None = None` (default → `StreamerTuning()`).
3. Remplacer les littéraux par `self._tuning.*`.

**Compatibilité :** `MoveQueue` est le seul créateur — aucun paramètre de construction
ne change côté appelant (le paramètre est optionnel).

**Risque :** faible.

---

## D2 — Codes HTTP incorrects dans `rpc_result()`

**Fichier :** `src/wendy/handlers.py`

**Problème :** tous les retours JSON-RPC en erreur mappent sur HTTP 502, y compris les
erreurs clientes (`-32602 Invalid params`). La RFC ne standardise pas le mapping HTTP
mais les conventions REST sont claires.

**Mapping recommandé :**

| Code JSON-RPC | Signification | HTTP |
|---|---|---|
| `-32700` | Parse error | 400 |
| `-32600` | Invalid request | 400 |
| `-32601` | Method not found | 404 |
| `-32602` | Invalid params | 400 |
| `-32000` à `-32099` | Server error | 502 |
| Autre | Erreur applicative | 502 |

**Approche :**

```python
ERROR_CODE_TO_HTTP = {
    -32700: 400,
    -32600: 400,
    -32601: 404,
    -32602: 400,
}

def _rpc_error_to_http(code: int) -> int:
    if code in ERROR_CODE_TO_HTTP:
        return ERROR_CODE_TO_HTTP[code]
    if -32099 <= code <= -32000:
        return 502
    return 502
```

**Risque :** faible, mais peut casser des clients wendy/windy qui testent le code 502
en dur pour les erreurs de validation. Vérifier les tests avant d'appliquer.

---

## D3 — Tests unitaires Phase 1

**Répertoire :** `tests/`

Priorité minimale — aucun test n'existe encore pour les classes A0–A3.

| Fichier de test | Cas à couvrir |
|---|---|
| `test_session_params.py` | `SessionParams.validate()` : `spindle_rpm <= 0` → `ValueError` |
| `test_session_params.py` | `SessionParams.validate()` : `window_low >= window_high` → `ValueError` |
| `test_adaptive_winding.py` | `AdaptiveWindingRuntime` : résolution `total_turns=None` → valeur du programme |
| `test_adaptive_winding.py` | `AdaptiveWindingRuntime` : résolution `window=None` → valeur du programme |
| `test_winding_program.py` | `WindingProgram.from_payload()` avec champ inconnu → pas d'erreur |
| `test_plan_next_chunk.py` | `plan_next_chunk()` : rebond sur bord de fenêtre |
| `test_session_handler.py` | `update_session()` avec champ inconnu → `JsonRpcError(-32602)` |
| `test_session_handler.py` | `update_session()` avec `spindle_rpm=0` → `JsonRpcError(-32602)` |

**Commande d'exécution :**

```bash
cd src/rpi && python -m pytest ../../tests/ -v
```

**Risque :** nul — tests seulement.

---

## Ordre recommandé

```
1. D2  — codes HTTP corrects dans wendy/handlers.py         (30 min)
2. D1  — StreamerTuning injectable                          (1 h)
3. D3  — tests unitaires                                    (ongoing)
```
