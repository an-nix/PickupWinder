# Architecture BeagleBone Black — PickupWinder

> Branch `beagle` — migration ESP32+FreeRTOS → BeagleBone Black (AM335x) avec PRU.
> Architecture **Option A** : paramètres continus partagés (pas de move rings).

---

## 1. Vue d'ensemble

Le BeagleBone Black expose deux **PRU** (Programmable Real-time Units, 200 MHz,
déterministes, 5 ns/cycle) aux côtés d'un **ARM Cortex-A8** exécutant Linux.

Architecture à 4 couches.  Le host écrit des commandes (vitesses cibles,
rampes d'accélération, moves) ; PRU0 orchestre et gère les segments de rampe ;
PRU1 génère les impulsions comme un pilote moteur aveugle.

```
┌───────────────────────────────────────────────────────────────────┐
│  Couche 4 — Application Python (asyncio, ARM Linux)               │
│  PruClient · pickup_test.py · (futur: WinderApp, WebUI, ...)      │
│  Envoie commandes JSON au daemon, reçoit télémétrie + événements  │
│                      │ Unix socket /run/pickup-winder.sock        │
├──────────────────────▼────────────────────────────────────────────┤
│  Couche 3 — Daemon C matériel (pickup_daemon, ARM Linux)          │
│  Mappe PRU Shared RAM via /dev/mem                                │
│  Pré-calcule les rampes Klipper (ramp_seg_t[16] par axe)          │
│  Écrit host_cmd_t + segments dans shared RAM                      │
│  Lit pru_status_t (statut agrégé de PRU0)                         │
│  Polling 10 ms, broadcast telem + events vers Python              │
│  Communique UNIQUEMENT avec PRU0                                  │
│                      │ /dev/mem mmap (PRU Shared RAM 0x4A310000)  │
├──────────────────────▼────────────────────────────────────────────┤
│  Couche 2 — PRU0 orchestration (200 MHz, 5 ns/cycle)              │
│  Lit host_cmd_t du daemon (via shared RAM)                        │
│  Machine d'état homing (IDLE → APPROACH → HIT)                    │
│  Gestion séquentielle des segments de rampe (ramp_tick)           │
│  Coordination spindle ↔ latéral (Q6 ratio)                       │
│  Écrit motor_params_t (motor_ctl_t par axe : interval, ramp_arm)  │
│  Lit motor_telem_t de PRU1 (seg_done, step_count, position)      │
│  Publie pru_status_t (statut agrégé pour le daemon)               │
│  PAS de contrôle moteur. PAS de STEP/DIR/EN.                      │
│                      │ PRU Shared RAM                             │
├──────────────────────▼────────────────────────────────────────────┤
│  Couche 1 — PRU1 contrôle moteur (200 MHz, propriétaire IEP)      │
│  Lit motor_params_t de PRU0 en continu                            │
│  Détecte ramp_arm=1 → arme pulse_set_ramp → clear ramp_arm=0     │
│  Génération d'impulsions IEP : pulse_gen_t par axe                │
│  GPIO STEP/DIR/EN pour spindle + lateral (PRU1 R30)               │
│  Quand seg terminé (accel_count==0) → seg_done=1 dans telem      │
│  Publie motor_telem_t (compteurs pas, positions, seg_done)        │
│  PILOTE MOTEUR AVEUGLE — aucune logique homing, aucune commande   │
└───────────────────────────────────────────────────────────────────┘
```

**Flux de communication** (608 octets en shared RAM) :
```
Host ──host_cmd_t + ramp_seg_t[16]×2──→ PRU0 ──motor_params_t──→ PRU1
                                         PRU0 ←──motor_telem_t── PRU1
Host ←─pru_status_t─────────────────── PRU0
```

---

## 2. Règle de découpage

| Critère                                          | PRU1 (L1) | PRU0 (L2) | Linux (L3-4) |
|--------------------------------------------------|:---------:|:---------:|:------------:|
| Latence < 1 µs (timing IEP des edges)           | ✓         |           |              |
| Génération STEP/DIR/ENABLE                       | ✓         |           |              |
| Comptage pas / position                          | ✓         |           |              |
| Exécution `interval += add` (boucle Klipper)     | ✓         |           |              |
| Détection seg_done (accel_count==0)              | ✓         |           |              |
| Publication motor_telem_t                        | ✓         |           |              |
| Traitement commandes host (host_cmd_t)           |           | ✓         |              |
| Machine d'état homing (IDLE/APPROACH/HIT)        |           | ✓         |              |
| Gestion séquentielle segments rampe (ramp_tick)  |           | ✓         |              |
| Coordination spindle ↔ latéral (coord_tick)      |           | ✓         |              |
| Lecture endstops (R31) + arrêt de sécurité       |           | ✓         |              |
| Écriture motor_params_t + ramp_arm handshake     |           | ✓         |              |
| Publication pru_status_t                         |           | ✓         |              |
| Détection événements → event_pending             |           | ✓         |              |
| Conversion Hz → intervalles IEP                  |           |           | ✓            |
| Calcul rampes Klipper (ramp_seg_t[16])           |           |           | ✓            |
| Protocole JSON socket (cmds/events/telem)        |           |           | ✓            |
| Machine d'état winding (IDLE/PAUSED/WINDING/…)   |           |           | ✓            |
| UI WebSocket / REST / UART                       |           |           | ✓            |
| Persistance recettes (JSON/fichiers)             |           |           | ✓            |
| Session arbitration (pot/IHM/footswitch)         |           |           | ✓            |

**Principes absolus** :
- Aucun flottant, aucune alloc dynamique, aucun appel OS dans les PRU.
- Pas de division dans la boucle PRU (intervalles et segments pré-calculés côté ARM).
- PRU1 = pilote moteur aveugle (ne connaît ni les commandes, ni le homing, ni les segments).
- PRU0 = cerveau/orchestrateur (gère les segments un par un via ramp_arm handshake).
- Le daemon ne parle qu'à PRU0 (jamais directement à PRU1).

---

## 3. IPC — Mémoire Partagée PRU ↔ Linux

Zone d'échange : **PRU Shared RAM** (AM335x)
- Adresse PRU locale : `0x00010000`
- Adresse physique hôte : `0x4A310000`
- Taille : 12 KB (608 octets utilisés)

### 3.1 Plan mémoire

```
Offset   Taille  Struct              Direction       Description
0x0000   64 o    host_cmd_t          Host → PRU0     Commandes + paramètres
0x0040   32 o    motor_params_t      PRU0 → PRU1     motor_ctl_t[2] (interval, ramp_arm, etc.)
0x0060   64 o    motor_telem_t       PRU1 → PRU0     motor_t[2] (step_count, position, seg_done)
0x00A0   64 o    pru_status_t        PRU0 → Host     Statut agrégé pour daemon broadcast
0x00E0   192 o   ramp_seg_t[16]      Host → PRU0     Segments rampe axe 0 (spindle)
0x01A0   192 o   ramp_seg_t[16]      Host → PRU0     Segments rampe axe 1 (latéral)
Total :  608 octets
```

### 3.2 Abstraction moteur (`pru_ipc.h`)

**Convention d'index :**
- `MOTOR_0` = 0 → spindle (Motor B, pins paires P8)
- `MOTOR_1` = 1 → latéral (Motor A, pins impaires P8)

**Structures clés (version actuelle) :**

```c
/* motor_t — télémétrie universelle par moteur (24 octets) */
typedef struct {
    uint32_t interval;          /* intervalle IEP courant                    */
    uint8_t  dir, enable, run;  /* état moteur                               */
    uint8_t  seg_done;          /* 1 = segment rampe terminé (PRU1→PRU0)     */
    uint32_t step_count;        /* pas depuis dernier reset                  */
    int32_t  position;          /* position signée (pas)                     */
    uint32_t interval_actual;   /* intervalle IEP mesuré                     */
    uint8_t  state, faults;     /* MOTOR_STATE_*, FAULT_*                    */
    uint8_t  _pad[2];
} motor_t;

/* motor_ctl_t — contrôle par moteur PRU0→PRU1 (16 octets) */
typedef struct {
    uint32_t interval;          /* intervalle IEP / start_iv rampe           */
    uint8_t  dir, enable, run;  /* commandes                                 */
    uint8_t  ramp_arm;          /* 1 = nouveau segment prêt (PRU0→PRU1)      */
    int32_t  ramp_add;          /* delta intervalle par pas (Klipper)        */
    uint32_t ramp_count;        /* nombre de pas dans le segment             */
} motor_ctl_t;

/* motor_params_t — PRU0 → PRU1 (32 octets) */
typedef struct {
    motor_ctl_t motor[2];       /* spindle + latéral                         */
} motor_params_t;

/* motor_telem_t — PRU1 → PRU0 (64 octets) */
typedef struct {
    motor_t  motor[2];          /* 48 o — télémétrie par moteur              */
    uint8_t  endstop_mask;      /* bit0=ES1, bit1=ES2                        */
    uint8_t  _pad[3];
    uint32_t seq;               /* compteur monotone                         */
    uint32_t _reserved[2];
} motor_telem_t;

/* pru_status_t — PRU0 → Host (64 octets) */
typedef struct {
    uint32_t seq;
    uint8_t  pru1_state, event_pending, event_type, endstop_mask;
    motor_t  motor[2];          /* 48 o — copie de la télémétrie             */
    uint32_t _reserved[2];
} pru_status_t;

/* ramp_seg_t — segment de rampe universel (12 octets) */
typedef struct {
    uint32_t start_iv;          /* intervalle forcé au début du segment      */
    int32_t  add;               /* delta signé par pas (0 = cruise)          */
    uint32_t count;             /* nombre de pas (0 = skip)                  */
} ramp_seg_t;
```

### 3.3 Opcodes de commande

| Opcode                | Valeur | Description |
|-----------------------|--------|-------------|
| `HOST_CMD_NOP`        | 0      | Idle / acquitté |
| `HOST_CMD_SET_SPEED`  | 1      | Vitesse constante (intervalle direct) |
| `HOST_CMD_ENABLE`     | 2      | Activer/désactiver drivers |
| `HOST_CMD_ESTOP`      | 3      | Arrêt d'urgence immédiat |
| `HOST_CMD_HOME_START` | 4      | Démarrer séquence homing latéral |
| `HOST_CMD_ACK_EVENT`  | 5      | Acquitter dernier événement PRU |
| `HOST_CMD_RESET_POS`  | 6      | Remettre compteurs/position à zéro |
| `HOST_CMD_SET_LIMITS` | 7      | Définir limites logicielles (min, max) |
| `HOST_CMD_MOVE_TO`    | 8      | Mouvement position (trapèze) |
| `HOST_CMD_RAMP_TO`    | 9      | Rampe vitesse (accélération Klipper) |

### 3.4 Événements (PRU0 → Host)

| Événement              | Code | Condition |
|------------------------|------|-----------|
| `EVENT_ENDSTOP_HIT`    | 1    | Fin de course activé (hors homing) |
| `EVENT_HOME_COMPLETE`  | 2    | Homing terminé |
| `EVENT_FAULT`          | 3    | Fault moteur détecté |
| `EVENT_LIMIT_HIT`      | 4    | Limite logicielle atteinte |
| `EVENT_MOVE_COMPLETE`  | 5    | Profil move_to terminé |
| `EVENT_RAMP_COMPLETE`  | 6    | Rampe ramp_to terminée |

### 3.5 Protocole commande/acquittement

1. Host écrit `cmd` dans `host_cmd_t` (+ paramètres + segments si RAMP_TO/MOVE_TO).
2. PRU0 lit `cmd`, exécute, copie l'opcode dans `cmd_ack`, remet `cmd = NOP`.
3. Host poll `cmd` — quand il voit `NOP`, la commande est acquittée.
4. Host ne doit PAS écrire de nouvelle commande avant acquittement.

---

## 4. PRU1 — Pilote Moteur Aveugle (Couche 1)

**Propriétaire du timer IEP** : PRU1 initialise le compteur IEP au démarrage.
PRU0 le lit seulement — ne le réinitialise jamais.

### 4.1 Responsabilités

PRU1 est un générateur d'impulsions minimal (~180 lignes de C).  Il ne connaît
ni les commandes host, ni le homing, ni les tableaux de segments.

1. Détecter `ramp_arm=1` → armer `pulse_set_ramp()` → clear `ramp_arm=0`
2. Lire `motor_ctl_t` : interval, dir, enable, run
3. Appliquer STEP/DIR/EN sur R30
4. `pulse_update()` par axe : génération IEP continue avec `interval += add`
5. Quand segment terminé (`accel_count==0`) : écrire l'intervalle final dans
   `params` (prévient le saut de vitesse) et `seg_done=1` dans telem
6. Publier `motor_telem_t` (~400 Hz)

### 4.2 Moteur de génération d'impulsions

Le `pulse_gen_t` (dans `pru_stepper.h`) gère la génération continue
d'impulsions basée sur le timer IEP :

```c
typedef struct {
    uint32_t next_edge_time;    /* timestamp IEP pour prochain toggle STEP  */
    uint32_t interval;          /* cycles IEP courant entre edges           */
    uint32_t step_count;        /* pas total depuis dernier reset           */
    int32_t  position;          /* position signée (pas)                    */
    int32_t  accel_add;         /* Klipper: delta intervalle par pas        */
    uint32_t accel_count;       /* Klipper: pas restants dans ce segment    */
    uint8_t  direction;         /* direction cachée                         */
    uint8_t  step_pin_state;    /* état courant du pin STEP (0 ou 1)       */
    uint8_t  running;           /* génère activement des impulsions          */
    uint8_t  _pad;
} pulse_gen_t;
```

**Boucle interne Klipper** (`pulse_update`, sur chaque rising edge) :
```c
if (accel_count > 0) {
    interval += accel_add;     // interval += add (Klipper stepper_event_edge)
    accel_count--;
}
```

Quand `accel_count` atteint 0, `pulse_update` passe en mode vitesse constante
et accepte les changements d'intervalle externes immédiatement.

### 4.3 Handshake rampe (ramp_arm / seg_done)

```
PRU0                                    PRU1
  ├─ Écrit interval=start_iv, ──────────→
  │  ramp_add, ramp_count              │
  │  ramp_arm=1                        │
  │                                     │
  │                    ramp_arm==1 ─────┤
  │                    pulse_set_ramp()  │
  │  ←───── ramp_arm=0 ────────────────┤
  │                    seg_done=0        │
  │                                     │
  │                    exécute segment   │
  │                    interval += add   │
  │                    count--           │
  │                    ...               │
  │                    accel_count==0    │
  │                    params.iv=final   │
  │  ←───── seg_done=1 ────────────────┤
  │                                     │
  ├─ (ramp_tick détecte seg_done=1)    │
  ├─ Charge segment suivant...         │
```

### 4.4 Table des pins PRU1

| Pin header | Bit R30    | Signal   | Notes |
|------------|------------|----------|-------|
| P8_45      | R30\[0\]   | LAT_STEP | Step latéral |
| P8_46      | R30\[1\]   | SP_STEP  | Step spindle |
| P8_43      | R30\[2\]   | LAT_DIR  | Direction latéral |
| P8_44      | R30\[3\]   | SP_DIR   | Direction spindle |
| P8_41      | R30\[4\]   | LAT_EN   | Enable latéral (actif-bas) |
| P8_42      | R30\[5\]   | SP_EN    | Enable spindle (actif-bas) |

---

## 5. PRU0 — Orchestrateur (Couche 2)

**Lecteur IEP** : PRU0 lit le timer IEP de PRU1.  Ne le réinitialise jamais.

### 5.1 Responsabilités

- Lire et traiter `host_cmd_t` du daemon
- **Gestion séquentielle des segments de rampe** pour les deux axes (`ramp_tick`)
- Machine d'état homing (IDLE → APPROACH → HIT)
- Lecture endstops R31 + arrêt de sécurité latéral inconditionnel
- Coordination spindle ↔ latéral (Q6 ratio, `coord_tick`)
- Vérification des limites logicielles (`limits_check`)
- Publication `pru_status_t` pour le daemon

### 5.2 Architecture rampe générique (ramp_tick)

L'architecture de rampe est **identique pour les deux axes**.  Le daemon
pré-calcule `ramp_seg_t[MAX_RAMP_SEGS]` dans la shared RAM, et PRU0 les
charge un par un via le handshake ramp_arm/seg_done :

```
État par axe :
  g_ramp_active[ax]     — rampe en cours ?
  g_ramp_seg_idx[ax]    — index du segment courant (0..seg_count-1)
  g_ramp_seg_count[ax]  — nombre total de segments
  g_ramp_op[ax]         — RAMP_OP_RAMP_TO ou RAMP_OP_MOVE_TO

ramp_tick() :
  pour chaque axe :
    si !g_ramp_active[ax] → skip
    si !telem->motor[ax].seg_done → skip  (segment pas encore terminé)
    avancer g_ramp_seg_idx++
    si < seg_count → charger segment suivant (load_ramp_seg)
    sinon → lever EVENT_RAMP_COMPLETE ou EVENT_MOVE_COMPLETE
```

Types d'opérations :
- `RAMP_OP_RAMP_TO` : rampe vitesse → EVENT_RAMP_COMPLETE, moteur continue à cruiser
- `RAMP_OP_MOVE_TO` : mouvement position → EVENT_MOVE_COMPLETE, moteur s'arrête

### 5.3 Machine d'état homing

```
IDLE ──(HOST_CMD_HOME_START)──→ APPROACH
  (lat_dir=HOMING_DIR, lat_interval=HOMING_INTERVAL, lat_run=1)

APPROACH ──(endstop_mask != 0)──→ HIT
  (lat_run=0, reset position, EVENT_HOME_COMPLETE)

HIT ──(immédiat)──→ IDLE
```

### 5.4 Endstops (R31 inputs)

| Pin header | Bit R31 | Signal     |
|------------|---------|------------|
| P9_28      | R31[3]  | ENDSTOP_1  |
| P9_30      | R31[2]  | ENDSTOP_2  |

Les endstops sont **actifs-HIGH** avec pull-up.  Sur front montant :
- Arrêt latéral inconditionnel (`run=0`, cancel_ramp)
- Restauration spindle depuis coordination vers vitesse demandée
- Événement `EVENT_ENDSTOP_HIT` (si hors homing)

### 5.5 Coordination spindle ↔ latéral

Pendant un `MOVE_TO`, le daemon calcule un ratio Q6 :
`sp_lat_coord = (sp_iv × 64) / lat_cruise_iv`

PRU0 applique `coord_tick()` à chaque itération :
```c
sp_adj = (sp_lat_coord × lat_interval_actual) >> 6;
```
Ceci maintient le spindle proportionnel au latéral pendant les rampes,
assurant une densité de bobinage constante (tours/mm).

---

## 6. Couche Linux

### 6.1 Daemon C (`pickup_daemon`)

Le daemon est la **seule interface** entre le monde matériel (PRU, /dev/mem)
et le monde applicatif (Python, recettes, UI).

- Mappe `/dev/mem` sur PRU Shared RAM
- Écrit `host_cmd_t` + `ramp_seg_t[]`, lit `pru_status_t`
- **Pré-calcule les rampes Klipper** (16 sous-segments par axe) via `compute_ramp()`
- Serveur Unix socket `/run/pickup-winder.sock`
- Protocole : JSON ligne par ligne (newline-delimited)
- Polling à 10 ms : broadcast telem + détection événements

Commandes : `set_speed`, `enable`, `e_stop`, `home_start`, `set_limits`,
`move_to`, `ramp_to`, `ack_event`, `reset_pos`

Événements : `endstop_hit`, `home_complete`, `fault`, `limit_hit`,
`move_complete`, `ramp_complete`, `telem`

### 6.2 Application Python

- `pru_client.py` : Client async socket pour le daemon
- `pickup_test.py` : Sketch de test validant toute la chaîne
- `scripts/test_motor.py` : Tests automatisés (rotation + rampes)
- (futur) `WinderApp` : machine d'état winding, recettes, patterns, UI WebSocket

Python ne doit JAMAIS accéder directement à /dev/mem.  Toute interaction PRU
passe par `PruClient` → socket → `pickup_daemon`.

### 6.3 Contrat HAL

Le contrat entre daemon et Python est petit et stable :

```
Commandes (Python → C) :  set_speed / enable / e_stop / home_start /
                           set_limits / move_to / ramp_to /
                           ack_event / reset_pos
Événements (C → Python) :  endstop_hit / home_complete / fault /
                            limit_hit / move_complete / ramp_complete / telem
```

Aucun détail matériel n'est exposé.  Si le matériel change, seul le daemon change.
Si l'application change, seul Python change.

---

## 7. Modèle d'accélération — Klipper multi-segments

### 7.1 Pourquoi un seul segment `interval += add` ne suffit pas

Le modèle `interval += add` (add constant) est le cœur de Klipper.
Mais `v = clock / iv` est hyperbolique :

| Vitesse   | iv      | Effet de add=-14 | Δv relative |
|-----------|---------|-------------------|-------------|
| 10 RPM    | 93 809  | 93809→93795       | 0.015%      |
| 1500 RPM  | 625     | 625→611           | 2.24%       |

L'accélération est ~150× plus rapide à haute vitesse.  Avec un seul segment,
le moteur passe une éternité à basse vitesse puis stall à haute vitesse.

### 7.2 Solution : 16 sous-segments avec force-load

Le **daemon ARM** pré-calcule `MAX_RAMP_SEGS=16` sous-segments couvrant
des plages d'intervalle uniformes.  Chaque segment a son propre
`{start_iv, add, count}`.  L'intervalle est **force-loaded** au début de
chaque segment (= `stepper_load_next()` de Klipper).

**Algorithme `compute_ramp(start_iv, end_iv, accel_s2)` :**

1. Diviser `[start_iv, end_iv]` en 16 sous-plages uniformes
2. Pour chaque segment i :
   - `ivs, ive` = bornes de la sous-plage
   - `vs, ve` = vitesses correspondantes (Hz = clock/2/iv)
   - `count = |v²_hi - v²_lo| / (2 × accel)` — cinématique
   - `add = (ive - ivs) / count` — tronqué, signe correct
3. Écrire les segments dans `g_ramp_segs[ax][]` (shared RAM)

### 7.3 Exécution (PRU1 via PRU0 séquentiel)

```
Pour chaque segment (géré par PRU0 ramp_tick) :
  PRU0 → load_ramp_seg(ax, idx) :
    params->motor[ax].interval   = seg.start_iv   // force-load
    params->motor[ax].ramp_add   = seg.add
    params->motor[ax].ramp_count = seg.count
    params->motor[ax].ramp_arm   = 1

  PRU1 → détecte ramp_arm=1 :
    pulse_set_ramp(pg, start_iv, add, count)
    ramp_arm = 0, seg_done = 0

  PRU1 → boucle IEP (par pas) :
    interval += add
    count--
    ... jusqu'à accel_count == 0

  PRU1 → segment terminé :
    params.interval = interval_final  (empêche saut de vitesse)
    seg_done = 1

  PRU0 → ramp_tick détecte seg_done :
    charge segment N+1 ou lève EVENT_RAMP_COMPLETE
```

### 7.4 Décel (ralentissement)

Le même mécanisme supporte l'accélération et la décélération.  Pour une décel :
- `start_iv < end_iv` (intervalles croissants = vitesse décroissante)
- `add > 0` (intervalle augmente à chaque pas)
- `compute_ramp()` gère les deux cas avec arithmétique signée

---

## 8. Build System

```bash
make all         # Tout construire (DTBOs + PRU firmware + daemon)
make dtbo        # Overlays device-tree seulement
make pru         # Firmware PRU seulement
make daemon      # Daemon C seulement
make clean       # Supprimer tous les artefacts

# Docker cross-compile (recommandé) :
./docker-build.sh --jobs 4

# Déploiement :
./scripts/deploy.sh                # build + deploy
./scripts/deploy.sh --no-build     # deploy seulement
```

Sorties dans `build/` :
- `build/dtbo/*.dtbo` — Overlays DT compilés
- `build/pru/am335x-pru0-fw` — Firmware PRU0 (orchestrateur)
- `build/pru/am335x-pru1-fw` — Firmware PRU1 (moteur)
- `build/daemon/pickup_daemon` — Binaire daemon

### 8.1 Tailles firmware (contrainte 8 KB IRAM)

| Firmware | text | data | bss | total | Marge |
|----------|------|------|-----|-------|-------|
| PRU0 (orchestrateur) | 7256 | 654 | 544 | 8454 | ~0.7 KB |
| PRU1 (moteur) | 5388 | 648 | 544 | 6580 | ~2.6 KB |

---

## 9. Arborescence

```
.github/                        ← CI + copilot instructions
src/
  pru/                          ← Firmware PRU (actif)
    include/                    ← pru_ipc.h, pru_stepper.h, pru_regs.h
    motor_control/              ← firmware moteur (tourne sur PRU1)
    orchestrator/               ← firmware orchestrateur (tourne sur PRU0)
    Makefile                    ← Cross-compile PRU
  linux/
    daemon/                     ← pickup_daemon.c (Couche 3)
  python/                       ← Application Python (Couche 4)
    pickup_test.py              ← Sketch de test
    pru_client.py               ← Client async socket
  dts/                          ← Overlays device-tree
build/                          ← Sorties de build
scripts/
  deploy.sh                     ← Déploiement automatisé (build + SSH + service)
  test_motor.py                 ← Tests automatisés (rotation + rampes)
doc/                            ← Documentation architecture
resources/                      ← Référence (ESP32, eQEP, Klipper)
test/                           ← Tests unitaires (planifié)
Makefile                        ← Orchestrateur de build racine
```

---

## 10. Déploiement et procédure de démarrage

### 10.1 Mapping remoteproc ↔ PRU (critique)

| sysfs | Adresse physique | Rôle | Firmware |
|-------|-----------------|------|----------|
| `remoteproc1` | `4a334000.pru` = PRU0 | **Orchestrateur** | `am335x-pru0-fw` |
| `remoteproc2` | `4a338000.pru` = PRU1 | **Contrôle moteur** | `am335x-pru1-fw` |

### 10.2 Service systemd

```bash
sudo systemctl start pickup-winder   # Démarre PRU + daemon
sudo systemctl stop pickup-winder    # Arrête tout
sudo systemctl status pickup-winder  # État
sudo journalctl -fu pickup-winder    # Logs temps réel
```

### 10.3 Tests

```bash
# Tests complets (spindle rotation + rampes Klipper) :
python3 scripts/test_motor.py --host 192.168.74.171

# Spindle seulement :
python3 scripts/test_motor.py --host 192.168.74.171 --spindle-only

# Rampes seulement :
python3 scripts/test_motor.py --host 192.168.74.171 --ramp
```

---

## 11. Problèmes rencontrés et solutions

### 11.1 Shared RAM garbage → PRU bloqué au boot

**Cause** : la shared RAM conserve son contenu entre reboots logiciels.
**Fix** : `*host_cmd = (host_cmd_t){0}` au début de `orchestrator/main.c`.

### 11.2 JSON avec espaces → "unknown cmd"

**Cause** : `json.dumps()` Python avec espaces.
**Fix** : `compact_json()` dans le daemon normalise avant dispatch.

### 11.3 Socket permissions + read EAGAIN

**Fix** : `chmod(0666)` après `bind()`, garde `O_NONBLOCK`, EAGAIN → continue.

### 11.4 Événement ramp_complete non reçu

**Cause 1** : `read_telem()` dans test_motor.py filtrait les événements ;
`ramp_complete` manquait dans la liste.  Fix : ajout à la liste de filtres.

**Cause 2** : débounce par type (`g_last_event_sent == event_type`) bloquait les
événements consécutifs du même type.  Fix : remplacement par flag one-shot
`g_event_broadcast` qui se reset sur `ack_event` ou quand `event_pending→0`.

### 11.5 Rampe décel non supportée

**Cause** : `send_host_ramp_to` clampait `start_hz > target_hz → start_hz = target_hz`.
`compute_ramp` utilisait `start_iv - end_iv` (underflow si décel).
**Fix** : arithmétique signée dans `compute_ramp`, suppression du clamp dans
`send_host_ramp_to`.

---

## 12. Références

- `.github/copilot-instructions.md` — conventions de code et règles
- `resources/klipper/stepper.c` — implémentation Klipper de référence
- AM335x PRU Reference Guide (TI SPRUHF8)
- AM335x Technical Reference Manual (TI SPRUH73)
