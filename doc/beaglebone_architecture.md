# Architecture BeagleBone Black — PickupWinder

> Branch `beagle` — migration ESP32+FreeRTOS → BeagleBone Black (AM335x) avec PRU.
> Architecture **Option A** : paramètres continus partagés (pas de move rings).

---

## 1. Vue d'ensemble

Le BeagleBone Black expose deux **PRU** (Programmable Real-time Units, 200 MHz,
déterministes, 5 ns/cycle) aux côtés d'un **ARM Cortex-A8** exécutant Linux.

Architecture à 4 couches. Le host écrit des vitesses cibles ; PRU1 orchestre ;
PRU0 génère les impulsions à partir de paramètres continus.

```
┌───────────────────────────────────────────────────────────────────┐
│  Couche 4 — Application Python (asyncio, ARM Linux)               │
│  PruClient · pickup_test.py · (futur: WinderApp, WebUI, ...)      │
│  Envoie commandes JSON au daemon, reçoit télémétrie + événements  │
│                      │ Unix socket /run/pickup-winder.sock        │
├──────────────────────▼────────────────────────────────────────────┤
│  Couche 3 — Daemon C matériel (pickup_daemon, ARM Linux)          │
│  Mappe PRU Shared RAM via /dev/mem                                │
│  Écrit host_cmd_t   (commandes: set_speed, enable, estop, home)   │
│  Lit pru_status_t   (statut agrégé de PRU1)                       │
│  Polling 10 ms, broadcast telem + events vers Python              │
│  Communique UNIQUEMENT avec PRU1                                  │
│                      │ /dev/mem mmap (PRU Shared RAM 0x4A310000)  │
├──────────────────────▼────────────────────────────────────────────┤
│  Couche 2 — PRU1 orchestration (200 MHz, 5 ns/cycle)              │
│  Lit host_cmd_t du daemon (via shared RAM)                        │
│  Machine d'état homing (IDLE → APPROACH → HIT)                    │
│  Écrit motor_params_t (intervalles, dirs, enable, run)            │
│  Lit motor_telem_t de PRU0                                        │
│  Publie pru_status_t (statut agrégé pour le daemon)               │
│  PAS de contrôle moteur. PAS de STEP/DIR/EN.                      │
│                      │ PRU Shared RAM                             │
├──────────────────────▼────────────────────────────────────────────┤
│  Couche 1 — PRU0 contrôle moteur (200 MHz, propriétaire IEP)      │
│  Lit motor_params_t de PRU1 en continu                            │
│  Génération d'impulsions IEP : pulse_gen_t par axe                │
│  GPIO STEP/DIR/EN pour spindle + lateral                          │
│  Lit R31 endstops → arrêt latéral de sécurité inconditionnel      │
│  Publie motor_telem_t (compteurs pas, positions, faults, endstop) │
│  PILOTE MOTEUR AVEUGLE — aucune logique homing, aucune commande   │
└───────────────────────────────────────────────────────────────────┘
```

**Flux de communication** (224 octets en shared RAM) :
```
Host ──host_cmd_t──→ PRU1 ──motor_params_t──→ PRU0
                     PRU1 ←──motor_telem_t── PRU0
Host ←─pru_status_t─ PRU1
```

---

## 2. Règle de découpage

| Critère                                          | PRU0 | PRU1 | Linux |
|--------------------------------------------------|:----:|:----:|:-----:|
| Latence < 1 µs (timing IEP des edges)           | ✓    |      |       |
| Génération STEP/DIR/ENABLE                       | ✓    |      |       |
| Lecture endstops (R31) + arrêt de sécurité       | ✓    |      |       |
| Comptage pas / position                          | ✓    |      |       |
| Publication motor_telem_t                        | ✓    |      |       |
| Orchestration (traitement commandes host)        |      | ✓    |       |
| Machine d'état homing (IDLE/APPROACH/HIT)        |      | ✓    |       |
| Écriture motor_params_t                          |      | ✓    |       |
| Publication pru_status_t                         |      | ✓    |       |
| Détection événements (endstop, home, fault)      |      | ✓    |       |
| Conversion Hz → intervalles IEP                  |      |      | ✓     |
| Compensation spindle ↔ lateral                   |      |      | ✓     |
| Rampes d'accélération (set_speed progressifs)    |      |      | ✓     |
| Machine d'état winding (IDLE/PAUSED/WINDING/…)   |      |      | ✓     |
| UI WebSocket / REST / UART                       |      |      | ✓     |
| Persistance recettes (JSON/fichiers)             |      |      | ✓     |
| Session arbitration (pot/IHM/footswitch)         |      |      | ✓     |

**Principes absolus** :
- Aucun flottant, aucune alloc dynamique, aucun appel OS dans les PRU.
- Pas de division dans la boucle PRU0 (intervalles pré-calculés côté host).
- PRU0 = pilote moteur aveugle. PRU1 = cerveau/orchestrateur.
- Le daemon ne parle qu'à PRU1 (jamais directement à PRU0).

---

## 3. IPC — Mémoire Partagée PRU ↔ Linux

Zone d'échange : **PRU Shared RAM** (AM335x)
- Adresse PRU locale : `0x00010000`
- Adresse physique hôte : `0x4A310000`
- Taille : 12 KB (seuls 224 octets utilisés)

### 3.1 Plan mémoire (Option A)

```
Offset   Taille  Struct              Direction       Description
0x0000   64 o    host_cmd_t          Host → PRU1     Commandes (set_speed, enable, estop, home, etc.)
0x0040   32 o    motor_params_t      PRU1 → PRU0     Intervalles moteur, directions, flags enable/run
0x0060   64 o    motor_telem_t       PRU0 → PRU1     Compteurs pas, positions, faults, masque endstop
0x00A0   64 o    pru_status_t        PRU1 → Host     Statut agrégé pour broadcast daemon
Total :  224 octets
```

### 3.2 Structures clés (`pru_ipc.h`)

```c
/* Host → PRU1 : commande (64 octets) */
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;                    /* HOST_CMD_* opcode                    */
    uint8_t  axis;                   /* AXIS_SPINDLE / LATERAL / ALL         */
    uint8_t  sp_dir;                 /* direction spindle (0=fwd, 1=rev)     */
    uint8_t  lat_dir;                /* direction latéral (0=fwd, 1=rev)     */
    uint32_t sp_interval_target;     /* intervalle IEP cible spindle         */
    uint32_t lat_interval_target;    /* intervalle IEP cible latéral         */
    uint32_t value_a;                /* usage général (ex: enable=1/0)       */
    uint32_t value_b;                /* usage général                        */
    uint8_t  cmd_ack;                /* PRU1 copie l'opcode ici quand traité */
    uint8_t  _pad[3];
    uint32_t _reserved[9];
} host_cmd_t;

/* PRU1 → PRU0 : paramètres moteur (32 octets) */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t sp_interval;            /* cycles IEP entre edges spindle       */
    uint32_t lat_interval;           /* cycles IEP entre edges latéral       */
    uint8_t  sp_dir, lat_dir;        /* directions                           */
    uint8_t  sp_enable, lat_enable;  /* 1=driver activé                      */
    uint8_t  sp_run, lat_run;        /* 1=générer des impulsions             */
    uint8_t  _pad[2];
    uint32_t _reserved[4];
} motor_params_t;

/* PRU0 → PRU1 : télémétrie moteur (64 octets) */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t sp_step_count;          /* pas spindle depuis dernier reset     */
    uint32_t lat_step_count;         /* pas latéral depuis dernier reset     */
    int32_t  lat_position;           /* position latérale signée (pas)       */
    uint32_t sp_interval_actual;     /* intervalle spindle actuel            */
    uint32_t lat_interval_actual;    /* intervalle latéral actuel            */
    uint8_t  sp_state, lat_state;    /* MOTOR_STATE_* flags                  */
    uint8_t  sp_faults, lat_faults;  /* FAULT_* flags                        */
    uint8_t  endstop_mask;           /* bit0=ES1, bit1=ES2                   */
    uint8_t  _pad[3];
    uint32_t seq;                    /* compteur monotone (wrap)             */
    uint32_t _reserved[8];
} motor_telem_t;

/* PRU1 → Host : statut agrégé (64 octets) */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t seq;
    uint8_t  pru1_state;             /* PRU1_STATE_* flags                   */
    uint8_t  event_pending;          /* 1 = événement en attente d'ack host  */
    uint8_t  event_type;             /* EVENT_* type                         */
    uint8_t  _pad0;
    /* Champs copiés de motor_telem_t : */
    uint32_t sp_step_count, lat_step_count;
    int32_t  lat_position;
    uint32_t sp_interval_actual, lat_interval_actual;
    uint8_t  endstop_mask, sp_faults, lat_faults, _pad1;
    uint32_t _reserved[7];
} pru_status_t;
```

### 3.3 Opcodes de commande

| Opcode                | Valeur | Description |
|-----------------------|--------|-------------|
| `HOST_CMD_NOP`        | 0      | Idle / acquitté |
| `HOST_CMD_SET_SPEED`  | 1      | Définir intervalles IEP cibles + directions |
| `HOST_CMD_ENABLE`     | 2      | Activer/désactiver drivers |
| `HOST_CMD_ESTOP`      | 3      | Arrêt d'urgence immédiat |
| `HOST_CMD_HOME_START` | 4      | Démarrer séquence homing latéral |
| `HOST_CMD_ACK_EVENT`  | 5      | Acquitter dernier événement PRU |
| `HOST_CMD_RESET_POS`  | 6      | Remettre compteurs/position à zéro |

### 3.4 Protocole commande/acquittement

1. Host écrit `cmd` dans `host_cmd_t` (+ paramètres).
2. PRU1 lit `cmd`, exécute, copie l'opcode dans `cmd_ack`, remet `cmd = NOP`.
3. Host poll `cmd` — quand il voit `NOP`, la commande est acquittée.
4. Host ne doit PAS écrire de nouvelle commande avant acquittement.

---

## 4. PRU0 — Pilote Moteur (Couche 1)

**Propriétaire du timer IEP** : PRU0 initialise le compteur IEP au démarrage.
PRU1 le lit seulement — ne le réinitialise jamais.

### 4.1 Boucle principale

```
Initialiser IEP (IEP_INIT)
Boucle infinie :
  1. Lire motor_params_t
  2. Appliquer enable/direction sur R30 (STEP/DIR/EN)
  3. Lire R31 → endstop_mask
  4. Si endstop actif ET latéral en marche → arrêt immédiat latéral
  5. pulse_update(&sp_gen, STEP_A_BIT, iep_now)
  6. pulse_update(&lat_gen, STEP_B_BIT, iep_now)
  7. (tous les N cycles) Publier motor_telem_t
```

### 4.2 Moteur de génération d'impulsions

Le `pulse_gen_t` (dans `pru_stepper.h`) gère la génération continue
d'impulsions basée sur le timer IEP :

```c
typedef struct {
    uint32_t next_edge;    /* timestamp IEP pour prochain toggle STEP  */
    uint32_t interval;     /* cycles IEP entre edges                   */
    uint32_t step_count;   /* pas total depuis dernier reset           */
    int32_t  position;     /* position signée (pas)                    */
    uint8_t  phase;        /* 0=montée, 1=descente                     */
    uint8_t  direction;    /* direction cachée pour tracking position   */
    uint8_t  running;      /* génère activement des impulsions          */
    uint8_t  _pad;
} pulse_gen_t;
```

`pulse_update()` à chaque edge :
- Si pas running ou interval==0 : skip
- Si IEP a dépassé `next_edge` : toggle pin STEP, incrémenter compteur,
  avancer next_edge
- Gère rising et falling edges (duty cycle 50%)

### 4.3 Table des pins PRU0

| Pin header | Bit PRU   | Fonction   | Notes |
|------------|-----------|------------|-------|
| P9_25      | R30\[7\]  | EN_A       | Enable spindle (actif-bas) |
| P9_29      | R30\[1\]  | DIR_A      | Direction spindle |
| P9_31      | R30\[0\]  | STEP_A     | Step spindle |
| P9_41      | R30\[6\]  | EN_B       | Enable latéral (actif-bas) |
| P9_28      | R30\[3\]  | DIR_B      | Direction latéral |
| P9_30      | R30\[2\]  | STEP_B     | Step latéral |
| P8_15      | R31\[15\] | ENDSTOP_1  | Fin de course |
| P8_16      | R31\[14\] | ENDSTOP_2  | Fin de course |

---

## 5. PRU1 — Orchestrateur (Couche 2)

**Lecteur IEP** : PRU1 lit le timer IEP de PRU0. Ne le réinitialise jamais.

### 5.1 Responsabilités

- Lire `host_cmd_t` du daemon
- Traiter les commandes : SET_SPEED, ENABLE, ESTOP, HOME_START, ACK_EVENT, RESET_POS
- Machine d'état homing (IDLE → APPROACH → HIT)
- Écrire `motor_params_t` pour PRU0
- Lire `motor_telem_t` de PRU0
- Publier `pru_status_t` pour le daemon
- Détecter événements (endstop, home complete, fault) et signaler au host

### 5.2 Machine d'état homing

```
IDLE ──(HOST_CMD_HOME_START)──→ APPROACH
  (lat_dir=HOMING_DIR, lat_interval=HOMING_INTERVAL, lat_run=1)

APPROACH ──(endstop_mask != 0)──→ HIT
  (lat_run=0, EVENT_HOME_COMPLETE, PRU1_STATE_AT_HOME)

HIT ──(host acquitte)──→ IDLE
```

### 5.3 Détection d'événements

PRU1 surveille `motor_telem_t` et signale les événements au host via
`pru_status_t.event_pending` + `event_type` :

| Événement             | Code | Condition |
|-----------------------|------|-----------|
| `EVENT_ENDSTOP_HIT`   | 1    | `lat_faults & FAULT_ENDSTOP_HIT` (hors homing) |
| `EVENT_HOME_COMPLETE` | 2    | Homing FSM atteint l'état HIT |
| `EVENT_FAULT`         | 3    | Fault moteur détecté |

---

## 6. Couche Linux

### 6.1 Daemon C (`pickup_daemon`)

Le daemon est la **seule interface** entre le monde matériel (PRU, /dev/mem)
et le monde applicatif (Python, recettes, UI).

- Mappe `/dev/mem` sur PRU Shared RAM
- Écrit `host_cmd_t`, lit `pru_status_t`
- Serveur Unix socket `/run/pickup-winder.sock`
- Protocole : JSON ligne par ligne (newline-delimited)
- Polling à 10 ms : broadcast telem + détection événements

Commandes : `set_speed`, `enable`, `e_stop`, `home_start`, `ack_event`, `reset_pos`
Événements : `endstop_hit`, `home_complete`, `fault`, `telem`

### 6.2 Application Python

- `pru_client.py` : Client async socket pour le daemon
- `pickup_test.py` : Sketch de test (9 fonctions) validant toute la chaîne
- (futur) `WinderApp` : machine d'état winding, recettes, patterns, UI WebSocket

Python ne doit JAMAIS accéder directement à /dev/mem. Toute interaction PRU
passe par `PruClient` → socket → `pickup_daemon`.

### 6.3 Contrat HAL

Le contrat entre daemon et Python est petit et stable :

```
Commandes (Python → C) :  set_speed / enable / e_stop / home_start / ack_event / reset_pos
Événements (C → Python) :  endstop_hit / home_complete / fault / telem
```

Aucun détail matériel n'est exposé : pas d'adresses mémoire, pas de valeurs
de registres, pas d'opcodes PRU. Si le matériel change, seul le daemon change.
Si l'application change, seul Python change.

---

## 7. Build System

```bash
make all         # Tout construire (DTBOs + PRU firmware + daemon)
make dtbo        # Overlays device-tree seulement
make pru         # Firmware PRU seulement
make daemon      # Daemon C seulement
make clean       # Supprimer tous les artefacts
make deploy BBB_IP=192.168.x.x  # Déployer via SSH
```

Sorties dans `build/` :
- `build/dtbo/*.dtbo` — Overlays DT compilés
- `build/pru/am335x-pru0-fw` — Firmware PRU0
- `build/pru/am335x-pru1-fw` — Firmware PRU1
- `build/daemon/pickup_daemon` — Binaire daemon

Toolchain PRU : `pru-unknown-elf-gcc` (crosstool-NG, auto-détecté dans `~/x-tools`).

---

## 8. Arborescence

```
.github/                        ← CI + copilot instructions
src/
  pru/                          ← Firmware PRU (actif)
    include/                    ← pru_ipc.h, pru_stepper.h, pru_regs.h
    pru0_motor_control/         ← PRU0 : pilote moteur aveugle
    pru1_orchestration/         ← PRU1 : orchestrateur / cerveau
    Makefile                    ← Cross-compile PRU
  linux/
    daemon/                     ← pickup_daemon.c (Couche 3)
  python/                       ← Application Python (Couche 4)
    pickup_test.py              ← Sketch de test (9 fonctions)
    pru_client.py               ← Client async socket
  dts/                          ← Overlays device-tree
build/                          ← Sorties de build
doc/                            ← Documentation architecture
resources/                      ← Référence (ESP32, eQEP, Klipper)
test/                           ← Tests unitaires (planifié)
Makefile                        ← Orchestrateur de build racine
```

---

## 9. Synchronisation Spindle ↔ Lateral

La vitesse latérale est calculée par Python en fonction de :
- Largeur effective de la bobine (mm)
- Vitesse de bobinage (Hz)
- Tours par passe (TPP)
- Pattern (STRAIGHT / SCATTER / HUMAN)

Python envoie des commandes `set_speed` progressives à ~10 ms de cadence.
Les changements de vitesse prennent effet immédiatement (pas de flush de ring
nécessaire). Les rampes d'accélération sont gérées côté host.

---

## 10. Option A — Pourquoi pas de move rings

Ce projet utilise le modèle à **paramètres continus partagés** :

- Python définit les vitesses cibles en Hz via `set_speed`
- Le daemon convertit Hz → intervalles IEP et écrit `host_cmd_t`
- PRU1 valide et copie dans `motor_params_t`
- PRU0 lit `motor_params_t` et génère des impulsions continues

Avantages :
- **Pas d'underrun possible** : PRU0 a toujours des paramètres valides
- **Firmware PRU plus simple** : pas de pointeurs de ring, pas de logique de frontière de move
- **Changements de vitesse instantanés** : le nouvel intervalle prend effet au cycle suivant
- **Debug plus facile** : un seul struct à inspecter

---

## 11. Références

- `.github/copilot-instructions.md` — conventions de code et règles
- `src/pru/README.md` — firmware PRU détails
- `src/dts/README.md` — overlays device-tree
- `src/pru/PRU_DEPLOY.md` — déploiement et scripts runtime
- AM335x PRU Reference Guide (TI SPRUHF8)
- AM335x Technical Reference Manual (TI SPRUH73)
