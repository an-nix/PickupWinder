# Architecture BeagleBone Black — PickupWinder

> Branch `beagle` — migration ESP32+FreeRTOS → BeagleBone Black (AM335x) avec PRU.
> Architecture **Option A** : paramètres continus partagés (pas de move rings).

---

## 1. Vue d'ensemble

Le BeagleBone Black expose deux **PRU** (Programmable Real-time Units, 200 MHz,
déterministes, 5 ns/cycle) aux côtés d'un **ARM Cortex-A8** exécutant Linux.

Architecture à 4 couches. Le host écrit des vitesses cibles ; PRU0 orchestre ;
PRU1 génère les impulsions à partir de paramètres continus.

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
│  Lit pru_status_t   (statut agrégé de PRU0)                       │
│  Polling 10 ms, broadcast telem + events vers Python              │
│  Communique UNIQUEMENT avec PRU0                                  │
│                      │ /dev/mem mmap (PRU Shared RAM 0x4A310000)  │
├──────────────────────▼────────────────────────────────────────────┤
│  Couche 2 — PRU0 orchestration (200 MHz, 5 ns/cycle)              │
│  Lit host_cmd_t du daemon (via shared RAM)                        │
│  Machine d'état homing (IDLE → APPROACH → HIT)                    │
│  Écrit motor_params_t (intervalles, dirs, enable, run)            │
│  Lit motor_telem_t de PRU1                                        │
│  Publie pru_status_t (statut agrégé pour le daemon)               │
│  PAS de contrôle moteur. PAS de STEP/DIR/EN.                      │
│                      │ PRU Shared RAM                             │
├──────────────────────▼────────────────────────────────────────────┤
│  Couche 1 — PRU1 contrôle moteur (200 MHz, propriétaire IEP)      │
│  Lit motor_params_t de PRU0 en continu                            │
│  Génération d'impulsions IEP : pulse_gen_t par axe                │
│  GPIO STEP/DIR/EN pour spindle + lateral                          │
│  Lit R31 endstops → arrêt latéral de sécurité inconditionnel      │
│  Publie motor_telem_t (compteurs pas, positions, faults, endstop) │
│  PILOTE MOTEUR AVEUGLE — aucune logique homing, aucune commande   │
└───────────────────────────────────────────────────────────────────┘
```

**Flux de communication** (224 octets en shared RAM) :
```
Host ──host_cmd_t──→ PRU0 ──motor_params_t──→ PRU1
                     PRU0 ←──motor_telem_t── PRU1
Host ←─pru_status_t─ PRU0
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
| P8_41      | R30\[7\]  | EN_A       | Enable spindle (actif-bas) |
| P8_43      | R30\[5\]  | DIR_A      | Direction spindle |
| P8_45      | R30\[1\]  | STEP_A     | Step spindle |
| P8_42      | R30\[3\]  | EN_B       | Enable latéral (actif-bas) |
| P8_44      | R30\[0\]  | DIR_B      | Direction latéral |
| P8_46      | R30\[2\]  | STEP_B     | Step latéral |
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
    motor_control/          ← firmware moteur (tourne sur PRU1)
    orchestrator/           ← firmware orchestrateur (tourne sur PRU0)
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

## 10. Modèle d'accélération — Klipper multi-segments

### 10.1 Pourquoi un seul segment `interval += add` ne suffit pas

Le modèle `interval += add` (add constant) est le cœur de Klipper
(`stepper_event_edge()` dans `stepper.c`). Mais il ne produit une accélération
constante que sur une **plage de vitesse étroite**.

La relation entre intervalle IEP et vitesse est **hyperbolique** :
`v = 100 MHz / iv`. Un `add` constant donne :

| Vitesse   | iv      | Effet de add=-14 | Δv relative |
|-----------|---------|-------------------|-------------|
| 10 RPM    | 93 809  | 93809→93795       | 0.015%      |
| 1500 RPM  | 625     | 625→611           | 2.24%       |

L'accélération est ~150× plus rapide à haute vitesse qu'à basse vitesse.
Avec un seul segment, le moteur passe une éternité à basse vitesse puis
traverse la haute vitesse instantanément → **stall**.

### 10.2 Solution Klipper : file de segments

Dans Klipper, le **host** pré-calcule de nombreux petits segments
`{interval, count, add}`. Le MCU (ou PRU) fait en boucle :

```c
// stepper_event_edge() — resources/klipper/stepper.c
interval += add;
count--;
if (count == 0) stepper_load_next();  // charge le segment suivant
```

Chaque segment couvre une plage de vitesse étroite où l'approximation linéaire
`iv(n) ≈ iv_start + n × add` est précise. C'est l'équivalent de
`queue_step(interval, count, add)` envoyé par le host.

Types de données Klipper :
- `interval` : `uint32_t` (ticks d'horloge MCU)
- `add` : `int16_t` (±32767 max dans `stepper_move`, promu `int16_t` dans `stepper`)
- `count` : `uint16_t` (max 65535 dans `stepper_move`, promu `uint32_t` dans `stepper`)

### 10.3 Notre implémentation : `compute_accel_ramp()` + `run_ramp_forward/reverse`

Puisque le test_spindle est autonome (pas de host Python pendant la rampe),
le PRU pré-calcule **N_SEG=16 segments** avant la boucle moteur.

**Algorithme `compute_accel_ramp(start_iv, end_iv, accel_s2)` :**

1. Convertir intervalles en vitesses : `v_start = 100M/start_iv`, `v_end = 100M/end_iv`
2. Diviser la plage en N_SEG sous-plages de vitesse égales : `Δv_seg = (v_end - v_start) / N_SEG`
3. Pour chaque segment i :
   - `vs = v_start + i × Δv_seg` → `ivs = 100M / vs` (force-loaded au début du segment)
   - `ve = v_start + (i+1) × Δv_seg` → `ive = 100M / ve`
   - `count = (vs + ve) × (ve - vs) / (2 × accel)` — équation cinématique $s = (v_e^2 - v_s^2) / (2a)$
   - `add = (ive - ivs) / count` — tronqué vers zéro

**Exécution :**
```c
for (seg = 0; seg < N_SEG; seg++) {
    iv = segs[seg].start_iv;       // force-load = stepper_load_next()
    for (s = 0; s < segs[seg].count; s++) {
        one_step(iv);
        iv += segs[seg].add;       // Klipper: interval += add
    }
}
```

La **décel** ré-utilise les mêmes segments en ordre inverse avec `add` négé.

### 10.4 Pourquoi le force-load est critique

L'erreur de troncature de `add` fait que chaque segment n'atteint pas
exactement l'intervalle cible. Sans correction, l'erreur s'accumule.

En chargeant `start_iv` au début de chaque segment (= `stepper_load_next()`
dans Klipper qui charge `m->interval`), l'erreur est confinée au segment
courant et ne se propage pas. L'erreur maximale dans un segment est
`|gap mod count|` cycles, soit < 1 step de déviation.

### 10.5 Intégration future dans le firmware de production

Le test_spindle valide le modèle. Pour la production (`motor_control/main.c`
+ `orchestrator/main.c`) :

- **PRU0 (orchestrator)** reçoit du daemon `move_to(target, start_hz, max_hz, accel_steps)`.
  Il calcule les segments (comme `compute_accel_ramp`) et les écrit dans
  `motor_params_t` en séquence, un segment à la fois.
- **PRU1 (motor_control)** exécute `interval += add; count--` dans la boucle IEP.
  Quand `count == 0`, il signale à PRU0 qui charge le segment suivant.
- **Le daemon** ne calcule PAS les segments — il envoie des paramètres cinématiques
  (vitesses Hz + accélération). PRU0 fait la conversion Hz→intervalles et le
  découpage en segments.
- Les divisions (N_SEG divisions par rampe) se font dans PRU0 avant le démarrage
  de la rampe, pas dans la boucle moteur de PRU1.

---

## 11. Option A — Pourquoi pas de move rings (rappel)

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

## 12. Déploiement et procédure de démarrage

### 12.1 Mapping remoteproc ↔ PRU (critique)

Sur Debian 12 / kernel 6.12, le mapping est **inversé par rapport aux noms** :

| sysfs | Adresse physique | Rôle réel | Firmware chargé |
|-------|-----------------|-----------|-----------------|
| `remoteproc1` | `4a334000.pru` = PRU0 | **Orchestrateur** | `am335x-pru0-fw` |
| `remoteproc2` | `4a338000.pru` = PRU1 | **Contrôle moteur** | `am335x-pru1-fw` |

> ⚠️ PRU0 = orchestrateur, PRU1 = moteur. L'architecture nommée (couche 1/2)
> correspond aux rôles, pas aux indices sysfs.

### 12.2 IRAM PRU — chargement via remoteproc uniquement

L'IRAM des PRU n'est **pas accessible via `/dev/mem`** : les écritures sont
silencieusement ignorées par le kernel même quand le PRU est à l'arrêt.
Le seul mécanisme valide est le driver `remoteproc` :

```bash
# Arrêt
sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc1/state'
sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc2/state'
# Copier les binaires
sudo cp build/pru/am335x-pru0-fw /lib/firmware/
sudo cp build/pru/am335x-pru1-fw /lib/firmware/
# Démarrage (charge l'IRAM depuis /lib/firmware/)
sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state'
sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state'
```

### 12.3 Initialisation critique de la shared RAM

**CRITIQUE** : à chaque démarrage PRU, l'orchestrateur (PRU0) commence par
zérer toute la `host_cmd_t` :

```c
/* orchestrator/main.c — début de main() */
*host_cmd  = (host_cmd_t){0};   /* CRITIQUE : pas de cmd garbage au boot */
*params    = (motor_params_t){0};
*status    = (pru_status_t){0};
host_cmd->cmd     = HOST_CMD_NOP;
host_cmd->cmd_ack = HOST_CMD_NOP;
```

Sans ce zéro, des octets aléatoires de boot précédent dans la shared RAM
peuvent faire interpréter un opcode fantôme par l'orchestrateur
(ex. `cmd=0x44`), causant un comportement indéfini ou un blocage.

### 12.4 Build et déploiement complet

```bash
# Depuis le répertoire racine du projet (Linux build host)
cd /home/nicolas/Documents/PlatformIO/Projects/PickupWinder

# Build PRU uniquement
cd src/pru && make bbb-deploy BBB_IP=192.168.74.171 BBB_USER=beagle

# Build + déploiement daemon (compilé nativement sur BBB)
scp src/linux/daemon/pickup_daemon.c  beagle@BBB:/tmp/build/src/linux/daemon/
scp src/pru/include/pru_ipc.h         beagle@BBB:/tmp/build/src/pru/include/
scp src/pru/include/pru_stepper.h     beagle@BBB:/tmp/build/src/pru/include/
ssh beagle@BBB "cd /tmp/build/src/linux/daemon && \
    gcc -O2 -Wall -Wextra pickup_daemon.c -lm -o /tmp/daemon && \
    sudo cp /tmp/daemon /usr/local/bin/pickup_daemon"
```

### 12.5 Procédure de démarrage sur BBB

```bash
sudo pkill pickup_daemon 2>/dev/null
sudo rm -f /run/pickup-winder.sock
sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; true'
sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; true'
sleep 0.5
sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state'
sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state'
sleep 0.5
sudo /usr/local/bin/pickup_daemon &
```

---

## 13. Problèmes rencontrés et solutions

### 13.1 Shared RAM garbage → PRU bloqué au boot

**Symptôme** : le moteur ne tourne pas, le PC du PRU est bloqué à une adresse
fixe, les step counters ne progressent pas.

**Cause** : la shared RAM conserve son contenu entre reboots logiciels (arrêt/
démarrage remoteproc sans power cycle). Un `cmd` résiduel non-NOP dans
`host_cmd_t` est traité comme une commande fantôme avant même que le daemon
n'écrive quoi que ce soit.

**Fix** : `*host_cmd = (host_cmd_t){0}` au début de `orchestrator/main.c`.

---

### 13.2 JSON avec espaces → "unknown cmd"

**Symptôme** : toutes les commandes Python retournent `{"ok":false,"error":"unknown cmd"}`.

**Cause** : `json.dumps()` Python produit `{"cmd": "enable"}` (espace après `:`).
Le daemon cherchait `"cmd":"enable"` (sans espace) via `strstr()`.

**Fix** : `compact_json()` dans le daemon normalise l'entrée en place avant dispatch.

```c
/* pickup_daemon.c */
static void compact_json(char *s) {
    char *r = s, *w = s;
    while (*r) {
        *w++ = *r;
        if (*r == ':' || *r == ',') { ++r; while (*r == ' ' || *r == '\t') ++r; }
        else { ++r; }
    }
    *w = '\0';
}
```

---

### 13.3 Socket Unix — permissions et read() EAGAIN

**Symptôme** : connexions immédiatement refusées (Broken pipe) ou silencieusement
fermées sans réponse.

**Causes et fixes** :

1. **Permissions 755** sur le socket (créé par root via `sudo`) : seul root
   peut se connecter. Fix : `chmod(SOCKET_PATH, 0666)` après `bind()`.

2. **`read()` non-bloquant retourne EAGAIN** : le socket client est mis en
   `O_NONBLOCK` pour que `broadcast()` ne bloque pas. Si poll retourne POLLIN
   mais que `read()` retourne -1/EAGAIN (race condition), `n <= 0` fermait
   le socket. Fix :
   ```c
   ssize_t n = read(g_clients[i], buf, sizeof(buf) - 1);
   if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
   if (n <= 0) { /* fermer le client */ }
   ```

---

### 13.4 Bitmasks des pins moteur

Pin layout confirmé via `cat /sys/kernel/debug/pinctrl/44e10800.pinmux-pinctrl-single/pins` :

| Pin | Bit R30 | Signal |
|-----|---------|--------|
| P8_45 | R30[0] | LAT_STEP |
| P8_46 | R30[1] | SP_STEP |
| P8_43 | R30[2] | LAT_DIR |
| P8_44 | R30[3] | SP_DIR |
| P8_41 | R30[4] | LAT_EN (active-low) |
| P8_42 | R30[5] | SP_EN (active-low) |

Endstop P9_28 → `pr1_pru0_pru_r31_3` (MODE6) → `ES1_BIT = (1u << 3)`.

---

## 14. Références

- `.github/copilot-instructions.md` — conventions de code et règles
- `src/pru/README.md` — firmware PRU détails
- `src/dts/README.md` — overlays device-tree
- `src/pru/PRU_DEPLOY.md` — déploiement et scripts runtime
- AM335x PRU Reference Guide (TI SPRUHF8)
- AM335x Technical Reference Manual (TI SPRUH73)
