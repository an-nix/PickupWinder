# Architecture BeagleBone Black — PickupWinder

> Branch `beagle` — migration ESP32+FreeRTOS → BeagleBone Black (AM335x) avec PRU.

---

## 1. Vue d'ensemble

Le BeagleBone Black expose deux **PRU** (Programmable Real-time Units, 200 MHz,
déterministes, 5 ns/cycle) aux côtés d'un **ARM Cortex-A8** exécutant Linux.
Cette dualité map parfaitement sur la séparation de responsabilités du projet :

```
┌─────────────────────────────────────────────────────────┐
│                    LINUX (userspace)                    │
│  WinderApp · Session · Recipe · WebUI · CommandRegistry │
│              Patterns · Telemetry · WiFi                │
│                                                         │
│  StepperPRU           LateralPRU                        │
│  MoveQueue (spindle)  MoveQueue (lateral)               │
│      Pre-compute (interval, count, add) move triples    │
│                                                         │
│         IPC (PRU Shared RAM 0x4A310000, 12 KB)          │
├───────────────────┬─────────────────────────────────────┤
│   PRU0 — Spindle  │     PRU1 — Lateral + Sensors        │
│  IEP timer owner  │   IEP timer reader                  │
│  Move ring pop    │   Move ring pop                     │
│  Edge generation  │   Edge generation                   │
│  Position count   │   Home sensor (NO+NC) + debounce    │
│  Emergency stop   │   Position count + direction        │
└───────────────────┴─────────────────────────────────────┘
```

---

## 2. Règle de découpage

| Critère                                          | PRU | Linux |
|--------------------------------------------------|:---:|:-----:|
| Latence exigée < 1 µs (edge timing IEP)          | ✓   |       |
| Génération de signaux STEP/DIR/ENABLE            | ✓   |       |
| Lecture capteur avec débounce (IEP-based 500 µs) | ✓   |       |
| Arrêt d'urgence immédiat (CMD_EMERGENCY_STOP)    | ✓   |       |
| Comptage pas / position                          | ✓   |       |
| Calcul move triples (interval, count, add)       |     | ✓     |
| Compensation spindle ↔ lateral                   |     | ✓     |
| Calcul de profil (ramps, patterns, géométrie)    |     | ✓     |
| Machine d'état (IDLE/WINDING/PAUSED/…)           |     | ✓     |
| UI WebSocket / REST / UART                       |     | ✓     |
| Persistance recettes (JSON/fichiers)             |     | ✓     |
| Validation commandes / session arbitration       |     | ✓     |
| Supervision watchdog + logging                   |     | ✓     |

**Principe absolu** : aucun flottant lourd, aucune alloc dynamique, aucun appel
OS dans les PRU. Tout passe par des entiers et de la mémoire partagée.

---

## 3. IPC — Mémoire Partagée PRU ↔ Linux

La zone d'échange est la **PRU Shared RAM** (AM335x) :
- Adresse PRU locale : `0x00010000`
- Adresse physique hôte : `0x4A310000`
- Taille : 12 KB

### 3.1 Plan mémoire

```
Offset  Taille  Nom                     Propriétaire
0x0000   512 B  Command ring buffer     Linux écrit, PRU(s) lisent
0x0200     4 B  cmd_whead               Linux écrit (index d'écriture)
0x0204     4 B  cmd_rhead               PRU met à jour (index de lecture)
0x0210    48 B  spindle_telem           PRU0 écrit, Linux lit
0x0240    48 B  lateral_telem           PRU1 écrit, Linux lit
0x0270    16 B  pru_sync                PRU0↔PRU1 (intervals actifs)
0x0280     4 B  sp_move_whead           Linux écrit
0x0284     4 B  sp_move_rhead           PRU0 écrit
0x0288  2048 B  spindle move ring       128 × pru_move_t (16 B chacun)
0x0A88     4 B  lat_move_whead          Linux écrit
0x0A8C     4 B  lat_move_rhead          PRU1 écrit
0x0A90  2048 B  lateral move ring       128 × pru_move_t
```

### 3.2 Structures clés (`pru_ipc.h`)

```c
/* Commande lifecycle (ring buffer) */
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       /* CMD_NOP | CMD_ENABLE | CMD_EMERGENCY_STOP |  */
                        /* CMD_RESET_POSITION | CMD_HOME_START | CMD_QUEUE_FLUSH */
    uint8_t  axis;      /* AXIS_SPINDLE | AXIS_LATERAL | AXIS_ALL */
    uint8_t  flags;
    uint8_t  _pad;
    uint32_t value_a;
    uint32_t value_b;
    uint32_t _reserved;
} pru_cmd_t; /* 16 bytes */

/* Move triple — unité atomique de génération de pas */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t interval;   /* IEP cycles entre deux fronts (STEP edges) */
    uint32_t count;      /* nombre de fronts à générer (2 × nb pas) */
    int32_t  add;        /* delta interval par front (rampe linéaire) */
    uint8_t  direction;  /* 0 = forward, 1 = backward */
    uint8_t  _pad[3];
} pru_move_t; /* 16 bytes */

/* Télémétrie par axe (PRU → Linux) */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t seq;               /* compteur monotone */
    uint32_t step_count;        /* pas absolus depuis reset */
    uint32_t current_interval;  /* demi-période courante (IEP cycles) */
    uint32_t moves_pending;     /* slots non encore consommés */
    int32_t  position_steps;    /* position signée (lateral) */
    uint16_t state;             /* flags d'état */
    uint16_t faults;            /* bits de faute */
    uint32_t _reserved[2];
} pru_axis_telem_t; /* 48 bytes */

/* Sync PRU0 ↔ PRU1 */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t spindle_interval;  /* intervalle courant PRU0 (IEP cycles) */
    uint32_t lateral_interval;  /* intervalle courant PRU1 (IEP cycles) */
    uint32_t control_flags;
    uint32_t _reserved;
} pru_sync_t; /* 16 bytes */
```

### 3.3 Jeu de commandes (6 opcodes)

| Commande             | Valeur | Description |
|----------------------|--------|-------------|
| `CMD_NOP`            | 0      | Pas d'opération |
| `CMD_ENABLE`         | 1      | Active/désactive le driver (value_a=1/0) |
| `CMD_EMERGENCY_STOP` | 2      | Coupe STEP immédiatement, vide le ring |
| `CMD_RESET_POSITION` | 3      | Remet step_count et position_steps à 0 |
| `CMD_HOME_START`     | 4      | PRU1 : active homing_mode |
| `CMD_QUEUE_FLUSH`    | 5      | Vide le move ring sans arrêt d'urgence |

---

## 4. PRU0 — Spindle Step Generator

### Responsabilités
- Initialiser et posséder le **IEP timer** (200 MHz free-running counter)
- Consommer les move triples `(interval, count, add)` depuis le spindle move ring
- Générer les fronts STEP/DIR avec précision < 5 ns (IEP edge timing absolu)
- Compter les pas absolus et publier `spindle_telem`
- Publier `pru_sync.spindle_interval` pour la couche Linux

### GPIO
```
R30[0] → P9_31  STEP_A
R30[1] → P9_29  DIR_A
R30[7] → P9_25  EN_A
R30[2] → P9_30  SPINDLE_ENABLE  (actif bas)
```

### Algorithme IEP (pru_stepper.h)

```c
/* Boucle chaude — aucun __delay_cycles */
loop:
  poll command ring (every CMD_CHECK_STRIDE = 1000 iters)
  if (IEP_NOW() >= stepper.next_edge_time):
    toggle STEP pin
    stepper.count--
    if stepper.count == 0:
      pop next move from ring (chain seamlessly, pre-apply add)
      or mark running=false + set underrun fault
    else:
      stepper.next_edge_time += stepper.interval
      stepper.interval += stepper.add   /* linear ramp */
  publish telem (every TELEM_STRIDE = 500000 iters)
```

On move boundary, `interval += add` is pre-applied after loading the new move
(matches Klipper `stepper_load_next`). `FAULT_MOVE_UNDERRUN` is set if the ring
drains while the stepper was running (host didn't push moves fast enough).

### Paramètres move triple
| Champ      | Type     | Description |
|------------|----------|-------------|
| `interval` | uint32_t | IEP cycles entre deux fronts (min 625 = 3,125 µs) |
| `count`    | uint32_t | Nombre de fronts = 2 × pas |
| `add`      | int32_t  | Delta interval/front (rampe linéaire, peut être 0) |
| `direction`| uint8_t  | 0=forward, 1=backward |

### Fréquences supportées
| Paramètre         | Valeur        |
|-------------------|---------------|
| F_PRU             | 200 MHz       |
| Step max          | 160 kHz       |
| Interval min      | 625 cycles    |
| Résolution timing | 5 ns (1 cycle) |

---

## 5. PRU1 — Lateral Step Generator + Sensors

### Responsabilités
- Lire le **IEP timer** initialisé par PRU0 (ne jamais le réinitialiser)
- Consommer les move triples depuis le lateral move ring
- Générer les fronts STEP/DIR pour l'axe lateral
- Lire le capteur home dual-contact (NO+NC) avec débounce IEP (500 µs)
- En mode `homing_mode` : vider le ring et stopper quand home est détecté
- Compter la position signée et détecter les dépassements de course
- Publier `pru_sync.lateral_interval` et `lateral_telem`

### GPIO
```
R30[2]  → P9_30  STEP_B
R30[3]  → P9_28  DIR_B
R30[6]  → P9_41  EN_B           (actif bas)
R31[15] → P8_15  ENDSTOP_1      (entrée, currently PRU0 sampled; PRU1 pins used by eQEP2)
R31[14] → P8_16  ENDSTOP_2      (entrée, currently PRU0 sampled; PRU1 pins used by eQEP2)
```

### Débounce capteur home (IEP-based)
```
Débounce via IEP_NOW() — 100 000 cycles = 500 µs
NO=LOW + NC=HIGH → home détecté
NO=HIGH + NC=HIGH → FAUTE (câble débranché)
En homing_mode : home_hit_latch → flush ring + stop + reset position
```

---

## 6. Couche Linux

### Processus daemon

Unique processus userspace : `pickupwinderd`

| Module              | Fichier                        | Rôle |
|---------------------|-------------------------------|------|
| `IpcChannel`        | `linux/src/IpcChannel.cpp`    | Mappe PRU shared RAM; `sendMove()`, `moveQueueFreeSlots()` |
| `MoveQueue`         | `linux/src/MoveQueue.cpp`     | Calcule les move triples (ramp math flottant) |
| `StepperPRU`        | `linux/src/StepperPRU.cpp`    | API spindle; `tick()` pour refill du ring |
| `LateralPRU`        | `linux/src/LateralPRU.cpp`    | API lateral; poussée de traversées par `_pushTraverse()` |
| `WinderApp.Core`    | `src/WinderApp.Core.cpp`      | Machine d'état (inchangée) |
| `WinderApp.*`       | `src/WinderApp.*.cpp`         | Commands, patterns, recipe (inchangés) |
| `WebInterface`      | `src/WebInterface.cpp`        | HTTP + WebSocket (inchangé) |
| `SessionController` | `src/SessionController.cpp`   | Arbitrage (inchangé) |

### Accès au PRU Shared RAM depuis Linux

```
Option A : prussdrv (libprussdrv-dev) — legacy mais bien documenté
Option B : remoteproc + /dev/mem mmap — plus standard kernel 5.x
```

Voir `linux/src/IpcChannel.cpp` pour l'implémentation.

---

## 7. Build System

### PRU
- Compilateur : `pru-gcc` (GNU PRU toolchain) ou `clpru` (TI CGT)
- `port/beaglebone/pru/Makefile` construit les deux firmwares
- Les binaires (`.out`/`.fw`) sont copiés dans `/lib/firmware/`

### Linux daemon
- CMake ≥ 3.16
- `port/beaglebone/linux/CMakeLists.txt`
- Dépendances : `libprussdrv`, `libwebsockets` (ou équivalent), `nlohmann/json`

### CI
- Pipeline GitHub Actions séparé (workflow `beaglebone-build.yml`)
- Build PRU cross (toolchain pru-gcc dans container)
- Build Linux croisé ARM (arm-linux-gnueabihf-g++)
- Tests unitaires natifs (host x86-64, gtest)

---

## 8. Arborescence cible

```
port/beaglebone/
├── pru/
│   ├── include/
│   │   ├── pru_ipc.h           ← layout mémoire partagée; pru_move_t; fault flags
│   │   └── pru_stepper.h       ← IEP timer + Klipper step engine (inline, underrun)
│   ├── pru0_motor_control/
│   │   └── main.c
│   ├── pru1_orchestration/
│   │   └── main.c
│   └── Makefile
├── linux/
│   ├── include/
│   │   ├── IpcChannel.h
│   │   ├── MoveQueue.h         ← planificateur host (pushAccelSegment, etc.)
│   │   ├── StepperPRU.h
│   │   └── LateralPRU.h
│   ├── src/
│   │   ├── IpcChannel.cpp
│   │   ├── MoveQueue.cpp
│   │   ├── StepperPRU.cpp
│   │   └── LateralPRU.cpp
│   └── CMakeLists.txt
├── dts/
│   └── pickupwinder-pru-00A0.dts   ← overlay pinmux + PRU enable
└── scripts/
    ├── load_pru.sh
    ├── deploy.sh
    ├── pickupwinder-pru.service
    └── pru_spin_test.py
```

---

## 9. Synchronisation Spindle ↔ Lateral (rappel de contrainte critique)

La densité de fil doit rester constante à tout instant :

```
Ratio = Spindle_Hz / Lateral_Hz = CONSTANT
```

Avec l'architecture Klipper, la compensation est appliquée **côté Linux** avant
de pousser les move triples dans le ring :

```cpp
/* Dans StepperPRU::tick() / WinderApp */
uint32_t compHz = StepperPRU::calculateCompensatedSpindleHz(
    nominalSpindleHz, currentLatHz, nominalLatHz);
stepper.setSpeedHz(compHz);  /* met à jour _speedHz + _speedChanged */
/* tick() détecte _speedChanged → flush + repush via MoveQueue */
```

Le calcul de ramp (triples) suit la cinématique Klipper :
```
N_edges = 2 × (v1² - v0²) / (2 × accel_hz_per_s)
iv0     = PRU_CLOCK / (2 × v0)
add     = round((iv1 - iv0) / N_edges)
```

La zone de sync `pru_sync` contient maintenant les **intervals IEP** (cycles)
au lieu des fréquences Hz, permettant une reconstitution Hz exacte :
```
hz = PRU_CLOCK_HZ / (2 × current_interval)
```

Cette implémentation remplace l'ancienne compensation en PRU0.

---

## 10. Références

- [AM335x TRM](https://www.ti.com/lit/ug/spruh73q/spruh73q.pdf) — chapitre 4 (PRU-ICSS)
- [PRU Cookbook](https://beagleboard.org/static/prucookbook/)
- [pru-software-support-package](https://github.com/beagleboard/am335x_pru_package)
- [prussdrv](https://github.com/beagleboard/am335x_pru_package/tree/master/pru_sw/utils/prussdrv)
