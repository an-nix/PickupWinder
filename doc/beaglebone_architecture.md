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
│         IPC (PRU Shared RAM 0x4A310000, 12 KB)          │
├───────────────────┬─────────────────────────────────────┤
│   PRU0 — Spindle  │     PRU1 — Lateral + Sensors        │
│  Step generation  │   Step generation                   │
│  Accel ramp       │   Accel ramp                        │
│  Compensation RPM │   Home sensor (NO+NC)               │
│  Position count   │   Position count + direction        │
│  Emergency stop   │   Reversal detection                │
└───────────────────┴─────────────────────────────────────┘
```

---

## 2. Règle de découpage

| Critère                                          | PRU | Linux |
|--------------------------------------------------|:---:|:-----:|
| Latence exigée < 10 µs                           | ✓   |       |
| Génération de signaux STEP/DIR/ENABLE            | ✓   |       |
| Lecture capteur avec débounce matériel           | ✓   |       |
| Arrêt d'urgence immédiat                         | ✓   |       |
| Comptage pas / position                          | ✓   |       |
| Compensation spindle ↔ lateral en real-time      | ✓   |       |
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
0x0270    16 B  pru_sync                PRU0↔PRU1 (vitesses actuelles)
```

### 3.2 Structures clés (`pru_ipc.h`)

```c
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       /* CMD_* opcode */
    uint8_t  axis;      /* AXIS_SPINDLE | AXIS_LATERAL | AXIS_ALL */
    uint8_t  flags;
    uint8_t  _pad;
    uint32_t value_a;   /* vitesse cible en Hz */
    uint32_t value_b;   /* accélération en Hz/ms */
    uint32_t _reserved;
} pru_cmd_t; /* 16 bytes */

typedef struct __attribute__((packed, aligned(4))) {
    uint32_t seq;               /* compteur monotone */
    uint32_t step_count;        /* pas absolus depuis reset */
    uint32_t current_hz;        /* vitesse réelle (Hz) */
    uint32_t target_hz;         /* consigne (Hz) */
    int32_t  position_steps;    /* position signée (lateral) */
    uint16_t state;             /* flags d'état */
    uint16_t faults;            /* bits de faute */
    uint32_t _reserved[2];
} pru_axis_telem_t; /* 48 bytes */
```

### 3.3 Ring Buffer de commandes

```
Capacité : 32 slots × 16 bytes = 512 bytes
cmd_whead : Linux incrémente après écriture
cmd_rhead : PRU incrémente après traitement
Condition vide : whead == rhead
Condition pleine : (whead+1) % 32 == rhead
```

---

## 4. PRU0 — Spindle Step Generator

### Responsabilités
- Générer les impulsions STEP/DIR sur les GPIO R30 dédiés
- Exécuter la rampe d'accélération/décélération (fixed-point, ±1 Hz/ms)
- Appliquer la compensation de vitesse spindle en lisant `pru_sync.lateral_hz`
- Écrire `pru_sync.spindle_hz` pour que PRU1 puisse en prendre connaissance
- Compter les pas absolus et publier `spindle_telem` toutes les 256 impulsions

### GPIO (à confirmer via device-tree overlay)
```
R30[0] → P9_31  SPINDLE_STEP
R30[1] → P9_29  SPINDLE_DIR
R30[2] → P9_30  SPINDLE_ENABLE  (actif bas)
```

### Algorithme de génération

```
loop:
  read commands from ring buffer (every STEP_POLL_STRIDE = 128 steps)
  update ramp: current_hz → target_hz (±accel_hz_per_ms per 1ms)
  period_cycles = 200_000_000 / current_hz
  half_period = period_cycles / 2
  SET STEP=1
  __delay_cycles(half_period - OVERHEAD)
  SET STEP=0
  __delay_cycles(half_period - OVERHEAD)
  step_count++
  write current_hz to pru_sync
  apply compensation: spindle_hz = nominal × (lateral_actual / lateral_nominal)
```

### Fréquences supportées
| Paramètre    | Valeur        |
|--------------|---------------|
| F_PRU        | 200 MHz       |
| Step max     | 160 kHz       |
| Half-period min | 625 cycles |
| Step min (mesure) | ~10 Hz  |
| Résolution ramp | 1 Hz / ms |

---

## 5. PRU1 — Lateral Step Generator + Sensors

### Responsabilités
- Générer les impulsions STEP/DIR pour l'axe lateral
- Exécuter la rampe (même moteur que PRU0)
- Lire le capteur home dual-contact (NO+NC) avec débounce 10 µs
- Détecter début/fin de traversée et signaler via `state` dans `lateral_telem`
- Écrire `pru_sync.lateral_hz` pour la compensation spindle
- Détecter dépassement de course et lever une faute (`faults`)

### GPIO
```
R30[0] → P8_45  LATERAL_STEP
R30[1] → P8_46  LATERAL_DIR
R30[2] → P8_43  LATERAL_ENABLE  (actif bas)
R31[0] → P8_45  HOME_NO         (entrée)
R31[1] → P8_46  HOME_NC         (entrée)
```

### Débounce capteur home
```
Lecture R31 toutes les 10 µs (2000 cycles)
État validé si stable 3 lectures consécutives
NO=LOW + NC=HIGH → home détecté
NO=HIGH + NC=HIGH → FAUTE (câble débranché)
```

---

## 6. Couche Linux

### Processus daemon

Unique processus userspace : `pickupwinderd`

| Module              | Fichier                        | Rôle |
|---------------------|-------------------------------|------|
| `IpcChannel`        | `linux/src/IpcChannel.cpp`    | Mappe PRU shared RAM via `/dev/mem` ou prussdrv |
| `StepperPRU`        | `linux/src/StepperPRU.cpp`    | API compatible `StepperController` → commandes IPC |
| `LateralPRU`        | `linux/src/LateralPRU.cpp`    | API compatible `LateralController` → commandes IPC |
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
│   │   ├── pru_ipc.h           ← layout mémoire partagée (C pur)
│   │   └── pru_ramp.h          ← moteur rampe fixed-point (inline)
│   ├── pru0_spindle/
│   │   ├── main.c
│   │   └── Makefile
│   ├── pru1_lateral/
│   │   ├── main.c
│   │   └── Makefile
│   └── Makefile
├── linux/
│   ├── include/
│   │   ├── IpcChannel.h
│   │   ├── StepperPRU.h
│   │   └── LateralPRU.h
│   ├── src/
│   │   ├── IpcChannel.cpp
│   │   ├── StepperPRU.cpp
│   │   └── LateralPRU.cpp
│   └── CMakeLists.txt
├── dts/
│   └── pickupwinder-pru-00A0.dts   ← overlay pinmux + PRU enable
└── scripts/
    ├── load_pru.sh
    └── deploy.sh
```

---

## 9. Synchronisation Spindle ↔ Lateral (rappel de contrainte critique)

La densité de fil doit rester constante à tout instant :

```
Ratio = Spindle_Hz / Lateral_Hz = CONSTANT
```

Sur BBB, la compensation est appliquée **dans PRU0** à chaque impulsion spindle,
en lisant `pru_sync.lateral_hz` écrit par PRU1 :

```c
/* PRU0 — appliqué à chaque ramp-tick (1ms) */
uint32_t lat_hz = pru_sync->lateral_hz;
if (lat_hz > LAT_MIN_HZ_THRESHOLD && nominal_lat_hz > 0) {
    current_spindle_hz = (uint64_t)nominal_spindle_hz
                         * lat_hz / nominal_lat_hz;
    /* clamp SPEED_HZ_MIN..SPEED_HZ_MAX */
}
```

Cette implémentation remplace `StepperController::calculateCompensatedSpindleHz()`
de l'ESP32, maintenant exécutée dans le PRU au lieu du `controlTask`.

---

## 10. Références

- [AM335x TRM](https://www.ti.com/lit/ug/spruh73q/spruh73q.pdf) — chapitre 4 (PRU-ICSS)
- [PRU Cookbook](https://beagleboard.org/static/prucookbook/)
- [pru-software-support-package](https://github.com/beagleboard/am335x_pru_package)
- [prussdrv](https://github.com/beagleboard/am335x_pru_package/tree/master/pru_sw/utils/prussdrv)
