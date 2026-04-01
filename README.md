# PickupWinder

ESP32/PlatformIO pickup winding controller with:

- winding motor speed control from potentiometer
- lateral carriage homing and synchronized traverse
- web UI (browser)
- window shift & trim controls in run mode
- session pause/resume and target tracking
- recipe persistence in NVS
- safe Wi-Fi credential handling for CI

## Quickstart

1. Install PlatformIO and dependencies.
2. Open repository and build with `platformio run`.
3. Flash with `platformio run --target upload`.
4. Connect to device Wi-Fi or set your own in `include/Config.h`.
5. Open http://<device-ip>/ in browser.

## Secure Wi-Fi config for CI

Wi-Fi credential resolution behavior:

1. Try NVS credentials (namespace `wifi`, keys `ssid` + `pwd`).
2. If NVS exists, use it; if config `WIFI_SSID`/`WIFI_PASSWORD` are non-default and differ from NVS, persist new values to NVS (overwrite) then use them.
3. If no NVS exists:
   - if config `WIFI_SSID`/`WIFI_PASSWORD` are default placeholders (`your_ssid_here` / `your_password_here`) or empty: Wi-Fi does not start.
   - else persist config values to NVS and use them.

This is implemented in `src/WifiManager.cpp` and used by `src/WebInterface.cpp`.

### Decoupled WiFi service

`WebInterface` uses `WifiManager` as a dependency for connectivity. This keeps WiFi/NVS handling independent from WebSocket/HTTP routing, allowing future non-web consumers to reuse WiFi.

### Runtime NVS (post-flash, without rebuild)

At startup, the firmware checks NVS namespace `wifi` for stored credentials:

- If NVS contains `ssid` and `pwd`, they are used.
- Otherwise `WIFI_SSID` / `WIFI_PASSWORD` from `include/Config.h` are used.
- Upload via `cmd('wifi_set', 'ssid|password')` updates NVS.
- If new values are identical to existing NVS values, write is skipped (idempotent).
Runtime update command:

`cmd('wifi_set', 'ssid|password')`

- Saves to NVS keys `ssid` and `pwd` in namespace `wifi`.
- Effective on next reboot.

## Build and flash (local)

```bash
platformio run -e esp32dev
platformio run -e esp32dev --target upload
```

### Build profiles

- `esp32dev`: production-oriented profile (`DIAG_VERBOSE=0`, `-Os`, `-flto`)
- `esp32dev-lab`: lab diagnostics profile (`DIAG_VERBOSE=1`, `-O0`)

Example:

```bash
platformio run -e esp32dev-lab --target upload
```

## Protocol and capabilities

The firmware now exposes explicit protocol versions and command capabilities:

- `GET /protocol.json`: `{ ws, uart, recipe }`
- `GET /capabilities.json`: command list with type, mutability and help text

This allows UI clients to discover supported commands dynamically and stay decoupled from hardcoded firmware assumptions.

UART also emits a startup protocol banner:

- `P|<uartProtocolVersion>`

followed by regular status frames.

## Command validation and normalization

Incoming commands are now validated before entering the control queue:

- alias normalization (`max-rpm` -> `max_rpm`, `windows_shift` -> `window_shift`)
- value schema checking (boolean, integer ranges, float ranges, enum values, JSON payload)
- explicit rejection metrics for oversize and schema-invalid payloads

Command ingress counters are logged periodically in RTOS metrics.

## Runtime metrics

Periodic metrics now include:

- heap free and minimum heap observed
- stack high-water mark for control/comms tasks
- control/comms worst loop execution time
- command ingress/drop/reject counters

These metrics are intended to validate memory and timing headroom on target hardware.

## CI and test workflow

GitHub Actions now compiles:

- firmware for `esp32dev`
- firmware for `esp32dev-lab`
- unit tests for both environments (compile-only, no upload)

Useful local commands:

```bash
platformio run -e esp32dev
platformio test -e esp32dev --without-uploading
platformio test -e esp32dev-lab --without-uploading
```

Engineering references:

- `doc/review_checklist_realtime_heap.md`
- `doc/benchmark_control_loop.md`

## Runtime modes

- `IDLE`
- `PAUSED`
- `WINDING`
- `TARGET_REACHED`
 - `RODAGE` (break-in)

## Web UI overview

This project can load Wi-Fi credentials from multiple sources:

1. Build-time defines (preferred for CI):
   - `BUILD_WIFI_SSID` (e.g. `-D BUILD_WIFI_SSID=\"mi-ssid\"`)
   - `BUILD_WIFI_PASSWORD` (e.g. `-D BUILD_WIFI_PASSWORD=\"mi-password\"`)
2. Optional local `include/Secrets.h` (ignored by git via `.gitignore`).
3. Fallback placeholders (`<redacted>`) to prevent hard-coding unsafe values.

Example PlatformIO command in CI:

```bash
platformio run -e esp32dev \
  -D BUILD_WIFI_SSID=\"your_ssid\" \
  -D BUILD_WIFI_PASSWORD=\"your_password\"
```

If using local development:

```cpp
// include/Secrets.h
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
```

The file is in `.gitignore` so it won’t be stored in the repo.

# PickupWinder

ESP32/PlatformIO pickup winding controller with:

- winding motor speed control from potentiometer
- lateral carriage homing and synchronized traverse
- web UI
- verification flow for low/high winding bounds
- manual carriage mode
- lateral-axis break-in mode
- recipe persistence in NVS
- predictive final-position handling

## Hardware pinout

This project uses the following pins on an ESP32:

```
   +-------------------+    
   |  ESP32            |    
   |                   |    
   |  D26  STEP        |--> Spindle stepper STEP
   |  D27  DIR         |--> Spindle stepper DIR
   |  D14  ENABLE      |--> Spindle stepper ENABLE (LOW = on)
   |                   |    
   |  D32  STEP_LAT    |--> Lateral stepper STEP
   |  D33  DIR_LAT     |--> Lateral stepper DIR
   |  D25  ENABLE_LAT  |--> Lateral stepper ENABLE (LOW = on)
   |                   |    
   |  D23  HOME_NO     |--> Lateral home sensor NO
   |  D22  HOME_NC     |--> Lateral home sensor NC
   |                   |    
   |  D19  ENC1_CLK    |--> Encoder A
   |  D18  ENC1_DT     |--> Encoder B
   |                   |    
   |  D34  POT        |--> Potentiometer ADC (speed control)
   |  D13  FOOTSWITCH |--> Footswitch input
   |  D02  LED        |--> Status LED
   +-------------------+
```

## Variable machine parameters

The following constants can be tuned in `include/Config.h` and related headers:

- `STEP_PIN`, `DIR_PIN`, `ENABLE_PIN`: main spindle stepper pins.
- `STEP_PIN_LAT`, `DIR_PIN_LAT`, `ENABLE_PIN_LAT`: lateral carriage stepper pins.
- `HOME_PIN_NO`, `HOME_PIN_NC`: homing switches.
- `ENC1_CLK`, `ENC1_DT`: encoder signals for manual trim in PAUSED mode.
- `POT_PIN`, `POT_INVERTED`: speed potentiometer input and inversion flag.
- `FOOTSWITCH_PIN`, `FOOTSWITCH_ACTIVE_LOW`: physical footswitch pin/polarity.
- `STEP_PIN`, `DIR_PIN`, `ENABLE_PIN`: stepper pins (swap `DIR_PIN` signal to invert motor direction)
- `MICROSTEPPING`: microstepping factor (32 by default).
- `LAT_HOME_OFFSET_DEFAULT_MM`: carriage offset after homing.
- `LAT_TRAVERSE_MM`: total lateral travel range used for calibration.
- `LAT_ACCEL`, `LAT_HOME_SPEED_HZ`, `LAT_TRAVERSE_SPEED_HZ`: lateral motor motion profile.

### Axis geometry and drive ratio

- `LAT_MOTOR_STEPS` and `MICROSTEPPING` define the steps/mm for lateral movement.
- For 1 mm screw pitch and 96-step motor:
  - `LAT_STEPS_PER_MM = LAT_MOTOR_STEPS * MICROSTEPPING` (default 3072).
- Adjust `MICROSTEPPING` and mechanical setup as needed for your specific carriage.

### Winding geometry

Variable in `include/WindingGeometry.h` and runtime commands:
- `totalWidth_mm`, `flangeBottom_mm`, `flangeTop_mm`, `margin_mm`
- `windingStartTrim_mm`, `windingEndTrim_mm`
- `wireDiameter_mm` (used for TPP calculations)
- UI controls under **Configuration** > **Geometry** reflect these.

## Web UI overview

The web UI is split into two tabs:

- **Winding**: main setup and run-time controls
- **Configuration**: machine-level settings and break-in tools

---

## Winding tab — setup page

### Machine settings

#### Direction
- **CW**: clockwise winding direction
- **CCW**: counter-clockwise winding direction

This changes the main winding motor direction only.

#### Mode
- **Target**: stop automatically when the target turn count is reached
- **FreeRun**: no automatic stop at a target count

#### Target turns
Desired total turn count for the winding job.

- In **Target** mode, the job stops when this count is reached.
- This value can also be adjusted while winding.

#### Seed
Random seed used by non-straight winding profiles.

- Same recipe + same seed = repeatable pattern behavior
- Changing the seed changes the random-looking profile variations

#### Final position
Controls how the carriage should finish the job.

- **Free**: no forced final bound
- **High bound**: finish at the upper winding bound
- **Low bound**: finish at the lower winding bound

#### Hold turns
Number of final turns to do while the carriage is already resting on the selected final bound.

Purpose:
- helps lock the wire in place at the end of winding

---

## Geometry

These parameters define the usable winding window.

#### Total width
Total available width of the bobbin/tonework region.

#### Margin
Safety margin kept away from the ends.

#### Bottom tonework
Lower dead zone / non-winding area.

#### Top tonework
Upper dead zone / non-winding area.

#### Low-bound trim
Manual correction applied to the computed winding start position.

Use it when the theoretical start point is slightly off in the real machine.

#### High-bound trim
Manual correction applied to the computed winding end position.

Use it when the theoretical end point is slightly off in the real machine.

#### Wire (mm)
Wire diameter used to estimate turns per pass.

This strongly affects traverse density and packing.

---

## Winding profile

These settings control how the traverse behaves from one pass to another.

### Profile type

#### Straight
Regular winding.

- no intentional random variation
- most predictable behavior

#### Scatter
Adds per-layer variation.

- useful to avoid perfectly repetitive lay patterns
- can help produce a more scattered winding distribution

#### Human
Adds smoother, more organic variation across a pass.

- meant to imitate a less perfectly uniform hand-guided feel
- combines slower-varying traverse and speed fluctuations

### Scatter
Geometry-side scatter factor.

This affects how much the winding distribution is spread.
Higher values generally make the pattern less strictly uniform.

### Turns/pass offset
Manual correction added to the computed turns-per-pass.

Use it to compensate if the real machine packs slightly tighter or looser than the simple geometry estimate.

- positive value: more turns before reversal
- negative value: fewer turns before reversal

### Layer jitter
Random turn-per-pass variation applied per pass/layer.

Effect:
- varies pass width from one pass to the next
- stronger values make layering less regular

### Layer speed jitter
Random traverse speed variation applied per pass/layer.

Effect:
- changes lateral speed from pass to pass
- useful for breaking repetitive patterns

### Human traverse jitter
Smooth within-pass variation of traverse behavior.

Effect:
- changes traverse character gradually during a pass
- mainly relevant in **Human** profile

### Human speed jitter
Smooth within-pass variation of traverse speed.

Effect:
- speed changes gradually during a pass
- mainly relevant in **Human** profile

### First-pass traverse factor
Extra multiplier applied only to the **first traverse pass**.

Range:
- `< 1.0`: slower first pass
- `= 1.0`: unchanged
- `> 1.0`: faster first pass

Use cases:
- soften the first layer start
- compensate for the first pass packing differently from later passes
- make the first pass denser or looser without changing the whole recipe

---

## Setup summary block

### Carriage position
Current measured lateral carriage position.

### Winding window
Current effective winding start and end positions.

### Turns/pass
Current computed nominal turns per pass.

---

## Winding tab — run page

### Live values

#### RPM
Measured winding motor speed.

#### Hz
Current step frequency command for the winding motor.

#### Turns
Current turn counter.

#### Pass
Current lateral pass index.

#### Progress
Job completion percentage relative to target turns.

#### Active turns/pass
Actual turns-per-pass currently being used by the planner.

This may differ from the nominal value when profile modulation is active.

#### Lateral scale
Current traverse speed scale applied by the planner.

This can change because of:
- selected winding profile
- predictive final-position logic
- reversal slowdown logic

#### Layer progress
Normalized 0–100% progress across the current lateral stroke.

#### Low / high trim
Current live trim corrections.

#### Mode
Current winding mode:
- **Target**
- **FreeRun**

---

## In-run actions

### Adjust target while running
Changes the target turns during an active job.

Useful if you decide to stop earlier or continue longer.

### Stop at next bound (soft)
Requests a smooth stop when the carriage reaches the next selected bound.

- **Stop @ high bound**
- **Stop @ low bound**

This is different from an immediate stop.

### Position correction (web/encoder)
Small nudges for live correction of:
- winding start position
- high bound position

Use these to fine-tune the real carriage position without editing raw recipe data.

### Reset turns
Resets the turn counter.

---

## Local build and debugging

- build command:

```bash
./build.sh
```

- tools:
  - `platformio` CLI
  - `pio` (alias dans certains environnements)

- debug trace (session input arbitration):
  - `src/SessionController.cpp` a maintenant des `Serial.printf` dans
    `recordIntent` (`intent`, `src`, `potAboveZero`, `potNeedsRearm`).

- flow :
  1. `ControlHardware` lit pot/footswitch/encodeur.
  2. `CommandController` draine les commandes série/WebSocket.
  3. `SessionController::tick()` intègre toutes les sources et applique
     un `ControlIntent` selon le **dernier événement reçu**.
  4. `WinderApp` reçoit `setControlHz` et avance le modèle.


### Re-arm Start
Re-enters the startup flow.

---

## Configuration tab

### Lateral axis home offset
Offset applied after homing.

Instead of treating the home switch position as the final usable zero, the carriage moves by this offset and treats that as the real working zero.

Use this when:
- the home switch is not physically located at the desired working zero
- you want a reproducible safe gap after homing

Changing this triggers an automatic rehome.

### Lateral axis break-in
Runs repeated back-and-forth motion for mechanical testing or break-in.

#### Max distance (mm)
Maximum carriage travel for the break-in cycle.

#### Passes (round trips)
Number of full back-and-forth cycles.

Use this after:
- assembling a new axis
- changing mechanics
- checking smooth motion and repeatability

---


## Final-position logic

When **Final position** is enabled, the firmware tries to make the carriage reach the selected bound at the correct moment before the end of the job.

The logic is based on:
- remaining turns
- current pass progress
- estimated remaining path to the chosen bound
- active turns-per-pass
- predictive slowing of traverse speed

Goal:
- avoid abrupt end-of-run repositioning
- avoid “reach bound, leave, come back” behavior
- start the hold turns with the carriage already on the requested bound

---

## Winding Uniformity and Wire Distribution Logic

### Core Principle: Constant Turns-Per-Millimeter Ratio

The fundamental requirement for uniform wire density across the entire bobbin width is that the **ratio between spindle RPM and carriage velocity must remain constant at all times**:

$$\text{Turns per mm} = \frac{\text{Spindle RPM}}{\text{Carriage velocity (mm/s)}} = \text{CONSTANT}$$

If this ratio varies during winding, the wire distribution becomes uneven:
- **Excessive ratio** (spindle fast, carriage slow) → wire bunches up → **bulge**
- **Reduced ratio** (spindle slow, carriage fast) → wire spreads out → **gap**

### Problem Scenario: Carriage Reversals and Acceleration

Without compensation, the following occurs during each carriage reversal:

1. **Deceleration phase**: Carriage slows down as it approaches the reversal point.
2. **Spindle uncompensated**: Spindle continues at nominal RPM → the ratio increases.
3. **Result**: Extra wire deposited at edges during carriage slowdown.
4. **Acceleration phase**: Carriage speeds back up after reversal.
5. **Spindle still uncompensated**: Ratio decreases as carriage accelerates.
6. **Result**: Less wire deposited mid-pass → gap at center.

### Solution: Dynamic Spindle-Carriage Compensation

The firmware implements real-time compensation to maintain constant ratio during all carriage states.

**Compensation formula:**

$$\text{Spindle}_{\text{Hz, compensated}} = \text{Spindle}_{\text{Hz, nominal}} \times \frac{\text{Lateral}_{\text{Hz, current}}}{\text{Lateral}_{\text{Hz, nominal}}}$$

**When carriage velocity drops** (during reversal deceleration):
- Compensation reduces spindle speed proportionally
- Maintains constant turns-per-mm ratio
- Prevents edge bulges

**When carriage velocity rises** (during acceleration):
- Compensation increases spindle speed proportionally
- Maintains ratio stability
- Prevents mid-bobbin gaps

### Traverse Ratio: Integer vs. Non-Integer

The **turns-per-pass ÷ effective-width ratio** must be **non-integer** (e.g., 3.7, 4.2, 5.3) to create a uniform cross-hatch (losange) pattern.

**Why integer ratios are problematic:**

| Ratio Type | Result | Visual Effect |
|-----------|--------|--------------|
| Integer (3, 4, 5) | Wires stack vertically | Visible banding, uneven density |
| Non-integer (3.7, 4.2) | Wires offset per layer | Uniform cross-hatch pattern |

**Example:**
- Bobbin width: 10 mm
- Turns per pass: 37
- Ratio: 37 ÷ 10 = **3.7** turns/mm ✓ (GOOD)

If you adjust the geometry and observe integer ratios (within ±0.15 of any whole number), adjust either:
- **Turns per pass offset** (in the UI or recipe)
- **Effective winding width** (by adjusting flange/margin/trim values)
- **Wire diameter** (though less practical)

### Boundary Margins: Symmetry Requirement

The carriage has intentional margins (safe distance from physical bobbin edges):

**Default margins:**
- Bottom margin: 15 mm (from `LAT_HOME_OFFSET_DEFAULT_MM`)
- Top margin: 15 mm (symmetric)
- Effective winding width: 100 - (2 × 15) = **70 mm**

**Critical requirement: margins MUST be SYMMETRIC on both sides.**

If margins are asymmetric:
- Winding center shifts relative to the physical bobbin
- Wire distribution becomes unequal between left and right halves
- One side gets denser, the other gets looser

The firmware enforces symmetry by using a single offset constant applied to both bounds. Verify your mechanical setup:
- Measure physical distance from left edge to left sensor trigger
- Measure physical distance from right edge to right sensor trigger
- Both distances should be equal

### Expected Results After Compensation

**Nominal regime** (constant speed):
- Spindle RPM constant
- Carriage velocity constant
- Ratio remains perfectly stable
- Wire deposits uniformly

**Deceleration phase** (carriage approaches reversal):
- Spindle RPM decreases proportionally with carriage velocity
- Ratio maintained within ±5%
- No edge bulging

**Acceleration phase** (carriage speeds up):
- Spindle RPM increases proportionally with carriage velocity
- Ratio maintained within ±5%
- No mid-bobbin gaps

**Reversal artifact**:
- Brief 100–600 ms deviation as carriage changes direction
- Creates ~0.5–2 mm bulge at reversal point (acceptable)
- Spindle begins compensation as carriage re-accelerates

### Validation and Troubleshooting

#### How to validate compensation is working

1. Wind a test pickup with a simple profile (Straight, no jitter).
2. Inspect the bobbin under good lighting.
3. Look for the **cross-hatch (losange) pattern** across the entire surface.
4. Pattern should be uniform left-to-right with no visible banding or gaps.

#### Symptoms of poor compensation

| Symptom | Likely Cause | Fix |
|---------|------------|-----|
| Bulge at center edges | Carriage reversal slowdown inadequate | Check `LAT_ACCEL`, verify compensation enabled |
| Gap at bobbin center | Carriage acceleration too fast for spindle | Increase `ACCELERATION` or adjust traverse ratio |
| Integer banding pattern | Traverse ratio is whole number | Adjust turns-per-pass offset or width |
| Asymmetric distribution | Boundary margins mismatched | Verify home sensor position symmetry |
| All loose / all dense | Wire diameter incorrect | Adjust in Geometry settings |

#### Logging and debugging

The control loop logs worst-case timing every 5 seconds:

```
[RTOS] control_worst_loop_us=892
```

Ensure this stays below ~4000 µs (10 ms tick = 10000 µs available, budget ~40% for motion control).

Compensation calculations are lightweight (~0.5 ms) and should not impact timing.

---

## Notes

- Geometry and trim settings are critical. Start conservative.
- Randomized profiles are deterministic for a given seed.
- If a parameter seems unclear, start from **Straight** profile with all jitter values near zero and `firstPassTraverseFactor = 1.0`.
