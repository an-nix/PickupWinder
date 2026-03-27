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

## Notes

- Geometry and trim settings are critical. Start conservative.
- Randomized profiles are deterministic for a given seed.
- If a parameter seems unclear, start from **Straight** profile with all jitter values near zero and `firstPassTraverseFactor = 1.0`.
