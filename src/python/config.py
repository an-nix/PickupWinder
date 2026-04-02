"""Hardware and application constants - mirrors Config.h / Types.h."""

# ── Protocol versions ─────────────────────────────────────────────────────────
PICKUP_WS_PROTOCOL_VERSION = "1.1"
PICKUP_UART_PROTOCOL_VERSION = "1.1"
PICKUP_CAPABILITIES_VERSION = "1"
PICKUP_RECIPE_FORMAT_VERSION = 2

# ── Spindle stepper ───────────────────────────────────────────────────────────
MICROSTEPPING = 32
MOTOR_FULL_STEPS = 200
STEPS_PER_REV = MOTOR_FULL_STEPS * MICROSTEPPING   # 6400
WINDING_MOTOR_INVERTED = True

ACCELERATION = 150000          # steps/s²
SPEED_HZ_MIN = 9600 // 9       # ~1067 Hz  (~10 RPM)
SPEED_HZ_MAX = 160000          # 160 kHz   (~1500 RPM)
SPEED_HZ_START = 800

# ── Lateral stepper ───────────────────────────────────────────────────────────
LAT_MOTOR_STEPS = 96
LAT_STEPS_PER_MM = LAT_MOTOR_STEPS * MICROSTEPPING  # 3072 steps/mm
LAT_TRAVERSE_MM = 100
LAT_HOME_SPEED_HZ = 4800
LAT_HOME_OFFSET_DEFAULT_MM = 15.0
LAT_ACCEL = 40000
LAT_TRAVERSE_SPEED_HZ = 4800
LAT_RODAGE_SPEED_HZ = 12000
LAT_REVERSAL_SLOWDOWN = 0.50

# ── Approach slow-down ────────────────────────────────────────────────────────
APPROACH_TURNS = 80
APPROACH_SPEED_HZ_FLOOR = 200 * STEPS_PER_REV // 60   # ~21333 Hz

# ── Physical: encoder step ────────────────────────────────────────────────────
ENC_STEP_MM = 0.05

# ── Winding defaults ──────────────────────────────────────────────────────────
DEFAULT_TARGET_TURNS = 8000
WINDING_DEFAULT_SEED = 424242

WINDING_LAYER_JITTER_DEFAULT = 0.10
WINDING_LAYER_SPEED_JITTER_DEFAULT = 0.08
WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT = 0.16
WINDING_HUMAN_SPEED_JITTER_DEFAULT = 0.12

# ── WebSocket / HTTP ──────────────────────────────────────────────────────────
WEB_PORT = 80
WS_UPDATE_MS = 200
