# Control Loop Benchmark Guide

## Goal

Measure timing and memory headroom on target ESP32 during realistic operation.

## Build profile

- Production-like benchmark: `esp32dev`
- Lab diagnostics benchmark: `esp32dev-lab`

## Procedure

1. Flash firmware and run for at least 10 minutes under normal use.
2. Trigger representative command traffic from UART and Web UI.
3. Observe periodic `[RTOS]` logs:
   - `control_worst_loop_us`
   - `comms_worst_loop_us`
   - `heap_free`, `heap_min`
   - `ctrl_hwm`, `comm_hwm`
   - `cmd_enq`, `cmd_drop_q`, `cmd_rej_len`, `cmd_rej_schema`
4. Repeat with maximal expected spindle speed and command churn.

## Pass criteria (recommended)

- `control_worst_loop_us` comfortably below control period budget.
- No command queue saturation in expected workload (`cmd_drop_q == 0`).
- Stable heap floor (`heap_min`) with no downward drift trend.
- Stack high-water marks keep a healthy safety margin.

## Notes

When changing command schema or telemetry payload, rerun this benchmark and update baseline values.
