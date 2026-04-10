# Synchronization Sequence — Host → Daemon → PRU Orchestrator → PRU Motor

This document contains a Mermaid sequence diagram that illustrates the `move_to` flow and the spindle↔lateral coordination path. The diagram is in English and references the key functions in the codebase.

```mermaid
sequenceDiagram
    participant Host
    participant Daemon
    participant PRU0 as Orchestrator_PRU0
    participant PRU1 as Motor_PRU1

    Host->>Daemon: JSON move_to(pos, start_hz, cruise_hz, accel_steps)
    Daemon->>Daemon: send_host_move_to() compute start_iv, cruise_iv, delta_iv, sp_lat_coord (Q6)
    Daemon->>PRU0: write host_cmd_t (HOST_CMD_MOVE_TO)

    PRU0->>PRU0: process_host_cmd() arm motor_params_t (lat_target, lat_start_iv, lat_cruise_iv, lat_delta_iv)
    PRU0->>PRU1: publish motor_params_t (lat_move_active = 1)

    PRU1->>PRU1: lat_move_arm() (hot-retarget or cold start)
    PRU1->>PRU1: lat_move_on_step() trapezoidal FSM: ACCEL -> CRUISE -> DECEL
    PRU1-->>PRU0: set motor_telem_t.lat_move_done = 1

    PRU0->>PRU0: move_complete_check() set pru_status_t.event_type = EVENT_MOVE_COMPLETE
    PRU0->>Daemon: publish status with move_complete event
    Daemon->>Host: JSON event move_complete

    Note right of PRU0: coord_tick() runs periodically and applies
    Note right of PRU0: sp_adj = (sp_lat_coord * lat_interval) >> 6 to motor_params_t.sp_interval
    Note right of PRU0: sp_lat_coord computed by the daemon as (sp_cruise_iv * 64) / lat_cruise_iv (Q6)
```

## Annotated function references

- `send_host_move_to()` — [src/linux/daemon/pickup_daemon.c](src/linux/daemon/pickup_daemon.c#L191)
- `process_host_cmd()` — [src/pru/orchestrator/main.c](src/pru/orchestrator/main.c#L174)
- `coord_tick()` — [src/pru/orchestrator/main.c](src/pru/orchestrator/main.c#L329)
- `move_complete_check()` — [src/pru/orchestrator/main.c](src/pru/orchestrator/main.c#L341)
- `lat_move_arm()` — [src/pru/motor_control/main.c](src/pru/motor_control/main.c#L77)
- `lat_move_on_step()` — [src/pru/motor_control/main.c](src/pru/motor_control/main.c#L127)

## Notes

- The diagram assumes the orchestrator runs on **PRU0** and the motor firmware runs on **PRU1** (see function headers for confirmation).
- The Q6 ratio avoids runtime division on the PRU; the daemon performs the integer division once when arming the move.
- Safety: endstops and software limits are checked by the orchestrator and will abort moves on fault.

