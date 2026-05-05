from __future__ import annotations

import dataclasses
import time
from typing import TYPE_CHECKING, Any, Callable

from core.config import AppConfiguration
from core.shared_state import SharedState
from transport.messages import StatusPayload

if TYPE_CHECKING:
    from core.lateral import LateralAxisController


def serialize_firmware_status(status: StatusPayload) -> dict[str, Any]:
    return {
        "uptime_ms": status.uptime_ms,
        "queue_free_slots": list(status.queue_free_slots),
        "ring_free_slots": list(status.ring_free_slots),
        "underrun_count": list(status.underrun_count),
        "last_rx_sequence": status.last_rx_sequence,
        "last_rx_type": status.last_rx_type,
        "last_result": status.last_result,
        "protocol_version": status.protocol_version,
        "enabled_mask": status.enabled_mask,
        "running_mask": status.running_mask,
        "lateral_endstop_state": status.lateral_endstop_state,
        "endstop_armed_mask": status.endstop_armed_mask,
        "endstop_hit_mask": status.endstop_hit_mask,
        "last_executed_sequence": status.last_executed_sequence,
        "multi_axis_queue_free": status.multi_axis_queue_free,
        "planner_queue_free": status.planner_queue_free,
        "last_planned_sequence": status.last_planned_sequence,
        "segments_dropped": status.segments_dropped,
    }


def serialize_configuration(config: AppConfiguration) -> dict[str, Any]:
    snapshot = dataclasses.asdict(config)
    snapshot["lateral_steps_per_mm"] = config.lateral_steps_per_mm
    snapshot["lateral_soft_limit_min_steps"] = config.lateral_soft_limit_min_steps
    snapshot["lateral_soft_limit_max_steps"] = config.lateral_soft_limit_max_steps
    snapshot["spindle_max_acceleration_steps_per_s2"] = (
        config.spindle_max_acceleration_steps_per_s2
    )
    snapshot["spindle_max_deceleration_steps_per_s2"] = (
        config.spindle_max_deceleration_steps_per_s2
    )
    snapshot["lateral_max_acceleration_steps_per_s2"] = (
        config.lateral_max_acceleration_steps_per_s2
    )
    snapshot["lateral_max_deceleration_steps_per_s2"] = (
        config.lateral_max_deceleration_steps_per_s2
    )
    return snapshot


class RuntimeStatusService:
    """Build explicit status snapshots for RPC and diagnostics."""

    def __init__(
        self,
        *,
        shared_state: SharedState,
        move_queue_status_provider: Callable[[], dict[str, Any]],
        lateral_controller: LateralAxisController,
        config: AppConfiguration,
        transport_diagnostics_provider: Callable[[], dict[str, Any]] | None = None,
        engine_health_provider: Callable[[], dict[str, Any]] | None = None,
        adaptive_health_provider: Callable[[], dict[str, Any]] | None = None,
        rpc_health_provider: Callable[[], dict[str, Any]] | None = None,
        started_at_monotonic: float | None = None,
    ) -> None:
        self._shared_state = shared_state
        self._move_queue_status_provider = move_queue_status_provider
        self._lateral = lateral_controller
        self._config = config
        self._transport_diagnostics_provider = transport_diagnostics_provider
        self._engine_health_provider = engine_health_provider
        self._adaptive_health_provider = adaptive_health_provider
        self._rpc_health_provider = rpc_health_provider
        self._started_at_monotonic = (
            time.monotonic()
            if started_at_monotonic is None
            else started_at_monotonic
        )

    def _workers_status(self) -> dict[str, Any]:
        workers: dict[str, Any] = {}
        if self._engine_health_provider is not None:
            workers["engine"] = self._engine_health_provider()
        if self._adaptive_health_provider is not None:
            workers["adaptive_winding"] = self._adaptive_health_provider()
        if self._rpc_health_provider is not None:
            workers["rpc_server"] = self._rpc_health_provider()
        return workers

    @staticmethod
    def _worker_faulted(worker: dict[str, Any]) -> bool:
        return bool(
            worker.get("thread_faulted")
            or worker.get("accept_thread_faulted")
            or worker.get("notify_thread_faulted")
        )

    @staticmethod
    def _worker_alive(worker: dict[str, Any]) -> bool:
        alive_keys = [
            key
            for key in (
                "thread_alive",
                "accept_thread_alive",
                "notify_thread_alive",
            )
            if key in worker
        ]
        if not alive_keys:
            return True
        return all(bool(worker[key]) for key in alive_keys)

    @staticmethod
    def _derive_health(workers: dict[str, Any], engine_state: str) -> str:
        if engine_state == "FAULT":
            return "faulted"
        if any(RuntimeStatusService._worker_faulted(worker) for worker in workers.values()):
            return "faulted"
        if any(not RuntimeStatusService._worker_alive(worker) for worker in workers.values()):
            return "degraded"
        return "healthy"

    def application_status(self) -> dict[str, Any]:
        workers = self._workers_status()
        engine_state = self._shared_state.engine_state.name
        return {
            "uptime_s": round(time.monotonic() - self._started_at_monotonic, 2),
            "configured": True,
            "engine_state": engine_state,
            "health": self._derive_health(workers, engine_state),
            "workers": workers,
            "rpc_socket_path": self._config.rpc_socket_path,
            "spi_device": self._config.spi_device,
        }

    def configuration_status(self) -> dict[str, Any]:
        return serialize_configuration(self._config)

    def engine_status(self) -> dict[str, Any]:
        self._lateral.refresh_home_state()
        status = {
            "shared_state": self._shared_state.snapshot(),
            "move_queue": self._move_queue_status_provider(),
            "workers": self._workers_status(),
        }
        if self._transport_diagnostics_provider is not None:
            status["transport"] = self._transport_diagnostics_provider()
        return status

    def axis_state(self, axis_id: int) -> dict[str, Any]:
        state = self._shared_state.axis_states.get(axis_id)
        if state is None:
            raise RuntimeError(f"Unknown axis_id: {axis_id}")
        return state.snapshot()
