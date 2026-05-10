from __future__ import annotations

import json
import logging
import re
import shutil
import threading
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_BACKUP_KEEP = 5

from winding.program import WindingProgram

logger = logging.getLogger(__name__)

_DERIVED_PROGRAM_FIELDS = {
    "id",
    "turns_per_mm",
    "layer_duration_s",
    "revision",
    "created_at",
    "updated_at",
}


class ProgramNotFoundError(FileNotFoundError):
    """Raised when a persisted winding program cannot be found."""


class ProgramStore:
    """Persist winding programs as JSON files on the Raspberry Pi host."""

    def __init__(self, storage_dir: str | Path) -> None:
        self._storage_dir = Path(storage_dir)
        self._lock = threading.RLock()

    @property
    def storage_dir(self) -> Path:
        return self._storage_dir

    def list_programs(self, *, include_content: bool = False) -> list[dict[str, Any]]:
        with self._lock:
            if not self._storage_dir.exists():
                return []

            programs: list[dict[str, Any]] = []
            for path in sorted(self._storage_dir.glob("*.json")):
                try:
                    program = self._read_program(path)
                except Exception as exc:
                    logger.warning("Skipping unreadable program file %s: %s", path, exc)
                    continue
                programs.append(
                    program.snapshot() if include_content else self._summary(program)
                )

            programs.sort(
                key=lambda item: (
                    str(item.get("updated_at") or ""),
                    str(item.get("created_at") or ""),
                    str(item.get("id") or ""),
                ),
                reverse=True,
            )
            return programs

    def get_program(self, program_id: str) -> WindingProgram:
        normalized_id = self._normalize_existing_id(program_id)
        with self._lock:
            return self._read_program(self._program_path(normalized_id))

    def save_program(
        self,
        program: WindingProgram,
        *,
        program_id: str | None = None,
    ) -> WindingProgram:
        requested_id = program_id or program.program_id
        normalized_id = self._normalize_new_id(requested_id, fallback_name=program.name)
        now = self._utc_now()

        with self._lock:
            existing = self._try_read_program(normalized_id)
            persisted = replace(
                program,
                program_id=normalized_id,
                revision=(existing.revision + 1) if existing is not None else max(program.revision, 1),
                created_at=(existing.created_at if existing is not None else program.created_at or now),
                updated_at=now,
            )
            persisted.validate()
            self._write_program(persisted)
            return persisted

    def update_program(self, program_id: str, changes: dict[str, Any]) -> WindingProgram:
        if not isinstance(changes, dict):
            raise ValueError("program changes must be an object")

        normalized_id = self._normalize_existing_id(program_id)
        with self._lock:
            existing = self._read_program(self._program_path(normalized_id))
            merged_payload = existing.to_dict()
            sanitized_changes = self._sanitize_changes(changes)
            requested_id = sanitized_changes.pop("program_id", None)
            if requested_id is not None and requested_id != normalized_id:
                raise ValueError("program_id cannot be changed once created")
            merged_payload.update(sanitized_changes)
            updated = replace(
                WindingProgram.from_payload(merged_payload),
                program_id=normalized_id,
                revision=existing.revision + 1,
                created_at=existing.created_at or self._utc_now(),
                updated_at=self._utc_now(),
            )
            updated.validate()
            self._write_program(updated)
            return updated

    def delete_program(self, program_id: str) -> None:
        normalized_id = self._normalize_existing_id(program_id)
        with self._lock:
            path = self._program_path(normalized_id)
            if not path.exists():
                raise ProgramNotFoundError(f"Unknown program_id: {normalized_id}")
            try:
                existing = self._read_program(path)
                backup_dir = self._storage_dir / ".backup"
                backup_dir.mkdir(exist_ok=True)
                backup_name = f"{normalized_id}_rev{existing.revision:04d}_deleted.json"
                path.rename(backup_dir / backup_name)
            except Exception as exc:
                logger.warning("Could not backup before delete %s: %s", normalized_id, exc)
                path.unlink()

    def list_revisions(self, program_id: str) -> list[dict[str, Any]]:
        """Return backup revisions for *program_id*, newest first."""
        normalized_id = self._normalize_existing_id(program_id)
        backup_dir = self._storage_dir / ".backup"
        if not backup_dir.exists():
            return []
        revisions: list[dict[str, Any]] = []
        for path in sorted(backup_dir.glob(f"{normalized_id}_rev[0-9]*.json"), reverse=True):
            try:
                revisions.append(self._summary(self._read_program(path)))
            except Exception as exc:
                logger.warning("Skipping unreadable backup %s: %s", path, exc)
        return revisions

    def restore_revision(self, program_id: str, revision: int) -> "WindingProgram":
        """Restore *program_id* from backup *revision* and save as the new head."""
        normalized_id = self._normalize_existing_id(program_id)
        backup_path = (
            self._storage_dir / ".backup" / f"{normalized_id}_rev{revision:04d}.json"
        )
        if not backup_path.exists():
            raise ProgramNotFoundError(
                f"Backup revision {revision} not found for program {program_id!r}"
            )
        with self._lock:
            return self.save_program(
                self._read_program(backup_path), program_id=normalized_id
            )

    def _summary(self, program: WindingProgram) -> dict[str, Any]:
        snapshot = program.snapshot()
        return {
            "id": snapshot["id"],
            "program_id": snapshot["program_id"],
            "name": snapshot["name"],
            "revision": snapshot["revision"],
            "created_at": snapshot["created_at"],
            "updated_at": snapshot["updated_at"],
            "num_layers": snapshot["num_layers"],
            "spindle_rpm": snapshot["spindle_rpm"],
            "layer_pitch_mm": snapshot["layer_pitch_mm"],
            "wire_diameter_mm": snapshot["wire_diameter_mm"],
            "bobbin_width_mm": snapshot["bobbin_width_mm"],
        }

    def _sanitize_changes(self, changes: dict[str, Any]) -> dict[str, Any]:
        sanitized = dict(changes)
        if "id" in sanitized and "program_id" not in sanitized:
            sanitized["program_id"] = sanitized.pop("id")
        for derived_field in _DERIVED_PROGRAM_FIELDS:
            sanitized.pop(derived_field, None)
        return sanitized

    def _program_path(self, program_id: str) -> Path:
        return self._storage_dir / f"{program_id}.json"

    def _try_read_program(self, program_id: str) -> WindingProgram | None:
        path = self._program_path(program_id)
        if not path.exists():
            return None
        return self._read_program(path)

    def _read_program(self, path: Path) -> WindingProgram:
        if not path.exists():
            raise ProgramNotFoundError(f"Unknown program_id: {path.stem}")
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        program = WindingProgram.from_payload(payload)
        if not program.program_id:
            program = replace(program, program_id=path.stem)
        program.validate()
        return program

    def _write_program(self, program: WindingProgram) -> None:
        self._storage_dir.mkdir(parents=True, exist_ok=True)
        path = self._program_path(
            program.program_id or self._normalize_new_id(None, fallback_name=program.name)
        )
        # Backup existing revision before overwriting
        if path.exists():
            try:
                existing = self._read_program(path)
                backup_dir = self._storage_dir / ".backup"
                backup_dir.mkdir(exist_ok=True)
                backup_name = f"{existing.program_id}_rev{existing.revision:04d}.json"
                shutil.copy2(path, backup_dir / backup_name)
                self._prune_backups(backup_dir, existing.program_id)
            except Exception as exc:
                logger.warning(
                    "Could not create backup for %s: %s", program.program_id, exc
                )
        temp_path = path.with_suffix(".json.tmp")
        with temp_path.open("w", encoding="utf-8") as handle:
            json.dump(program.to_dict(), handle, indent=2, sort_keys=True)
            handle.write("\n")
        temp_path.replace(path)

    def _prune_backups(self, backup_dir: Path, program_id: str) -> None:
        """Keep only the last _BACKUP_KEEP non-deleted revisions for *program_id*."""
        backups = sorted(backup_dir.glob(f"{program_id}_rev[0-9]*.json"))
        for old in backups[:-_BACKUP_KEEP]:
            try:
                old.unlink()
            except OSError as exc:
                logger.warning("Could not prune backup %s: %s", old, exc)

    @staticmethod
    def _utc_now() -> str:
        return (
            datetime.now(timezone.utc)
            .replace(microsecond=0)
            .isoformat()
            .replace("+00:00", "Z")
        )

    @staticmethod
    def _slugify(name: str) -> str:
        slug = re.sub(r"[^a-z0-9]+", "-", name.strip().lower()).strip("-")
        return slug or "program"

    def _normalize_existing_id(self, program_id: str) -> str:
        normalized_id = self._normalize_new_id(program_id, fallback_name="program")
        if not self._program_path(normalized_id).exists():
            raise ProgramNotFoundError(
                f"Unknown program_id: {program_id!r} (normalized: {normalized_id!r})"
            )
        return normalized_id

    def _normalize_new_id(self, program_id: str | None, *, fallback_name: str) -> str:
        if program_id is not None:
            candidate = str(program_id).strip().lower()
        else:
            candidate = self._slugify(fallback_name)
        candidate = re.sub(r"[^a-z0-9._-]+", "-", candidate).strip("-._")
        if not candidate:
            candidate = self._slugify(fallback_name)
        if not candidate:
            raise ValueError("program_id cannot be empty")
        return candidate
