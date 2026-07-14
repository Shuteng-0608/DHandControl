"""Record and replay validated Vision Pro hand frames for deterministic debugging."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import time
from typing import Dict, List, Optional

import numpy as np

from visionpro_stream import VisionProHandFrame


SESSION_FORMAT_VERSION = 1
SESSION_PHASES = ("neutral", "range", "teleop")


class HandSessionRecorder:
    """Collect raw hand transforms and atomically save a compressed NPZ session."""

    def __init__(self, path: str, metadata: Optional[Dict] = None) -> None:
        self.path = Path(path).expanduser()
        self.metadata = dict(metadata or {})
        self.timestamps: List[float] = []
        self.transforms: List[np.ndarray] = []
        self.phases: List[str] = []
        self.hands: List[str] = []
        self._first_timestamp: Optional[float] = None
        self.saved = False

    def record(self, frame: VisionProHandFrame, phase: str) -> None:
        if phase not in SESSION_PHASES:
            raise ValueError(f"unknown recording phase: {phase}")
        transforms = np.asarray(frame.transforms, dtype=float)
        if transforms.shape != (27, 4, 4) or not np.all(np.isfinite(transforms)):
            raise ValueError("recorded transforms must be finite with shape (27,4,4)")
        timestamp = float(frame.timestamp)
        if not np.isfinite(timestamp):
            raise ValueError("recorded frame timestamp must be finite")
        if self._first_timestamp is None:
            self._first_timestamp = timestamp
        self.timestamps.append(timestamp - self._first_timestamp)
        self.transforms.append(transforms.copy())
        self.phases.append(phase)
        self.hands.append(str(frame.hand))

    def save(self) -> Optional[Path]:
        if self.saved:
            return self.path
        if not self.transforms:
            return None

        self.path.parent.mkdir(parents=True, exist_ok=True)
        metadata = dict(self.metadata)
        metadata.update(
            {
                "format_version": SESSION_FORMAT_VERSION,
                "frame_count": len(self.transforms),
                "phases": sorted(set(self.phases)),
            }
        )
        arrays = {
            "format_version": np.asarray(SESSION_FORMAT_VERSION, dtype=np.int64),
            "timestamps": np.asarray(self.timestamps, dtype=np.float64),
            "transforms": np.stack(self.transforms).astype(np.float64, copy=False),
            "phases": np.asarray(self.phases, dtype="U16"),
            "hands": np.asarray(self.hands, dtype="U8"),
            "metadata_json": np.asarray(json.dumps(metadata, ensure_ascii=False)),
        }

        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb",
                suffix=".npz",
                prefix=f".{self.path.name}.",
                dir=self.path.parent,
                delete=False,
            ) as temporary:
                temporary_path = Path(temporary.name)
                np.savez_compressed(temporary, **arrays)
            os.replace(temporary_path, self.path)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()
        self.saved = True
        return self.path


class ReplayHandStream:
    """VisionProHandStream-compatible source backed by a recorded NPZ session."""

    is_replay = True

    def __init__(
        self,
        path: str,
        *,
        speed: float = 1.0,
        loop: bool = False,
        no_wait: bool = False,
    ) -> None:
        if speed <= 0.0:
            raise ValueError("replay speed must be positive")
        self.path = Path(path).expanduser()
        self.speed = float(speed)
        self.loop = bool(loop)
        self.no_wait = bool(no_wait)
        self.timestamps: Optional[np.ndarray] = None
        self.transforms: Optional[np.ndarray] = None
        self.phases: Optional[np.ndarray] = None
        self.hands: Optional[np.ndarray] = None
        self.metadata: Dict = {}
        self.current_phase: Optional[str] = None
        self._phase_indices = np.empty(0, dtype=int)
        self._phase_times = np.empty(0, dtype=float)
        self._cursor = 0
        self._wall_start: Optional[float] = None
        self._timeline_start: Optional[float] = None
        self._timeline_offset = 0.0

    def start(self) -> None:
        try:
            with np.load(self.path, allow_pickle=False) as archive:
                version = int(np.asarray(archive["format_version"]).item())
                if version != SESSION_FORMAT_VERSION:
                    raise RuntimeError(
                        f"unsupported replay format {version}; "
                        f"expected {SESSION_FORMAT_VERSION}"
                    )
                self.timestamps = np.asarray(archive["timestamps"], dtype=float)
                self.transforms = np.asarray(archive["transforms"], dtype=float)
                self.phases = np.asarray(archive["phases"], dtype=str)
                self.hands = np.asarray(archive["hands"], dtype=str)
                metadata_text = str(np.asarray(archive["metadata_json"]).item())
                self.metadata = json.loads(metadata_text)
        except RuntimeError:
            raise
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"failed to load replay session: {self.path}") from exc

        count = len(self.timestamps)
        if self.transforms.shape != (count, 27, 4, 4):
            raise RuntimeError("replay transforms have an invalid shape")
        if len(self.phases) != count or len(self.hands) != count:
            raise RuntimeError("replay frame arrays have inconsistent lengths")
        if count == 0 or not np.all(np.isfinite(self.transforms)):
            raise RuntimeError("replay session contains no valid frames")
        if not np.all(np.isfinite(self.timestamps)):
            raise RuntimeError("replay timestamps must be finite")
        for phase in SESSION_PHASES:
            phase_times = self.timestamps[self.phases == phase]
            if len(phase_times) > 1 and np.any(np.diff(phase_times) < 0.0):
                raise RuntimeError(f"replay timestamps go backwards in phase '{phase}'")

    def stop(self) -> None:
        self.current_phase = None
        self._phase_indices = np.empty(0, dtype=int)
        self._phase_times = np.empty(0, dtype=float)
        self._cursor = 0
        self._wall_start = None
        self._timeline_start = None
        self._timeline_offset = 0.0

    def set_phase(self, phase: str) -> None:
        if phase not in SESSION_PHASES:
            raise ValueError(f"unknown replay phase: {phase}")
        if self.phases is None:
            raise RuntimeError("replay stream has not been started")
        indices = np.flatnonzero(self.phases == phase)
        if len(indices) == 0:
            raise RuntimeError(f"replay session has no '{phase}' frames")
        phase_times = self.timestamps[indices]
        self.current_phase = phase
        self._phase_indices = indices
        self._phase_times = phase_times - phase_times[0]
        self._cursor = 0
        self._wall_start = time.monotonic()
        self._timeline_start = self._wall_start
        self._timeline_offset = 0.0

    @property
    def phase_finished(self) -> bool:
        return self._cursor >= len(self._phase_indices)

    def _restart_teleop_loop(self, now: float) -> bool:
        if not (self.loop and self.current_phase == "teleop" and self.phase_finished):
            return False
        cycle_duration = float(self._phase_times[-1]) / self.speed
        if len(self._phase_times) > 1:
            positive_steps = np.diff(self._phase_times)
            positive_steps = positive_steps[positive_steps > 0.0]
            cycle_gap = (
                float(np.median(positive_steps)) / self.speed
                if len(positive_steps)
                else 1e-6
            )
        else:
            cycle_gap = 1e-6
        self._timeline_offset += cycle_duration + cycle_gap
        self._cursor = 0
        self._wall_start = now
        if self._timeline_start is None:
            self._timeline_start = now
        return True

    def get_latest_frame(self) -> Optional[VisionProHandFrame]:
        if self.transforms is None or self.current_phase is None:
            raise RuntimeError("replay phase must be selected before reading frames")
        now = time.monotonic()
        self._restart_teleop_loop(now)
        if self.phase_finished:
            return None

        if self.no_wait:
            selected_cursor = self._cursor
            self._cursor += 1
        else:
            elapsed_recording_time = (now - self._wall_start) * self.speed
            if self._phase_times[self._cursor] > elapsed_recording_time:
                return None
            selected_cursor = self._cursor
            self._cursor += 1

        frame_index = int(self._phase_indices[selected_cursor])
        relative_time = float(self._phase_times[selected_cursor]) / self.speed
        frame_timestamp = (
            float(self._timeline_start) + self._timeline_offset + relative_time
        )
        transforms = self.transforms[frame_index].copy()
        return VisionProHandFrame(
            points=transforms[:, :3, 3].copy(),
            transforms=transforms,
            timestamp=frame_timestamp,
            hand=str(self.hands[frame_index]),
        )
