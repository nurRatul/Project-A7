"""
VehicleState
------------
Thread-safe singleton that holds the live vehicle pose, path history,
and (future) obstacle / detection layers.

Any module — real sensors, test_run.py, or a control loop — updates
state through the public methods.  The radar frontend only reads.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from threading import Lock
from typing import Any, Deque, Dict, List, Optional, Tuple

from .coordinate_transformer import MapCoordinateTransformer


@dataclass
class PoseSample:
    """One recorded pose on the path trail."""
    x: float          # local metres (East)
    y: float          # local metres (North)
    heading: float    # degrees, 0 = North, positive clockwise (radar convention)
    latitude: float
    longitude: float
    timestamp: float


@dataclass
class Obstacle:
    """Placeholder for future ultrasonic / LIDAR / vision objects."""
    x: float
    y: float
    radius: float = 0.3
    label: str = "obstacle"
    source: str = "unknown"
    timestamp: float = field(default_factory=time.time)


class VehicleState:
    """
    Central live state for the radar map.

    update_position / update_heading are the only writers that external
    code should call.  All reads are lock-protected snapshots.
    """

    MAX_PATH_POINTS = 5000          # keep memory bounded
    MIN_PATH_DISTANCE_M = 0.15      # ignore micro-jitter when recording trail

    def __init__(self, scale_px_per_m: float = 8.0):
        self._lock = Lock()
        self.transformer = MapCoordinateTransformer(scale_px_per_m=scale_px_per_m)

        # Live pose
        self._latitude: Optional[float] = None
        self._longitude: Optional[float] = None
        self._x: float = 0.0
        self._y: float = 0.0
        self._heading: float = 0.0          # degrees
        self._timestamp: float = 0.0
        self._has_fix: bool = False

        # History
        self._path: Deque[PoseSample] = deque(maxlen=self.MAX_PATH_POINTS)

        # Future expansion layers
        self._obstacles: List[Obstacle] = []
        self._coverage_cells: Dict[Tuple[int, int], int] = {}  # heatmap grid

        # Metadata
        self._meta: Dict[str, Any] = {
            "mode": "idle",
            "message": "Waiting for first GPS fix",
        }

    # ------------------------------------------------------------------
    # Writers (called by sensors / simulator)
    # ------------------------------------------------------------------

    def update_position(
        self,
        latitude: float,
        longitude: float,
        timestamp: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Ingest a new GPS fix.  Automatically sets the map origin on the
        first call.  Returns the local {x, y, heading} snapshot.
        """
        ts = timestamp if timestamp is not None else time.time()
        x, y = self.transformer.to_local_metres(latitude, longitude, auto_origin=True)

        with self._lock:
            self._latitude = float(latitude)
            self._longitude = float(longitude)
            self._x = x
            self._y = y
            self._timestamp = ts
            self._has_fix = True

            # Record path point only if we moved enough
            if not self._path or self._distance_to_last(x, y) >= self.MIN_PATH_DISTANCE_M:
                self._path.append(
                    PoseSample(
                        x=x,
                        y=y,
                        heading=self._heading,
                        latitude=float(latitude),
                        longitude=float(longitude),
                        timestamp=ts,
                    )
                )

            # Simple coverage heatmap (0.5 m cells)
            cell = (int(round(x * 2)), int(round(y * 2)))
            self._coverage_cells[cell] = self._coverage_cells.get(cell, 0) + 1

            return {"x": self._x, "y": self._y, "heading": self._heading}

    def update_heading(self, yaw_angle: float, timestamp: Optional[float] = None) -> None:
        """
        Ingest a heading update (degrees).

        Convention used by the radar UI:
            0°   = North (+Y)
            90°  = East  (+X)
            positive = clockwise (matches typical GPS course / compass)

        The MPU-6050 yaw is relative and increases with gyro Z; the
        bridge layer is responsible for any sign/offset conversion.
        """
        ts = timestamp if timestamp is not None else time.time()
        with self._lock:
            # Normalise to [0, 360)
            h = float(yaw_angle) % 360.0
            if h < 0:
                h += 360.0
            self._heading = h
            self._timestamp = ts

    def update_pose(
        self,
        latitude: float,
        longitude: float,
        heading: float,
        timestamp: Optional[float] = None,
    ) -> Dict[str, float]:
        """Convenience: position + heading in one atomic update."""
        self.update_heading(heading, timestamp)
        return self.update_position(latitude, longitude, timestamp)

    def set_origin(self, latitude: float, longitude: float) -> None:
        self.transformer.set_origin(latitude, longitude)
        with self._lock:
            self._x = 0.0
            self._y = 0.0
            self._path.clear()
            self._coverage_cells.clear()

    def add_obstacle(
        self,
        x: float,
        y: float,
        radius: float = 0.3,
        label: str = "obstacle",
        source: str = "unknown",
    ) -> None:
        with self._lock:
            self._obstacles.append(
                Obstacle(x=x, y=y, radius=radius, label=label, source=source)
            )

    def clear_obstacles(self) -> None:
        with self._lock:
            self._obstacles.clear()

    def clear_path(self) -> None:
        with self._lock:
            self._path.clear()
            self._coverage_cells.clear()

    def set_meta(self, **kwargs: Any) -> None:
        with self._lock:
            self._meta.update(kwargs)

    # ------------------------------------------------------------------
    # Readers
    # ------------------------------------------------------------------

    def snapshot(self) -> Dict[str, Any]:
        """Full JSON-serialisable state for the radar WebSocket push."""
        with self._lock:
            path = [
                {"x": p.x, "y": p.y, "heading": p.heading, "t": p.timestamp}
                for p in self._path
            ]
            obstacles = [
                {
                    "x": o.x,
                    "y": o.y,
                    "radius": o.radius,
                    "label": o.label,
                    "source": o.source,
                }
                for o in self._obstacles
            ]
            origin = self.transformer.get_origin()
            return {
                "x": self._x,
                "y": self._y,
                "heading": self._heading,
                "latitude": self._latitude,
                "longitude": self._longitude,
                "timestamp": self._timestamp,
                "has_fix": self._has_fix,
                "path": path,
                "obstacles": obstacles,
                "path_count": len(self._path),
                "origin": {"lat": origin[0], "lon": origin[1]} if origin else None,
                "scale_px_per_m": self.transformer.scale_px_per_m,
                "meta": dict(self._meta),
            }

    def get_local_pose(self) -> Dict[str, float]:
        with self._lock:
            return {
                "x": self._x,
                "y": self._y,
                "heading": self._heading,
            }

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _distance_to_last(self, x: float, y: float) -> float:
        if not self._path:
            return float("inf")
        last = self._path[-1]
        return math.hypot(x - last.x, y - last.y)


# Avoid circular import of math only for the helper
import math  # noqa: E402


# ---------------------------------------------------------------------------
# Process-wide singleton
# ---------------------------------------------------------------------------
_instance: Optional[VehicleState] = None
_instance_lock = Lock()


def get_vehicle_state(scale_px_per_m: float = 8.0) -> VehicleState:
    """Return the shared VehicleState (created on first call)."""
    global _instance
    with _instance_lock:
        if _instance is None:
            _instance = VehicleState(scale_px_per_m=scale_px_per_m)
        return _instance
