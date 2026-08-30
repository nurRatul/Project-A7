"""
MapCoordinateTransformer
------------------------
Converts WGS-84 latitude/longitude into a local East-North metre frame
relative to a fixed origin, then optionally scales to canvas pixels.

The origin is set on the first valid GPS fix (or can be forced).
Future obstacle / LIDAR points can use the same transform.
"""

from __future__ import annotations

import math
from threading import Lock
from typing import Optional, Tuple


# Mean Earth radius (WGS-84 approximation) in metres
_EARTH_RADIUS_M = 6_371_000.0


class MapCoordinateTransformer:
    """Thread-safe GPS → local-metres → optional canvas-pixel converter."""

    def __init__(self, scale_px_per_m: float = 8.0):
        self._lock = Lock()
        self._origin_lat: Optional[float] = None
        self._origin_lon: Optional[float] = None
        self._origin_lat_rad: Optional[float] = None
        self.scale_px_per_m = float(scale_px_per_m)

    # ------------------------------------------------------------------
    # Origin management
    # ------------------------------------------------------------------

    def set_origin(self, latitude: float, longitude: float) -> None:
        """Force the local origin (e.g. start of a coverage run)."""
        with self._lock:
            self._origin_lat = float(latitude)
            self._origin_lon = float(longitude)
            self._origin_lat_rad = math.radians(self._origin_lat)

    def has_origin(self) -> bool:
        with self._lock:
            return self._origin_lat is not None

    def get_origin(self) -> Optional[Tuple[float, float]]:
        with self._lock:
            if self._origin_lat is None:
                return None
            return (self._origin_lat, self._origin_lon)

    def clear_origin(self) -> None:
        with self._lock:
            self._origin_lat = None
            self._origin_lon = None
            self._origin_lat_rad = None

    # ------------------------------------------------------------------
    # Core conversion
    # ------------------------------------------------------------------

    def to_local_metres(
        self, latitude: float, longitude: float, auto_origin: bool = True
    ) -> Tuple[float, float]:
        """
        Convert (lat, lon) → (x_east, y_north) in metres relative to origin.

        If no origin has been set and auto_origin is True, the first call
        becomes the origin (returns 0, 0).
        """
        lat = float(latitude)
        lon = float(longitude)

        with self._lock:
            if self._origin_lat is None:
                if not auto_origin:
                    raise RuntimeError("Map origin has not been set")
                self._origin_lat = lat
                self._origin_lon = lon
                self._origin_lat_rad = math.radians(lat)
                return (0.0, 0.0)

            dlat = math.radians(lat - self._origin_lat)
            dlon = math.radians(lon - self._origin_lon)
            # Equirectangular approximation — excellent for < a few km
            x = _EARTH_RADIUS_M * dlon * math.cos(self._origin_lat_rad)
            y = _EARTH_RADIUS_M * dlat
            return (x, y)

    def to_canvas(
        self,
        latitude: float,
        longitude: float,
        canvas_cx: float,
        canvas_cy: float,
        auto_origin: bool = True,
    ) -> Tuple[float, float]:
        """
        Convert GPS → canvas pixel coordinates.
        Canvas Y is inverted so +North points up on screen.
        """
        x_m, y_m = self.to_local_metres(latitude, longitude, auto_origin)
        px = canvas_cx + x_m * self.scale_px_per_m
        py = canvas_cy - y_m * self.scale_px_per_m
        return (px, py)

    def metres_to_canvas(
        self, x_m: float, y_m: float, canvas_cx: float, canvas_cy: float
    ) -> Tuple[float, float]:
        """Convert already-local metres to canvas pixels."""
        px = canvas_cx + x_m * self.scale_px_per_m
        py = canvas_cy - y_m * self.scale_px_per_m
        return (px, py)

    def set_scale(self, px_per_m: float) -> None:
        self.scale_px_per_m = max(0.1, float(px_per_m))
