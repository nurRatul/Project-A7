"""
RadarBridge
-----------
Background thread that continuously samples the existing GPSManager and
IMUManager (or any objects exposing the same interface) and feeds
VehicleState.

This is the only place that talks to the hardware managers for radar
purposes, so we never create a second GPS/IMU instance.
"""

from __future__ import annotations

import time
from threading import Event, Lock, Thread
from typing import Any, Optional

from .vehicle_state import VehicleState, get_vehicle_state


class RadarBridge:
    """
    Poll sensors → update VehicleState.

    Parameters
    ----------
    gps :
        Object with get_location() → (lat, lon) | None
        and optionally get_telemetry().
    imu :
        Object with get_orientation() → {"yaw": float, ...} | None
    vehicle_state :
        Shared state (defaults to the process singleton).
    poll_hz :
        How often to sample sensors.
    heading_offset_deg :
        Added to raw IMU yaw so that 0° aligns with map North.
        (MPU-6050 yaw is relative; calibrate once at start of a run.)
    invert_yaw :
        If True, negate the IMU yaw (some mounting orientations need this).
    """

    def __init__(
        self,
        gps: Any = None,
        imu: Any = None,
        vehicle_state: Optional[VehicleState] = None,
        poll_hz: float = 10.0,
        heading_offset_deg: float = 0.0,
        invert_yaw: bool = False,
    ):
        self.gps = gps
        self.imu = imu
        self.state = vehicle_state or get_vehicle_state()
        self.poll_interval = 1.0 / max(1.0, poll_hz)
        self.heading_offset_deg = float(heading_offset_deg)
        self.invert_yaw = bool(invert_yaw)

        self._stop = Event()
        self._thread: Optional[Thread] = None
        self._lock = Lock()
        self._running = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._stop.clear()
            self._running = True
            self._thread = Thread(target=self._loop, name="RadarBridge", daemon=True)
            self._thread.start()
            self.state.set_meta(mode="live", message="Radar bridge running")

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            if self._thread is not None:
                self._thread.join(timeout=2.0)
                self._thread = None
            self._running = False
            self.state.set_meta(mode="idle", message="Radar bridge stopped")

    @property
    def is_running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # Manual injection (used by test_run.py and external modules)
    # ------------------------------------------------------------------

    def inject_position(self, latitude: float, longitude: float) -> dict:
        """Same interface real sensors use — for simulators."""
        return self.state.update_position(latitude, longitude)

    def inject_heading(self, yaw_angle: float) -> None:
        """Same interface real sensors use — for simulators."""
        self.state.update_heading(self._normalise_heading(yaw_angle))

    def inject_pose(
        self, latitude: float, longitude: float, heading: float
    ) -> dict:
        return self.state.update_pose(
            latitude, longitude, self._normalise_heading(heading)
        )

    # ------------------------------------------------------------------
    # Internal polling
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._sample_once()
            except Exception:
                # Never let a sensor glitch kill the bridge
                self.state.set_meta(message="Sensor sample error (see logs)")
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, self.poll_interval - elapsed))

    def _sample_once(self) -> None:
        # --- GPS ---
        if self.gps is not None:
            loc = None
            try:
                loc = self.gps.get_location()
            except Exception:
                pass
            if loc is not None:
                lat, lon = loc
                if lat is not None and lon is not None:
                    self.state.update_position(lat, lon)

        # --- IMU heading ---
        if self.imu is not None:
            orient = None
            try:
                orient = self.imu.get_orientation()
            except Exception:
                pass
            if orient is not None and "yaw" in orient:
                self.state.update_heading(self._normalise_heading(orient["yaw"]))

    def _normalise_heading(self, raw_yaw: float) -> float:
        yaw = float(raw_yaw)
        if self.invert_yaw:
            yaw = -yaw
        yaw += self.heading_offset_deg
        return yaw
