# localization.py

"""
localization.py

Fuses GPSManager + IMUManager into the single (x_m, y_m, heading_deg)
local pose that everything else in mapping/ works in. This is a
best-effort approach for OUTDOOR / OPEN-SKY use — see the module
docstrings on neom8n.py and mpu6050.py for the honest per-sensor
limits this inherits (GPS: multipath, a few meters of noise even with
a fix; IMU: yaw drifts because there's no magnetometer, position
drifts because it's double-integrated acceleration).

Fusion strategy:
    * Heading (heading_deg) comes from IMU yaw, corrected toward the
      GPS-implied heading (direction of travel between consecutive GPS
      fixes) whenever the rover is moving fast enough for that implied
      heading to be trustworthy (see MIN_GPS_HEADING_SPEED_KMH) — a
      lightweight complementary filter, not a full Kalman filter. This
      bounds IMU yaw drift over a mapping run without a magnetometer.
    * Position (x_m, y_m) is primarily GPS: every NEW fix is converted
      to local meters via an equirectangular projection around the
      mapping origin (accurate to within ~0.1% at the scale of a
      single mapping plot — nowhere near where earth curvature matters).
    * Between GPS fixes (NEO-M8N commonly updates around 1 Hz — far
      slower than the control loop rate you'll want for obstacle
      reactions), position is dead-reckoned forward using the
      commanded speed you pass into update(), then snapped to the next
      real GPS fix when it arrives. That snap can visibly jump the
      estimated position — RoverMap's coverage marking tolerates this
      fine (cells are 0.5m by default) but a precision-sensitive
      consumer would need real sensor fusion (EKF/UKF) instead of this
      snap-and-drift approach — see ARCHITECTURE.md future improvements.
    * If GPS has no fix at all (has_fix() False — e.g. just booted, or
      indoors), position free-runs on dead reckoning alone, which
      accumulates the error mpu6050.py's docstring already warns
      about. wait_for_fix() exists so MappingManager can choose to
      block in INITIALIZING until GPS is trustworthy, rather than
      mapping blind.

CALIBRATION REQUIRED — read before trusting navigation:
    heading_deg here uses the convention 0 = the rover's heading at
    set_origin() time, positive = clockwise/right (i.e. compatible
    with math.atan2(dx, dy), not the standard math.atan2(dy, dx)).
    Whether physically turning the rover right INCREASES or DECREASES
    the raw yaw IMUManager reports depends entirely on which way the
    gyro's Z axis is mounted — this is NOT something the code can know
    for you. Before trusting MappingManager's navigation, verify on
    the real rover: does a physical right turn increase IMU yaw? If
    not, negate raw_yaw below.
"""

import math
import time

from .mapData import Pose

EARTH_RADIUS_M = 6371000.0
MIN_GPS_HEADING_SPEED_KMH = 0.5   # below this, GPS course is too noisy to trust for heading correction
HEADING_CORRECTION_GAIN = 0.3     # how strongly a trusted GPS-implied heading pulls IMU yaw toward it


class PositionEstimator:
    def __init__(self, gps, imu):
        self.gps = gps
        self.imu = imu

        self.origin_lat = None
        self.origin_lon = None

        self.x_m = 0.0
        self.y_m = 0.0
        self.heading_deg = 0.0

        self._last_fix_time = None
        self._yaw_offset_deg = 0.0   # aligns IMU's relative yaw to heading_deg's 0=start-heading frame
        self._last_update = None

    # ---- setup ---------------------------------------------------------

    def wait_for_fix(self, timeout=60.0):
        """Block until GPS has a usable fix, or raise TimeoutError. Call
        this during INITIALIZING before trusting local coordinates."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.gps.has_fix():
                return
            time.sleep(0.25)
        raise TimeoutError(f"No GPS fix after {timeout:.1f}s — check antenna/sky view")

    def set_origin(self, latitude=None, longitude=None):
        """
        Call once at start_mapping(). Captures the current GPS fix (or
        an explicit lat/lon) as local (0, 0), and the current IMU yaw
        as heading_deg's 0 degrees, so every later pose is relative to
        the rover's actual position/orientation at mission start.
        """
        if latitude is None or longitude is None:
            location = self.gps.get_location()
            if location is None:
                raise RuntimeError(
                    "No GPS fix yet — call wait_for_fix() first, or pass latitude/longitude explicitly"
                )
            latitude, longitude = location

        self.origin_lat = latitude
        self.origin_lon = longitude
        self.x_m = 0.0
        self.y_m = 0.0

        orientation = self.imu.get_orientation()
        self._yaw_offset_deg = orientation["yaw"] if orientation else 0.0
        self.heading_deg = 0.0
        self._last_update = time.time()

    # ---- fusion loop ---------------------------------------------------

    def update(self, commanded_speed_mps=0.0):
        """
        Call once per control-loop tick. commanded_speed_mps is the
        rover's current REAL-WORLD forward speed in meters/second (not
        the 0..1 motor duty cycle) — used for dead reckoning between
        GPS fixes. Pass 0.0 while stationary or turning in place.
        """
        if self.origin_lat is None:
            raise RuntimeError("set_origin() must be called before update()")

        now = time.time()
        dt = (now - self._last_update) if self._last_update else 0.0
        self._last_update = now

        orientation = self.imu.get_orientation()
        raw_yaw = orientation["yaw"] if orientation else (self.heading_deg + self._yaw_offset_deg)
        imu_heading = raw_yaw - self._yaw_offset_deg

        fix = self.gps.get_telemetry()
        fix_time = fix.get("last_update")
        is_new_fix = (
            fix.get("has_fix")
            and fix.get("latitude") is not None
            and fix_time is not None
            and fix_time != self._last_fix_time
        )

        if is_new_fix:
            gps_x, gps_y = self._to_local(fix["latitude"], fix["longitude"])

            had_prior_fix = self._last_fix_time is not None
            speed_kmh = fix.get("speed_kmh") or 0.0
            if had_prior_fix and speed_kmh >= MIN_GPS_HEADING_SPEED_KMH:
                implied_heading = math.degrees(math.atan2(gps_x - self.x_m, gps_y - self.y_m))
                imu_heading = self._blend_angles(imu_heading, implied_heading, HEADING_CORRECTION_GAIN)

            self.x_m, self.y_m = gps_x, gps_y
            self._last_fix_time = fix_time
        else:
            # Dead-reckon forward using commanded speed + heading until the next fix arrives.
            heading_rad = math.radians(imu_heading)
            self.x_m += commanded_speed_mps * math.sin(heading_rad) * dt
            self.y_m += commanded_speed_mps * math.cos(heading_rad) * dt

        self.heading_deg = imu_heading
        return self.pose()

    def pose(self):
        latitude, longitude = self._to_global(self.x_m, self.y_m)
        return Pose(
            x_m=self.x_m,
            y_m=self.y_m,
            heading_deg=self.heading_deg,
            latitude=latitude,
            longitude=longitude,
        )

    # ---- projection helpers --------------------------------------------

    def _to_local(self, latitude, longitude):
        lat_rad = math.radians(self.origin_lat)
        dlat = math.radians(latitude - self.origin_lat)
        dlon = math.radians(longitude - self.origin_lon)
        y = dlat * EARTH_RADIUS_M
        x = dlon * EARTH_RADIUS_M * math.cos(lat_rad)
        return x, y

    def _to_global(self, x_m, y_m):
        if self.origin_lat is None:
            return None, None
        lat_rad = math.radians(self.origin_lat)
        dlat = y_m / EARTH_RADIUS_M
        dlon = x_m / (EARTH_RADIUS_M * math.cos(lat_rad))
        return self.origin_lat + math.degrees(dlat), self.origin_lon + math.degrees(dlon)

    @staticmethod
    def _blend_angles(a_deg, b_deg, gain):
        """Blend b into a by `gain`, the short way around the circle."""
        diff = (b_deg - a_deg + 180) % 360 - 180
        return a_deg + gain * diff
