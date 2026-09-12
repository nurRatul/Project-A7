"""
mpu6050.py

Driver + lightweight sensor fusion for the GY-521 breakout board
(InvenSense MPU-6050, 3-axis accelerometer + 3-axis gyroscope = 6DOF),
read over I2C.

Wiring (default I2C address 0x68):
    VCC  -> 3.3V or 5V (check your GY-521 board's onboard regulator)
    GND  -> GND
    SCL  -> GPIO3 (SCL)
    SDA  -> GPIO2 (SDA)
    AD0  -> GND for address 0x68, or 3.3V for address 0x69

Before use on the Pi:
    sudo raspi-config          # Interface Options -> I2C -> enable
    pip install smbus2
    i2cdetect -y 1              # should show a device at 68 (or 69)

------------------------------------------------------------------
Honest limits of what this can measure — read before trusting output
------------------------------------------------------------------
* Roll & pitch (tilt) are solid: the accelerometer directly senses the
  gravity vector, blended with the gyro through a complementary filter
  to stay smooth and (near) drift-free.
* Yaw (heading) is gyro-only on this chip — there's no magnetometer —
  so it is a *relative* angular shift since the tracker was started or
  last reset, and it will slowly drift, worse the longer it runs.
* Position comes from double-integrating acceleration. Any tiny bias or
  vibration noise turns into growing velocity error and unbounded
  position error within seconds. Treat x/y here as a rough "how far
  has it drifted since reset" indicator for a short run — not a GPS or
  encoder replacement. If you need real position tracking, pair this
  with wheel encoders or a GPS module and fuse the two.
"""

import logging
import math
import time
from threading import Lock, Thread

from logger.logger_manager import LoggerManager
logger = LoggerManager.get_logger("car_controller.mpu6050")



class MPU6050:
    """Low-level register access + calibration for the MPU-6050."""

    DEFAULT_ADDRESS = 0x68

    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    WHO_AM_I = 0x75

    ACCEL_SCALE = 16384.0  # LSB per g, at the +-2g range configured below
    GYRO_SCALE = 131.0     # LSB per deg/s, at the +-250 dps range configured below
    GRAVITY = 9.80665       # m/s^2

    def __init__(self, bus=1, address=DEFAULT_ADDRESS, sample_rate_hz=100):
        try:
            from smbus2 import SMBus
        except ImportError as error:
            raise RuntimeError(
                "MPU6050 requires smbus2; install the project requirements before use"
            ) from error

        self.address = address
        self.bus = SMBus(bus)
        self._configure(sample_rate_hz)
        self.accel_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.gyro_offset = {"x": 0.0, "y": 0.0, "z": 0.0}

    def _configure(self, sample_rate_hz):
        who = self.bus.read_byte_data(self.address, self.WHO_AM_I)

        if who == 0x68:
            logger.info("MPU6050 detected")
        elif who == 0x70:
            logger.info("MPU6500 detected")
        elif who == 0x71:
            logger.info("MPU9250 detected")
        else:
            logger.warning(
                "Unknown IMU WHO_AM_I=0x%02X",
                who
            )

        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)  # wake up
        time.sleep(0.1)
        divider = max(0, min(255, int(1000 / sample_rate_hz) - 1))
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, divider)
        self.bus.write_byte_data(self.address, self.CONFIG, 0x04)       # DLPF ~44Hz
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)  # +-250 dps
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)  # +-2g

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value

    def read_raw(self):
        """Raw 16-bit register values, no scaling or offsets applied."""
        return {
            "ax": self._read_word(self.ACCEL_XOUT_H),
            "ay": self._read_word(self.ACCEL_XOUT_H + 2),
            "az": self._read_word(self.ACCEL_XOUT_H + 4),
            "gx": self._read_word(self.GYRO_XOUT_H),
            "gy": self._read_word(self.GYRO_XOUT_H + 2),
            "gz": self._read_word(self.GYRO_XOUT_H + 4),
        }

    def read(self):
        """Calibrated (accel_g, gyro_dps) dicts: accel in g, gyro in deg/s."""
        raw = self.read_raw()
        accel = {
            "x": raw["ax"] / self.ACCEL_SCALE - self.accel_offset["x"],
            "y": raw["ay"] / self.ACCEL_SCALE - self.accel_offset["y"],
            "z": raw["az"] / self.ACCEL_SCALE - self.accel_offset["z"],
        }
        gyro = {
            "x": raw["gx"] / self.GYRO_SCALE - self.gyro_offset["x"],
            "y": raw["gy"] / self.GYRO_SCALE - self.gyro_offset["y"],
            "z": raw["gz"] / self.GYRO_SCALE - self.gyro_offset["z"],
        }
        return accel, gyro

    def calibrate(self, samples=200, delay=0.005):
        """
        Average `samples` readings to remove sensor bias. Call this once
        at startup with the car sitting still on a level surface — that's
        what CarController does automatically unless you disable it.
        """
        sums = {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}
        for _ in range(samples):
            raw = self.read_raw()
            for key in sums:
                sums[key] += raw[key]
            time.sleep(delay)

        self.accel_offset = {
            "x": sums["ax"] / samples / self.ACCEL_SCALE,
            "y": sums["ay"] / samples / self.ACCEL_SCALE,
            "z": sums["az"] / samples / self.ACCEL_SCALE - 1.0,  # gravity reads ~1g on Z
        }
        self.gyro_offset = {
            "x": sums["gx"] / samples / self.GYRO_SCALE,
            "y": sums["gy"] / samples / self.GYRO_SCALE,
            "z": sums["gz"] / samples / self.GYRO_SCALE,
        }
        logger.info(
            "IMU calibrated — accel offset=%s gyro offset=%s",
            self.accel_offset, self.gyro_offset,
        )

    def close(self):
        self.bus.close()


class IMUTracker:
    """
    Background thread that polls an MPU6050 at a fixed rate and maintains
    a running estimate of orientation and a rough relative position.

    See the module docstring for what's trustworthy here (roll/pitch) vs.
    what drifts (yaw, position). Access the latest values with snapshot(),
    which is thread-safe.
    """

    def __init__(self, mpu, sample_rate_hz=100, still_accel_tol=0.02, still_gyro_tol=1.5):
        self.mpu = mpu
        self.dt = 1.0 / sample_rate_hz
        self.still_accel_tol = still_accel_tol  # g, deviation from 1g magnitude
        self.still_gyro_tol = still_gyro_tol    # deg/s, magnitude

        self._lock = Lock()
        self._thread = None
        self._running = False

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro_dps = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.accel_g = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.linear_accel = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_update = None

    def reset(self):
        """Zero orientation, velocity, and position — clears accumulated drift."""
        with self._lock:
            self.roll = self.pitch = self.yaw = 0.0
            self.velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
            self.position = {"x": 0.0, "y": 0.0, "z": 0.0}

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _loop(self):
        while self._running:
            started = time.time()
            try:
                self._update()
            except Exception:
                logger.exception("IMU read/update failed")
            elapsed = time.time() - started
            time.sleep(max(0.0, self.dt - elapsed))

    def _update(self):
        accel, gyro = self.mpu.read()  # accel in g, gyro in deg/s

        # Tilt straight from the gravity vector, in degrees.
        accel_roll = math.degrees(math.atan2(accel["y"], accel["z"]))
        accel_pitch = math.degrees(
            math.atan2(-accel["x"], math.sqrt(accel["y"] ** 2 + accel["z"] ** 2))
        )

        with self._lock:
            alpha = 0.98  # complementary filter: trust gyro short-term, accel long-term
            self.roll = alpha * (self.roll + gyro["x"] * self.dt) + (1 - alpha) * accel_roll
            self.pitch = alpha * (self.pitch + gyro["y"] * self.dt) + (1 - alpha) * accel_pitch
            self.yaw = self.yaw + gyro["z"] * self.dt  # gyro-only — will drift

            # First-order tilt compensation to estimate gravity-free linear
            # acceleration (yaw is not corrected for, so x/y stay in a frame
            # that rotates with the car's heading rather than true world axes).
            roll_r = math.radians(self.roll)
            pitch_r = math.radians(self.pitch)
            ax_g, ay_g, az_g = accel["x"], accel["y"], accel["z"]

            level_x = ax_g * math.cos(pitch_r) + az_g * math.sin(pitch_r)
            level_y = ay_g * math.cos(roll_r) - az_g * math.sin(roll_r)
            level_z = (
                -ax_g * math.sin(pitch_r)
                + ay_g * math.sin(roll_r) * math.cos(pitch_r)
                + az_g * math.cos(roll_r) * math.cos(pitch_r)
            )

            lin_x = level_x * self.mpu.GRAVITY
            lin_y = level_y * self.mpu.GRAVITY
            lin_z = (level_z - 1.0) * self.mpu.GRAVITY  # subtract gravity

            gyro_mag = math.sqrt(gyro["x"] ** 2 + gyro["y"] ** 2 + gyro["z"] ** 2)
            accel_mag_err = abs(math.sqrt(ax_g ** 2 + ay_g ** 2 + az_g ** 2) - 1.0)
            is_still = accel_mag_err < self.still_accel_tol and gyro_mag < self.still_gyro_tol

            if is_still:
                # Crude zero-velocity update: while the car looks stationary,
                # bleed off velocity so standing-still noise doesn't quietly
                # accumulate into a "drifting" position.
                self.velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
            else:
                self.velocity["x"] += lin_x * self.dt
                self.velocity["y"] += lin_y * self.dt
                self.velocity["z"] += lin_z * self.dt

            self.position["x"] += self.velocity["x"] * self.dt
            self.position["y"] += self.velocity["y"] * self.dt
            self.position["z"] += self.velocity["z"] * self.dt

            self.gyro_dps = dict(gyro)
            self.accel_g = dict(accel)
            self.linear_accel = {"x": lin_x, "y": lin_y, "z": lin_z}
            self.last_update = time.time()

    def snapshot(self):
        """Thread-safe read of the latest telemetry as plain dicts (JSON-friendly)."""
        with self._lock:
            return {
                "orientation_deg": {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw},
                "gyro_dps": dict(self.gyro_dps),
                "accel_g": dict(self.accel_g),
                "linear_acceleration_ms2": dict(self.linear_accel),
                "velocity_ms": dict(self.velocity),
                "position_m": dict(self.position),
                "last_update": self.last_update,
            }