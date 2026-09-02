# ultrasonicManager.py

"""
ultrasonicManager.py

Driver + background tracker for a 3-sensor HC-SR04 ultrasonic array
(front / left / right), covering the obstacle-detection sensors used
by MappingManager for side-pass avoidance. Mirrors gpsManager.py /
imuManager.py's shape exactly: a low-level driver, a background
tracker thread, and a high-level *Manager wrapper — so CarController,
ArmController, and MappingManager can all depend on ONE consistent
interface across every sensor.

Wiring (module -> Pi), one set per sensor:
    VCC  -> 5V
    GND  -> GND
    TRIG -> any free GPIO (output)
    ECHO -> any free GPIO **through a voltage divider** (Pi GPIO is
            3.3V, HC-SR04 ECHO is 5V — skipping the divider risks
            damaging the pin)

Before use on the Pi:
    pip install gpiozero
    # HC-SR04 needs no serial/I2C setup, just correct wiring + divider.

Pins are read from CAR_ULTRASONIC_FRONT_PINS / _LEFT_PINS / _RIGHT_PINS
env vars as "trigger,echo" (same pattern as CarController's
_pins_from_environment for the motors), or passed directly.

------------------------------------------------------------------
Honest limits of what this can measure — read before trusting output
------------------------------------------------------------------
* Usable range is roughly 2cm-400cm. Below ~2cm and above ~400cm the
  sensor/gpiozero reports max_distance (a "nothing in range" value),
  which looks identical to "clear path" — don't treat a max reading as
  a guaranteed-clear reading if the object could instead be very close.
* The ping is a ~15 degree cone, not a laser: an object off to the side
  of where you *think* the sensor points can still trigger a short
  reading, and soft/angled/absorptive surfaces (fabric, foam, glass at
  an angle) can scatter the echo and under-report or miss it entirely.
* Reading three sensors back-to-back can cross-talk (one sensor's echo
  picked up by another). This driver staggers reads with a small delay
  to reduce that risk; still worth treating suspiciously-identical
  readings across all three sensors with suspicion.
* Speed-of-sound is temperature-dependent; gpiozero's DistanceSensor
  assumes a fixed value, so accuracy drifts a little in extreme heat or
  cold. Fine for obstacle-avoidance thresholds, not for precision work.
"""

import os
import time
from threading import Lock, Thread

from gpiozero import DistanceSensor

from logger.logger_manager import LoggerManager

logger = LoggerManager.get_logger("car_controller.ultrasonicManager")


DEFAULT_MAX_DISTANCE_M = 4.0    # HC-SR04 practical ceiling (~400 cm)
DEFAULT_THRESHOLD_M = 0.30      # "obstacle ahead" distance used by mapping/avoidance
DEFAULT_POLL_HZ = 20            # reads/sec per sensor
_INTER_SENSOR_DELAY_S = 0.01    # stagger to reduce cross-talk between sensors


def _empty_reading():
    return {
        "front_m": None,
        "left_m": None,
        "right_m": None,
        "last_update": None,
    }


class UltrasonicArray:
    """Low-level access to the three HC-SR04 sensors via gpiozero."""

    def __init__(self, front_pins, left_pins, right_pins, max_distance_m=DEFAULT_MAX_DISTANCE_M):
        self.max_distance_m = max_distance_m
        self.front = DistanceSensor(trigger=front_pins[0], echo=front_pins[1], max_distance=max_distance_m)
        self.left = DistanceSensor(trigger=left_pins[0], echo=left_pins[1], max_distance=max_distance_m)
        self.right = DistanceSensor(trigger=right_pins[0], echo=right_pins[1], max_distance=max_distance_m)

    def read(self):
        """One staggered round of front/left/right reads, in meters.
        DistanceSensor.distance is already scaled to meters — no manual
        conversion needed."""
        reading = _empty_reading()
        reading["front_m"] = self.front.distance
        time.sleep(_INTER_SENSOR_DELAY_S)
        reading["left_m"] = self.left.distance
        time.sleep(_INTER_SENSOR_DELAY_S)
        reading["right_m"] = self.right.distance
        reading["last_update"] = time.time()
        return reading

    def close(self):
        for sensor in (self.front, self.left, self.right):
            sensor.close()


class UltrasonicTracker:
    """
    Background daemon thread that continuously polls the array and
    keeps a thread-safe snapshot of the latest reading. Mirrors
    GPSTracker / IMUTracker exactly, so every sensor manager in this
    project behaves the same way from a caller's point of view.
    """

    def __init__(self, array, poll_hz=DEFAULT_POLL_HZ):
        self.array = array
        self.dt = 1.0 / poll_hz
        self._lock = Lock()
        self._thread = None
        self._running = False
        self._reading = _empty_reading()

    @property
    def is_running(self):
        return self._running

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
        self._thread = None

    def _loop(self):
        while self._running:
            started = time.time()
            try:
                reading = self.array.read()
                with self._lock:
                    self._reading = reading
            except Exception:
                logger.exception("Ultrasonic read/update failed")
            elapsed = time.time() - started
            time.sleep(max(0.0, self.dt - elapsed))

    def snapshot(self):
        with self._lock:
            return dict(self._reading)


class UltrasonicManager:
    """
    High-level wrapper around UltrasonicArray + UltrasonicTracker,
    mirroring how GPSManager wraps NEOM8NGPS + GPSTracker and
    IMUManager wraps MPU6050 + IMUTracker. This is the class Car, Arm,
    and MappingManager should all depend on — never talk to gpiozero
    DistanceSensor objects directly.

    Usage:
        ultrasonic = UltrasonicManager()   # auto_start=True by default
        ... ultrasonic.get_telemetry() / ultrasonic.has_obstacle() ...
        ultrasonic.close()
    """

    def __init__(
        self,
        front_pins=None,
        left_pins=None,
        right_pins=None,
        max_distance_m=DEFAULT_MAX_DISTANCE_M,
        threshold_m=DEFAULT_THRESHOLD_M,
        poll_hz=DEFAULT_POLL_HZ,
        auto_start=True,
    ):
        front_pins = front_pins or self._pins_from_environment("CAR_ULTRASONIC_FRONT_PINS", (5, 6))
        left_pins = left_pins or self._pins_from_environment("CAR_ULTRASONIC_LEFT_PINS", (17, 27))
        right_pins = right_pins or self._pins_from_environment("CAR_ULTRASONIC_RIGHT_PINS", (22, 23))

        self.threshold_m = threshold_m

        self.array = UltrasonicArray(front_pins, left_pins, right_pins, max_distance_m=max_distance_m)
        self.tracker = UltrasonicTracker(self.array, poll_hz=poll_hz)

        if auto_start:
            self.start()

    @staticmethod
    def _pins_from_environment(name, default):
        value = os.getenv(name)
        if not value:
            return default
        try:
            pins = tuple(int(pin.strip()) for pin in value.split(","))
        except ValueError as error:
            raise ValueError(f"{name} must contain two GPIO numbers (trigger,echo)") from error
        if len(pins) != 2:
            raise ValueError(f"{name} must contain two GPIO numbers (trigger,echo)")
        return pins

    # ---- lifecycle -----------------------------------------------------

    def start(self):
        self.tracker.start()

    def stop(self):
        self.tracker.stop()

    def close(self):
        self.tracker.stop()
        try:
            self.array.close()
        except Exception:
            logger.exception("Error closing ultrasonic sensors")

    def wait_until_ready(self, timeout=5.0):
        """Block until the first reading has arrived, or raise TimeoutError.
        Useful in MappingManager's INITIALIZING state."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.tracker.snapshot()["last_update"] is not None:
                return
            time.sleep(0.02)
        raise TimeoutError(f"No ultrasonic reading after {timeout:.1f}s — check wiring")

    # ---- reads -----------------------------------------------------------

    def get_telemetry(self):
        data = self.tracker.snapshot()
        data["available"] = True
        data["threshold_m"] = self.threshold_m
        return data

    @property
    def front_distance_m(self):
        return self.tracker.snapshot()["front_m"]

    @property
    def left_distance_m(self):
        return self.tracker.snapshot()["left_m"]

    @property
    def right_distance_m(self):
        return self.tracker.snapshot()["right_m"]

    def has_obstacle(self, direction="front", threshold_m=None):
        """
        True if the named sensor ('front' | 'left' | 'right') is
        reading closer than threshold_m (defaults to self.threshold_m).

        Fails CLOSED: if no reading is available yet (tracker not
        started, or hasn't completed its first poll), this returns
        True. An obstacle check that silently reports "clear" when it
        actually has no data is more dangerous than one that is overly
        cautious for a fraction of a second at startup.
        """
        threshold = self.threshold_m if threshold_m is None else threshold_m
        reading = self.tracker.snapshot()
        key = f"{direction}_m"
        if key not in reading:
            raise ValueError("direction must be 'front', 'left', or 'right'")
        distance = reading[key]
        if distance is None:
            return True
        return distance < threshold
