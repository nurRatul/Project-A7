# gpsManager.py

from math import atan2, radians, sqrt, sin, cos
import os
import time

from .neom8n import NEOM8NGPS, GPSTracker
from logger.logger_manager import LoggerManager

logger = LoggerManager.get_logger("car_controller.gpsManager")


class GPSManager:
    """
    High-level wrapper around NEOM8NGPS + GPSTracker, mirroring how
    IMUManager wraps MPU6050 + IMUTracker.

    Two independent ways to get data — use either, or both:

    1) Daemon-thread mode (live telemetry for a control loop):
           gps = GPSManager()
           gps.start()
           ... gps.get_telemetry() / gps.get_location() as often as you like ...
           gps.close()

    2) One-shot mode (no background thread at all):
           gps = GPSManager()
           fix = gps.read_once()      # blocks until a real fix comes in
           gps.close()

    Unlike IMUManager, this does NOT start the background thread
    automatically on construction — pass auto_start=True if you want that.
    Reading a serial port from two places at once corrupts data, so don't
    call read_once() while the daemon thread (start()) is running; use
    get_telemetry() instead, or stop() first.
    """

    def __init__(self, port=None, baudrate=None, timeout=1.0, auto_start=False):
        port = port or os.getenv("CAR_GPS_PORT", "/dev/serial0")
        baudrate = int(baudrate or os.getenv("CAR_GPS_BAUDRATE", "9600"))

        self.gps = NEOM8NGPS(port=port, baudrate=baudrate, timeout=timeout)
        self.tracker = GPSTracker(self.gps)
        self.old_location = {'latitude': None,'longitude': None }

        if auto_start:
            self.start()

    # ---- lifecycle -------------------------------------------------

    def start(self):
        """Begin polling the GPS in a background daemon thread."""
        self.tracker.start()
        time.sleep(0.5)  # let the first fix come in
        location = self.get_location()
        print(f"GPSManager started, first fix: {location}")
        self.old_location['latitude'] = location[0]
        self.old_location['longitude'] = location[1]
   
    def reset(self):
        """Reset the GPS tracker and clear the last known location."""
        self.tracker.reset()
        location = self.get_location()
        self.old_location['latitude'] = location[0]
        self.old_location['longitude'] = location[1]

    def stop(self):
        """Stop the background thread. The serial port stays open — start() again anytime."""
        self.tracker.stop()

    def close(self):
        """Stop the background thread (if running) and close the serial port for good."""
        self.tracker.stop()
        try:
            self.gps.close()
        except Exception:
            logger.exception("Error closing GPS serial port")

    # ---- one-shot, no daemon required -------------------------------

    def read_once(self, timeout=30.0, raw=False):
        """
        Block until one real position fix comes in, straight off the
        serial port — no background thread needed, safe to call whether
        or not start() has ever been used. Do NOT call this while the
        daemon thread is running (see class docstring).
        """
        if self.tracker.is_running:
            raise RuntimeError(
                "GPS background thread is running — reading the serial "
                "port from two places at once will corrupt data. Use "
                "get_telemetry() instead, or call stop() first."
            )
        return self.gps.read(timeout=timeout, raw=raw)

    # ---- daemon-thread telemetry -------------------------------------

    def get_telemetry(self, raw=False):
        """Latest snapshot from the background thread. Call start() first."""
        return self.tracker.snapshot(raw=raw)

    def get_location(self):
        """(latitude, longitude) tuple, or None if there's no fix yet."""
        fix = self.tracker.snapshot()
        if fix["has_fix"] and fix["latitude"] is not None:
            return (fix["latitude"], fix["longitude"])
        return None

    def get_speed_kmh(self):
        return self.tracker.snapshot()["speed_kmh"]

    def get_altitude_m(self):
        return self.tracker.snapshot()["altitude_m"]

    def get_satellites(self):
        return self.tracker.snapshot()["satellites"]

    def has_fix(self):
        return self.tracker.snapshot()["has_fix"]

    def distance_meters(self):
        R = 6371000  # Earth radius in meters
        lat1, lon1 = self.old_location['latitude'], self.old_location['longitude']
        lat2, lon2 = self.get_location()

        if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
            return None

        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)

        a = (
            sin(dlat / 2) ** 2
            + cos(radians(lat1))
            * cos(radians(lat2))
            * sin(dlon / 2) ** 2
        )

        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c


if __name__ == "__main__":
    # Quick wiring/bring-up check: run this file directly on the Pi
    #   python3 -m Car.sensors.basic.gps.gpsManager
    # (adjust the module path to wherever you place it)
    gps = GPSManager()
    try:
        print("Waiting for first GPS fix (Ctrl+C to stop early)...")
        first_fix = gps.read_once(timeout=60.0)
        print("First fix:", first_fix)

        print("\nSwitching to daemon mode for 10s of live telemetry...")
        gps.start()
        for _ in range(10):
            time.sleep(1)
            print(gps.get_telemetry())
    except TimeoutError as error:
        print(error)
    except KeyboardInterrupt:
        pass
    finally:
        gps.close()