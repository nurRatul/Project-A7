"""
test_run.py
-----------
Real-hardware lawnmower / boustrophedon coverage for the rover.

Uses the existing CarController (motors + IMU) and GPSManager exactly as
your other scripts do.  While the rover drives, live GPS fixes and IMU
yaw are pushed into the radar stack via HTTP (/api/vehicle/update) so
the /radar page shows the true path.

Distance along a leg is measured from GPS displacement (fallback: timed
drive if GPS has no fix yet).  90° turns use the IMU angle-based
turn_left / turn_right on CarController.

Usage (on the Pi, with hardware):
    # Terminal 1 — radar server (reuses sensors if you prefer; or --sim
    # and let this script own GPS/IMU exclusively)
    python radar_server.py --sim

    # Terminal 2 — this script owns the motors + GPS + IMU
    python test_run.py
    python test_run.py --length 15 --width 8 --car-width 1.2 --speed 0.25

Safety:
    Ctrl+C always stops the motors.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from threading import Event, Thread

# ---------------------------------------------------------------------------
# Project path — same pattern as your existing test.py
# ---------------------------------------------------------------------------
if __package__ in (None, ""):
    # Prefer repo root that contains the Car/ package
    here = Path(__file__).resolve().parent
    candidates = [here, here.parent, here.parents[1] if len(here.parents) > 1 else here]
    for root in candidates:
        if (root / "Car").is_dir():
            sys.path.insert(0, str(root))
            break
    else:
        # Still allow local radar package next to this file
        sys.path.insert(0, str(here))

from Car.carController import CarController
from Car.sensors.basic.gps.gpsManager import GPSManager

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
AREA_LENGTH_X = 10.0    # metres along X (first leg)
AREA_WIDTH_Y = 6.0      # metres along Y (coverage depth)
CAR_WIDTH = 1.2         # lateral step between swaths (m)
DRIVE_SPEED = 0.25      # motor speed 0..1 (keep low outdoors until tuned)
TURN_SPEED = 0.22
GPS_PORT = os.getenv("CAR_GPS_PORT", "/dev/serial1")
GPS_BAUD = int(os.getenv("CAR_GPS_BAUDRATE", "9600"))
SERVER_URL = os.getenv("RADAR_SERVER", "http://127.0.0.1:5000")
TELEMETRY_HZ = 10.0
GPS_WAIT_S = 60.0
# If GPS never gets a fix, fall back to open-loop timed legs.
# Rough calibration: seconds per metre at DRIVE_SPEED (tune on your rover).
SEC_PER_METRE = 2.5


# ---------------------------------------------------------------------------
# Radar HTTP helpers (optional — script still works if server is down)
# ---------------------------------------------------------------------------

def _http_json(method: str, url: str, payload: dict | None = None, timeout: float = 1.5):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json"} if data else {}
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8") or "{}")


def radar_available(base_url: str) -> bool:
    try:
        _http_json("GET", f"{base_url}/api/status")
        return True
    except Exception:
        return False


def post_pose(base_url: str, lat: float, lon: float, heading: float) -> None:
    try:
        _http_json(
            "POST",
            f"{base_url}/api/vehicle/update",
            {"latitude": lat, "longitude": lon, "heading": heading},
        )
    except Exception:
        pass


def post_reset(base_url: str) -> None:
    try:
        _http_json("POST", f"{base_url}/api/vehicle/reset", {})
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def normalize_heading(deg: float) -> float:
    h = deg % 360.0
    if h < 0:
        h += 360.0
    return h


# ---------------------------------------------------------------------------
# Live telemetry broadcaster (GPS + IMU → radar) while motors run
# ---------------------------------------------------------------------------

class TelemetryPublisher:
    """
    Background thread: sample GPS + IMU and push into the radar server.
    Shares the *same* GPSManager / CarController instances — no duplicates.
    """

    def __init__(self, car: CarController, gps: GPSManager, base_url: str, hz: float = 10.0):
        self.car = car
        self.gps = gps
        self.base_url = base_url.rstrip("/")
        self.dt = 1.0 / max(1.0, hz)
        self._stop = Event()
        self._thread: Thread | None = None
        self.enabled = False
        self.last_lat = None
        self.last_lon = None
        self.last_heading = 0.0

    def start(self) -> None:
        if not radar_available(self.base_url):
            print(f"[telemetry] Radar server not at {self.base_url} — continuing without map feed")
            self.enabled = False
            return
        self.enabled = True
        post_reset(self.base_url)
        self._stop.clear()
        self._thread = Thread(target=self._loop, name="TelemetryPublisher", daemon=True)
        self._thread.start()
        print(f"[telemetry] Publishing to {self.base_url}/radar at {1/self.dt:.0f} Hz")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.sample_once()
            time.sleep(max(0.0, self.dt - (time.monotonic() - t0)))

    def sample_once(self) -> None:
        loc = None
        try:
            loc = self.gps.get_location()
        except Exception:
            pass

        heading = self.last_heading
        try:
            orient = self.car.get_orientation()
            if orient and "yaw" in orient:
                # MPU yaw is relative; still useful for live rotation on the map
                heading = normalize_heading(orient["yaw"])
                self.last_heading = heading
        except Exception:
            pass

        if loc is not None:
            lat, lon = loc
            if lat is not None and lon is not None:
                self.last_lat, self.last_lon = lat, lon
                if self.enabled:
                    post_pose(self.base_url, lat, lon, heading)
        elif self.enabled and self.last_lat is not None:
            # Keep heading updates flowing even between GPS fixes
            post_pose(self.base_url, self.last_lat, self.last_lon, heading)


# ---------------------------------------------------------------------------
# Motion primitives (real motors)
# ---------------------------------------------------------------------------

def wait_for_gps_fix(gps: GPSManager, timeout: float = GPS_WAIT_S):
    print(f"Waiting for GPS fix (timeout {timeout:.0f}s)...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        loc = gps.get_location()
        if loc is not None:
            print(f"  GPS fix: lat={loc[0]:.7f}, lon={loc[1]:.7f}")
            return loc
        tel = gps.get_telemetry()
        sats = tel.get("satellites")
        print(f"  ... no fix yet (sats={sats}, has_fix={tel.get('has_fix')})")
        time.sleep(1.0)
    print("  WARNING: no GPS fix — will use timed open-loop legs")
    return None


def drive_distance_m(
    car: CarController,
    gps: GPSManager,
    distance_m: float,
    speed: float,
    sec_per_metre: float = SEC_PER_METRE,
) -> float:
    """
    Drive forward until GPS reports ~distance_m travelled, or timed fallback.
    Returns estimated metres driven.
    """
    start = gps.get_location()
    car.move_forward(speed=speed)  # continuous until stop()

    if start is None:
        # Open-loop timed drive
        duration = max(0.1, distance_m * sec_per_metre)
        print(f"  open-loop forward {distance_m:.2f} m ≈ {duration:.1f}s @ speed={speed}")
        time.sleep(duration)
        car.stop()
        return distance_m

    print(f"  GPS-guided forward {distance_m:.2f} m @ speed={speed}")
    t0 = time.monotonic()
    max_time = max(3.0, distance_m * sec_per_metre * 2.5)
    travelled = 0.0

    try:
        while True:
            loc = gps.get_location()
            if loc is not None:
                travelled = haversine_m(start[0], start[1], loc[0], loc[1])
                if travelled >= distance_m:
                    break
            if time.monotonic() - t0 > max_time:
                print(f"  timeout after {travelled:.2f} m — stopping leg")
                break
            time.sleep(0.05)
    finally:
        car.stop()

    print(f"  travelled ≈ {travelled:.2f} m")
    return travelled


def turn_degrees(car: CarController, direction: str, angle: float, speed: float) -> None:
    """
    direction: 'left' or 'right'
    Uses IMU angle-based turn when available; otherwise timed fallback.
    """
    angle = abs(float(angle))
    print(f"  turn {direction} {angle:.0f}° @ speed={speed}")

    orient = car.get_orientation()
    if orient is None or "yaw" not in orient:
        # Timed fallback (~1.2 s per 90° at speed 0.22 — tune as needed)
        duration = (angle / 90.0) * 1.2
        if direction == "left":
            car.turn_left(deltaT=duration, speed=speed)
        else:
            car.turn_right(deltaT=duration, speed=speed)
        return

    if direction == "left":
        car.turn_left(speed=speed, angle=angle)
    else:
        car.turn_right(speed=speed, angle=angle)


# ---------------------------------------------------------------------------
# Coverage pattern (real hardware)
# ---------------------------------------------------------------------------

def run_coverage(
    length_x: float,
    width_y: float,
    car_width: float,
    drive_speed: float,
    turn_speed: float,
    server_url: str,
    gps_port: str,
    gps_baud: int,
    sec_per_metre: float,
) -> None:
    print("=== Real rover coverage run ===")
    print(f"  area: {length_x} m × {width_y} m, swath={car_width} m")
    print(f"  drive_speed={drive_speed}, turn_speed={turn_speed}")
    print(f"  GPS: {gps_port} @ {gps_baud}")
    print(f"  radar: {server_url}")
    print()

    car = CarController()
    gps = GPSManager(port=gps_port, baudrate=gps_baud, timeout=1.0)
    gps.start()

    pub = TelemetryPublisher(car, gps, server_url, hz=TELEMETRY_HZ)

    try:
        wait_for_gps_fix(gps, timeout=GPS_WAIT_S)
        pub.start()
        # Give publisher one sample at origin
        time.sleep(0.3)
        pub.sample_once()

        y_covered = 0.0
        going_forward = True  # alternate leg direction
        swath = 0

        # Initial heading assumed "along +X"; we don't need absolute compass —
        # relative IMU turns are enough for the lawnmower sequence.
        while y_covered < width_y - 1e-3:
            # --- long leg ---
            print(f"\n[swath {swath}] long leg ({'outbound' if going_forward else 'return'})")
            drive_distance_m(car, gps, length_x, drive_speed, sec_per_metre)

            remaining = width_y - y_covered
            if remaining < car_width * 0.4:
                break

            step = min(car_width, remaining)

            # Lawnmower turn sequence:
            # After outbound (forward): turn right 90 → step → turn right 90
            # After return:             turn left  90 → step → turn left  90
            if going_forward:
                turn_degrees(car, "right", 90, turn_speed)
                print(f"[swath {swath}] lateral step {step:.2f} m")
                drive_distance_m(car, gps, step, drive_speed, sec_per_metre)
                turn_degrees(car, "right", 90, turn_speed)
            else:
                turn_degrees(car, "left", 90, turn_speed)
                print(f"[swath {swath}] lateral step {step:.2f} m")
                drive_distance_m(car, gps, step, drive_speed, sec_per_metre)
                turn_degrees(car, "left", 90, turn_speed)

            y_covered += step
            going_forward = not going_forward
            swath += 1
            print(f"  y_covered ≈ {y_covered:.2f} / {width_y:.2f} m")

        print("\n=== Coverage finished — stopping ===")

    except KeyboardInterrupt:
        print("\nInterrupted — emergency stop")
    finally:
        try:
            car.stop()
        except Exception:
            pass
        pub.stop()
        try:
            car.shutdown()
        except Exception:
            pass
        try:
            gps.close()
        except Exception:
            pass
        print("Motors stopped, GPS closed.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Real rover lawnmower coverage + radar feed")
    parser.add_argument("--length", type=float, default=AREA_LENGTH_X, help="Long leg length (m)")
    parser.add_argument("--width", type=float, default=AREA_WIDTH_Y, help="Coverage depth (m)")
    parser.add_argument("--car-width", type=float, default=CAR_WIDTH, help="Swath / lateral step (m)")
    parser.add_argument("--speed", type=float, default=DRIVE_SPEED, help="Drive speed 0..1")
    parser.add_argument("--turn-speed", type=float, default=TURN_SPEED, help="Turn speed 0..1")
    parser.add_argument("--server", type=str, default=SERVER_URL, help="Radar server base URL")
    parser.add_argument("--gps-port", type=str, default=GPS_PORT)
    parser.add_argument("--gps-baud", type=int, default=GPS_BAUD)
    parser.add_argument(
        "--sec-per-metre",
        type=float,
        default=SEC_PER_METRE,
        help="Open-loop timing fallback (seconds per metre)",
    )
    args = parser.parse_args()

    if not 0.0 < args.speed <= 1.0:
        parser.error("--speed must be in (0, 1]")
    if not 0.0 < args.turn_speed <= 1.0:
        parser.error("--turn-speed must be in (0, 1]")

    run_coverage(
        length_x=args.length,
        width_y=args.width,
        car_width=args.car_width,
        drive_speed=args.speed,
        turn_speed=args.turn_speed,
        server_url=args.server.rstrip("/"),
        gps_port=args.gps_port,
        gps_baud=args.gps_baud,
        sec_per_metre=args.sec_per_metre,
    )


if __name__ == "__main__":
    main()
