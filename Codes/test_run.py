"""
test_run.py
-----------
Simulates autonomous lawnmower / boustrophedon coverage of a rectangular
area and feeds GPS + heading into the *same* interfaces that real sensors
use.

Because the Flask server runs in another process, pose updates are sent
via HTTP POST to /api/vehicle/update — identical payload shape to what
RadarBridge produces from real GPSManager + IMUManager.

Usage:
    # Terminal 1
    python radar_server.py --sim

    # Terminal 2
    python test_run.py
    python test_run.py --length 20 --width 12 --car-width 1.5 --speed 1.0
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import urllib.error
import urllib.request
import json
from pathlib import Path

# ---------------------------------------------------------------------------
# Coverage parameters (overridable via CLI)
# ---------------------------------------------------------------------------
AREA_LENGTH_X = 20.0   # metres (East-West)
AREA_WIDTH_Y  = 12.0   # metres (North-South)
CAR_WIDTH     = 1.5    # metres (lateral swath step)
SPEED_MPS     = 0.8    # simulated ground speed
TURN_RATE_DPS = 45.0   # simulated yaw rate while turning
UPDATE_HZ    = 15.0   # pose injection rate

SERVER_URL = "http://127.0.0.1:5000"

# Origin GPS (arbitrary fixed point; only relative motion matters)
ORIGIN_LAT = 23.8103
ORIGIN_LON = 90.4125

_M_PER_DEG_LAT = 111_320.0
_M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(ORIGIN_LAT))


def local_to_gps(x_m: float, y_m: float) -> tuple[float, float]:
    lat = ORIGIN_LAT + y_m / _M_PER_DEG_LAT
    lon = ORIGIN_LON + x_m / _M_PER_DEG_LON
    return lat, lon


def post_pose(latitude: float, longitude: float, heading: float, base_url: str) -> bool:
    """
    Push one pose update through the same HTTP interface external modules
    (and RadarBridge) use.  Returns False on connection failure.
    """
    payload = {
        "latitude": latitude,
        "longitude": longitude,
        "heading": heading,
    }
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{base_url}/api/vehicle/update",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            return 200 <= resp.status < 300
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        print(f"[test_run] server unreachable: {exc}", file=sys.stderr)
        return False


def post_reset(base_url: str) -> None:
    req = urllib.request.Request(
        f"{base_url}/api/vehicle/reset",
        data=b"{}",
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        urllib.request.urlopen(req, timeout=2.0)
    except Exception:
        pass


def move_straight(
    x0: float,
    y0: float,
    heading_deg: float,
    distance_m: float,
    speed: float,
    update_hz: float,
    base_url: str,
) -> tuple[float, float]:
    """Drive straight. heading: 0=North, 90=East. Returns final (x, y)."""
    rad = math.radians(heading_deg)
    dx = math.sin(rad)   # East component
    dy = math.cos(rad)   # North component

    duration = distance_m / max(speed, 1e-6)
    steps = max(1, int(duration * update_hz))
    dt = duration / steps

    x, y = x0, y0
    for i in range(1, steps + 1):
        x = x0 + dx * (distance_m * i / steps)
        y = y0 + dy * (distance_m * i / steps)
        lat, lon = local_to_gps(x, y)
        if not post_pose(lat, lon, heading_deg, base_url):
            raise SystemExit("Lost connection to radar server")
        time.sleep(dt)
    return x, y


def turn_to(
    x: float,
    y: float,
    current_hdg: float,
    target_hdg: float,
    turn_rate: float,
    update_hz: float,
    base_url: str,
) -> float:
    """Rotate in place (shortest direction). Returns final heading."""
    delta = (target_hdg - current_hdg + 180.0) % 360.0 - 180.0
    if abs(delta) < 0.5:
        lat, lon = local_to_gps(x, y)
        post_pose(lat, lon, target_hdg, base_url)
        return target_hdg

    duration = abs(delta) / max(turn_rate, 1e-6)
    steps = max(1, int(duration * update_hz))
    dt = duration / steps

    for i in range(1, steps + 1):
        h = current_hdg + delta * (i / steps)
        lat, lon = local_to_gps(x, y)
        if not post_pose(lat, lon, h, base_url):
            raise SystemExit("Lost connection to radar server")
        time.sleep(dt)

    lat, lon = local_to_gps(x, y)
    post_pose(lat, lon, target_hdg, base_url)
    return target_hdg


def run_coverage(
    length_x: float = AREA_LENGTH_X,
    width_y: float = AREA_WIDTH_Y,
    car_width: float = CAR_WIDTH,
    speed: float = SPEED_MPS,
    turn_rate: float = TURN_RATE_DPS,
    update_hz: float = UPDATE_HZ,
    base_url: str = SERVER_URL,
) -> None:
    print(f"Waiting for radar server at {base_url} ...")
    for _ in range(30):
        try:
            urllib.request.urlopen(f"{base_url}/api/status", timeout=1.0)
            break
        except Exception:
            time.sleep(0.5)
    else:
        print("ERROR: radar server not reachable. Start it with:")
        print("  python radar_server.py --sim")
        sys.exit(1)

    post_reset(base_url)

    # Seed at origin, facing East
    post_pose(ORIGIN_LAT, ORIGIN_LON, 90.0, base_url)

    x, y = 0.0, 0.0
    heading = 90.0
    going_east = True
    swath = 0

    print(f"Coverage area: {length_x} m × {width_y} m, swath={car_width} m")
    print(f"Origin GPS: {ORIGIN_LAT}, {ORIGIN_LON}")
    print("Open http://localhost:5000/radar to watch\n")

    while y < width_y - 1e-6:
        # Drive along X
        target_hdg = 90.0 if going_east else 270.0
        heading = turn_to(x, y, heading, target_hdg, turn_rate, update_hz, base_url)
        x, y = move_straight(x, y, heading, length_x, speed, update_hz, base_url)

        remaining = width_y - y
        if remaining < car_width * 0.5:
            break

        step = min(car_width, remaining)
        # Step North
        heading = turn_to(x, y, heading, 0.0, turn_rate, update_hz, base_url)
        x, y = move_straight(x, y, heading, step, speed, update_hz, base_url)

        going_east = not going_east
        swath += 1
        print(f"  swath {swath}: now at ({x:.2f}, {y:.2f})")

    print("\nCoverage finished.")
    print(f"Final pose: x={x:.2f} m, y={y:.2f} m, heading={heading:.1f}°")


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulated lawnmower coverage for radar map")
    parser.add_argument("--length", type=float, default=AREA_LENGTH_X)
    parser.add_argument("--width", type=float, default=AREA_WIDTH_Y)
    parser.add_argument("--car-width", type=float, default=CAR_WIDTH)
    parser.add_argument("--speed", type=float, default=SPEED_MPS)
    parser.add_argument("--turn-rate", type=float, default=TURN_RATE_DPS)
    parser.add_argument("--server", type=str, default=SERVER_URL, help="Radar server base URL")
    args = parser.parse_args()

    try:
        run_coverage(
            length_x=args.length,
            width_y=args.width,
            car_width=args.car_width,
            speed=args.speed,
            turn_rate=args.turn_rate,
            base_url=args.server.rstrip("/"),
        )
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
