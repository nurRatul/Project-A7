"""
radar_server.py
---------------
Flask + SocketIO server that hosts the existing car-control page and the
new live radar map at /radar.

Integration rules
-----------------
* Reuses a single CarController (and therefore a single IMU) when hardware
  is present.
* Optionally starts a GPSManager (one instance only).
* Never creates a second sensor instance.
* All radar state lives in the VehicleState singleton.
* Existing /api/move and /api/drive routes are preserved unchanged.

Run (simulation / development, no hardware required):
    python radar_server.py --sim

Run on the Pi with real sensors:
    python radar_server.py
"""

from __future__ import annotations

import argparse
import atexit
import os
import sys
import time
from threading import Event, Lock, Thread

from flask import Flask, jsonify, render_template, request, send_from_directory
from flask_socketio import SocketIO, emit

# ---------------------------------------------------------------------------
# Make sure the project root (parent of Car/) and this artifacts dir are
# importable whether we are on the Pi or in the sandbox.
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = (
    _THIS_DIR
    if os.path.isdir(os.path.join(_THIS_DIR, "Car"))
    else os.path.dirname(_THIS_DIR)
)
for p in (_PROJECT_ROOT, _THIS_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

from radar.vehicle_state import get_vehicle_state
from radar.radar_bridge import RadarBridge

# ---------------------------------------------------------------------------
# Optional hardware imports — graceful degradation when not on the Pi
# ---------------------------------------------------------------------------
CarController = None
GPSManager = None
_hardware_available = False

try:
    from Car.carController import CarController as _CC
    from sensors.basic.gps.gpsManager import GPSManager as _GM

    CarController = _CC
    GPSManager = _GM
    _hardware_available = True
except ImportError:
    pass

# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------
app = Flask(
    __name__,
    template_folder=os.path.join(_THIS_DIR, "templates"),
    static_folder=os.path.join(_THIS_DIR, "static"),
)
app.config["SECRET_KEY"] = os.getenv("CAR_SECRET", "radar-dev-key")

socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

vehicle_state = get_vehicle_state(scale_px_per_m=8.0)
car_lock = Lock()
car = None
gps = None
bridge: RadarBridge | None = None

# Broadcast loop
_broadcast_stop = Event()
_broadcast_thread: Thread | None = None
BROADCAST_HZ = 15.0


# ---------------------------------------------------------------------------
# Hardware / simulation bootstrap
# ---------------------------------------------------------------------------

def _init_hardware(use_sim: bool = False) -> None:
    global car, gps, bridge

    if use_sim or not _hardware_available:
        print("[radar] Running in simulation mode (no hardware)")
        bridge = RadarBridge(gps=None, imu=None, vehicle_state=vehicle_state)
        # Bridge is not auto-started; test_run.py or external code injects data
        vehicle_state.set_meta(mode="sim", message="Simulation mode — inject via test_run.py")
        return

    try:
        car = CarController()
        print("[radar] CarController ready (IMU attached)")
    except Exception as exc:
        print(f"[radar] CarController failed: {exc}")
        car = None

    try:
        gps = GPSManager(port=os.getenv("CAR_GPS_PORT", "/dev/serial0"), auto_start=True)
        print("[radar] GPSManager started")
    except Exception as exc:
        print(f"[radar] GPSManager failed: {exc}")
        gps = None

    imu = car.imu if car is not None else None
    bridge = RadarBridge(
        gps=gps,
        imu=imu,
        vehicle_state=vehicle_state,
        poll_hz=10.0,
        # Adjust these once on the real vehicle if needed:
        heading_offset_deg=float(os.getenv("CAR_HEADING_OFFSET", "0")),
        invert_yaw=os.getenv("CAR_INVERT_YAW", "0") == "1",
    )
    bridge.start()
    print("[radar] RadarBridge started")


def _start_broadcast() -> None:
    global _broadcast_thread
    _broadcast_stop.clear()
    _broadcast_thread = Thread(target=_broadcast_loop, name="RadarBroadcast", daemon=True)
    _broadcast_thread.start()


def _broadcast_loop() -> None:
    interval = 1.0 / BROADCAST_HZ
    while not _broadcast_stop.is_set():
        started = time.monotonic()
        try:
            snap = vehicle_state.snapshot()
            socketio.emit("vehicle_update", snap)
        except Exception:
            pass
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, interval - elapsed))


def _shutdown() -> None:
    _broadcast_stop.set()
    if bridge is not None:
        bridge.stop()
    if gps is not None:
        try:
            gps.close()
        except Exception:
            pass
    if car is not None:
        try:
            car.shutdown()
        except Exception:
            pass


atexit.register(_shutdown)

# ---------------------------------------------------------------------------
# Routes — existing control page is left intact if templates exist;
# we serve a minimal control stub + the new radar page.
# ---------------------------------------------------------------------------

CONTROL_PAGE_FALLBACK = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Car Controller</title>
<style>body{background:#101820;color:#f4f7f9;font-family:system-ui;display:grid;place-items:center;min-height:100vh;margin:0}
a{color:#55c2a3}</style></head>
<body><main style="text-align:center">
<h1>Car Controller</h1>
<p>Radar map: <a href="/radar">/radar</a></p>
<p>API status: <a href="/api/status">/api/status</a></p>
</main></body></html>"""


@app.get("/")
def index():
    return CONTROL_PAGE_FALLBACK


@app.get("/radar")
def radar_page():
    return render_template("radar.html")


@app.get("/api/status")
def status():
    snap = vehicle_state.snapshot()
    return jsonify(
        status="ok",
        has_fix=snap["has_fix"],
        mode=snap["meta"].get("mode"),
        path_count=snap["path_count"],
        hardware=_hardware_available and car is not None,
    )


@app.get("/api/vehicle")
def api_vehicle():
    """REST fallback for clients that cannot use WebSocket."""
    return jsonify(vehicle_state.snapshot())


@app.post("/api/vehicle/update")
def api_vehicle_update():
    """
    External modules (or test_run.py over HTTP) can push pose here.
    Body: {"latitude": ..., "longitude": ..., "heading": ...}

    Uses the same VehicleState methods that RadarBridge / real sensors call.
    """
    payload = request.get_json(silent=True) or {}
    try:
        has_pos = "latitude" in payload and "longitude" in payload
        has_hdg = "heading" in payload
        if has_pos and has_hdg:
            vehicle_state.update_pose(
                float(payload["latitude"]),
                float(payload["longitude"]),
                float(payload["heading"]),
            )
        elif has_pos:
            vehicle_state.update_position(
                float(payload["latitude"]), float(payload["longitude"])
            )
        elif has_hdg:
            vehicle_state.update_heading(float(payload["heading"]))
        return jsonify(status="ok", pose=vehicle_state.get_local_pose())
    except (TypeError, ValueError) as exc:
        return jsonify(error=str(exc)), 400


@app.post("/api/vehicle/reset")
def api_vehicle_reset():
    vehicle_state.clear_path()
    vehicle_state.clear_obstacles()
    vehicle_state.transformer.clear_origin()
    vehicle_state.set_meta(message="Path and origin cleared")
    return jsonify(status="ok")


# ---- Optional: keep original move/drive if hardware present -------------

@app.post("/api/move")
def move():
    if car is None:
        return jsonify(error="No hardware car available"), 503
    payload = request.get_json(silent=True) or {}
    command = payload.get("command")
    try:
        speed = float(payload.get("speed", 0.6))
    except (TypeError, ValueError):
        return jsonify(error="invalid speed"), 400
    try:
        with car_lock:
            if command == "stop":
                car.stop()
            elif command == "forward":
                car.move_forward(speed=speed)
            elif command == "backward":
                car.move_backward(speed=speed)
            elif command == "left":
                car.turn_left(speed=speed)
            elif command == "right":
                car.turn_right(speed=speed)
            else:
                return jsonify(error="unknown command"), 400
    except Exception:
        return jsonify(error="command failed"), 500
    return jsonify(status="ok", command=command)


@app.post("/api/drive")
def drive():
    if car is None:
        return jsonify(error="No hardware car available"), 503
    payload = request.get_json(silent=True) or {}
    try:
        speed = float(payload.get("speed", 0.6))
        left = max(-1.0, min(1.0, float(payload.get("left", 0)))) * speed
        right = max(-1.0, min(1.0, float(payload.get("right", 0)))) * speed
    except (TypeError, ValueError):
        return jsonify(error="invalid payload"), 400
    try:
        with car_lock:
            car.drive(left, right)
    except Exception:
        return jsonify(error="drive failed"), 500
    return jsonify(status="ok", left=left, right=right)


# ---------------------------------------------------------------------------
# SocketIO events
# ---------------------------------------------------------------------------

@socketio.on("connect")
def on_connect():
    emit("vehicle_update", vehicle_state.snapshot())


@socketio.on("request_snapshot")
def on_request_snapshot():
    emit("vehicle_update", vehicle_state.snapshot())


@socketio.on("reset_path")
def on_reset_path():
    vehicle_state.clear_path()
    vehicle_state.clear_obstacles()
    emit("vehicle_update", vehicle_state.snapshot())


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Autonomous car radar server")
    parser.add_argument("--sim", action="store_true", help="Force simulation mode")
    parser.add_argument("--host", default=os.getenv("CAR_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.getenv("CAR_PORT", "5000")))
    args = parser.parse_args()

    _init_hardware(use_sim=args.sim)
    _start_broadcast()

    print(f"[radar] Serving on http://{args.host}:{args.port}/radar")
    socketio.run(app, host=args.host, port=args.port, debug=False, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
