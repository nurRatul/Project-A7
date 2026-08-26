import atexit
import os
import sys
from threading import Lock

from flask import Flask, jsonify, render_template_string, request
from werkzeug.exceptions import HTTPException

from logger.logger_manager import LoggerManager

LoggerManager.initialize(print_on_console=False)

logger = LoggerManager.get_logger("serverController")

# carController.py does "from .btsMotor import BTSMotor" — a relative import
# that only works if it's loaded as part of the "Car" package. So instead of
# falling back to a bare "import carController" (which would break that
# relative import), we make sure the directory *containing* Car/ is on
# sys.path and always import it as "Car.carController". This works whether
# app.py lives next to Car/ or inside Car/ itself.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = _THIS_DIR if os.path.isdir(os.path.join(_THIS_DIR, "Car")) else os.path.dirname(_THIS_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from Car.carController import CarController


# ---------------------------------------------------------------------------
# Logging setup — writes to loggs/car_controller.log (created if missing)
# ---------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "loggs")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, "car_controller.log")




app = Flask(__name__)
car_lock = Lock()

try:
    car = CarController()
except Exception:
    logger.exception("Failed to initialize CarController — check hardware/wiring")
    raise


CONTROL_PAGE = """<!doctype html>
<html lang="en">
<head>
	<meta charset="utf-8">
	<meta name="viewport" content="width=device-width, initial-scale=1">
	<title>Car Controller</title>
	<style>
		:root { color-scheme: dark; font-family: system-ui, sans-serif; }
		body { display: grid; min-height: 100vh; place-items: center; margin: 0; background: #101820; color: #f4f7f9; }
		main { width: min(92vw, 420px); text-align: center; }
		h1 { margin: 0 0 1.5rem; font-size: 1.8rem; }
		.speed { display: flex; align-items: center; gap: .75rem; margin-bottom: 1.5rem; }
		input { flex: 1; accent-color: #55c2a3; }
		output { min-width: 3rem; text-align: right; }
		.controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: .75rem; }
		button { min-height: 72px; border: 0; border-radius: 8px; background: #263746; color: inherit; font-size: 1.8rem; touch-action: manipulation; }
		button:active, button.stop { background: #d95d39; }
		button.stop { grid-column: 2; font-size: 1rem; font-weight: 700; }
		.joysticks { display: flex; justify-content: space-between; gap: 1rem; margin: 1.5rem 0; }
		.joystick { width: min(38vw, 160px); aspect-ratio: 1; position: relative; border: 2px solid #3d5660; border-radius: 50%; background: #17232a; touch-action: none; }
		.joystick::after { content: ''; position: absolute; inset: 18%; border: 1px dashed #55717b; border-radius: 50%; }
		.knob { width: 32%; aspect-ratio: 1; position: absolute; left: 34%; top: 34%; z-index: 1; border-radius: 50%; background: #55c2a3; box-shadow: 0 0 18px #55c2a388; }
		.status { min-height: 1.5rem; margin-top: 1.25rem; color: #9bb3c2; }
		@media (max-width: 420px) { .joysticks { gap: .5rem; } }
	</style>
</head>
<body>
	<main>
		<h1>Car Controller</h1>
		<label class="speed" for="speed">Speed
			<input id="speed" type="range" min="0" max="1" step="0.05" value="0.6">
			<output id="speedValue">0.60</output>
		</label>
		<section class="controls" aria-label="Driving controls">
			<span></span><button data-command="forward" aria-label="Forward">&#9650;</button><span></span>
			<button data-command="left" aria-label="Turn left">&#9664;</button>
			<button class="stop" data-command="stop">STOP</button>
			<button data-command="right" aria-label="Turn right">&#9654;</button>
			<span></span><button data-command="backward" aria-label="Backward">&#9660;</button><span></span>
		</section>
		<section class="joysticks" aria-label="Twin joysticks">
			<div class="joystick" data-stick="left" aria-label="Left joystick"><div class="knob"></div></div>
			<div class="joystick" data-stick="right" aria-label="Right joystick"><div class="knob"></div></div>
		</section>
		<div class="status" id="status" role="status">Ready</div>
	</main>
	<script>
		const speed = document.querySelector('#speed');
		const speedValue = document.querySelector('#speedValue');
		const status = document.querySelector('#status');
		speed.addEventListener('input', () => speedValue.value = Number(speed.value).toFixed(2));

		async function send(command) {
			try {
				const response = await fetch('/api/move', {
					method: 'POST', headers: {'Content-Type': 'application/json'},
					body: JSON.stringify({command, speed: Number(speed.value)})
				});
				const result = await response.json();
				status.textContent = result.message || result.error;
			} catch (err) {
				status.textContent = 'Connection error';
			}
		}
		async function sendDrive(left, right) {
			try {
				await fetch('/api/drive', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({left, right, speed: Number(speed.value)})});
			} catch (err) {
				status.textContent = 'Connection error';
			}
		}
		function stopDrive() { send('stop').catch(() => {}); }
		const driveValues = {left: 0, right: 0};

		document.querySelectorAll('[data-command]').forEach((button) => {
			const command = button.dataset.command;
			button.addEventListener('pointerdown', (event) => { event.preventDefault(); send(command); });
			if (command !== 'stop') {
				button.addEventListener('pointerup', stopDrive);
				button.addEventListener('pointerleave', stopDrive);
			}
		});
		document.querySelectorAll('.joystick').forEach((stick) => {
			const knob = stick.querySelector('.knob');
			let pointerId = null;
			function update(event) {
				const box = stick.getBoundingClientRect();
				const radius = box.width * .34;
				const x = Math.max(-radius, Math.min(radius, event.clientX - (box.left + box.width / 2)));
				const y = Math.max(-radius, Math.min(radius, event.clientY - (box.top + box.height / 2)));
				knob.style.transform = `translate(${x}px, ${y}px)`;
				const side = stick.dataset.stick;
				driveValues[side] = Math.max(-1, Math.min(1, -y / radius));
				sendDrive(driveValues.left, driveValues.right);
			}
			stick.addEventListener('pointerdown', (event) => { pointerId = event.pointerId; stick.setPointerCapture(pointerId); update(event); });
			stick.addEventListener('pointermove', (event) => { if (event.pointerId === pointerId) update(event); });
			stick.addEventListener('pointerup', () => { pointerId = null; knob.style.transform = ''; driveValues[stick.dataset.stick] = 0; sendDrive(driveValues.left, driveValues.right); });
			stick.addEventListener('pointercancel', () => { pointerId = null; knob.style.transform = ''; driveValues[stick.dataset.stick] = 0; sendDrive(driveValues.left, driveValues.right); });
		});
		window.addEventListener('blur', stopDrive);
		document.addEventListener('keydown', (event) => {
			const keys = {ArrowUp: 'forward', ArrowDown: 'backward', ArrowLeft: 'left', ArrowRight: 'right'};
			if (keys[event.key] && !event.repeat) { event.preventDefault(); send(keys[event.key]); }
		});
		document.addEventListener('keyup', (event) => { if (event.key.startsWith('Arrow')) stopDrive(); });
	</script>
</body>
</html>"""


COMMANDS = {
    "forward": "move_forward",
    "backward": "move_backward",
    "left": "turn_left",
    "right": "turn_right",
    "stop": "stop",
}


def clamp(value):
    return max(-1.0, min(1.0, float(value)))


@app.get("/")
def index():
    return render_template_string(CONTROL_PAGE)


@app.post("/api/move")
def move():
    payload = request.get_json(silent=True) or {}
    print(payload)
    command = payload.get("command")
    if command not in COMMANDS:
        logger.error("Rejected /api/move: invalid command=%r", command)
        return jsonify(error="command must be forward, backward, left, right, or stop"), 400

    try:
        speed = float(payload.get("speed", 0.6))
    except (TypeError, ValueError):
        logger.error("Rejected /api/move: invalid speed=%r", payload.get("speed"))
        return jsonify(error="speed must be a number from 0 to 1"), 400
    if not 0 <= speed <= 1:
        logger.error("Rejected /api/move: speed out of range=%s", speed)
        return jsonify(error="speed must be a number from 0 to 1"), 400

    try:
        with car_lock:
            if command == "stop":
                car.stop()
            else:
                getattr(car, COMMANDS[command])(speed=speed)
    except Exception:
        logger.exception("Car command failed: command=%s speed=%.2f", command, speed)
        return jsonify(error="failed to execute command on the car"), 500

    message = f"{command} at {speed:.2f}"
    logger.info("/api/move -> %s", message)
    return jsonify(status="ok", command=command, message=message)


@app.get("/api/status")
def status():
    return jsonify(status="ok")


@app.post("/api/drive")
def drive():
    payload = request.get_json(silent=True) or {}
    try:
        speed = float(payload.get("speed", 0.6))
        left = clamp(payload.get("left", 0)) * speed
        right = clamp(payload.get("right", 0)) * speed
    except (TypeError, ValueError):
        logger.error("Rejected /api/drive: bad payload=%r", payload)
        return jsonify(error="left, right, and speed must be numbers"), 400
    if not 0 <= speed <= 1:
        logger.error("Rejected /api/drive: speed out of range=%s", speed)
        return jsonify(error="speed must be a number from 0 to 1"), 400

    try:
        with car_lock:
            car.drive(left, right)
    except Exception:
        logger.exception("Car drive failed: left=%.2f right=%.2f", left, right)
        return jsonify(error="failed to execute drive on the car"), 500

    message = f"drive left={left:.2f} right={right:.2f}"
    logger.info("/api/drive -> %s", message)
    return jsonify(status="ok", left=left, right=right)


@app.errorhandler(Exception)
def handle_unexpected_error(error):
    if isinstance(error, HTTPException):
        return error
    logger.exception("Unhandled exception on %s %s", request.method, request.path)
    return jsonify(error="internal server error"), 500


@atexit.register
def stop_car():
    logger.info("Shutting down — stopping car")
    try:
        car.stop()
    except Exception:
        logger.exception("Error while stopping car during shutdown")


if __name__ == "__main__":
    host = os.getenv("CAR_HOST", "0.0.0.0")
    port = int(os.getenv("CAR_PORT", "5000"))
    logger.info("Starting car controller server on %s:%s", host, port)
    try:
        app.run(host=host, port=port, debug=False, threaded=True)
    except Exception:
        logger.exception("Server crashed")
        sys.exit(1)