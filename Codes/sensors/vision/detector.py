"""
detector.py

Camera + YOLO detection, split the same way neom8n.py splits GPS:

    Detector        - low-level, blocking, no thread. read_frame() /
                      detect() / draw() do exactly one thing each time
                      you call them.
    DetectorTracker - background daemon thread that keeps grabbing
                      frames and running detect() at a fixed rate,
                      exposing a thread-safe snapshot() of the latest
                      result. Mirrors GPSTracker / IMUTracker.

Wrap both with DetectorManager (see detectorManager.py) the same way
GPSManager wraps NEOM8NGPS + GPSTracker.
"""

import sys
import time
from pathlib import Path
from threading import Lock, Thread

import cv2
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO

sys.path.append(str(Path(__file__).resolve().parent))
import Arm.config as cfg

from logger.logger_manager import LoggerManager

logger = LoggerManager.get_logger("car_controller.detector")


class Detector:
    """
    Low-level camera + model access. Every method blocks and does one
    thing — no background thread here, that's DetectorTracker's job.
    Use this directly (via DetectorManager.detect_once()) if you just
    want a single on-demand detection instead of a continuous feed.
    """

    def __init__(self, model_path=cfg.MODEL_PATH):
        self.model = YOLO(model_path)

        self.camera = Picamera2()

        camera_config = self.camera.create_video_configuration(
            main={
                "format": "BGR888",
                "size": (cfg.IMAGE_WIDTH, cfg.IMAGE_HEIGHT),
            },
            buffer_count=2,  # minimizes DMA RAM consumption
        )
        self.camera.configure(camera_config)
        self.camera.start()

        # Continuous autofocus
        self.camera.set_controls({"AfMode": 2})
        full_size = self.camera.camera_properties["PixelArraySize"]

        self.camera.set_controls({
            # Continuous autofocus
            "AfMode": controls.AfModeEnum.Continuous,
            "AfTrigger": controls.AfTriggerEnum.Start,

            # Maximum field of view (no digital zoom)
            "ScalerCrop": (
                0,
                0,
                full_size[0],
                full_size[1],
            ),
        })

    def read_frame(self):
        """Blocks until the next raw BGR frame is available."""
        return self.camera.capture_array()

    def detect(self, frame):
        """
        Runs YOLO on `frame`. Always returns a dict — an empty-scene
        frame is not an error, it's just an empty detections list:

            {
                "frame": frame,
                "detections": [ {...}, ... ],   # [] if nothing found
                "has_detection": bool,
                "timestamp": time.time(),
            }

        Each entry in "detections":
            class_id, class_name, confidence, bbox [x1,y1,x2,y2],
            center [cx, cy]
        """
        results = self.model(
            frame,
            conf=cfg.YOLO_CONFIDENCE,
            verbose=False,
        )

        detections = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                detections.append({
                    "class_id": class_id,
                    "class_name": result.names[class_id],
                    "confidence": confidence,
                    "bbox": [float(x1), float(y1), float(x2), float(y2)],
                    "center": [float(cx), float(cy)],
                })

        return {
            "frame": frame,
            "detections": detections,
            "has_detection": bool(detections),
            "timestamp": time.time(),
        }

    def draw(self, frame, result=None):
        """
        Draws boxes/labels for whatever is in result["detections"].
        Pass the dict from detect() (or None, or one with an empty
        list) — with nothing to draw, the frame comes back untouched.
        """
        detections = (result or {}).get("detections", [])

        if not detections:
            return frame

        for detection in detections:
            x1, y1, x2, y2 = map(int, detection["bbox"])
            cx, cy = map(int, detection["center"])
            confidence = detection["confidence"]
            class_name = detection["class_name"]

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            cv2.putText(
                frame,
                f"{class_name} {confidence:.2f}",
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                frame,
                f"({cx},{cy})",
                (x1, y2 + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        return frame

    def release(self):
        self.camera.stop()
        self.camera.close()


class DetectorTracker:
    """
    Background daemon thread that continuously grabs a frame and runs
    detect() at a fixed rate, keeping a thread-safe snapshot of the
    latest result. Mirrors GPSTracker / IMUTracker in this codebase.

    Change the rate any time with set_rate_hz() — takes effect on the
    next loop iteration, no restart needed.
    """

    def __init__(self, detector, rate_hz=20.0):
        self.detector = detector
        self._rate_hz = max(0.1, rate_hz)

        self._lock = Lock()
        self._thread = None
        self._running = False

        self._result = {
            "frame": None,
            "detections": [],
            "has_detection": False,
            "timestamp": None,
        }

    @property
    def is_running(self):
        return self._running

    def set_rate_hz(self, rate_hz):
        with self._lock:
            self._rate_hz = max(0.1, rate_hz)

    def get_rate_hz(self):
        with self._lock:
            return self._rate_hz

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._thread = None

    def _loop(self):
        while self._running:
            started = time.time()

            try:
                self._update()
            except Exception:
                logger.exception("Detector read/update failed")

            period = 1.0 / self.get_rate_hz()
            elapsed = time.time() - started
            time.sleep(max(0.0, period - elapsed))

    def _update(self):
        frame = self.detector.read_frame()
        result = self.detector.detect(frame)

        with self._lock:
            self._result = result

    def snapshot(self):
        """Thread-safe copy of the latest {frame, detections, has_detection, timestamp}."""
        with self._lock:
            return dict(self._result)
