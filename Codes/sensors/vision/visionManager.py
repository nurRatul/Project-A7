# visionManager.py

"""
visionManager.py

Independent YOLO + camera sensor, generalized from Arm/detector.py's
BottleDetector so it can be shared by CarController, MappingManager,
and (optionally) ArmController instead of each owning its own camera
and model. Same background-thread + snapshot() shape as every other
*Manager in sensors/.

Relationship to Arm/detector.py — READ THIS FIRST:
    Arm/detector.py's BottleDetector is untouched and keeps working
    exactly as it does today ("don't touch the arm code"). This is a
    NEW, separate class, not a replacement. It generalizes the same
    detect-and-annotate pattern to configurable target classes (not
    just bottles).

    The one real risk: if VisionManager AND Arm's own BottleDetector
    both try to open the same camera_id at once, they will conflict —
    only one process/object can usually hold a given /dev/videoN
    handle. Controller.__init__ already accepts an optional
    `detector=` parameter for exactly this situation. Use the
    VisionManagerDetectorAdapter below to make ArmController share
    this VisionManager's camera instead of opening a second one —
    zero changes to controller.py or detector.py required:

        vision = VisionManager(target_classes={"bottle": 39})
        arm = Controller(detector=VisionManagerDetectorAdapter(vision))
        mapper = MappingManager(..., vision=vision, arm=arm, ...)

------------------------------------------------------------------
Honest limits of what this can tell you — read before trusting output
------------------------------------------------------------------
* YOLO gives a 2D pixel box, not distance. Depth has to come from
  somewhere else (ultrasonic, stereo, a depth camera, or a fixed
  assumed distance like Arm/controller.py's temporary self.distance_mm)
  — detections alone don't produce trustworthy XYZ.
* Detection quality depends on lighting, motion blur, and how well
  your target classes match what the model was actually trained on —
  "confidence" is a model score, not a calibrated probability.
* Stock COCO-trained YOLO (yolo26n.pt) only has a real class for
  "bottle" among this project's example object types (bottle, box,
  rock, trash — see MappingManager's object_policy). "box", "rock",
  and "trash" are NOT COCO classes. Detecting them for real needs a
  custom-trained/fine-tuned model (swap model_path + target_classes)
  or a fallback heuristic elsewhere (e.g. treat any ultrasonic-flagged
  obstacle vision can't classify as a generic "unknown object"). Don't
  assume you're detecting box/rock/trash until you've verified against
  the real model's class list.
"""

import time
from threading import Lock, Thread

import cv2
from ultralytics import YOLO

from logger.logger_manager import LoggerManager

logger = LoggerManager.get_logger("car_controller.visionManager")


MODEL_PATH = "yolo26n.pt"
CAMERA_ID = 0
IMAGE_WIDTH = 1080
IMAGE_HEIGHT = 720
CONFIDENCE = 0.50
DEFAULT_POLL_HZ = 10

# COCO class ids VisionManager reports on out of the box. Extend or
# replace via VisionManager(target_classes=...) — see the module
# docstring above on why "box"/"rock"/"trash" aren't included.
DEFAULT_TARGET_CLASSES = {
    "bottle": 39,
}


def _empty_snapshot():
    return {
        "detections": [],
        "frame_shape": None,
        "last_update": None,
    }


class VisionDetector:
    """Low-level camera + YOLO access — a generalized BottleDetector."""

    def __init__(self, model_path=MODEL_PATH, camera_id=CAMERA_ID,
                 target_classes=None, confidence=CONFIDENCE,
                 image_width=IMAGE_WIDTH, image_height=IMAGE_HEIGHT):

        self.target_classes = dict(target_classes or DEFAULT_TARGET_CLASSES)
        self.confidence = confidence
        self._class_id_to_name = {v: k for k, v in self.target_classes.items()}

        self.model = YOLO(model_path)
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)

        if not self.camera.isOpened():
            raise RuntimeError("Could not open camera.")

    def read_frame(self):
        success, frame = self.camera.read()
        if not success:
            return None
        return frame

    def detect(self, frame):
        results = self.model(frame, conf=self.confidence, verbose=False)
        detections = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                class_id = int(box.cls[0])
                if class_id not in self._class_id_to_name:
                    continue

                confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

                detections.append({
                    "class_id": class_id,
                    "class_name": self._class_id_to_name[class_id],
                    "confidence": confidence,
                    "bbox": [float(x1), float(y1), float(x2), float(y2)],
                    "center": [float(cx), float(cy)],
                })

        return detections

    def draw(self, frame, detections):
        for detection in detections:
            x1, y1, x2, y2 = map(int, detection["bbox"])
            cx, cy = map(int, detection["center"])

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f'{detection["class_name"]} {detection["confidence"]:.2f}',
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )

        return frame

    def release(self):
        self.camera.release()


class VisionTracker:
    """Background daemon thread — matches IMUTracker/GPSTracker/UltrasonicTracker."""

    def __init__(self, detector, poll_hz=DEFAULT_POLL_HZ):
        self.detector = detector
        self.dt = 1.0 / poll_hz
        self._lock = Lock()
        self._thread = None
        self._running = False
        self._snapshot = _empty_snapshot()
        self._latest_frame = None

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
            self._thread.join(timeout=2.0)
        self._thread = None

    def _loop(self):
        while self._running:
            started = time.time()
            try:
                self._update()
            except Exception:
                logger.exception("Vision read/update failed")
            elapsed = time.time() - started
            time.sleep(max(0.0, self.dt - elapsed))

    def _update(self):
        frame = self.detector.read_frame()
        if frame is None:
            return
        detections = self.detector.detect(frame)
        with self._lock:
            self._latest_frame = frame
            self._snapshot = {
                "detections": detections,
                "frame_shape": frame.shape,
                "last_update": time.time(),
            }

    def snapshot(self):
        with self._lock:
            return dict(self._snapshot)

    def latest_frame(self):
        with self._lock:
            return self._latest_frame


class VisionManager:
    """
    High-level wrapper around VisionDetector + VisionTracker, mirroring
    GPSManager/IMUManager/UltrasonicManager. CarController,
    MappingManager, and (via the adapter below) ArmController should
    all depend on this instead of touching cv2/YOLO/ultralytics directly.
    """

    def __init__(self, model_path=MODEL_PATH, camera_id=CAMERA_ID,
                 target_classes=None, confidence=CONFIDENCE,
                 poll_hz=DEFAULT_POLL_HZ, auto_start=True, detector=None):

        self.detector = detector or VisionDetector(
            model_path=model_path,
            camera_id=camera_id,
            target_classes=target_classes,
            confidence=confidence,
        )
        self.tracker = VisionTracker(self.detector, poll_hz=poll_hz)

        if auto_start:
            self.start()

    def start(self):
        self.tracker.start()

    def stop(self):
        self.tracker.stop()

    def close(self):
        self.tracker.stop()
        try:
            self.detector.release()
        except Exception:
            logger.exception("Error releasing camera")

    def get_telemetry(self):
        data = self.tracker.snapshot()
        data["available"] = True
        return data

    def detect(self, frame=None):
        """Detections from the background tracker's latest frame, or a
        fresh synchronous detect() against `frame` if one is supplied
        (e.g. a frame ArmController already grabbed this cycle)."""
        if frame is not None:
            return self.detector.detect(frame)
        return self.tracker.snapshot()["detections"]

    def read_frame(self):
        return self.detector.read_frame()

    def latest_frame(self):
        return self.tracker.latest_frame()

    def get_detections_by_class(self, class_name):
        return [d for d in self.detect() if d["class_name"] == class_name]

    def wait_until_ready(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.tracker.snapshot()["last_update"] is not None:
                return
            time.sleep(0.05)
        raise TimeoutError(f"No camera frame after {timeout:.1f}s — check camera_id/connection")


class VisionManagerDetectorAdapter:
    """
    Wraps a VisionManager to match the shape ArmController's
    Controller(detector=...) already expects from Arm/detector.py's
    BottleDetector (read_frame / detect / draw / release) — WITHOUT
    modifying controller.py or detector.py at all.

    Why this exists: Controller opens its own camera via BottleDetector
    unless you inject a `detector`. If VisionManager ALSO opens
    camera_id=0, both objects fight over the same device. Constructing
    ArmController as:

        Controller(detector=VisionManagerDetectorAdapter(vision))

    makes the arm use the exact same shared VisionManager/camera that
    MappingManager and CarController use, so there is only ever one
    camera handle open system-wide.
    """

    def __init__(self, vision_manager):
        self._vision = vision_manager

    def read_frame(self):
        return self._vision.read_frame()

    def detect(self, frame):
        return self._vision.detect(frame)

    def draw(self, frame, detections):
        return self._vision.detector.draw(frame, detections)

    def release(self):
        # Intentionally a no-op: VisionManager is shared and owned by
        # whoever constructed it (MappingManager's caller), not by the
        # Controller instance that borrowed it — Controller.close()
        # calling this must not kill the camera out from under
        # MappingManager/CarController.
        pass
