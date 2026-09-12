# detectorManager.py

from .detector import Detector, DetectorTracker
from logger.logger_manager import LoggerManager

logger = LoggerManager.get_logger("car_controller.detectorManager")


class DetectorManager:
    """
    High-level wrapper around Detector + DetectorTracker, mirroring how
    GPSManager wraps NEOM8NGPS + GPSTracker and IMUManager wraps
    MPU6050 + IMUTracker.

    Two independent ways to get data — use either, or both:

    1) Daemon-thread mode (continuous feed for a control loop):
           det = DetectorManager(rate_hz=20)
           det.start()
           ... det.get_snapshot() / det.get_frame() as often as you like ...
           det.close()

    2) One-shot mode (no background thread at all):
           det = DetectorManager()
           result = det.detect_once()      # blocks for one frame + inference
           det.close()

    Reading the camera from two places at once wastes a capture, so
    don't call detect_once() while the daemon thread (start()) is
    running; use get_snapshot() instead, or stop() first.
    """

    def __init__(self, model_path=None, rate_hz=20.0, auto_start=False):
        self.detector = Detector(model_path=model_path) if model_path else Detector()
        self.tracker = DetectorTracker(self.detector, rate_hz=rate_hz)

        if auto_start:
            self.start()

    # ---- lifecycle -------------------------------------------------

    def start(self):
        """Begin continuous capture + detection in a background daemon thread."""
        self.tracker.start()

    def stop(self):
        """Stop the background thread. The camera stays open — start() again anytime."""
        self.tracker.stop()

    def close(self):
        """Stop the background thread (if running) and release the camera for good."""
        self.tracker.stop()
        try:
            self.detector.release()
        except Exception:
            logger.exception("Error releasing camera")

    def set_rate_hz(self, rate_hz):
        """Change the polling rate on the fly, e.g. 5Hz while idle, 20Hz while closing in."""
        self.tracker.set_rate_hz(rate_hz)

    def get_rate_hz(self):
        return self.tracker.get_rate_hz()

    # ---- one-shot, no daemon required -------------------------------

    def detect_once(self):
        """
        Block for exactly one frame + one detection pass, straight off
        the camera — no background thread needed. Do NOT call this
        while the daemon thread is running (see class docstring).
        """
        if self.tracker.is_running:
            raise RuntimeError(
                "Detector background thread is running — reading the "
                "camera from two places at once wastes frames. Use "
                "get_snapshot() instead, or call stop() first."
            )
        frame = self.detector.read_frame()
        return self.detector.detect(frame)

    # ---- daemon-thread telemetry -------------------------------------

    def get_snapshot(self):
        """Latest {frame, detections, has_detection, timestamp}. Call start() first."""
        return self.tracker.snapshot()

    def get_frame(self):
        """Just the latest raw frame, or None if nothing captured yet."""
        return self.tracker.snapshot()["frame"]

    def get_detections(self):
        """Just the latest detections list (empty list if none)."""
        return self.tracker.snapshot()["detections"]

    def has_detection(self):
        return self.tracker.snapshot()["has_detection"]

    def draw(self, frame, result=None):
        """
        Convenience passthrough — draws detections onto `frame`.
        Accepts either the manager's {frame, detections, ...} dict, a
        plain detections list (old-Detector style), or nothing (uses the
        latest snapshot).
        """
        if result is None:
            result = self.get_snapshot()
        elif isinstance(result, list):
            result = {"detections": result}
        return self.detector.draw(frame, result)

    # ---- compatibility shim for old-style consumers -------------------
    #
    # Code written against the original bare Detector (read_frame() ->
    # detect(frame) -> list -> draw(frame, list) -> release()) can take a
    # DetectorManager instance instead with no changes, e.g. ArmController
    # can be handed the same manager instance Rover already started.
    #
    # IMPORTANT: when the daemon thread is running (the normal case, e.g.
    # a shared instance from Rover), read_frame()/detect() here do NOT
    # trigger a fresh camera capture — they hand back whatever the
    # background thread most recently captured/detected. Two consumers
    # calling the real camera at once would fight over frames the same
    # way two callers of GPSManager.read_once() would fight over the
    # serial port; reading the shared snapshot instead avoids that, at
    # the cost of the data being up to one tracker cycle (1/rate_hz) old.
    # If the thread isn't running, this falls back to a real one-shot
    # capture via detect_once(), so a standalone (non-shared) instance
    # still behaves exactly like the old Detector.

    def read_frame(self):
        """Old-Detector-compatible: latest frame, from the shared snapshot if the thread is running."""
        if self.tracker.is_running:
            return self.get_frame()
        return self.detector.read_frame()

    def detect(self, frame=None):
        """
        Old-Detector-compatible: returns a plain detections list (not the
        {frame, detections, ...} dict). `frame` is accepted for interface
        compatibility but ignored while the daemon thread is running,
        since detection has already been run on the shared snapshot.
        """
        if self.tracker.is_running:
            return self.get_detections()
        if frame is None:
            frame = self.detector.read_frame()
        return self.detector.detect(frame)["detections"]

    def release(self):
        """Old-Detector-compatible alias for close(). Prefer close() in new code."""
        self.close()
