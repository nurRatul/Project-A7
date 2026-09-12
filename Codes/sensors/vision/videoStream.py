# videoStream.py
#
# Flask routes for a shared DetectorManager: an MJPEG live feed and a
# JSON endpoint for the latest detections. Register these against
# whatever Flask app + detector instance you're already running
# (see the integration notes below for serverController.py).

import cv2
from flask import Response, jsonify

from logger.logger_manager import LoggerManager

logger = LoggerManager.get_logger("car_controller.videoStream")


def _mjpeg_generator(detector, jpeg_quality=80):
    """
    Endless generator of multipart JPEG frames for an <img> tag / MJPEG
    stream. Reads whatever the detector's background thread most
    recently captured -- does NOT trigger a new camera read, so this can
    run alongside the arm, mapping, etc. reading the same detector
    without fighting over the camera.
    """
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

    while True:
        result = detector.get_snapshot()
        frame = result.get("frame")

        if frame is None:
            continue  # nothing captured yet -- skip, don't crash the stream

        frame = detector.draw(frame, result)

        ok, jpg = cv2.imencode(".jpg", frame, encode_params)
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
        )


def build_video_app(detector, url_prefix=""):
    """
    Builds a standalone Flask app with just the video routes, for when
    you don't want to touch your existing car-control Flask app at all.
    """
    from flask import Flask
    app = Flask(__name__)
    register_video_routes(app, detector, url_prefix=url_prefix)
    return app


def run_video_server_in_thread(detector, host="0.0.0.0", port=5001, url_prefix=""):
    """
    Builds the video-only Flask app and runs it in a background daemon
    thread -- call this once from Rover.__init__ (after the detector is
    already started) and it just runs alongside everything else:

        self.init_detector(rate_hz=20)                    # from DetectorMixin
        self.video_thread = run_video_server_in_thread(self.detector)

    Returns the Thread so you can .join() it if you ever need to, though
    being a daemon thread it dies automatically when the main program
    exits -- no explicit shutdown needed.

    use_reloader is forced off: Flask's reloader tries to fork/re-exec
    the process, which only makes sense when Flask itself owns main() --
    it will fight with a thread it didn't start.
    """
    from threading import Thread

    app = build_video_app(detector, url_prefix=url_prefix)

    thread = Thread(
        target=app.run,
        kwargs={"host": host, "port": port, "debug": False, "threaded": True, "use_reloader": False},
        daemon=True,
    )
    thread.start()

    logger.info("Video server running at http://%s:%s%s/video_feed", host, port, url_prefix)
    return thread


def register_video_routes(app, detector, url_prefix=""):
    """
    Call once, after both `app` and `detector` exist:

        register_video_routes(app, detector)

    Adds:
        GET {url_prefix}/video_feed      -- MJPEG stream for <img src="...">
        GET {url_prefix}/api/detections  -- latest detections as JSON, no frame
    """

    @app.get(f"{url_prefix}/video_feed")
    def video_feed():
        return Response(
            _mjpeg_generator(detector),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get(f"{url_prefix}/api/detections")
    def api_detections():
        result = detector.get_snapshot()
        return jsonify(
            has_detection=result["has_detection"],
            detections=[
                {
                    "class_name": d["class_name"],
                    "confidence": d["confidence"],
                    "center": d["center"],
                }
                for d in result["detections"]
            ],
            timestamp=result["timestamp"],
        )

    logger.info(
        "Video routes registered at %s/video_feed and %s/api/detections",
        url_prefix, url_prefix,
    )
