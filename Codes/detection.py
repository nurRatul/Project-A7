import cv2
import numpy as np


class FieldTracker:
    FIELD_IDS = [1, 2, 3, 4]

    def __init__(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.corners = None
        self.seen = False

    def update(self, frame):
        marker_corners, marker_ids, _ = self.detector.detectMarkers(frame)
        if marker_ids is None:
            return self.seen
        ordered = [None, None, None, None]
        for corner, marker_id in zip(marker_corners, marker_ids):
            index = int(marker_id[0]) - 1
            if index in range(4):
                ordered[index] = corner[0][0]
        if all(corner is not None for corner in ordered):
            self.corners = np.array(
                [[int(corner[0]), int(corner[1])] for corner in ordered], dtype=np.float32
            )
            self.seen = True
        return self.seen

    def is_ready(self):
        return self.seen and self.corners is not None

    def get_corners(self):
        return self.corners


def warp_field(frame, corners, output_size=480):
    source = np.array(
        [corners[0], corners[1], corners[2], corners[3]], dtype=np.float32
    )
    target = np.array(
        [[0, 0], [output_size, 0], [output_size, output_size], [0, output_size]],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(source, target)
    warped = cv2.warpPerspective(frame, homography, (output_size, output_size))
    return warped, homography


class YoloObjectDetector:
    def __init__(self, model_path, confidence=0.35, allowed_classes=None):
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.confidence = confidence
        self.allowed_classes = allowed_classes

    def detect(self, image):
        detections = []
        result = self.model(image, verbose=False, conf=self.confidence)[0]
        for box in result.boxes:
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0]]
            detections.append(
                {
                    "name": result.names[int(box.cls[0])],
                    "confidence": float(box.conf[0]),
                    "box": (x1, y1, x2, y2),
                    "center": ((x1 + x2) // 2, (y1 + y2) // 2),
                    "footprint": ((x1 + x2) // 2, y2),
                }
            )
        return detections

    def pick_best(self, image):
        detections = self.detect(image)
        if self.allowed_classes:
            detections = [
                detection
                for detection in detections
                if detection["name"] in self.allowed_classes
            ]
        if not detections:
            return None
        return max(detections, key=lambda detection: detection["confidence"])


def image_to_robot_xy(center, field_size_px, field_span_mm):
    width_mm, height_mm = field_span_mm
    width_px, height_px = field_size_px
    x_mm = int(center[1] * width_mm / height_px - width_mm / 2)
    y_mm = int(center[0] * height_mm / width_px)
    return x_mm, y_mm


def draw_field_boundary(frame, corners, color=(0, 255, 0), thickness=2):
    points = np.array([corner for corner in corners], dtype=np.int32)
    cv2.polylines(frame, [points], isClosed=True, color=color, thickness=thickness)


def draw_detection(frame, detection, color=(0, 255, 0)):
    x1, y1, x2, y2 = detection["box"]
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    label = f"{detection['name']} {detection['confidence']:.2f}"
    cv2.putText(frame, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.circle(frame, detection["center"], 6, (0, 0, 255), -1)
    cv2.circle(frame, detection["footprint"], 6, (255, 0, 0), -1)


def draw_crosshair(frame, center, color=(0, 0, 255)):
    height, width = frame.shape[:2]
    cv2.line(frame, (center[0], 0), (center[0], height), color, 2)
    cv2.line(frame, (0, center[1]), (width, center[1]), color, 2)
