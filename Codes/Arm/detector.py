import cv2
from ultralytics import YOLO


MODEL_PATH = "yolo26n.pt"

CAMERA_ID = 0

IMAGE_WIDTH = 1080 # 640
IMAGE_HEIGHT = 720 # 480

CONFIDENCE = 0.50

BOTTLE_CLASS_ID = 39


class BottleDetector:

    def __init__(
        self,
        model_path=MODEL_PATH,
        camera_id=CAMERA_ID
    ):

        self.model = YOLO(model_path)

        self.camera = cv2.VideoCapture(camera_id)

        self.camera.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            IMAGE_WIDTH
        )

        self.camera.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            IMAGE_HEIGHT
        )

        if not self.camera.isOpened():
            raise RuntimeError(
                "Could not open camera."
            )


    def read_frame(self):

        success, frame = self.camera.read()

        if not success:
            return None

        return frame


    def detect(self, frame):

        results = self.model(
            frame,
            conf=CONFIDENCE,
            verbose=False
        )

        detections = []

        for result in results:

            if result.boxes is None:
                continue

            for box in result.boxes:

                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                if class_id != BOTTLE_CLASS_ID:
                    continue

                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                )

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                detections.append({
                    "class_id": class_id,
                    "class_name": result.names[class_id],
                    "confidence": confidence,

                    "bbox": [
                        float(x1),
                        float(y1),
                        float(x2),
                        float(y2)
                    ],

                    "center": [
                        float(cx),
                        float(cy)
                    ]
                })

        return detections


    def draw(self, frame, detections):

        for detection in detections:

            x1, y1, x2, y2 = map(
                int,
                detection["bbox"]
            )

            cx, cy = map(
                int,
                detection["center"]
            )

            confidence = detection["confidence"]

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

            cv2.circle(
                frame,
                (cx, cy),
                5,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                frame,
                f"bottle {confidence:.2f}",
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"({cx},{cy})",
                (x1, y2 + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

        return frame


    def release(self):

        self.camera.release()