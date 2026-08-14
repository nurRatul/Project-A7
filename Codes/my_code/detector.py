import cv2
from ultralytics import YOLO


# ============================================================
# SETTINGS
# ============================================================

MODEL_PATH = "yolo26n.pt"

CAMERA_ID = 0

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

CONFIDENCE = 0.50

# True  -> show camera window
# False -> run without displaying video
SHOW_VIDEO = True

WINDOW_NAME = "YOLO26n Detection"


# COCO class ID:
# bottle = 39
BOTTLE_CLASS_ID = 39


# ============================================================
# YOLO DETECTOR
# ============================================================

class BottleDetector:

    def __init__(self):

        print("Loading YOLO26n...")

        self.model = YOLO(MODEL_PATH)

        print("YOLO26n loaded.")


    def detect(self, frame):

        """
        Detect bottles in one frame.

        Returns a list:

        [
            {
                "class_id": 39,
                "class_name": "bottle",
                "confidence": 0.92,
                "x1": ...,
                "y1": ...,
                "x2": ...,
                "y2": ...,
                "cx": ...,
                "cy": ...
            }
        ]
        """

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


                # Only bottle
                if class_id != BOTTLE_CLASS_ID:
                    continue


                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                )


                x1 = float(x1)
                y1 = float(y1)
                x2 = float(x2)
                y2 = float(y2)


                # Center of bounding box

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2


                detection = {

                    "class_id": class_id,

                    "class_name":
                        result.names[class_id],

                    "confidence":
                        confidence,

                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,

                    "cx": cx,
                    "cy": cy
                }


                detections.append(detection)


        return detections


# ============================================================
# CAMERA
# ============================================================

def open_camera():

    cap = cv2.VideoCapture(CAMERA_ID)


    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        IMAGE_WIDTH
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        IMAGE_HEIGHT
    )


    if not cap.isOpened():

        raise RuntimeError(
            "Could not open camera."
        )


    return cap


# ============================================================
# DRAW DETECTIONS
# ============================================================

def draw_detections(frame, detections):

    for detection in detections:

        x1 = int(detection["x1"])
        y1 = int(detection["y1"])

        x2 = int(detection["x2"])
        y2 = int(detection["y2"])

        cx = int(detection["cx"])
        cy = int(detection["cy"])

        confidence = detection["confidence"]


        # Bounding box

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )


        # Center point

        cv2.circle(
            frame,
            (cx, cy),
            5,
            (0, 0, 255),
            -1
        )


        # Label

        label = (
            f"bottle "
            f"{confidence:.2f}"
        )


        cv2.putText(
            frame,
            label,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )


        # Center coordinates

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


# ============================================================
# TEST DETECTOR DIRECTLY
# ============================================================

if __name__ == "__main__":

    detector = BottleDetector()

    cap = open_camera()


    print()
    print("==============================")
    print("YOLO26n BOTTLE DETECTOR")
    print("==============================")
    print("Press Q or ESC to quit.")
    print()


    while True:

        ret, frame = cap.read()


        if not ret:

            print("Failed to read camera.")

            break


        detections = detector.detect(frame)


        # ----------------------------------------------------
        # Print detection information
        # ----------------------------------------------------

        if detections:

            for detection in detections:

                print(
                    f"Bottle | "
                    f"confidence="
                    f"{detection['confidence']:.2f} | "
                    f"center="
                    f"({detection['cx']:.0f},"
                    f"{detection['cy']:.0f})"
                )


        # ----------------------------------------------------
        # Video
        # ----------------------------------------------------

        if SHOW_VIDEO:

            display_frame = draw_detections(
                frame,
                detections
            )


            cv2.imshow(
                WINDOW_NAME,
                display_frame
            )


            # IMPORTANT:
            # waitKey must be called when using imshow.

            key = cv2.waitKey(1) & 0xFF


            if key == ord("q") or key == 27:

                print("Stopping...")

                break


    cap.release()

    cv2.destroyAllWindows()