import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "controller"))

import cv2
import keyboard
from detection import (
    FieldTracker,
    YoloObjectDetector,
    warp_field,
    image_to_robot_xy,
    draw_field_boundary,
    draw_detection,
    draw_crosshair,
)
import braccio_control_python


def choose_camera():
    choice = input("Camera: type a number (0, 1, ...) for webcam, paste a URL (rtsp/http), or a path to an image file: ").strip()
    if choice.isdigit():
        cap = cv2.VideoCapture(int(choice))
        label = "webcam " + choice
        is_image = False
    elif os.path.isfile(choice):
        cap = cv2.VideoCapture(choice)
        label = "image " + choice
        is_image = True
    else:
        cap = cv2.VideoCapture(choice)
        label = choice
        is_image = False
    if not cap.isOpened():
        print("Could not open camera:", label)
        cap.release()
        return None, None, False
    print("Opened camera:", label)
    return cap, label, is_image


def main():
    cap, label, is_image = choose_camera()
    if cap is None:
        return

    field_tracker = FieldTracker()
    object_detector = YoloObjectDetector(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolov8n.pt"),
        confidence=0.35,
    )

    current_grab_point = [[0, 0]]
    best_detection = None
    using_field_warp = False
    work_size = (0, 0)
    frame_count = 0

    print("Tracking field markers 1-4 (4x4 dict) to define the working area.")
    print("Object detection: YOLOv8n (COCO classes).")
    print("Press 'p' to pick up the best detected object, 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            if is_image:
                cap.release()
                cap = cv2.VideoCapture(label)
                continue
            print("Lost camera feed")
            break

        display = frame.copy()

        field_tracker.update(frame)
        if field_tracker.is_ready():
            corners = field_tracker.get_corners()
            draw_field_boundary(display, corners)
            working_image, _ = warp_field(frame, corners)
            using_field_warp = True
        else:
            cv2.putText(display, "NO FIELD MARKERS - using full frame", (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            working_image = frame
            using_field_warp = False

        if frame_count % 3 == 0:
            best_detection = object_detector.pick_best(working_image)
        if best_detection is not None:
            current_grab_point[0] = best_detection["footprint"]
            draw_detection(working_image, best_detection)
            draw_crosshair(working_image, best_detection["center"])

        cv2.imshow("detection_view", working_image)

        if keyboard.is_pressed("p"):
            if best_detection is None:
                print("No object detected yet!")
            elif not using_field_warp:
                print("FIELD MARKERS NOT FOUND - arm disabled. Place markers 1-4 forming an exact 600x300 mm rectangle.")
            else:
                work_size = (working_image.shape[1], working_image.shape[0])
                x_mm, y_mm = image_to_robot_xy(current_grab_point[0], work_size, (600, 300))
                print("Detected:", best_detection["name"], best_detection["confidence"])
                print("Camera position:", x_mm, ",", y_mm)
                try:
                    x_comp, y_comp = braccio_control_python.camera_compensation(x_mm, y_mm)
                    print("Compensated (info only):", x_comp, ",", y_comp)
                except ZeroDivisionError:
                    print("Compensation skipped (zero coordinate)")
                braccio_control_python.pick_up(x_mm, y_mm)
                print("Object placed!")

        frame_count += 1

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    braccio_control_python.home()
    main()
