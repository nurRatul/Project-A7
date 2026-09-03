import sys
from pathlib import Path
import cv2

# Ensure local imports inside Arm/ resolve properly
sys.path.append(str(Path(__file__).resolve().parent))
from Arm.detector  import BottleDetector


def main():
    print("Initializing BottleDetector and Pi Camera...")
    detector = BottleDetector()

    print("Camera feed active. Press 'q' inside the video window to exit.")

    try:
        while True:
            frame = detector.read_frame()
            rgb_frame= cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            if frame is None:
                print("Failed to capture frame from camera.")
                continue

            # Run YOLO detection
            detections = detector.detect(frame)

            # Draw labels and bounding boxes
            annotated_frame = detector.draw(frame, detections)

            # Display feed
            cv2.imshow("Pi Camera - Bottle Detection Test", annotated_frame)

            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        detector.release()
        cv2.destroyAllWindows()
        print("Camera released and windows closed.")


if __name__ == "__main__":
    main()

