import cv2
import numpy as np
import sys
from picamera2 import Picamera2


# ============================================================
# CONFIGURATION
# ============================================================
CHECKERBOARD = (7, 7)  # Internal corners (width, height)
SQUARE_SIZE = 18.5     # Square size in mm
REQUIRED_IMAGES = 30   # Captures required
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# ============================================================
# PREPARE 3D POINTS
# ============================================================
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

object_points = []
image_points = []

# ============================================================
# INITIALIZE PICAMERA2 (Pi 5 Native Hardware Driver)
# ============================================================
print("Starting Picamera2 on Pi 5 ISP...")
try:
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (IMAGE_WIDTH, IMAGE_HEIGHT), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
except Exception as e:
    print(f"\n[ERROR] Failed to open Picamera2: {e}")
    print("Ensure no other camera process is running in the background.")
    sys.exit(1)

print("\nCamera started successfully!")
print("Controls: [SPACE] Capture frame | [Q] Quit")

# ============================================================
# MAIN LOOP
# ============================================================
while True:
    # Capture frame directly into numpy array
    frame_rgb = picam2.capture_array()
    
    # Convert RGB to BGR for OpenCV processing/display
    frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    display = frame.copy()

    if found:
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)
        cv2.putText(display, "Ready (Press SPACE)", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(display, "Checkerboard not detected", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.putText(display, f"Captured: {len(image_points)}/{REQUIRED_IMAGES}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv2.imshow("Pi 5 Camera Calibration", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord(' '):
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            object_points.append(objp.copy())
            image_points.append(refined_corners)
            print(f"Captured {len(image_points)}/{REQUIRED_IMAGES}")
            
            # Flash UI to confirm capture
            cv2.imshow("Pi 5 Camera Calibration", cv2.bitwise_not(display))
            cv2.waitKey(100)
        else:
            print("Checkerboard not visible.")

    elif key == ord('q'):
        print("Exiting...")
        break

    if len(image_points) >= REQUIRED_IMAGES:
        print("\nCollected enough images. Processing calibration...")
        break

picam2.stop()
cv2.destroyAllWindows()

# ============================================================
# CALIBRATION
# ============================================================
if len(image_points) > 0:
    ret, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, gray.shape[::-1], None, None
    )

    print("\n" + "="*40)
    print("CALIBRATION COMPLETE")
    print("="*40)
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", distortion)
    print(f"Reprojection Error: {ret:.4f} pixels")

    np.savez("camera_calibration.npz", 
             camera_matrix=camera_matrix, 
             distortion=distortion)
    print("\nSaved parameters to camera_calibration.npz")
