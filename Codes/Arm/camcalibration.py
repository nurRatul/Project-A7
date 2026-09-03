import cv2
import numpy as np
import json
from picamera2 import Picamera2
import config as cfg

# Checkerboard settings
CHECKERBOARD = (7, 7)
SQUARE_SIZE_MM = 18 

# Termination criteria for sub-pixel accuracy
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (0,0,0), (25,0,0), (50,0,0) ...
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

objpoints = [] # 3D points in real world space
imgpoints = [] # 2D points in image plane

print("Initializing camera...")
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (cfg.IMAGE_WIDTH, cfg.IMAGE_HEIGHT)}
)
picam2.configure(camera_config)
picam2.start()

print("Controls: Press 'c' to capture a good frame. Press 'q' to finish and calibrate.")
captures = 0

try:
    while True:
        frame_rgb = picam2.capture_array()
        if frame_rgb is None: continue
        
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display_frame = frame.copy()
        if ret:
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret)
            
        cv2.imshow('Calibration - Press c to capture, q to quit', display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c') and ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            captures += 1
            print(f"Captured {captures}/20 images.")

        elif key == ord('q'):
            if captures >= 10:
                break
            else:
                print("Need at least 10 images to calibrate safely.")
finally:
    picam2.stop()
    cv2.destroyAllWindows()

print("\nCalculating calibration... This may take a moment.")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

fx, fy = mtx[0, 0], mtx[1, 1]
cx, cy = mtx[0, 2], mtx[1, 2]

print(f"\nReprojection Error: {ret:.4f} pixels")
print(f"CAMERA_FX = {fx:.2f}")
print(f"CAMERA_FY = {fy:.2f}")
print(f"CAMERA_CX = {cx:.2f}")
print(f"CAMERA_CY = {cy:.2f}")
print(f"Distortion Coeffs: {dist.ravel()}")

# Save to file
calib_data = {
    "camera_matrix": mtx.tolist(),
    "dist_coeffs": dist.tolist(),
    "reprojection_error": ret,
    "fx": fx, "fy": fy, "cx": cx, "cy": cy
}

with open("camera_calibration.json", "w") as f:
    json.dump(calib_data, f, indent=4)
print("\nCalibration saved to camera_calibration.json")
