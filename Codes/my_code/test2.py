import cv2
import math
import numpy as np
from ultralytics import YOLO


# ============================================================
#                    YOLO SETTINGS
# ============================================================

MODEL = "yolo26n.pt"

# COCO class ID for bottle
BOTTLE_CLASS = 39

CONFIDENCE = 0.50

model = YOLO(MODEL)


# ============================================================
#                    CAMERA SETTINGS
# ============================================================

CAMERA_ID = 0

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# ------------------------------------------------------------
# IMPORTANT:
# Replace these with your actual laptop webcam FOV.
#
# These are only example values.
# ------------------------------------------------------------

HORIZONTAL_FOV = 70.0
VERTICAL_FOV = 55.0


# ============================================================
#                    CAMERA OFFSET
# ============================================================

# Camera position relative to J1, in mm.
#
# X = forward
# Y = left
# Z = upward
#
# CHANGE THESE VALUES TO YOUR ACTUAL MOUNTING POSITION.

XO = 0.0
YO = 0.0
ZO = 100.0


# ============================================================
#                    ARM DIMENSIONS
# ============================================================

L1 = 95.0       # J1 -> J2
L2 = 120.0      # J2 -> J3
L3 = 133.0      # J3 -> J4
LG = 35.0       # J4 -> gripper tip


# ============================================================
#                 GRIPPER ORIENTATION
# ============================================================

# Angle of gripper relative to horizontal.
#
# 0    = gripper points forward
# -90  = gripper points downward
# +90  = gripper points upward

PHI = -90.0


# ============================================================
#             SERVO CALIBRATION / OFFSETS
# ============================================================

# These MUST eventually be calibrated according to how
# your actual servos are mounted.
#
# Mathematical IK angle -> servo angle
#
# servo = OFFSET + SIGN * mathematical_angle

J1_OFFSET = 90.0
J2_OFFSET = 90.0
J3_OFFSET = 90.0
J4_OFFSET = 90.0

J1_SIGN = 1
J2_SIGN = 1
J3_SIGN = 1
J4_SIGN = 1


# ============================================================
#                 ULTRASONIC TEST VALUE
# ============================================================

# For Windows testing, we don't have the Raspberry Pi
# ultrasonic sensor connected.
#
# Enter the approximate distance from camera to bottle here.

DISTANCE_MM = 500.0


# ============================================================
#                  CAMERA INTRINSICS
# ============================================================

def calculate_camera_intrinsics():

    hfov = math.radians(HORIZONTAL_FOV)
    vfov = math.radians(VERTICAL_FOV)

    fx = IMAGE_WIDTH / (2 * math.tan(hfov / 2))
    fy = IMAGE_HEIGHT / (2 * math.tan(vfov / 2))

    cx = IMAGE_WIDTH / 2
    cy = IMAGE_HEIGHT / 2

    return fx, fy, cx, cy


# ============================================================
#              PIXEL + DISTANCE -> CAMERA XYZ
# ============================================================

def pixel_to_camera_xyz(px, py, distance):

    fx, fy, cx, cy = calculate_camera_intrinsics()

    # --------------------------------------------------------
    # Assume ultrasonic distance is approximately the
    # forward distance Z.
    # --------------------------------------------------------

    Zc = distance

    Xc = (px - cx) * Zc / fx

    Yc = (py - cy) * Zc / fy

    return Xc, Yc, Zc


# ============================================================
#             CAMERA XYZ -> ARM BASE XYZ
# ============================================================

def camera_to_arm(Xc, Yc, Zc):

    X = Xc + XO
    Y = Yc + YO
    Z = Zc + ZO

    return X, Y, Z


# ============================================================
#                       CLAMP
# ============================================================

def clamp(value, minimum, maximum):

    return max(minimum, min(value, maximum))


# ============================================================
#                  INVERSE KINEMATICS
# ============================================================

def inverse_kinematics(X, Y, Z, phi):

    # --------------------------------------------------------
    # J1
    # --------------------------------------------------------

    theta1 = math.atan2(Y, X)

    # Horizontal distance from J1 axis
    R = math.sqrt(X**2 + Y**2)

    # J2 is 95 mm above J1
    H = Z - L1

    # --------------------------------------------------------
    # Gripper orientation
    # --------------------------------------------------------

    phi_rad = math.radians(phi)

    # Position of J4 / wrist
    #
    # Remove the 35 mm gripper length from target.

    wrist_R = R - LG * math.cos(phi_rad)

    wrist_Z = H - LG * math.sin(phi_rad)

    # --------------------------------------------------------
    # Distance from J2 to wrist
    # --------------------------------------------------------

    D2 = wrist_R**2 + wrist_Z**2
    D = math.sqrt(D2)

    max_reach = L2 + L3
    min_reach = abs(L2 - L3)

    if D > max_reach:

        raise ValueError(
            f"Target too far: {D:.1f} mm "
            f"(maximum {max_reach:.1f} mm)"
        )

    if D < min_reach:

        raise ValueError(
            f"Target too close: {D:.1f} mm "
            f"(minimum {min_reach:.1f} mm)"
        )

    # --------------------------------------------------------
    # J3
    # --------------------------------------------------------

    cos_theta3 = (
        D2 - L2**2 - L3**2
    ) / (2 * L2 * L3)

    cos_theta3 = clamp(
        cos_theta3,
        -1.0,
        1.0
    )

    # Elbow-down solution
    theta3 = math.acos(cos_theta3)

    # --------------------------------------------------------
    # J2
    # --------------------------------------------------------

    alpha = math.atan2(
        wrist_Z,
        wrist_R
    )

    beta = math.atan2(
        L3 * math.sin(theta3),
        L2 + L3 * math.cos(theta3)
    )

    theta2 = alpha - beta

    # --------------------------------------------------------
    # J4
    #
    # phi = J2 + J3 + J4
    # --------------------------------------------------------

    theta4 = (
        phi_rad
        - theta2
        - theta3
    )

    # --------------------------------------------------------
    # Convert radians -> degrees
    # --------------------------------------------------------

    theta1 = math.degrees(theta1)
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)
    theta4 = math.degrees(theta4)

    return theta1, theta2, theta3, theta4


# ============================================================
#              MATHEMATICAL -> SERVO ANGLES
# ============================================================

def convert_to_servo_angles(
        theta1,
        theta2,
        theta3,
        theta4):

    j1 = J1_OFFSET + J1_SIGN * theta1
    j2 = J2_OFFSET + J2_SIGN * theta2
    j3 = J3_OFFSET + J3_SIGN * theta3
    j4 = J4_OFFSET + J4_SIGN * theta4

    angles = [j1, j2, j3, j4]

    # --------------------------------------------------------
    # Check servo limits
    # --------------------------------------------------------

    for i, angle in enumerate(angles):

        if angle < 0 or angle > 180:

            raise ValueError(
                f"J{i+1} requires "
                f"{angle:.2f} degrees "
                f"(outside 0-180)"
            )

    return angles


# ============================================================
#                       MAIN
# ============================================================

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

    print("ERROR: Could not open camera.")

    exit()


print()
print("======================================")
print("        ROVER YOLO + IK TEST")
print("======================================")
print()
print("Press Q to quit.")
print()


while True:

    ret, frame = cap.read()

    if not ret:

        print("Camera frame failed.")
        break


    # --------------------------------------------------------
    # YOLO detection
    # --------------------------------------------------------

    results = model(
        frame,
        conf=CONFIDENCE,
        verbose=False
    )

    bottle_found = False


    for result in results:

        if result.boxes is None:
            continue


        for box in result.boxes:

            class_id = int(
                box.cls[0]
            )

            confidence = float(
                box.conf[0]
            )


            # ------------------------------------------------
            # Only process bottle
            # ------------------------------------------------

            if class_id != BOTTLE_CLASS:
                continue


            bottle_found = True


            # ------------------------------------------------
            # Bounding box
            # ------------------------------------------------

            x1, y1, x2, y2 = (
                box.xyxy[0]
                .cpu()
                .numpy()
            )


            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)


            # ------------------------------------------------
            # Bottle center
            # ------------------------------------------------

            px = (x1 + x2) / 2
            py = (y1 + y2) / 2


            # ------------------------------------------------
            # Camera XYZ
            # ------------------------------------------------

            try:

                Xc, Yc, Zc = pixel_to_camera_xyz(
                    px,
                    py,
                    DISTANCE_MM
                )


                # ------------------------------------------------
                # Camera -> arm coordinates
                # ------------------------------------------------

                X, Y, Z = camera_to_arm(
                    Xc,
                    Yc,
                    Zc
                )


                # ------------------------------------------------
                # IK
                # ------------------------------------------------

                theta1, theta2, theta3, theta4 = (
                    inverse_kinematics(
                        X,
                        Y,
                        Z,
                        PHI
                    )
                )


                # ------------------------------------------------
                # Servo angles
                # ------------------------------------------------

                servo_angles = (
                    convert_to_servo_angles(
                        theta1,
                        theta2,
                        theta3,
                        theta4
                    )
                )


                # ------------------------------------------------
                # Print results
                # ------------------------------------------------

                print(
                    f"\nBottle "
                    f"Confidence={confidence:.2f}"
                )

                print(
                    f"Pixel: "
                    f"({px:.1f}, {py:.1f})"
                )

                print(
                    f"Camera XYZ: "
                    f"({Xc:.1f}, "
                    f"{Yc:.1f}, "
                    f"{Zc:.1f}) mm"
                )

                print(
                    f"Arm XYZ: "
                    f"({X:.1f}, "
                    f"{Y:.1f}, "
                    f"{Z:.1f}) mm"
                )

                print(
                    f"IK: "
                    f"{theta1:.1f}, "
                    f"{theta2:.1f}, "
                    f"{theta3:.1f}, "
                    f"{theta4:.1f}"
                )

                print(
                    f"SERVO: "
                    f"J1={servo_angles[0]:.1f} "
                    f"J2={servo_angles[1]:.1f} "
                    f"J3={servo_angles[2]:.1f} "
                    f"J4={servo_angles[3]:.1f}"
                )


                # ------------------------------------------------
                # Draw bounding box
                # ------------------------------------------------

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )


                # ------------------------------------------------
                # Draw center
                # ------------------------------------------------

                cv2.circle(
                    frame,
                    (int(px), int(py)),
                    5,
                    (0, 0, 255),
                    -1
                )


                # ------------------------------------------------
                # Display information
                # ------------------------------------------------

                cv2.putText(
                    frame,
                    f"Bottle {confidence:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"XYZ: {X:.0f},{Y:.0f},{Z:.0f}",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    f"J: {servo_angles[0]:.0f},"
                    f"{servo_angles[1]:.0f},"
                    f"{servo_angles[2]:.0f},"
                    f"{servo_angles[3]:.0f}",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )


            except ValueError as error:

                print(
                    "IK:",
                    error
                )

                cv2.putText(
                    frame,
                    "Target unreachable",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )


            # ------------------------------------------------
            # Only process first bottle
            # ------------------------------------------------

            break


    if not bottle_found:

        cv2.putText(
            frame,
            "No bottle",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )


    # --------------------------------------------------------
    # Show camera
    # --------------------------------------------------------

    cv2.imshow(
        "YOLO26n + IK",
        frame
    )


    if cv2.waitKey(1) & 0xFF == ord("q"):

        break


cap.release()

cv2.destroyAllWindows()