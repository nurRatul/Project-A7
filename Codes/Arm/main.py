import cv2

from detector import (
    BottleDetector,
    open_camera,
    draw_detections
)

from ik import inverse_kinematics


# ============================================================
# SETTINGS
# ============================================================

SHOW_VIDEO = True

# Temporary distance for testing.
#
# Later this will come from your ultrasonic sensor.

DISTANCE_MM = 120.0


# Camera FOV
#
# These are temporary values.
# We will calibrate the real camera later.

HORIZONTAL_FOV = 70.0
VERTICAL_FOV = 55.0


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


# ============================================================
# CAMERA -> 3D
# ============================================================

def pixel_to_camera(
        px,
        py,
        distance):

    import math


    hfov = math.radians(
        HORIZONTAL_FOV
    )

    vfov = math.radians(
        VERTICAL_FOV
    )


    # Camera focal lengths

    fx = IMAGE_WIDTH / (
        2 * math.tan(hfov / 2)
    )

    fy = IMAGE_HEIGHT / (
        2 * math.tan(vfov / 2)
    )


    cx = IMAGE_WIDTH / 2
    cy = IMAGE_HEIGHT / 2


    # Temporary simplified model.
    #
    # IMPORTANT:
    # This assumes distance is approximately
    # camera forward distance.

    z = distance

    x = (
        (px - cx) *
        z / fx
    )

    y = (
        (py - cy) *
        z / fy
    )


    return x, y, z


# ============================================================
# CAMERA OFFSET
# ============================================================

# Camera position relative to J1.
#
# Change these later.

XO = 0.0
YO = 0.0
ZO = 100.0


def camera_to_arm(
        xc,
        yc,
        zc):

    x = xc + XO
    y = yc + YO
    z = zc + ZO

    return x, y, z


# ============================================================
# MAIN
# ============================================================

def main():

    detector = BottleDetector()

    cap = open_camera()


    print()
    print("==============================")
    print("ROVER VISION + IK")
    print("==============================")
    print()
    print("Press Q or ESC to quit.")
    print()


    while True:

        ret, frame = cap.read()


        if not ret:

            print(
                "Camera read failed."
            )

            break


        # ====================================================
        # YOLO
        # ====================================================

        detections = (
            detector.detect(frame)
        )


        # ====================================================
        # PROCESS FIRST BOTTLE
        # ====================================================

        if detections:

            detection = detections[0]


            px = detection["cx"]
            py = detection["cy"]


            print()
            print(
                "------------------------------"
            )

            print(
                f"Bottle detected"
            )

            print(
                f"Confidence: "
                f"{detection['confidence']:.2f}"
            )

            print(
                f"Pixel center: "
                f"({px:.1f}, {py:.1f})"
            )


            # =================================================
            # DISTANCE
            # =================================================

            distance = DISTANCE_MM


            print(
                f"Distance: "
                f"{distance:.1f} mm"
            )


            # =================================================
            # CAMERA XYZ
            # =================================================

            xc, yc, zc = (
                pixel_to_camera(
                    px,
                    py,
                    distance
                )
            )


            print(
                f"Camera XYZ: "
                f"({xc:.1f}, "
                f"{yc:.1f}, "
                f"{zc:.1f}) mm"
            )

            


            # =================================================
            # ARM XYZ
            # =================================================

            x, y, z = (
                camera_to_arm(
                    xc,
                    yc,
                    zc
                )
            )


            print(
                f"Arm XYZ: "
                f"({x:.1f}, "
                f"{y:.1f}, "
                f"{z:.1f}) mm"
            )


            # =================================================
            # IK
            # =================================================

            try:

                theta = (
                    inverse_kinematics(
                        x,
                        y,
                        z,
                        phi=-90
                    )
                )


                print()
                print(
                    "IK mathematical angles:"
                )

                print(
                    f"J1 = {theta[0]:.2f}°"
                )

                print(
                    f"J2 = {theta[1]:.2f}°"
                )

                print(
                    f"J3 = {theta[2]:.2f}°"
                )

                print(
                    f"J4 = {theta[3]:.2f}°"
                )


            except ValueError as error:

                print(
                    "IK ERROR:",
                    error
                )


        # ====================================================
        # VIDEO
        # ====================================================

        if SHOW_VIDEO:

            display = draw_detections(
                frame,
                detections
            )


            cv2.imshow(
                "Rover Vision",
                display
            )


            # Keyboard handling
            key = cv2.waitKey(1)


            # Q / q / ESC

            if (
                key == ord("q") or
                key == ord("Q") or
                key == 27
            ):

                print(
                    "Stopping..."
                )

                break


    cap.release()

    cv2.destroyAllWindows()


# ============================================================
# START
# ============================================================

if __name__ == "__main__":

    main()