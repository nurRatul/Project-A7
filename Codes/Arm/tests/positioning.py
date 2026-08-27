"""
Test the camera pixel -> ultrasonic sensor selection.

No camera.
No GPIO.
No motors.

Current configuration:
    CAMERA_CX = 320
    threshold = 80

Therefore initially:
    px < 240       -> LEFT
    240 <= px <= 400 -> CENTER
    px > 400       -> RIGHT
"""

from . import config as cfg


def classify(px):

    offset_px = px - cfg.CAMERA_CX

    if offset_px < -cfg.SENSOR_SELECT_THRESHOLD_PX:
        return "LEFT"

    if offset_px > cfg.SENSOR_SELECT_THRESHOLD_PX:
        return "RIGHT"

    return "CENTER"


def main():

    print(
        f"Image width: {cfg.IMAGE_WIDTH}"
    )

    print(
        f"Camera center: {cfg.CAMERA_CX}"
    )

    print(
        f"Threshold: {cfg.SENSOR_SELECT_THRESHOLD_PX}"
    )

    print()

    test_pixels = [
        0,
        80,
        160,
        200,
        239,
        240,
        250,
        300,
        320,
        350,
        400,
        401,
        480,
        560,
        639,
    ]

    for px in test_pixels:

        print(
            f"px = {px:3d}  ->  {classify(px)}"
        )


if __name__ == "__main__":
    main()
