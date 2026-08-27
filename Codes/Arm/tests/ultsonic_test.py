"""
Test ONE HC-SR04 connected to the CENTER GPIO pins.

CENTER:
    TRIG = GPIO 20
    ECHO = GPIO 21

Motion/servos are not involved.
"""

import time
from gpiozero import DistanceSensor
from . import config as cfg


def main():
    trigger = cfg.CENTER_SENSOR_TRIGGER_PIN
    echo = cfg.CENTER_SENSOR_ECHO_PIN

    if trigger is None or echo is None:
        raise RuntimeError(
            "Set CENTER_SENSOR_TRIGGER_PIN and "
            "CENTER_SENSOR_ECHO_PIN in config.py first."
        )

    print(f"Testing CENTER HC-SR04")
    print(f"TRIG = GPIO {trigger}")
    print(f"ECHO = GPIO {echo}")
    print()
    print("Place an object in front of the sensor.")
    print("Press Ctrl+C to stop.\n")

    sensor = DistanceSensor(
        trigger=trigger,
        echo=echo,
        max_distance=4.0,
        queue_len=1,
    )

    try:
        while True:
            distance_mm = sensor.distance * 1000.0

            print(
                f"Center distance: "
                f"{distance_mm:7.1f} mm  "
                f"({distance_mm / 10:6.1f} cm)"
            )

            time.sleep(0.25)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        sensor.close()


if __name__ == "__main__":
    main()
