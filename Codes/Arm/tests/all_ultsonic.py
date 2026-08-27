"""
Test LEFT, CENTER and RIGHT HC-SR04 sensors.

Sensors are read sequentially to reduce ultrasonic cross-talk.
"""

import time
from gpiozero import DistanceSensor
from . import config as cfg


def make_sensor(trigger, echo, name):
    if trigger is None or echo is None:
        raise RuntimeError(
            f"{name} sensor GPIO pins are not configured."
        )

    return DistanceSensor(
        trigger=trigger,
        echo=echo,
        max_distance=4.0,
        queue_len=1,
    )


def main():

    sensors = {
        "LEFT": make_sensor(
            cfg.LEFT_SENSOR_TRIGGER_PIN,
            cfg.LEFT_SENSOR_ECHO_PIN,
            "LEFT",
        ),

        "CENTER": make_sensor(
            cfg.CENTER_SENSOR_TRIGGER_PIN,
            cfg.CENTER_SENSOR_ECHO_PIN,
            "CENTER",
        ),

        "RIGHT": make_sensor(
            cfg.RIGHT_SENSOR_TRIGGER_PIN,
            cfg.RIGHT_SENSOR_ECHO_PIN,
            "RIGHT",
        ),
    }

    print("Testing all three ultrasonic sensors.")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:

            for name, sensor in sensors.items():

                distance_mm = sensor.distance * 1000.0

                print(
                    f"{name:6s}: "
                    f"{distance_mm:7.1f} mm  "
                    f"({distance_mm / 10:6.1f} cm)"
                )

                time.sleep(0.08)

            print("-" * 40)

            time.sleep(0.15)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        for sensor in sensors.values():
            sensor.close()


if __name__ == "__main__":
    main()
