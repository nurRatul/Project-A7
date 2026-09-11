"""
Ultrasonic sensing for the trash-collecting robot.

UltrasonicSensor wraps one physical HC-SR04-style sensor (trigger +
echo pins) using gpiozero.DistanceSensor.

UltrasonicManager owns all three sensors (LEFT/CENTER/RIGHT),
decides which one to use based on where the camera says the trash
is in the image, reads it, and converts the raw reading into a
depth (Z) relative to the arm's J1 origin.

NONE OF THIS EXISTED IN YOUR ORIGINAL FILES
--------------------------------------------
The closest thing in your original controller.py was:

    self.distance_mm = 100.0  # Later replace this with ultrasonic reading.

There was no ultrasonic driver code, no GPIO handling, and no
mention of which library to use. Since this is genuinely new
hardware being wired in (not something to "preserve"), I built it
using gpiozero, which is the standard, pre-installed-on-Raspberry-Pi-OS
way to read HC-SR04-style trigger/echo ultrasonic sensors -- it
handles the pulse timing internally instead of needing manual
GPIO bit-banging. If your actual sensors use a different interface
(e.g. an I2C ultrasonic board, or they're read through the Arduino
instead of the Pi), tell me and I'll rebuild this part.

Install with: pip install gpiozero
"""

from gpiozero import DistanceSensor

from . import config as cfg


class UltrasonicSensor:
    """One physical ultrasonic sensor and its mounting geometry."""

    def __init__(self, trigger_pin, echo_pin, x_offset, z_offset):

        if trigger_pin is None or echo_pin is None:
            raise ValueError(
                "An ultrasonic sensor's GPIO pins are not configured. "
                "Set the matching *_TRIGGER_PIN / *_ECHO_PIN values "
                "in config.py before running."
            )

        self.x_offset = x_offset
        self.z_offset = z_offset

        # gpiozero's max_distance is in meters; give it a little
        # headroom past our own valid range so out-of-range readings
        # come back as a real (too-large) number we can reject
        # ourselves, rather than being silently clipped by gpiozero.
        self._sensor = DistanceSensor(
            echo=echo_pin,
            trigger=trigger_pin,
            max_distance=(cfg.ULTRASONIC_MAX_VALID_MM / 1000.0) + 0.5
        )

    def read_mm(self):
        """
        Returns the raw straight-line reading in mm, or None if it
        falls outside the configured valid range.
        """

        raw_mm = self._sensor.distance * 1000.0

        if raw_mm < cfg.ULTRASONIC_MIN_VALID_MM:
            return None

        if raw_mm > cfg.ULTRASONIC_MAX_VALID_MM:
            return None

        return raw_mm

    def close(self):
        self._sensor.close()


class UltrasonicManager:
    """
    Picks the correct ultrasonic sensor based on the camera's
    left/center/right classification, reads it, and returns the
    depth relative to the arm's J1 origin.
    """

    def __init__(self):

        self.sensors = {
            "LEFT": UltrasonicSensor(
                cfg.LEFT_SENSOR_TRIGGER_PIN,
                cfg.LEFT_SENSOR_ECHO_PIN,
                cfg.LEFT_SENSOR_X_OFFSET,
                cfg.LEFT_SENSOR_Z_OFFSET
            ),
            "CENTER": UltrasonicSensor(
                cfg.CENTER_SENSOR_TRIGGER_PIN,
                cfg.CENTER_SENSOR_ECHO_PIN,
                cfg.CENTER_SENSOR_X_OFFSET,
                cfg.CENTER_SENSOR_Z_OFFSET
            ),
            "RIGHT": UltrasonicSensor(
                cfg.RIGHT_SENSOR_TRIGGER_PIN,
                cfg.RIGHT_SENSOR_ECHO_PIN,
                cfg.RIGHT_SENSOR_X_OFFSET,
                cfg.RIGHT_SENSOR_Z_OFFSET
            ),
        }

    def classify_direction(self, px):
        """
        Decide LEFT / CENTER / RIGHT from the detected object's
        pixel x-coordinate, comparing its offset from the camera's
        calibrated center (CAMERA_CX) against
        SENSOR_SELECT_THRESHOLD_PX.

        This does NOT need to know the real-world depth -- it's a
        pixel-space comparison only, which is what lets sensor
        selection happen before we know Z.
        """

        offset_px = px - cfg.CAMERA_CX

        if offset_px < -cfg.SENSOR_SELECT_THRESHOLD_PX:
            return "LEFT"

        if offset_px > cfg.SENSOR_SELECT_THRESHOLD_PX:
            return "RIGHT"

        return "CENTER"

    def measure(self, px):
        """
        Given the pixel x-position of the detected trash, select
        the matching sensor, read it, and convert the reading into
        a Z distance relative to the arm's J1 origin.

        Returns (direction, raw_mm, z_arm):
            direction -- "LEFT" / "CENTER" / "RIGHT", whichever was selected
            raw_mm    -- that sensor's raw reading, or None if invalid
            z_arm     -- raw_mm + that sensor's configured Z offset,
                         or None if the reading was invalid
        """

        direction = self.classify_direction(px)
        sensor = self.sensors[direction]

        raw_mm = sensor.read_mm()

        if raw_mm is None:
            return direction, None, None

        z_arm = raw_mm + sensor.z_offset

        return direction, raw_mm, z_arm

    def close(self):
        for sensor in self.sensors.values():
            sensor.close()
