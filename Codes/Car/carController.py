import os
from time import sleep

from .btsMotor import BTSMotor

# ---------------------- GPIO Controll ---------------------------- #
from gpiozero import PWMOutputDevice, DigitalOutputDevice
# ----------------------------------------------------------------- #

class CarController:
    def __init__(self, left_pins=None, right_pins=None):
        left_pins = left_pins or self._pins_from_environment(
            "CAR_LEFT_PINS", (27,22, 23, 24)
        )
        right_pins = right_pins or self._pins_from_environment(
            "CAR_RIGHT_PINS", (6, 13, 19, 26)
        )
        self.left_motor = BTSMotor(*left_pins)  # RPWM, LPWM, R_EN, L_EN
        self.right_motor = BTSMotor(*right_pins)  # RPWM, LPWM, R_EN, L_EN
        self.frame = None
        self.detections = []

    @staticmethod
    def _pins_from_environment(name, default):
        value = os.getenv(name)
        if not value:
            return default
        try:
            pins = tuple(int(pin.strip()) for pin in value.split(","))
        except ValueError as error:
            raise ValueError(f"{name} must contain four GPIO numbers") from error
        if len(pins) != 4:
            raise ValueError(f"{name} must contain four GPIO numbers")
        return pins

    def update_vision(self, frame, detections):
        """Receive the frame and YOLO results from the shared vision loop."""
        self.frame = frame
        self.detections = detections or []

    def drive(self, left_speed, right_speed):
        """Drive each side from -1.0 (backward) to 1.0 (forward)."""
        left_speed = max(-1.0, min(1.0, float(left_speed)))
        right_speed = max(-1.0, min(1.0, float(right_speed)))
        if left_speed > 0:
            self.left_motor.forward(left_speed)
        elif left_speed < 0:
            self.left_motor.backward(abs(left_speed))
        else:
            self.left_motor.stop()
        if right_speed > 0:
            self.right_motor.forward(right_speed)
        elif right_speed < 0:
            self.right_motor.backward(abs(right_speed))
        else:
            self.right_motor.stop()

    def move_forward(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.drive(speed, speed)
        else:
            self.drive(speed, speed)
            sleep(deltaT)
            self.stop()

    def move_backward(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.drive(-speed, -speed)
        else:
            self.drive(-speed, -speed)
            sleep(deltaT)
            self.stop()

    def turn_left(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.drive(-speed, speed)
        else:
            self.drive(-speed, speed)
            sleep(deltaT)
            self.stop()

    def turn_right(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.drive(speed, -speed)
        else:
            self.drive(speed, -speed)
            sleep(deltaT)
            self.stop()


    def stop(self):         ## Stopes the both motors
        self.left_motor.stop()
        self.right_motor.stop()

    
    

    
