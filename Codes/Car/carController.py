import os
from time import sleep

try:
    from .btsMotor import BTSMotor
except ImportError:
    from btsMotor import BTSMotor

# ---------------------- GPIO Controll ---------------------------- #
 from gpiozero import PWMOutputDevice, DigitalOutputDevice
# ----------------------------------------------------------------- #

class CarController:
    def __init__(self):
        self.left_motor = BTSMotor(18, 19 , 23, 24)  # GPIO pins
        self.right_motor = BTSMotor(12,13,14,15)  # GPIO pins
        self.frame = None
        self.detections = []

    def update_vision(self, frame, detections):
        """Receive the frame and YOLO results from the shared vision loop."""
        self.frame = frame
        self.detections = detections or []

    def move_forward(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.left_motor.forward(speed)
            # self.right_motor.forward(speed)
        else:
            self.left_motor.forward(speed)
            self.right_motor.forward(speed)
            sleep(deltaT)
            self.left_motor.stop()
            self.right_motor.stop()

    def move_backward(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.left_motor.backward(speed)
            self.right_motor.backward(speed)
        else:
            self.left_motor.backward(speed)
            self.right_motor.backward(speed)
            sleep(deltaT)
            self.left_motor.stop()
            self.right_motor.stop()

    def turn_left(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.left_motor.backward(speed)
            self.right_motor.forward(speed)
        else:
            self.left_motor.backward(speed)
            self.right_motor.forward(speed)
            sleep(deltaT)
            self.left_motor.stop()
            self.right_motor.stop()
