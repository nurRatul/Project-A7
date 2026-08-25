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
    def __init__(self,lm = [22,23,24,27], rm = [6,13,19,26]):
        self.left_motor = BTSMotor(lm[0],lm[1],lm[2],lm[3])  # GPIO pins
        self.right_motor = BTSMotor(rm[0],rm[1],rm[2],rm[3])  # GPIO pins
        self.frame = None
        self.detections = []

    def update_vision(self, frame, detections):
        """Receive the frame and YOLO results from the shared vision loop."""
        self.frame = frame
        self.detections = detections or []

    def move_forward(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.left_motor.forward(speed)
            self.right_motor.forward(speed)
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

    def turn_right(self, deltaT=None, speed= 1.0):
        if deltaT == None:
            self.left_motor.forward(speed)
            self.right_motor.backward(speed)
        else:
            self.left_motor.forward(speed)
            self.right_motor.backward(speed)
            sleep(deltaT)
            self.left_motor.stop()
            self.right_motor.stop()

    def drive(self, left_speed, right_speed):
        """Drive both motors with signed speeds in the range -1 to 1."""
        for motor, speed in ((self.left_motor, left_speed), (self.right_motor, right_speed)):
            speed = max(-1.0, min(1.0, float(speed)))
            print(speed)
            if speed > 0:
                motor.forward(speed)
            elif speed < 0:
                motor.backward(-speed)
            else:
                motor.stop()


    def stop(self):         ## Stopes the both motors
        self.left_motor.stop()
        self.right_motor.stop()

    
    

    
