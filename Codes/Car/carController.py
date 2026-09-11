import os
from time import sleep

#-----user defined------#
from .btsMotor import BTSMotor
from logger.logger_manager import LoggerManager
from sensors.basic.imu.imuManager import IMUManager

logger = LoggerManager.get_logger('carController')

#-----------------------#

class CarController:
    def __init__(
        self,
        left_pins=None,
        right_pins=None,
        imu_manager = None,   # IMUManager
    ):
        left_pins = left_pins or self._pins_from_environment(
            "CAR_LEFT_PINS",
            (17, 22, 23, 24)
        )

        right_pins = right_pins or self._pins_from_environment(
            "CAR_RIGHT_PINS",
            (6, 13, 19, 26)
        )

        self.left_motor = BTSMotor(*left_pins)
        self.right_motor = BTSMotor(*right_pins)

        self.frame = None
        self.detections = []

        if imu_manager is None:
            try:
                self.imu = IMUManager()
            except Exception as e:
                logger.exception("Failed to initialize IMU: %s", e)
                self.imu = None
        else:
            self.imu = imu_manager

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


    def turn_left(self, deltaT=None, speed=1.0, angle=None):
        if angle is None:
            if deltaT is None:
                self.drive(-speed, speed)
            else:
                self.drive(-speed, speed)
                sleep(deltaT)
                self.stop()

            return

        # -----------------------------------
        # Angle based rotation
        # -----------------------------------

        start_yaw = self.get_orientation()["yaw"]

        self.drive(-speed, speed)

        while True:

            current_yaw = self.get_orientation()["yaw"]

            # signed difference
            rotated = current_yaw - start_yaw
            print(f"start_yaw: {start_yaw}, current_yaw: {current_yaw}, rotated: {rotated}, target_angle: {angle}")

            if abs(rotated) >= angle:
                break

            sleep(0.01)

        self.stop()

    def turn_right(self, deltaT=None, speed= 1.0, angle=None):
        if angle is None:
            # old behavior
            if deltaT is None:
                self.drive(speed, -speed)
            else:
                self.drive(speed, -speed)
                sleep(deltaT)
                self.stop()

            return

        # -----------------------------------
        # Angle based rotation
        # -----------------------------------

        start_yaw = self.get_orientation()["yaw"]

        self.drive(speed, -speed)

        while True:

            current_yaw = self.get_orientation()["yaw"]

            # signed difference
            rotated = current_yaw - start_yaw
            print(f"start_yaw: {start_yaw}, current_yaw: {current_yaw}, rotated: {rotated}, target_angle: {angle}")

            if abs(rotated) >= angle:
                break

            sleep(0.01)
        self.stop()



    def stop(self):         ## Stopes the both motors
        self.left_motor.stop()
        self.right_motor.stop()

    def pitch_to_speed(self, pitch=None,min_speed=0.1, max_speed=.4):
        if pitch is None:
            orientation = self.get_orientation()
            if orientation is None:
                return min_speed
            pitch = orientation["pitch"]
            print(pitch)

        # Map pitch to speed
        speed = min_speed + (abs(pitch) / 90.0) * (max_speed - min_speed)
        if pitch<0:
            return min_speed
        return max(-max_speed, min(max_speed, speed))


    ###-----------------------------Reading Imu datas Start --------------------###

    def get_orientation(self):
        if self.imu:
            return self.imu.get_orientation()
        return None


    def get_acceleration(self):
        if self.imu:
            return self.imu.get_acceleration()
        return None


    def get_position(self):
        if self.imu:
            return self.imu.get_position()
        return None


    def get_telemetry(self):
        if self.imu:
            return self.imu.get_telemetry()
        return {"available": False}



    ###----------------------------- Reading Imu datas End --------------------###

    ### ----------------------------- Cleanup Start --------------------###
    def shutdown(self):
        self.stop()

        for motor in (self.left_motor, self.right_motor):
            try:
                motor.close()
            except Exception:
                logger.exception("Error closing motor")

        if self.imu:
            try:
                self.imu.close()
            except Exception:
                logger.exception("Error closing IMU")

    
    

    
