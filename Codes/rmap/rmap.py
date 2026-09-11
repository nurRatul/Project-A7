from Car.carController import CarController
from Arm.armcontroller import ArmController
from sensors.basic.gps.gpsManager import GPSManager
from sensors.basic.imu.imuManager import IMUManager
from sensors.basic.ultrasonic.ultrasonicManager import UltrasonicManager
from logger.logger_manager import LoggerManager
from dataclasses import dataclass, field
import time
import math

@dataclass
class Object:
    name: str = field(default_factory=lambda: "Unknown")
    id: int = field(default_factory=lambda: -1)
    position: tuple = field(default_factory=lambda: (None, None))
    pickedUp: bool = field(default=False)

@dataclass
class RoverState:
    position: tuple = (None, None)
    object_detected: bool = False
    orientation: float = None
    gps_fix: bool = False
    imu_orientation: dict = field(default_factory=dict)
    ultrasonic_telemetry: dict = field(default_factory=dict)
    detected_objects: list = field(default_factory=list)
    


class mappingManager:
    def __init__(self, roverState: RoverState, gps_manager=None, imu_manager=None, ultrasonic_manager=None, car_controller=None, arm_controller=None, gps_port="/dev/ttyAMA0", gps_baudrate=9600, gps_timeout=1.0):
        self.roverState = roverState
        self.gps_manager = gps_manager or GPSManager(port=gps_port, baudrate=gps_baudrate, timeout=gps_timeout)
        self.gps_manager.start()
        self.imu_manager = imu_manager or IMUManager()
        self.ultrasonic_manager = ultrasonic_manager or UltrasonicManager()
        self.car_controller = car_controller or CarController()
        self.arm_controller = arm_controller or ArmController()

    def cover_area_nogps(self, x_direction=2, y_direction=2, speed=0.2):

        for i in range(1, math.ceil(y_direction)+1):
            deltaT = x_direction / speed
            print(f"Moving forward for {deltaT} seconds at speed {speed}")
            time_covered = 0
            starting = time.time()
            self.car_controller.move_forward(deltaT=None, speed=speed)
            while time_covered < deltaT:
                time_covered =time_covered + (time.time() - starting)
                print(self.roverState.object_detected)
                if self.roverState.object_detected:
                    self.car_controller.stop()
                    time.sleep(1)
                    self.arm_controller.dump_garbage_w1()
                     # wait for 5 seconds to pickup object. Here the arm code has to be implemented to pickup the object. After that the rover will continue to move forward.
                    self.car_controller.move_forward(deltaT=None, speed=speed)
                self.roverState.object_detected = False
                starting = time.time()
                time.sleep(0.1)
                print(f"\n\nTime covered: {time_covered:.2f} seconds")
            self.car_controller.stop()
                

            if i < math.ceil(y_direction):
                if i % 2 == 0:
                    self.car_controller.turn_right(speed=speed,angle=90)
                    self.car_controller.move_forward(deltaT=1, speed=speed)
                    self.car_controller.turn_right(speed=speed,angle=90)
                else:
                    self.car_controller.turn_left(speed=speed,angle=85)
                    self.car_controller.move_forward(deltaT=1, speed=speed)
                    self.car_controller.turn_left(speed=speed,angle=85)

    def cover_area_gps(self, x_direction=2, y_direction=2, speed=0.2):
        self.gps_manager.reset()
        self.imu_manager.reset()
        # implement the logic to cover the area using GPS and IMU data
        pass


        