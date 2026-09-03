from Car.carController import CarController
from rmap.rmap import RoverState
from sensors.basic.gps.gpsManager import GPSManager
from sensors.basic.imu.imuManager import IMUManager
from sensors.basic.ultrasonic.ultrasonic import UltrasonicSensor
from logger.logger_manager import LoggerManager
from dataclasses import dataclass, field
from .rmap import mappingManager
import threading
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
    




class Rover:
    def __init__(self, gps_port="/dev/ttyAMA0", gps_baudrate=9600, gps_timeout=1.0, hold_distance=150):
        self.gps_manager = GPSManager(port=gps_port, baudrate=gps_baudrate, timeout=gps_timeout)
        self.gps_manager.start()
        self.imu_manager = IMUManager()
        self.ultrasonic_sensor = UltrasonicSensor(9,10,0,0)
        self.car_controller = CarController()
        self.roverState = RoverState()
        self.mapping_manager = mappingManager(self.roverState, self.gps_manager, self.imu_manager, self.ultrasonic_sensor, self.car_controller, gps_port, gps_baudrate, gps_timeout)
        self.hold_distance = hold_distance

        # --------------------------------------------#
        self.thread = threading.Thread(target=self.monitor_ultrasonic, args=(hold_distance,), daemon=True)
        self.thread.start()

    def has_obstacle(self, threshold_m=None): ## put this code to the ultrasonicManager
        if threshold_m is None:
            threshold_m = self.hold_distance
        if self.ultrasonic_sensor.read_mm() < threshold_m:
            self.roverState.object_detected = True
            return True
        return False

    def monitor_ultrasonic(self, threshold_m= None):
        if threshold_m is None:
            threshold_m = self.hold_distance
        while True:
            if self.has_obstacle(threshold_m):
                print("Obstacle detected!")
            time.sleep(0.1)  # Adjust the sleep time as needed


    
