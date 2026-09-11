
from mapping.mappingManager import MappingManager
from Car.carController import CarController
from sensors.basic.ultrasonic.ultrasonicManager import UltrasonicManager
from sensors.basic.imu.imuManager import IMUManager
from sensors.basic.gps.gpsManager import GPSManager
# from sensors.vision.visionManager import VisionManager
import time



class FakeUltrasonic:
    """Clear by default. Tests can set .front_m low to simulate an obstacle."""

    def __init__(self):
        self.front_m = 4.0
        self.left_m = 4.0
        self.right_m = 4.0

    def get_telemetry(self):
        return {"front_m": self.front_m, "left_m": self.left_m, "right_m": self.right_m,
                "last_update": time.time()}

    def has_obstacle(self, direction="front", threshold_m=None):
        threshold = threshold_m if threshold_m is not None else 0.35
        distance = {"front": self.front_m, "left": self.left_m, "right": self.right_m}[direction]
        return distance is not None and distance < threshold


class FakeVision:
    def detect(self, frame=None):
        return []

    def read_frame(self):
        return None


class FakeArm:
    def update(self, frame=None, detections=None):
        return None


gps = GPSManager()
gps.start()

mapper = MappingManager(
    car=CarController(),
    arm=FakeArm(),
    gps=gps,
    imu=IMUManager(),
    ultrasonic=FakeUltrasonic(),
    vision=FakeVision() ,
    object_policy="ignore",
    cruise_speed=0.2,
)

mapper.start_mapping(width_m=4, height_m=5)

while mapper.state not in ("COMPLETED", "ERROR"):
    mapper.tick()