
from mapping.mappingManager import MappingManager
from Car.carController import CarController
from sensors.basic.ultrasonic.ultrasonicManager import UltrasonicManager
from sensors.basic.imu.imuManager import IMUManager
from sensors.basic.gps.gpsManager import GPSManager
from sensors.basic.vision.visionManager import VisionManager


mapper = MappingManager(
    car=car_controller,
    arm=arm_controller,
    gps=gps_manager,
    imu=imu_manager,
    ultrasonic=ultrasonic_manager,
    vision=vision_manager,
    object_policy="avoid"
)

mapper.start_mapping(width_m=5, height_m=7)

while mapper.state not in ("COMPLETED", "ERROR"):
    mapper.tick()