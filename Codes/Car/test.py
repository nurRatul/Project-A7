from Car.carController import CarController

car = CarController()

car.move_forward(deltaT=2, speed=0.2)

from Car.sensors.basic.imu.imuManager import IMUManager

imu = IMUManager()
print(imu.get_orientation())
print(imu.get_telemetry())