# import sys
# from pathlib import Path

# if __package__ in (None, ""):
# 	sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# from Car.carController import CarController

# car = CarController()

# car.move_forward(deltaT=2, speed=0.2)

# from Car.sensors.basic.imu.imuManager import IMUManager

# imu = IMUManager()
# print(imu.get_orientation())
# print(imu.get_telemetry())


import sys
from pathlib import Path

if __package__ in (None, ""):
	sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Car.carController import CarController

from Car.sensors.basic.gps.gpsManager import GPSManager

car = CarController()
gps = GPSManager(port="/dev/ttyAMA0", baudrate=9600, timeout=1.0)
gps.start()


commands = [
    (car.move_forward, 2, 0.3),
    (car.turn_left,    1, 0.3),
    (car.move_forward,2, 0.3),
    (car.turn_left,    1, 0.3),
]

for _ in range(2):
    for func, deltaT, speed in commands:
        func(deltaT=deltaT, speed=speed)
        #print(gps.get_location())
        print(gps.get_telemetry())

gps.stop()


# car.move_forward(deltaT=2, speed=0.2)
# car.turn_left(deltaT=1, speed=0.2)
# car.move_backward(deltaT=2, speed=0.2)
# car.turn_left(deltaT=1, speed=0.2)
# car.move_forward(deltaT=2, speed=0.2)
# car.turn_left(deltaT=1, speed=0.2)
# car.move_backward(deltaT=2, speed=0.2)
