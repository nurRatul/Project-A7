from logger.logger_manager import LoggerManager
from .interfaces import RoverState, Object
import time
import math




class mappingManager:
    def __init__(self, roverState: RoverState, gps_manager=None, imu_manager=None, ultrasonic_manager=None, car_controller=None, arm_controller=None, gps_port="/dev/ttyAMA0", gps_baudrate=9600, gps_timeout=1.0):
        self.roverState = roverState
        self.gps_manager = gps_manager
        self.imu_manager = imu_manager
        self.ultrasonic_manager = ultrasonic_manager
        self.car_controller = car_controller
        self.arm_controller = arm_controller

    def cover_area_nogps(self, x_direction=2, y_direction=2, speed=0.2,object_distance=50):
        row_spacing = .25
        for i in range(1, int(y_direction/row_spacing)):
            k = 1.36 #correction factor
            deltaT = x_direction / (speed * k)
            print(f"\rMoving forward for {deltaT} seconds at speed {speed}")
            time_covered = 0
            starting = time.time()
            self.car_controller.move_forward(deltaT=None, speed=speed)
            while time_covered < deltaT:
                time_covered =time_covered + (time.time() - starting)
                print(self.roverState.nearby)
                if self.roverState.nearby:
                    self.car_controller.stop()
                    time.sleep(1)
                    self.arm_controller.dump_garbage_w1()
                    self.roverState.disposible = self.roverState.disposible + 1
                     # wait for 5 seconds to pickup object. Here the arm code has to be implemented to pickup the object. After that the rover will continue to move forward.
                    self.car_controller.move_forward(deltaT=None, speed=speed)
                self.roverState.nearby = False
                starting = time.time()
                time.sleep(.1)

                print(f"\n\nTime covered: {time_covered:.2f} seconds")
            self.car_controller.stop()
                
            self.imu_manager.reset()
            if i % 2 == 0:
                # self.car_controller.turn_right(speed=speed,angle=65) # for low proced imu, 67 == 90 here lol
                self.car_controller.turn_right(deltaT=1.26,speed=speed)
                self.car_controller.move_forward(deltaT=2*int(speed/.1), speed=speed)
                self.car_controller.turn_right(speed=speed,angle=65)
            else:
                # self.car_controller.turn_left(speed=speed,angle=65)
                self.car_controller.turn_left(deltaT=1.26,speed=speed)
                self.car_controller.move_forward(deltaT=2*int(speed/.1), speed=speed)
                self.car_controller.turn_left(speed=speed,angle=65)

    def cover_area_gps(self, x_direction=2, y_direction=2, speed=0.2):

        self.gps_manager.reset()
        self.imu_manager.reset()
        self.imu_manager.calibrate()

        row_spacing = 1.0
        row_k = .25

        if self.gps_manager.has_fix():

            for row in range(1, math.ceil(y_direction/row_k) + 1):

                # Move along row
                self.gps_manager.reset()
                self.car_controller.move_forward(deltaT=None, speed=speed)

                while True:

                    dist = self.gps_manager.distance_meters()

                    if dist is None:
                        time.sleep(0.2)
                        continue

                    if self.roverState.nearby:
                        self.car_controller.stop()
                        self.arm_controller.dump_garbage_w1()
                        self.car_controller.move_forward(deltaT=None, speed=speed)
                        self.roverState.nearby = False

                    if dist >= x_direction:
                        break

                    time.sleep(0.1)

                self.car_controller.stop()

                if row >= math.ceil(y_direction):
                    break

                # Change lane
                turn = (
                    self.car_controller.turn_left
                    if row % 2 else
                    self.car_controller.turn_right
                )

                turn(speed=speed, angle=90)

                self.gps_manager.reset()
                self.car_controller.move_forward(deltaT=None, speed=speed)

                while True:
                    dist = self.gps_manager.distance_meters()

                    if dist is not None and dist >= row_spacing:
                        break

                    time.sleep(0.1)

                self.car_controller.stop()
                turn(speed=speed, angle=90)

            self.car_controller.stop()

        else:
            print("Gps not found yet. Aborting the whole thing.")


            
