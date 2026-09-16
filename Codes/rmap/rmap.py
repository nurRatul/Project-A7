from logger.logger_manager import LoggerManager
from .interfaces import RoverState, Object
import time
import math


# Camera detection ids routed to each arm routine.
DUMP_W1_IDS = {1, 2, 5}
DUMP_W2_IDS = {3, 4}


def _extract_detection_id(detection):
    """
    Best-effort extraction of a class/id value from a detection entry.
    Works whether detections come back as dicts (e.g. {"id": 3, ...})
    or as objects/dataclasses (e.g. Object(id=3, ...)).
    """
    if detection is None:
        return None
    if isinstance(detection, dict):
        return detection.get("id", detection.get("class_id"))
    return getattr(detection, "id", getattr(detection, "class_id", None))


class mappingManager:
    def __init__(self, roverState: RoverState, gps_manager=None, imu_manager=None, ultrasonic_manager=None, car_controller=None, arm_controller=None, gps_port="/dev/ttyAMA0", gps_baudrate=9600, gps_timeout=1.0):
        self.roverState = roverState
        self.gps_manager = gps_manager
        self.imu_manager = imu_manager
        self.ultrasonic_manager = ultrasonic_manager
        self.car_controller = car_controller
        self.arm_controller = arm_controller

    def _select_dump_method(self):
        """
        Look at what the vision system currently sees and pick the arm
        routine that handles it:
          - detection id in {1, 2, 5} -> dump_garbage_w1
          - detection id in {3, 4}    -> dump_garbage_w2
        Falls back to dump_garbage_w1 if nothing recognizable is found, so
        the rover doesn't stall on an unclassified object.
        """
        for detection in self.roverState.detected_objects:
            detection_id = _extract_detection_id(detection)
            if detection_id in DUMP_W1_IDS:
                return self.arm_controller.dump_garbage_w1
            if detection_id in DUMP_W2_IDS:
                return self.arm_controller.dump_garbage_w2

        print("No recognized detection id nearby; defaulting to dump_garbage_w1")
        return self.arm_controller.dump_garbage_w1

    def _dump_and_record(self):
        """
        Runs whichever arm routine matches the currently detected object,
        counts it, and records the pickup time so the rover switches to the
        tighter 50mm 'nearby' threshold for the next 5 seconds.
        """
        dump_method = self._select_dump_method()
        dump_method()
        self.roverState.disposible = self.roverState.disposible + 1
        self.roverState.mark_picked_up()

    def cover_area_nogps(self, x_direction=2, y_direction=2, speed=0.2, object_distance=None):
        row_spacing = .25
        self.roverState.requested_object_distance = object_distance
        for i in range(1, y_direction):
            k = 1.36 #correction factor
            deltaT = x_direction / (speed * k)
            print(f"\rMoving forward for {deltaT} seconds at speed {speed}")
            time_covered = 0
            starting = time.time()
            self.car_controller.move_forward(deltaT=None, speed=speed)
            while time_covered < deltaT:
                time_covered =time_covered + (time.time() - starting)
                starting = time.time()
                time.sleep(.1)
                print(self.roverState.nearby)
                if self.roverState.nearby:
                    self.car_controller.stop()
                    time.sleep(1)
                    self._dump_and_record()
                     # wait for 5 seconds to pickup object. Here the arm code has to be implemented to pickup the object. After that the rover will continue to move forward.
                    self.car_controller.move_forward(deltaT=None, speed=speed)
                self.roverState.nearby = False

                print(f"\n\nTime covered: {time_covered:.2f} seconds")
            self.car_controller.stop()
                

            if i % 2 == 0:
                self.car_controller.turn_right(speed=speed,angle=90)
                self.car_controller.move_forward(deltaT=1, speed=speed)
                self.car_controller.turn_right(speed=speed,angle=90)
            else:
                self.car_controller.turn_left(speed=speed,angle=90)
                self.car_controller.move_forward(deltaT=1, speed=speed)
                self.car_controller.turn_left(speed=speed,angle=90)

            # hi

    def cover_area_gps(self, x_direction=2, y_direction=2, speed=0.2, object_distance=None):

        self.gps_manager.reset()
        self.imu_manager.reset()
        self.imu_manager.calibrate()

        self.roverState.requested_object_distance = object_distance

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
                        self._dump_and_record()
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
