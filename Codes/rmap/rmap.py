from logger.logger_manager import LoggerManager
from .interfaces import RoverState, Object, get_disposal_method
import time
import math




class mappingManager:
    def __init__(self, roverState: RoverState, gps_manager=None, imu_manager=None, ultrasonic_manager=None, car_controller=None, arm_controller=None, gps_port="/dev/ttyAMA0", gps_baudrate=9600, gps_timeout=1.0, pickup_distance_mm=60, max_creep_time=3.0):
        self.roverState = roverState
        self.gps_manager = gps_manager
        self.imu_manager = imu_manager
        self.ultrasonic_manager = ultrasonic_manager
        self.car_controller = car_controller
        self.arm_controller = arm_controller
        # `nearby` (set by the telemetry loop, e.g. at 250mm) only means
        # "close enough that the rover needed to start braking" - it's the
        # stopping-distance margin, not the arm's actual reach. Once stopped,
        # we creep forward slowly and re-check the ultrasonic reading until
        # we're actually within pickup_distance_mm of the object before
        # calling the arm. This is what was missing before: the arm was
        # being triggered directly off the 250mm braking threshold, so it
        # kept "trying" on an object that was still ~250mm away.
        self.pickup_distance_mm = pickup_distance_mm
        self.max_creep_time = max_creep_time

    def _dispatch_arm(self, object_id):
        """Call the correct ArmController method for the given object id.
        Returns True if something was dispatched, False if we have no
        mapping for this id (caller should not count it as collected)."""
        method_name = get_disposal_method(object_id)
        if method_name is None:
            print(f"No disposal method mapped for object id {object_id}; skipping.")
            return False
        getattr(self.arm_controller, method_name)()
        return True

    def _approach_and_collect(self, speed):
        """Creep forward from the braking-distance stop until the ultrasonic
        reading is actually inside pickup_distance_mm, or we time out /
        never get closer. Returns True if the object is within reach."""
        creep_speed = min(speed, 0.08)
        self.car_controller.move_forward(deltaT=None, speed=creep_speed)

        reached = False
        start = time.time()
        while time.time() - start < self.max_creep_time:
            distance = self.ultrasonic_manager.read_mm()
            if distance is not None and distance <= self.pickup_distance_mm:
                reached = True
                break
            time.sleep(0.05)

        self.car_controller.stop()
        return reached

    def _handle_nearby_object(self, speed):
        """Full stop -> creep -> dispatch -> resume sequence, shared by both
        coverage patterns. Consumes roverState.nearby and current_object so
        the same physical object can't immediately re-trigger a pickup."""
        self.car_controller.stop()
        self.roverState.nearby = False  # consume this edge now

        current = self.roverState.current_object
        reached = self._approach_and_collect(speed)

        if reached and current.id != -1:
            if self._dispatch_arm(current.id):
                current.pickedUp = True
                self.roverState.disposible = self.roverState.disposible + 1
            # rover.py's vision loop archives + resets current_object once
            # it sees pickedUp == True, freeing the slot for the next one.
        else:
            print("Couldn't close the distance to the object (or nothing latched) - resuming.")

        self.car_controller.move_forward(deltaT=None, speed=speed)

    def cover_area_nogps(self, x_direction=2, y_direction=2, speed=0.2):
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
                    self._handle_nearby_object(speed)
                starting = time.time()
                time.sleep(.1)
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
                        self._handle_nearby_object(speed)

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


            