# mappingManager.py

"""
mappingManager.py

Orchestrates one autonomous mapping mission: boustrophedon coverage of
a width_m x height_m rectangle, ultrasonic side-pass avoidance, and
configurable object handling (ignore / avoid / pickup), all recorded
into a RoverMap.

Architectural rules this class exists to respect (see ARCHITECTURE.md):
    * Every dependency (car, arm, gps, imu, ultrasonic, vision) is
      constructor-injected — MappingManager never imports gpiozero,
      cv2, serial, or smbus2, and never constructs a *Manager itself.
      That's what makes it swappable for simulation: hand it fakes
      that satisfy interfaces.py's Protocols and nothing else changes.
    * MappingManager never touches a GPIO pin, camera, or serial port
      directly — every hardware action goes through car.* / arm.*,
      and every reading comes from a *Manager's public methods.
    * ArmController's internals are not modified or reached into
      beyond its existing public update() method — pickup behavior
      lives in ArmController; MappingManager only decides WHEN to
      call it.

Driving loop model:
    MappingManager does not run its own background thread. Call
    start_mapping(...) once (it blocks briefly for sensor warm-up and,
    if require_gps_fix, a GPS fix), then call tick() repeatedly from
    whatever loop already owns the robot's timing — a bare while-loop,
    a Flask background thread, or alongside CarController/
    ArmController's own per-frame update(). run_to_completion() is a
    convenience wrapper around that loop for simple standalone scripts.

    tick() is not guaranteed to return quickly: during MAPPING it may
    complete a bounded (~waypoint_spacing_m) forward move before
    returning, checking for obstacles at ~20 Hz internally throughout
    that move — see _drive_forward_m.
"""

import math
import time

from logger.logger_manager import LoggerManager

from .coveragePlanner import BoustrophedonPlanner
from .interfaces import ArmLike, CarLike, GPSLike, IMULike, UltrasonicLike, VisionLike
from .localization import PositionEstimator
from .mapData import DetectedObject, ObjectAction, RoverMap
from .obstacleAvoidance import ManeuverAction, SidePassPlanner
from .objectPolicy import PolicyContext, get_policy
from .stateMachine import MappingState, MappingStateMachine

logger = LoggerManager.get_logger("mapping.mappingManager")


DEFAULT_CRUISE_SPEED = 0.5              # 0..1 duty cycle passed to CarController
DEFAULT_ESTIMATED_SPEED_MPS = 0.3       # REQUIRES CALIBRATION — see ARCHITECTURE.md
DEFAULT_OBSTACLE_THRESHOLD_M = 0.35
DEFAULT_WAYPOINT_TOLERANCE_M = 0.15
DEFAULT_HEADING_TOLERANCE_DEG = 5.0


class MappingManager:
    def __init__(
        self,
        car: CarLike,
        arm: ArmLike,
        gps: GPSLike,
        imu: IMULike,
        ultrasonic: UltrasonicLike,
        vision: VisionLike,
        object_policy="ignore",
        cruise_speed=DEFAULT_CRUISE_SPEED,
        estimated_speed_mps=DEFAULT_ESTIMATED_SPEED_MPS,
        obstacle_threshold_m=DEFAULT_OBSTACLE_THRESHOLD_M,
        lane_spacing_m=1.0,
        waypoint_spacing_m=0.5,
        cell_size_m=0.5,
        side_pass_clearance_m=0.6,
        side_pass_distance_m=0.8,
        pickup_timeout_s=20.0,
        require_gps_fix=True,
    ):
        self.car = car
        self.arm = arm
        self.gps = gps
        self.imu = imu
        self.ultrasonic = ultrasonic
        self.vision = vision
        self._validate_dependencies()

        self.object_policy_name = object_policy
        self.object_policy = get_policy(object_policy)

        self.cruise_speed = cruise_speed
        self.estimated_speed_mps = estimated_speed_mps
        self.obstacle_threshold_m = obstacle_threshold_m
        self.lane_spacing_m = lane_spacing_m
        self.waypoint_spacing_m = waypoint_spacing_m
        self.cell_size_m = cell_size_m
        self.side_pass_clearance_m = side_pass_clearance_m
        self.side_pass_distance_m = side_pass_distance_m
        self.pickup_timeout_s = pickup_timeout_s
        self.require_gps_fix = require_gps_fix

        self.localization = PositionEstimator(gps, imu)
        self.state_machine = MappingStateMachine(on_transition=self._on_transition)

        self.rover_map = None
        self.plan = []
        self._waypoint_index = 0

        self._avoidance_plan = None
        self._avoidance_step_index = 0

        self._pickup_started_at = None

    def _validate_dependencies(self):
        checks = [
            (self.car, CarLike, "car"), (self.arm, ArmLike, "arm"),
            (self.gps, GPSLike, "gps"), (self.imu, IMULike, "imu"),
            (self.ultrasonic, UltrasonicLike, "ultrasonic"), (self.vision, VisionLike, "vision"),
        ]
        for obj, protocol, name in checks:
            if not isinstance(obj, protocol):
                raise TypeError(
                    f"{name}={obj!r} does not implement the {protocol.__name__} interface "
                    f"(see mapping/interfaces.py)"
                )

    # ====================================================================
    # PUBLIC: MISSION CONTROL
    # ====================================================================

    def start_mapping(self, width_m, height_m):
        """
        IDLE -> INITIALIZING -> MAPPING. Blocks briefly for sensor
        warm-up and (if require_gps_fix) a GPS fix, then builds the
        RoverMap + coverage plan and leaves the rover ready for tick().
        """
        if width_m <= 0 or height_m <= 0:
            raise ValueError("width_m and height_m must be > 0")

        self.state_machine.transition(MappingState.INITIALIZING)

        try:
            self.ultrasonic.get_telemetry()  # cheap reachability check
            if self.require_gps_fix:
                self.localization.wait_for_fix()
            self.localization.set_origin()

            origin = None
            if self.localization.origin_lat is not None:
                origin = (self.localization.origin_lat, self.localization.origin_lon)

            self.rover_map = RoverMap(width_m, height_m, cell_size_m=self.cell_size_m, origin=origin)
            self.plan = BoustrophedonPlanner(
                width_m, height_m,
                lane_spacing_m=self.lane_spacing_m,
                waypoint_spacing_m=self.waypoint_spacing_m,
            ).generate()
            self._waypoint_index = 0

            self.rover_map.record_pose(self.localization.pose())

        except Exception:
            logger.exception("start_mapping() failed during INITIALIZING")
            self.state_machine.transition(MappingState.ERROR)
            raise

        self.state_machine.transition(MappingState.MAPPING)

    def tick(self):
        """Run ONE step of whatever the current state needs. Call this
        repeatedly from your main loop. Returns the state after this
        tick, so a caller can do `while mapper.tick() not in (...)`."""
        state = self.state_machine.state
        handler = {
            MappingState.MAPPING: self._tick_mapping,
            MappingState.AVOIDING_OBSTACLE: self._tick_avoiding_obstacle,
            MappingState.PICKING_OBJECT: self._tick_picking_object,
            MappingState.RETURNING_TO_PATH: self._tick_returning_to_path,
        }.get(state)

        if handler is not None:
            try:
                handler()
            except Exception:
                logger.exception("tick() failed in state %s", state)
                self.car.stop()
                self.state_machine.transition(MappingState.ERROR)

        return self.state_machine.state

    def run_to_completion(self, rate_hz=10):
        """Convenience blocking loop for simple scripts/tests — ticks
        until COMPLETED or ERROR."""
        period = 1.0 / rate_hz
        while self.state_machine.state not in (MappingState.COMPLETED, MappingState.ERROR):
            started = time.time()
            self.tick()
            elapsed = time.time() - started
            time.sleep(max(0.0, period - elapsed))
        return self.state_machine.state

    def stop_mapping(self):
        """User-triggered abort from ANY in-progress state -> IDLE."""
        self.car.stop()
        if self.state_machine.state != MappingState.IDLE:
            self.state_machine.transition(MappingState.IDLE)

    def save_map(self, path):
        if self.rover_map is None:
            raise RuntimeError("No map to save — start_mapping() first")
        self.rover_map.save(path)

    def reset(self):
        """COMPLETED/ERROR -> IDLE, ready for another start_mapping().
        Assumes the car is already stopped (true in both source
        states) — use stop_mapping() instead to abort an active mission."""
        self.state_machine.transition(MappingState.IDLE)

    @property
    def state(self):
        return self.state_machine.state

    @property
    def coverage_ratio(self):
        return self.rover_map.coverage_ratio() if self.rover_map else 0.0

    # ====================================================================
    # STATE HANDLERS
    # ====================================================================

    def _tick_mapping(self):
        pose = self.localization.update(commanded_speed_mps=0.0)

        if self.ultrasonic.has_obstacle("front", self.obstacle_threshold_m):
            self.car.stop()
            self._begin_avoidance(pose)
            return

        detection = self._relevant_detection(pose)
        if detection is not None:
            self.car.stop()
            self._begin_object_handling(detection, pose)
            return

        if self._waypoint_index >= len(self.plan):
            self.car.stop()
            self.state_machine.transition(MappingState.COMPLETED)
            return

        self._navigate_to(self.plan[self._waypoint_index], pose)

    def _tick_avoiding_obstacle(self):
        if self._avoidance_plan is None or self._avoidance_step_index >= len(self._avoidance_plan.steps):
            self._avoidance_plan = None
            self._avoidance_step_index = 0
            self.state_machine.transition(MappingState.RETURNING_TO_PATH)
            return

        step = self._avoidance_plan.steps[self._avoidance_step_index]
        self._execute_maneuver_step(step)
        self._avoidance_step_index += 1

    def _tick_returning_to_path(self):
        # Local avoidance is done. Because BoustrophedonPlanner's
        # waypoints are absolute positions (not relative "turn X,
        # forward Y" moves), simply resuming normal MAPPING navigation
        # toward the same _waypoint_index re-homes the rover onto the
        # path — no special rejoin geometry needed here.
        self.state_machine.transition(MappingState.MAPPING)

    def _tick_picking_object(self):
        """
        IMPORTANT — see ARCHITECTURE.md "Object pickup workflow":
        Arm/controller.py's Controller.update() only computes target
        joint angles via inverse kinematics; it does NOT drive servos
        or the gripper (that logic lives outside the provided files,
        in whatever script currently reads controller.angles). So
        `angles is not None` below means "the arm knows where to
        move," not "the arm has physically grabbed the object" — this
        is a placeholder until ArmController exposes a real
        pick_up_object() -> bool that also drives the servos/gripper
        and reports genuine success or failure.
        """
        if self._pickup_started_at is None:
            self._pickup_started_at = time.time()

        if time.time() - self._pickup_started_at > self.pickup_timeout_s:
            logger.warning("Pickup timed out — abandoning object and resuming mapping")
            self._finish_pickup(ObjectAction.PICKUP_FAILED)
            return

        angles = self.arm.update()
        if angles is not None:
            self._finish_pickup(ObjectAction.PICKED_UP)

    # ====================================================================
    # HELPERS
    # ====================================================================

    def _on_transition(self, old_state, new_state):
        # Hook for future telemetry/logging integrations (e.g. pushing
        # state changes to serverController's Flask app over a queue).
        pass

    def _relevant_detection(self, pose):
        if self.object_policy_name == "ignore":
            return None
        detections = self.vision.detect()
        if not detections:
            return None
        detection = detections[0]
        if self._recently_handled(detection, pose):
            return None
        return detection

    def _recently_handled(self, detection, pose, radius_m=0.5):
        """Lightweight dedup heuristic — NOT real object re-identification
        (see ARCHITECTURE.md edge cases). Prevents immediately
        re-triggering avoid/pickup on the same still-visible object
        right after handling it."""
        if not self.rover_map or not self.rover_map.objects:
            return False
        for obj in reversed(self.rover_map.objects[-5:]):
            if obj.class_name == detection["class_name"]:
                if math.hypot(obj.x_m - pose.x_m, obj.y_m - pose.y_m) < radius_m:
                    return True
        return False

    def _begin_avoidance(self, pose):
        front = self.ultrasonic.get_telemetry()
        planner = SidePassPlanner(clearance_m=self.side_pass_clearance_m, pass_distance_m=self.side_pass_distance_m)
        self._avoidance_plan = planner.plan(
            pose,
            front_distance_m=front.get("front_m"),
            left_distance_m=front.get("left_m"),
            right_distance_m=front.get("right_m"),
        )
        self._avoidance_step_index = 0
        self.rover_map.record_obstacle(self._avoidance_plan.obstacle)
        self.state_machine.transition(MappingState.AVOIDING_OBSTACLE)

    def _begin_object_handling(self, detection, pose):
        result = self.object_policy.handle(
            detection,
            PolicyContext(car=self.car, arm=self.arm, rover_map=self.rover_map, pose=pose),
        )

        detected_object = DetectedObject(
            x_m=pose.x_m,
            y_m=pose.y_m,
            class_name=detection["class_name"],
            confidence=detection["confidence"],
            policy=self.object_policy_name,
            action_taken=result.action_taken,
        )
        self.rover_map.record_object(detected_object)

        if result.next_state == MappingState.AVOIDING_OBSTACLE:
            # AvoidPolicy reuses the side-pass maneuver, built from
            # CURRENT ultrasonic readings the same way a real obstacle
            # would be — vision doesn't give us a distance to plan from.
            self._begin_avoidance(pose)
        elif result.next_state == MappingState.PICKING_OBJECT:
            self._pickup_started_at = None
            self.state_machine.transition(MappingState.PICKING_OBJECT)
        # IgnorePolicy returns next_state=None -> stay in MAPPING, loop continues next tick()

    def _finish_pickup(self, action_taken):
        self._pickup_started_at = None
        if self.rover_map.objects:
            self.rover_map.objects[-1].action_taken = action_taken
        self.state_machine.transition(MappingState.RETURNING_TO_PATH)

    def _navigate_to(self, waypoint, pose):
        target_heading = math.degrees(math.atan2(waypoint.x_m - pose.x_m, waypoint.y_m - pose.y_m))
        heading_error = self._angle_diff(target_heading, pose.heading_deg)

        if abs(heading_error) > DEFAULT_HEADING_TOLERANCE_DEG:
            if heading_error > 0:
                self.car.turn_right(speed=self.cruise_speed, angle=abs(heading_error))
            else:
                self.car.turn_left(speed=self.cruise_speed, angle=abs(heading_error))
            self.localization.update(commanded_speed_mps=0.0)  # refresh heading after the turn
            return

        distance = math.hypot(waypoint.x_m - pose.x_m, waypoint.y_m - pose.y_m)
        if distance <= DEFAULT_WAYPOINT_TOLERANCE_M:
            self.rover_map.record_pose(pose)
            self._waypoint_index += 1
            return

        self._drive_forward_m(min(distance, self.waypoint_spacing_m))
        self.rover_map.record_pose(self.localization.pose())

    def _drive_forward_m(self, distance_m, cruise_speed=None):
        """
        Distance-based forward drive built on top of CarController's
        existing time-based drive() plus this class's own GPS/IMU
        localization — CarController itself is not modified (it only
        exposes time- and angle-based movement). Polls at ~20 Hz so an
        obstacle can interrupt a forward move quickly, not just between
        tick() calls.
        """
        cruise_speed = cruise_speed if cruise_speed is not None else self.cruise_speed
        start_pose = self.localization.pose()
        self.car.drive(cruise_speed, cruise_speed)
        traveled = 0.0
        while traveled < distance_m:
            time.sleep(0.05)
            pose = self.localization.update(commanded_speed_mps=self.estimated_speed_mps)
            traveled = math.hypot(pose.x_m - start_pose.x_m, pose.y_m - start_pose.y_m)
            if self.ultrasonic.has_obstacle("front", self.obstacle_threshold_m):
                break
        self.car.stop()

    def _execute_maneuver_step(self, step):
        if step.action == ManeuverAction.TURN_LEFT:
            self.car.turn_left(speed=self.cruise_speed, angle=step.amount)
            self.localization.update(commanded_speed_mps=0.0)
        elif step.action == ManeuverAction.TURN_RIGHT:
            self.car.turn_right(speed=self.cruise_speed, angle=step.amount)
            self.localization.update(commanded_speed_mps=0.0)
        elif step.action == ManeuverAction.FORWARD:
            self._drive_forward_m(step.amount)

    @staticmethod
    def _angle_diff(target_deg, current_deg):
        return (target_deg - current_deg + 180) % 360 - 180
