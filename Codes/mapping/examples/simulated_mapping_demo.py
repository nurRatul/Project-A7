# simulated_mapping_demo.py

"""
simulated_mapping_demo.py

A complete, runnable example of MappingManager driven entirely by
fakes — no Raspberry Pi, no GPIO, no camera required. Demonstrates:
  * dependency injection (rule 4: MappingManager never constructs a
    sensor or controller itself)
  * that MappingManager only needs objects satisfying interfaces.py's
    Protocols, not real *Manager instances (rule 6: reusable for
    simulation and real hardware)

Run it from your project root (next to your existing logger/ package):
    python3 -m mapping.examples.simulated_mapping_demo

This is a logic/integration smoke test, not a physics simulator — the
fake car doesn't model wheel slip or acceleration. It exists to prove
the state machine, coverage planner, and obstacle handling all wire
together correctly, not to validate real-world driving dynamics.
"""

import math
import time

from mapping import MappingManager


class FakeGPS:
    """Provides one initial fix (enough for wait_for_fix()/set_origin() to
    succeed), then reports no fix — simulating the 'testing indoors'
    scenario mpu6050.py's docstring already warns about. MappingManager
    should keep making progress via dead reckoning; coverage_ratio
    climbing confirms it."""

    ORIGIN_LAT = 23.8103
    ORIGIN_LON = 90.4125

    def __init__(self):
        self._fix_count = 0

    def has_fix(self):
        return True

    def get_location(self):
        return (self.ORIGIN_LAT, self.ORIGIN_LON)

    def get_telemetry(self, raw=False):
        self._fix_count += 1
        has_fix = self._fix_count <= 1   # only the very first read looks like a fix
        return {
            "latitude": self.ORIGIN_LAT if has_fix else None,
            "longitude": self.ORIGIN_LON if has_fix else None,
            "has_fix": has_fix,
            "speed_kmh": 0.0,
            "last_update": time.time(),
        }


class FakeIMU:
    def __init__(self):
        self.yaw = 0.0

    def get_orientation(self):
        return {"roll": 0.0, "pitch": 0.0, "yaw": self.yaw}

    def get_telemetry(self):
        return {"orientation_deg": self.get_orientation()}


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


class FakeCar:
    """No physics engine — turns update the injected FakeIMU's yaw
    immediately (matching CarController.turn_left/turn_right's
    contract: block until rotated by `angle` degrees). Straight driving
    doesn't need to touch position at all: PositionEstimator's own
    dead reckoning advances x_m/y_m using the commanded_speed_mps
    MappingManager passes it directly, not anything read back from
    this fake car — see localization.py."""

    def __init__(self, imu):
        self.imu = imu
        self.stop_count = 0

    def drive(self, left_speed, right_speed):
        pass

    def stop(self):
        self.stop_count += 1

    def move_forward(self, deltaT=None, speed=1.0):
        pass

    def turn_left(self, deltaT=None, speed=1.0, angle=None):
        self.imu.yaw -= angle if angle is not None else 0.0

    def turn_right(self, deltaT=None, speed=1.0, angle=None):
        self.imu.yaw += angle if angle is not None else 0.0


def build_mapper(**overrides):
    imu = FakeIMU()
    kwargs = dict(
        car=FakeCar(imu),
        arm=FakeArm(),
        gps=FakeGPS(),
        imu=imu,
        ultrasonic=FakeUltrasonic(),
        vision=FakeVision(),
        cruise_speed=0.6,
        estimated_speed_mps=1.2,     # fast, just to keep the demo short
        waypoint_spacing_m=0.5,
        lane_spacing_m=0.5,
    )
    kwargs.update(overrides)
    return MappingManager(**kwargs)


def demo_basic_coverage():
    print("\n=== demo 1: basic coverage, no obstacles ===")
    mapper = build_mapper()
    mapper.start_mapping(width_m=1.5, height_m=1.0)
    print(f"state after start_mapping: {mapper.state}")

    final_state = mapper.run_to_completion(rate_hz=30)

    print(f"final state: {final_state}")
    print(f"coverage ratio: {mapper.coverage_ratio:.2f}")
    print(f"trail points recorded: {len(mapper.rover_map.trail)}")
    print(f"car.stop() call count: {mapper.car.stop_count}")
    assert final_state == "COMPLETED", "expected the mission to complete"
    assert mapper.coverage_ratio > 0.5, "expected meaningful coverage"
    print("PASSED")


def demo_obstacle_avoidance():
    print("\n=== demo 2: an obstacle appears mid-mission ===")
    mapper = build_mapper()
    mapper.start_mapping(width_m=1.5, height_m=1.0)

    seen_avoiding = False
    seen_returning = False
    ticks = 0
    max_ticks = 2000

    while mapper.state not in ("COMPLETED", "ERROR") and ticks < max_ticks:
        # Once the rover has moved a bit, throw an obstacle in front of it.
        if ticks == 15:
            mapper.ultrasonic.front_m = 0.10
        # Clear it again once avoidance has actually engaged, like a
        # real obstacle sensor would once the rover has turned away.
        if mapper.state == "AVOIDING_OBSTACLE":
            seen_avoiding = True
            mapper.ultrasonic.front_m = 4.0
        if mapper.state == "RETURNING_TO_PATH":
            seen_returning = True

        mapper.tick()
        ticks += 1

    print(f"final state: {mapper.state}")
    print(f"ticks used: {ticks}")
    print(f"entered AVOIDING_OBSTACLE: {seen_avoiding}")
    print(f"entered RETURNING_TO_PATH: {seen_returning}")
    print(f"obstacles recorded on map: {len(mapper.rover_map.obstacles)}")
    assert seen_avoiding, "expected the obstacle to trigger AVOIDING_OBSTACLE"
    assert seen_returning, "expected the detour to end in RETURNING_TO_PATH"
    assert len(mapper.rover_map.obstacles) >= 1
    print("PASSED")


def demo_stop_mapping_from_every_state():
    print("\n=== demo 3: stop_mapping() aborts cleanly from any state ===")
    for target_state, setup in [
        ("MAPPING", lambda m: None),
        ("AVOIDING_OBSTACLE", lambda m: setattr(m.ultrasonic, "front_m", 0.1) or m.tick()),
    ]:
        mapper = build_mapper()
        mapper.start_mapping(width_m=1.0, height_m=1.0)
        setup(mapper)
        assert mapper.state == target_state, f"expected {target_state}, got {mapper.state}"
        mapper.stop_mapping()
        assert mapper.state == "IDLE"
        print(f"  aborted cleanly from {target_state} -> IDLE")
    print("PASSED")


def demo_save_and_reload_map():
    print("\n=== demo 4: map save/load round-trip ===")
    import os
    import tempfile

    mapper = build_mapper()
    mapper.start_mapping(width_m=1.0, height_m=1.0)
    mapper.run_to_completion(rate_hz=30)

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "mission.json")
        mapper.save_map(path)

        from mapping import RoverMap
        reloaded = RoverMap.load(path)

        assert abs(reloaded.coverage_ratio() - mapper.coverage_ratio) < 1e-6
        assert len(reloaded.trail) == len(mapper.rover_map.trail)
        print(f"saved + reloaded map: {len(reloaded.trail)} trail points, "
              f"coverage {reloaded.coverage_ratio():.2f}")
    print("PASSED")


def demo_object_pickup():
    print("\n=== demo 5: object_policy='pickup' end to end ===")

    class DetectOnceVision:
        """Reports one bottle for the first few ticks, then nothing —
        simulating the rover looking away/moving on after handling it."""
        def __init__(self):
            self.calls = 0

        def detect(self, frame=None):
            self.calls += 1
            return [{"class_name": "bottle", "confidence": 0.91}] if self.calls <= 3 else []

        def read_frame(self):
            return None

    class SucceedsSecondCallArm:
        """Returns IK angles on its second update() call — see the
        honest caveat in MappingManager._tick_picking_object() about
        what 'angles is not None' does and doesn't guarantee."""
        def __init__(self):
            self.calls = 0

        def update(self, frame=None, detections=None):
            self.calls += 1
            return [10.0, 20.0, 30.0, 40.0] if self.calls >= 2 else None

    mapper = build_mapper(vision=DetectOnceVision(), arm=SucceedsSecondCallArm(), object_policy="pickup")
    mapper.start_mapping(width_m=1.0, height_m=1.0)

    seen_picking = False
    ticks = 0
    while mapper.state not in ("COMPLETED", "ERROR") and ticks < 500:
        if mapper.state == "PICKING_OBJECT":
            seen_picking = True
        mapper.tick()
        ticks += 1

    print(f"final state: {mapper.state}")
    print(f"entered PICKING_OBJECT: {seen_picking}")
    print(f"objects recorded: {[(o.class_name, o.action_taken.value) for o in mapper.rover_map.objects]}")
    assert seen_picking, "expected the detection to trigger PICKING_OBJECT"
    assert mapper.rover_map.objects[0].action_taken.value == "picked_up"
    print("PASSED")


if __name__ == "__main__":
    demo_basic_coverage()
    demo_obstacle_avoidance()
    demo_stop_mapping_from_every_state()
    demo_save_and_reload_map()
    demo_object_pickup()
    print("\nAll simulated demos passed.")
