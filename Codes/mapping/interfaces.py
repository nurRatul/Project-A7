# interfaces.py

"""
interfaces.py

Structural (duck-typed) contracts that MappingManager depends on,
expressed as typing.Protocol classes instead of concrete base classes.

Why Protocols and not ABCs:
    MappingManager never constructs any of these — every dependency is
    injected (see mappingManager.py). Protocols let real hardware
    classes (GPSManager, IMUManager, ...) satisfy the contract just by
    having the right methods, with zero coupling and no need for them
    to inherit from anything here. The same protocols let a test suite
    or a simulator hand MappingManager a fake object with the same
    shape and have everything just work — that's what "reusable for
    simulation and real hardware" (requirement #7) means in practice.
    See mapping/examples/simulated_mapping_demo.py for a full example
    with zero real hardware involved.

Every method signature below was checked against the actual project
files (carController.py, controller.py, gpsManager.py, imuManager.py)
— your existing classes already satisfy these without modification.

If you add a new capability MappingManager needs from a sensor or
controller, add it to the matching Protocol here FIRST, then implement
it on the real manager class — that keeps this file as the single
source of truth for "what MappingManager is allowed to assume."
"""

from typing import Optional, Protocol, runtime_checkable


@runtime_checkable
class GPSLike(Protocol):
    def get_location(self) -> Optional[tuple]: ...
    def get_telemetry(self, raw: bool = False) -> dict: ...
    def has_fix(self) -> bool: ...


@runtime_checkable
class IMULike(Protocol):
    def get_orientation(self) -> dict: ...
    def get_telemetry(self) -> dict: ...


@runtime_checkable
class UltrasonicLike(Protocol):
    def get_telemetry(self) -> dict: ...
    def has_obstacle(self, direction: str = "front", threshold_m: Optional[float] = None) -> bool: ...


@runtime_checkable
class VisionLike(Protocol):
    def detect(self, frame=None) -> list: ...
    def read_frame(self): ...


@runtime_checkable
class CarLike(Protocol):
    def drive(self, left_speed: float, right_speed: float) -> None: ...
    def stop(self) -> None: ...
    def turn_left(self, deltaT=None, speed: float = 1.0, angle: Optional[float] = None) -> None: ...
    def turn_right(self, deltaT=None, speed: float = 1.0, angle: Optional[float] = None) -> None: ...
    def move_forward(self, deltaT=None, speed: float = 1.0) -> None: ...


@runtime_checkable
class ArmLike(Protocol):
    def update(self, frame=None, detections=None): ...
