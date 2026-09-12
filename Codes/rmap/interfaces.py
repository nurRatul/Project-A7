from dataclasses import dataclass, field

@dataclass
class Object:
    name: str = field(default_factory=lambda: "Unknown")
    id: int = field(default_factory=lambda: -1)
    position: tuple = field(default_factory=lambda: (None, None))
    pickedUp: bool = field(default=False)

@dataclass
class RoverState:
    position: tuple = (None, None)
    object_detected: bool = False
    nearby: bool = False
    orientation: float = None
    gps_fix: bool = False
    imu_orientation: dict = field(default_factory=dict)
    ultrasonic_telemetry: dict = field(default_factory=dict)
    detected_objects: list = field(default_factory=list)
    