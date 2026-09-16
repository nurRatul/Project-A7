from dataclasses import dataclass, field


@dataclass
class Object:
    name: str = field(default_factory=lambda: "Unknown")
    id: int = field(default_factory=lambda: -1)
    confidence: float = field(default_factory=lambda: -1.0)
    position: tuple = field(default_factory=lambda: (None, None))
    pickedUp: bool = field(default=False)


# Maps a detected object's class id to the ArmController method that should
# dispose of it. id == -1 is the "no object latched" sentinel and is
# intentionally not mapped here.
DISPOSAL_METHOD_BY_ID = {
    1: "dump_garbage_w1",
    3: "dump_garbage_w2",
    4: "dump_garbage_w2",
}


def get_disposal_method(object_id: int):
    """Return the ArmController method name for a given object id, or None
    if we have no disposal mapping for it."""
    return DISPOSAL_METHOD_BY_ID.get(object_id)


@dataclass
class RoverState:
    position: tuple = (None, None)
    object_detected: bool = False
    nearby: bool = False
    disposible: int = 0
    nondisposible: int = 0
    orientation: float = None
    gps_fix: bool = False
    imu_orientation: dict = field(default_factory=dict)
    ultrasonic_telemetry: dict = field(default_factory=dict)
    detected_objects: list = field(default_factory=list)
    # The object currently "latched" in RAM: the vision loop fills this in
    # when something is detected, and it stays here (id != -1) until it is
    # picked up. mappingManager reads its id to decide which arm routine to
    # call, and rover.py archives + resets it once pickedUp is True.
    current_object: Object = field(default_factory=Object)
    