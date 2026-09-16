from dataclasses import dataclass, field
import time

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
    disposible = 0
    nondisposible = 0
    orientation: float = None
    current_object = None
    gps_fix: bool = False
    imu_orientation: dict = field(default_factory=dict)
    ultrasonic_telemetry: dict = field(default_factory=dict)
    detected_objects: list = field(default_factory=list)

    # --- pickup / distance tracking, used to size the "nearby" threshold ---
    last_pickup_time: float = None          # time.time() of the most recent successful pickup
    requested_object_distance: float = None  # distance (mm) requested by the current mapping run, if any

    def mark_picked_up(self):
        """Call this right after a successful dump_garbage_w1/w2() call."""
        self.last_pickup_time = time.time()

    def recently_picked_up(self, window: float = 5.0) -> bool:
        """True if an object was picked up within the last `window` seconds."""
        if self.last_pickup_time is None:
            return False
        return (time.time() - self.last_pickup_time) < window
