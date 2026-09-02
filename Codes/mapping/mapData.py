# mapData.py

"""
mapData.py

Reusable data models for MappingManager's map structure: local grid
cells, obstacles, detected objects, and the overall RoverMap container.
Everything here is a plain dataclass with a to_dict() pair so the
whole map serializes to JSON for file storage or WiFi transmission
(see RoverMap.to_json() / save() / load()).

Coordinate frames used throughout mapping/:
    local (x_m, y_m)   - meters from the mapping origin (0, 0), the
                          rover's position when start_mapping() was
                          called. This is what the coverage planner,
                          obstacle avoidance, and the grid all work in.
    global (lat, lon)  - from GPSManager, stored alongside local
                          coordinates wherever available so the map
                          stays meaningful if the site is revisited
                          later or compared against real-world GPS.
    heading_deg        - compass-style bearing in degrees: 0 = the
                          rover's heading at mapping start, positive =
                          clockwise/right (matches math.atan2(dx, dy),
                          NOT the standard math.atan2(dy, dx)). See
                          localization.py for why, and the calibration
                          note about IMU yaw sign convention.
"""

import json
import math
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Optional


class ObjectAction(str, Enum):
    IGNORED = "ignored"
    AVOIDED = "avoided"
    PICKED_UP = "picked_up"
    PICKUP_FAILED = "pickup_failed"


@dataclass
class Pose:
    """A position + heading at a point in time, in both local and (if available) global frames."""
    x_m: float
    y_m: float
    heading_deg: float
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    timestamp: float = field(default_factory=time.time)

    def to_dict(self):
        return asdict(self)


@dataclass
class Obstacle:
    x_m: float
    y_m: float
    source: str                # "ultrasonic_front" | "ultrasonic_left" | "ultrasonic_right"
    distance_m: float
    timestamp: float = field(default_factory=time.time)

    def to_dict(self):
        return asdict(self)


@dataclass
class DetectedObject:
    x_m: float
    y_m: float
    class_name: str
    confidence: float
    policy: str                 # the object_policy active when this was recorded
    action_taken: ObjectAction = ObjectAction.IGNORED
    timestamp: float = field(default_factory=time.time)

    def to_dict(self):
        data = asdict(self)
        data["action_taken"] = self.action_taken.value
        return data


@dataclass
class MapCell:
    """One cell of the coverage grid."""
    row: int
    col: int
    x_m: float
    y_m: float
    visited: bool = False
    visited_at: Optional[float] = None

    def to_dict(self):
        return asdict(self)


class RoverMap:
    """
    The full map for one mapping mission: a coverage grid plus the
    visited-position trail, obstacles, and detected objects layered on
    top. Pure data + (de)serialization — RoverMap does not know about
    sensors, motors, or the state machine, so it's equally usable in a
    unit test, a simulator, or on the real rover.
    """

    def __init__(self, width_m, height_m, cell_size_m=0.5, origin=None):
        if width_m <= 0 or height_m <= 0 or cell_size_m <= 0:
            raise ValueError("width_m, height_m, and cell_size_m must be > 0")

        self.width_m = width_m
        self.height_m = height_m
        self.cell_size_m = cell_size_m
        self.created_at = time.time()

        # origin = the real-world (lat, lon) local (0, 0) corresponds
        # to, captured once at start_mapping() if GPS has a fix yet.
        self.origin = origin

        self.cols = max(1, int(math.ceil(width_m / cell_size_m)))
        self.rows = max(1, int(math.ceil(height_m / cell_size_m)))

        self.grid = [
            [
                MapCell(row=r, col=c, x_m=c * cell_size_m, y_m=r * cell_size_m)
                for c in range(self.cols)
            ]
            for r in range(self.rows)
        ]

        self.trail = []      # list[Pose]
        self.obstacles = []  # list[Obstacle]
        self.objects = []    # list[DetectedObject]

    # ---- writes ------------------------------------------------------

    def record_pose(self, pose):
        self.trail.append(pose)
        self.mark_visited(pose.x_m, pose.y_m)

    def mark_visited(self, x_m, y_m):
        cell = self.cell_at(x_m, y_m)
        if cell is not None:
            cell.visited = True
            cell.visited_at = time.time()

    def record_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def record_object(self, detected_object):
        self.objects.append(detected_object)

    # ---- reads ---------------------------------------------------------

    def cell_at(self, x_m, y_m):
        col = int(x_m // self.cell_size_m)
        row = int(y_m // self.cell_size_m)
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.grid[row][col]
        return None

    def coverage_ratio(self):
        total = self.rows * self.cols
        if total == 0:
            return 0.0
        visited = sum(1 for row in self.grid for cell in row if cell.visited)
        return visited / total

    def unvisited_cells(self):
        return [cell for row in self.grid for cell in row if not cell.visited]

    # ---- serialization ---------------------------------------------------

    def to_dict(self):
        return {
            "width_m": self.width_m,
            "height_m": self.height_m,
            "cell_size_m": self.cell_size_m,
            "origin": list(self.origin) if self.origin else None,
            "created_at": self.created_at,
            "coverage_ratio": self.coverage_ratio(),
            "grid": [[cell.to_dict() for cell in row] for row in self.grid],
            "trail": [pose.to_dict() for pose in self.trail],
            "obstacles": [o.to_dict() for o in self.obstacles],
            "objects": [o.to_dict() for o in self.objects],
        }

    def to_json(self, indent=2):
        return json.dumps(self.to_dict(), indent=indent)

    def save(self, path):
        with open(path, "w") as f:
            f.write(self.to_json())

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)

        origin = tuple(data["origin"]) if data.get("origin") else None
        rover_map = cls(data["width_m"], data["height_m"], data["cell_size_m"], origin=origin)
        rover_map.created_at = data.get("created_at", rover_map.created_at)

        for row_data, row in zip(data["grid"], rover_map.grid):
            for cell_data, cell in zip(row_data, row):
                cell.visited = cell_data["visited"]
                cell.visited_at = cell_data["visited_at"]

        rover_map.trail = [Pose(**p) for p in data["trail"]]
        rover_map.obstacles = [Obstacle(**o) for o in data["obstacles"]]

        for o in data["objects"]:
            o = dict(o)
            o["action_taken"] = ObjectAction(o["action_taken"])
            rover_map.objects.append(DetectedObject(**o))

        return rover_map
