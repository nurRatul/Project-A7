# obstacleAvoidance.py

"""
obstacleAvoidance.py

Side-pass local avoidance: when the front ultrasonic sensor trips
during mapping, this decides a short rectangular detour — turn toward
whichever side has more clearance, advance past the obstacle, turn
back parallel to the original heading, advance alongside it, then turn
back onto the original heading — and hands MappingManager a small,
ordered list of ManeuverStep objects to execute. This module only
PLANS the maneuver; it never touches CarController itself, so it's
unit-testable with plain numbers and reusable in simulation.

    original heading ----X (obstacle)---->
                     |                    ^
                     v  (side-pass detour)|
                     +------------------->+

All turns below are RELATIVE (turn_left/turn_right by N degrees from
wherever the rover currently faces), which is why the away/back/back/
away pattern produces a clean rectangle and returns the rover to its
original heading regardless of what that heading was in the global
frame — no absolute compass math needed here.

Assumptions / limits (see ARCHITECTURE.md "Edge Cases" for the full list):
  * Only handles ONE obstacle at a time — if a second obstacle appears
    mid-detour, MappingManager treats that as a fresh AVOIDING_OBSTACLE
    cycle (stop, re-plan) rather than composing plans.
  * clearance_m / pass_distance_m are fixed pass distances, not derived
    from the obstacle's actual measured size (ultrasonic gives
    distance, not shape) — tune them to comfortably exceed your
    rover's width plus the largest obstacle you expect.
  * Picks a side using ONLY the current left/right ultrasonic snapshot
    at the moment the front sensor tripped. If that side is a poor
    read (None, or left/right are within a few cm of each other), it
    falls back to default_side rather than guessing, and flags the
    plan as low_confidence so MappingManager can log or move more
    cautiously.
"""

from dataclasses import dataclass
from enum import Enum
from typing import List

from .mapData import Obstacle


class ManeuverAction(str, Enum):
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    FORWARD = "forward"


@dataclass
class ManeuverStep:
    action: ManeuverAction
    amount: float   # degrees for turns, meters for forward


@dataclass
class AvoidancePlan:
    steps: List[ManeuverStep]
    obstacle: Obstacle
    side_chosen: str
    low_confidence: bool = False


class SidePassPlanner:
    def __init__(self, clearance_m=0.6, pass_distance_m=0.8, default_side="right"):
        """
        clearance_m: how far sideways to move before treating the
            obstacle as cleared. Should exceed rover half-width plus a
            safety margin.
        pass_distance_m: how far to drive forward while alongside the
            obstacle before turning back onto the original heading —
            should exceed the obstacle's expected footprint. This is a
            fixed heuristic, not adaptive to how far the interrupted
            waypoint actually was (see edge cases).
        default_side: which way to detour when left/right readings are
            both unusable.
        """
        if default_side not in ("left", "right"):
            raise ValueError("default_side must be 'left' or 'right'")
        if clearance_m <= 0 or pass_distance_m <= 0:
            raise ValueError("clearance_m and pass_distance_m must be > 0")

        self.clearance_m = clearance_m
        self.pass_distance_m = pass_distance_m
        self.default_side = default_side

    def plan(self, pose, front_distance_m, left_distance_m, right_distance_m):
        """
        pose: current Pose (used only to tag the recorded Obstacle with
            a location).
        Returns an AvoidancePlan. Distances are meters, None if a
        reading isn't available.
        """
        obstacle = Obstacle(
            x_m=pose.x_m,
            y_m=pose.y_m,
            source="ultrasonic_front",
            distance_m=front_distance_m if front_distance_m is not None else -1.0,
        )

        side, low_confidence = self._choose_side(left_distance_m, right_distance_m)

        # Relative to CURRENT heading, not a global compass direction.
        away = ManeuverAction.TURN_RIGHT if side == "right" else ManeuverAction.TURN_LEFT
        back = ManeuverAction.TURN_LEFT if side == "right" else ManeuverAction.TURN_RIGHT

        steps = [
            ManeuverStep(away, 90.0),
            ManeuverStep(ManeuverAction.FORWARD, self.clearance_m),
            ManeuverStep(back, 90.0),
            ManeuverStep(ManeuverAction.FORWARD, self.pass_distance_m),
            ManeuverStep(back, 90.0),
            ManeuverStep(ManeuverAction.FORWARD, self.clearance_m),
            ManeuverStep(away, 90.0),
        ]

        return AvoidancePlan(steps=steps, obstacle=obstacle, side_chosen=side, low_confidence=low_confidence)

    def _choose_side(self, left_distance_m, right_distance_m):
        if left_distance_m is None and right_distance_m is None:
            return self.default_side, True
        if left_distance_m is None:
            return "right", True
        if right_distance_m is None:
            return "left", True
        if abs(left_distance_m - right_distance_m) < 0.05:
            return self.default_side, True
        return ("left" if left_distance_m > right_distance_m else "right"), False
