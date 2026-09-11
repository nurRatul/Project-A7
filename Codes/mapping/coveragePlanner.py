# coveragePlanner.py

"""
coveragePlanner.py

Generates a boustrophedon ("lawnmower") coverage path: parallel lanes
alternating direction, covering a width_m x height_m rectangle
starting from local (0, 0).

    ------->
            |
    <-------
    |
    ------->
            |
    <-------

This produces WAYPOINTS, not motor commands — MappingManager is
responsible for driving to each waypoint and reacting to obstacles or
objects along the way. Kept separate from MappingManager so the
planner is independently testable and swappable (e.g. for a future
SLAM-driven frontier explorer — see ARCHITECTURE.md).

Note on how this produces the lawnmower shape without any special
casing: each lane is a dense run of waypoints at a fixed y (or fixed
x, depending on orientation), and MappingManager's navigation always
just "turn to face the next waypoint, then drive to it." Within a
lane, consecutive waypoints differ only in x, so the natural heading
comes out ~90 degrees from the lane-to-lane heading — the shape falls
out of the waypoint geometry, not from any turn/straight distinction
MappingManager has to reason about.
"""

from dataclasses import dataclass


@dataclass
class Waypoint:
    x_m: float
    y_m: float
    lane_index: int
    is_lane_end: bool = False


class BoustrophedonPlanner:
    def __init__(self, width_m, height_m, lane_spacing_m=1.0, waypoint_spacing_m=0.5):
        """
        lane_spacing_m: distance between adjacent lanes. Set this from
            your effective sensor coverage width (ultrasonic cone +
            camera FOV at cruising height) so adjacent lanes actually
            overlap enough not to miss anything between them — this is
            NOT automatically derived from the sensors here, because
            how much overlap margin you want is a judgment call about
            how much you trust your sensors, not a fixed geometry fact.
        waypoint_spacing_m: how finely each lane is broken into
            waypoints. Smaller = smoother path + more frequent
            obstacle/GPS checks; larger = fewer waypoints + coarser
            coverage tracking.
        """
        if width_m <= 0 or height_m <= 0:
            raise ValueError("width_m and height_m must be > 0")
        if lane_spacing_m <= 0 or waypoint_spacing_m <= 0:
            raise ValueError("lane_spacing_m and waypoint_spacing_m must be > 0")

        self.width_m = width_m
        self.height_m = height_m
        self.lane_spacing_m = lane_spacing_m
        self.waypoint_spacing_m = waypoint_spacing_m

    def generate(self):
        """Returns an ordered list[Waypoint] covering the full rectangle."""
        waypoints = []

        y = 0.0
        lane_index = 0
        going_right = True

        while y <= self.height_m + 1e-9:
            xs = self._lane_xs()
            if not going_right:
                xs = list(reversed(xs))

            for i, x in enumerate(xs):
                waypoints.append(
                    Waypoint(x_m=x, y_m=y, lane_index=lane_index, is_lane_end=(i == len(xs) - 1))
                )

            y += self.lane_spacing_m
            lane_index += 1
            going_right = not going_right

        return waypoints

    def _lane_xs(self):
        xs = []
        x = 0.0
        while x <= self.width_m + 1e-9:
            xs.append(min(x, self.width_m))
            x += self.waypoint_spacing_m
        if xs[-1] != self.width_m:
            xs.append(self.width_m)
        return xs
