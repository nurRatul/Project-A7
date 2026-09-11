# objectPolicy.py

"""
objectPolicy.py

Strategy classes for MappingManager's object_policy setting
("ignore" | "avoid" | "pickup"). Each policy implements the same
handle(detection, context) interface, so MappingManager's main loop
never branches on the policy name itself — it just calls
self.object_policy.handle(...) and reacts to the returned
PolicyResult.

Adding a new policy later (e.g. "photograph": stop, log an image,
continue) means writing one new class here and registering it in
POLICIES — nothing in MappingManager's loop has to change.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from .mapData import ObjectAction
from .stateMachine import MappingState


@dataclass
class PolicyContext:
    """Everything a policy might need, handed in by MappingManager. A
    policy should read from this and call methods on car/arm — it
    should never reach into MappingManager itself, keeping policies
    testable with a fake context. The built-in three policies below
    don't need most of these fields; they're here for policies you add
    later."""
    car: object
    arm: object
    rover_map: object
    pose: object


@dataclass
class PolicyResult:
    action_taken: ObjectAction
    next_state: Optional[MappingState] = None   # None = stay in MAPPING, no maneuver needed
    note: str = ""


class ObjectPolicy(ABC):
    name = "base"

    @abstractmethod
    def handle(self, detection: dict, context: PolicyContext) -> PolicyResult:
        ...


class IgnorePolicy(ObjectPolicy):
    """Log it and keep going — no maneuver, no state change."""
    name = "ignore"

    def handle(self, detection, context):
        return PolicyResult(action_taken=ObjectAction.IGNORED)


class AvoidPolicy(ObjectPolicy):
    """
    Mark the object's location (MappingManager records it into
    rover_map right after handle() returns) and steer around it using
    the SAME side-pass maneuver as ultrasonic obstacle avoidance, since
    "an object I don't want to touch" and "an obstacle" need identical
    physical behavior even though they come from different sensors.
    """
    name = "avoid"

    def handle(self, detection, context):
        return PolicyResult(
            action_taken=ObjectAction.AVOIDED,
            next_state=MappingState.AVOIDING_OBSTACLE,
            note="Routed through the same side-pass planner as an ultrasonic obstacle.",
        )


class PickupPolicy(ObjectPolicy):
    """
    Stop, then hand off to ArmController for the align/grip sequence.
    ArmController's internals are untouched (per "don't touch the arm
    code") — this policy only sequences WHEN to call it and what to do
    with the result. See MappingManager._tick_picking_object() and
    ARCHITECTURE.md's "Object pickup workflow" for an important caveat
    about what "success" currently means here.
    """
    name = "pickup"

    def handle(self, detection, context):
        return PolicyResult(
            action_taken=ObjectAction.PICKED_UP,
            next_state=MappingState.PICKING_OBJECT,
            note="MappingManager._tick_picking_object() drives the align/call-arm/timeout sequence.",
        )


POLICIES = {
    "ignore": IgnorePolicy,
    "avoid": AvoidPolicy,
    "pickup": PickupPolicy,
}


def get_policy(name):
    try:
        return POLICIES[name]()
    except KeyError:
        raise ValueError(f"Unknown object_policy '{name}'. Choose from: {', '.join(POLICIES)}")
