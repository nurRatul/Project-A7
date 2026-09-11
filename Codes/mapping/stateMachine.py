# stateMachine.py

"""
stateMachine.py

The mapping mission's state machine: valid states, valid transitions
between them, and a small MappingStateMachine class that enforces the
transition table so MappingManager can't accidentally jump into an
invalid state from a bug elsewhere in the loop.

Transition diagram:

    IDLE -> INITIALIZING
    INITIALIZING -> MAPPING | ERROR | IDLE
    MAPPING -> AVOIDING_OBSTACLE | PICKING_OBJECT | COMPLETED | ERROR | IDLE
    AVOIDING_OBSTACLE -> RETURNING_TO_PATH | ERROR | IDLE
    PICKING_OBJECT -> RETURNING_TO_PATH | MAPPING | ERROR | IDLE
    RETURNING_TO_PATH -> MAPPING | ERROR | IDLE
    COMPLETED -> IDLE
    ERROR -> IDLE

Every in-progress state (INITIALIZING through RETURNING_TO_PATH) can
go straight to IDLE. That's stop_mapping() — a user-triggered abort —
which has to work no matter which sub-state the mission is currently
in, rather than forcing it to "finish" whatever maneuver it was
mid-way through first.
"""

import time
from enum import Enum

from logger.logger_manager import LoggerManager

logger = LoggerManager.get_logger("mapping.stateMachine")


class MappingState(str, Enum):
    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    MAPPING = "MAPPING"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    PICKING_OBJECT = "PICKING_OBJECT"
    RETURNING_TO_PATH = "RETURNING_TO_PATH"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"


TRANSITIONS = {
    MappingState.IDLE: {MappingState.INITIALIZING},
    MappingState.INITIALIZING: {MappingState.MAPPING, MappingState.ERROR, MappingState.IDLE},
    MappingState.MAPPING: {
        MappingState.AVOIDING_OBSTACLE,
        MappingState.PICKING_OBJECT,
        MappingState.COMPLETED,
        MappingState.ERROR,
        MappingState.IDLE,
    },
    MappingState.AVOIDING_OBSTACLE: {MappingState.RETURNING_TO_PATH, MappingState.ERROR, MappingState.IDLE},
    MappingState.PICKING_OBJECT: {
        MappingState.RETURNING_TO_PATH,
        MappingState.MAPPING,
        MappingState.ERROR,
        MappingState.IDLE,
    },
    MappingState.RETURNING_TO_PATH: {MappingState.MAPPING, MappingState.ERROR, MappingState.IDLE},
    MappingState.COMPLETED: {MappingState.IDLE},
    MappingState.ERROR: {MappingState.IDLE},
}


class InvalidTransition(Exception):
    pass


class MappingStateMachine:
    def __init__(self, on_transition=None):
        """
        on_transition: optional callback(old_state, new_state) fired
        after every successful transition — MappingManager uses this to
        log, update telemetry, or (as a future improvement) persist
        state changes into RoverMap.
        """
        self._state = MappingState.IDLE
        self._on_transition = on_transition
        self._history = [(self._state, time.time())]

    @property
    def state(self):
        return self._state

    @property
    def history(self):
        return list(self._history)

    def can_transition(self, new_state):
        return new_state in TRANSITIONS[self._state]

    def transition(self, new_state):
        if not self.can_transition(new_state):
            raise InvalidTransition(f"{self._state} -> {new_state} is not a valid transition")

        old_state = self._state
        self._state = new_state
        self._history.append((new_state, time.time()))
        logger.info("Mapping state: %s -> %s", old_state, new_state)

        if self._on_transition:
            self._on_transition(old_state, new_state)

    def is_terminal(self):
        return self._state in (MappingState.COMPLETED, MappingState.ERROR)
