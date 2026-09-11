"""
Radar mapping subsystem for the autonomous car.

Public API
----------
    from radar.vehicle_state import VehicleState, get_vehicle_state
    from radar.coordinate_transformer import MapCoordinateTransformer
    from radar.radar_bridge import RadarBridge

VehicleState is a process-wide singleton so that the Flask server,
sensor threads, and test_run.py all share the same live state.
"""

from .vehicle_state import VehicleState, get_vehicle_state
from .coordinate_transformer import MapCoordinateTransformer
from .radar_bridge import RadarBridge

__all__ = [
    "VehicleState",
    "get_vehicle_state",
    "MapCoordinateTransformer",
    "RadarBridge",
]
