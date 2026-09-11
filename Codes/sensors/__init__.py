"""Independent sensor managers shared by Car, Arm, and MappingManager.

Each manager (imu, gps, ultrasonic, vision) owns its own hardware and
exposes a background-thread + snapshot() interface. None of them are
constructed inside CarController or ArmController — see
mapping/interfaces.py for the contracts MappingManager depends on.
"""
