# imuManager.py

import logging
import os

from .mpu6050 import MPU6050, IMUTracker

logger = logging.getLogger("car_controller.imuManager")


class IMUManager:
    def __init__(
        self,
        imu_bus=None,
        imu_address=None,
        imu_sample_rate_hz=100,
        imu_autocalibrate=True,
    ):
        if imu_bus is None:
            imu_bus = int(os.getenv("CAR_IMU_BUS", "1"))

        if imu_address is None:
            imu_address = int(os.getenv("CAR_IMU_ADDRESS", "0x68"), 0)

        self.imu = MPU6050(
            bus=imu_bus,
            address=imu_address,
            sample_rate_hz=imu_sample_rate_hz,
        )

        if imu_autocalibrate:
            logger.info("Calibrating IMU...")
            self.imu.calibrate()

        self.imu_tracker = IMUTracker(
            self.imu,
            sample_rate_hz=imu_sample_rate_hz,
        )

        self.imu_tracker.start()

    def calibrate(self):
        self.imu.calibrate()

    def reset(self):
        self.imu_tracker.reset()

    def get_orientation(self):
        return self.imu_tracker.snapshot()["orientation_deg"]

    def get_acceleration(self):
        return self.imu_tracker.snapshot()["linear_acceleration_ms2"]

    def get_position(self):
        return self.imu_tracker.snapshot()["position_m"]

    def get_telemetry(self):
        data = self.imu_tracker.snapshot()
        data["available"] = True
        return data

    def close(self):
        self.imu_tracker.stop()
        self.imu.close()