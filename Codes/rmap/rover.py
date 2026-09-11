from Car.carController import CarController
from sensors.basic.gps.gpsManager import GPSManager
from sensors.basic.imu.imuManager import IMUManager
from sensors.basic.ultrasonic.ultrasonic import UltrasonicSensor
from Arm.armcontroller import ArmController
from logger.logger_manager import LoggerManager
from influxdb.influxdb import InfluxManager
from .interfaces import RoverState, Object
from .rmap import mappingManager
import threading
import time
import math




class Rover:
    def __init__(self, gps_port="/dev/ttyAMA0", gps_baudrate=9600, gps_timeout=1.0, hold_distance=150):
        self.gps_manager = GPSManager(port=gps_port, baudrate=gps_baudrate, timeout=gps_timeout)
        self.gps_manager.start()
        self.imu_manager = IMUManager()
        self.ultrasonic_sensor = UltrasonicSensor(9,10,0,0)
        self.car_controller = CarController()
        self.arm_controller = ArmController(ultrasonic_sensor= self.ultrasonic_sensor ,show_video=False)
        self.roverState = RoverState()
        self.mapping_manager = mappingManager(self.roverState, self.gps_manager, self.imu_manager, self.ultrasonic_sensor, self.car_controller, gps_port, gps_baudrate, gps_timeout)
        self.hold_distance = hold_distance
        self.influx = InfluxManager(
            url="http://localhost:8086",
            token="YOUR_TOKEN",
            org="robot",
            bucket="robot_data",
            robot_id="robot_01",
            run_id="test_001",
        )

        # --------------------------------------------#
        self.thread = threading.Thread(target=self.monitor_ultrasonic, args=(hold_distance,), daemon=True)
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop,daemon=True)
        self.telemetry_thread.start()
        self.thread.start()

        #---------------------------------------------#
        self.arm_controller._open_serial()
        self.arm_controller.home()

    def has_obstacle(self, threshold_m=None): ## put this code to the ultrasonicManager
        if threshold_m is None:
            threshold_m = self.hold_distance
        distance = self.ultrasonic_sensor.read_mm() or 500
        # print(f"Ultrasonic distance: {distance} mm, Threshold: {threshold_m} mm")
        if int(distance) < int(threshold_m):
            self.roverState.object_detected = True
            return True
        return False

    def monitor_ultrasonic(self, threshold_m= None):
        if threshold_m is None:
            threshold_m = self.hold_distance
        print("Monitor thread started")
        while True:
            # print("Monitor tick")
            if self.has_obstacle(threshold_m):
                pass
            time.sleep(0.1)  # Adjust the sleep time as needed

    def _telemetry_loop(self):

        while True:

            try:
                # Get latest IMU data
                imu_data = self.imu_manager.get_telemetry()

                # Get latest GPS data
                gps_data = self.gps_manager.get_telemetry()

                # Get ultrasonic reading
                ultrasonic_data = self.ultrasonic_sensor.read_mm()

                # Send everything to InfluxDB
                self.influx.write_telemetry(
                    gps=gps_data,
                    imu=imu_data,
                    ultrasonic=ultrasonic_data,
                )

            except Exception as e:
                print(f"Telemetry error: {e}")

            # 20 Hz = 50 ms
            time.sleep(0.05)

    def close(self):
        self.gps_manager.close()
        self.imu_manager.close()
        self.ultrasonic_sensor.close()
        self.influx.close()


    
