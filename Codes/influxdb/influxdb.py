from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS


class InfluxManager:

    def __init__(
        self,
        url="http://localhost:8086",
        token="YOUR_TOKEN",
        org="robot",
        bucket="robot_data",
        robot_id="robot_01",
        run_id="test_001",
    ):
        self.bucket = bucket
        self.org = org
        self.robot_id = robot_id
        self.run_id = run_id

        self.client = InfluxDBClient(
            url=url,
            token=token,
            org=org,
        )

        self.write_api = self.client.write_api(
            write_options=SYNCHRONOUS
        )

    def write_telemetry(self, gps, imu, ultrasonic,dispossible=0,nondispossible=0):

        points = []

        # --------------------------------------------------
        # IMU
        # --------------------------------------------------

        if imu:
            accel = imu.get("linear_acceleration_ms2", {})
            orientation = imu.get("orientation_deg", {})
            velocity = imu.get("velocity_ms", {})

            point = (
                Point("imu")
                .tag("robot_id", self.robot_id)
                .tag("run_id", self.run_id)
            )

            if accel:
                for axis in ("x", "y", "z"):
                    if axis in accel:
                        point.field(
                            f"accel_{axis}",
                            float(accel[axis])
                        )

            if orientation:
                for axis in ("roll", "pitch", "yaw"):
                    if axis in orientation:
                        point.field(
                            axis,
                            float(orientation[axis])
                        )

            if velocity:
                for axis in ("x", "y", "z"):
                    if axis in velocity:
                        point.field(
                            axis,
                            float(velocity[axis])
                        )

            if len(point._fields) > 0:
                points.append(point)

        # --------------------------------------------------
        # GPS
        # --------------------------------------------------

        if gps and gps.get("has_fix"):

            point = (
                Point("gps")
                .tag("robot_id", self.robot_id)
                .tag("run_id", self.run_id)
            )

            if gps.get("latitude") is not None:
                point.field(
                    "latitude",
                    float(gps["latitude"])
                )

            if gps.get("longitude") is not None:
                point.field(
                    "longitude",
                    float(gps["longitude"])
                )

            if gps.get("speed_kmh") is not None:
                point.field(
                    "speed_kmh",
                    float(gps["speed_kmh"])
                )

            if gps.get("altitude_m") is not None:
                point.field(
                    "altitude_m",
                    float(gps["altitude_m"])
                )

            if gps.get("satellites") is not None:
                point.field(
                    "satellites",
                    int(gps["satellites"])
                )

            points.append(point)

        # --------------------------------------------------
        # ULTRASONIC
        # --------------------------------------------------

        if ultrasonic is not None:

            point = (
                Point("ultrasonic")
                .tag("robot_id", self.robot_id)
                .tag("run_id", self.run_id)
                .field("distance_mm", float(ultrasonic))
            )

            points.append(point)

        if dispossible:
            point = ( 
                Point("Dispossible")
                .tag("robot_id", self.robot_id)
                .tag("run_id", self.run_id)
                .field("collected",dispossible)
            )

            points.append(point)

        if nondispossible:
                point = ( 
                    Point("NonDispossible")
                    .tag("robot_id", self.robot_id)
                    .tag("run_id", self.run_id)
                    .field("collected",nondispossible)
                )
    
                points.append(point)
        

        # --------------------------------------------------
        # WRITE EVERYTHING IN ONE BATCH
        # --------------------------------------------------

        if points:
            self.write_api.write(
                bucket=self.bucket,
                org=self.org,
                record=points,
            )

    def close(self):
        self.client.close()