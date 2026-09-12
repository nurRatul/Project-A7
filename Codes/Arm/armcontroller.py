import cv2
import serial
import time

from sensors.basic.ultrasonic.ultrasonic import UltrasonicSensor, UltrasonicManager
from .detector import Detector
from .ik import inverse_kinematics
from . import config as cfg

# CHANGED (see chat response for the full explanation):
# - pixel_to_camera() / camera_to_arm() are replaced by a single
#   pixel_to_lateral_x(), since Y is no longer computed from the
#   camera (it's fixed geometry) and Z is no longer computed from
#   the camera either (it comes from the ultrasonic sensors).
# - horizontal_fov / vertical_fov and the runtime fx/fy calculation
#   are removed -- fx/fy/cx/cy now come directly from config.py,
#   per your instructions to use real calibration parameters
#   instead of an FOV approximation.
# - self.distance_mm (the temporary placeholder) is replaced by a
#   real UltrasonicManager.
# - self.xo / self.yo / self.zo are replaced by CAMERA_X_OFFSET in
#   config.py (yo and zo are gone because Y and Z no longer come
#   from the camera at all).
# - self.gripper_angle is removed -- the IK no longer takes a phi
#   parameter, since it doesn't compute a wrist angle (see ik.py).
# - Added enable_motion, servo calibration, and Arduino serial
#   sending -- none of this existed before (see the "Arduino
#   communication" note in chat).


class ArmController:

    def __init__(
        self,
        ultrasonic_sensor = None,
        show_video=True,
        camera_id=cfg.CAMERA_ID,
        debug=True,
        enable_motion=False
    ):

        self.show_video = show_video
        self.debug = debug

        # Safety: the robot will NOT send anything to the Arduino
        # until you explicitly pass enable_motion=True. With this
        # False (the default), update() still runs the full
        # detection + math pipeline and prints debug info if
        # debug=True, so you can verify the numbers before ever
        # commanding a motor.
        self.enable_motion = enable_motion

        self.window_name = "Rover Controller"

        self.detector = Detector()

        # self.ultrasonic = UltrasonicManager()    #for three ultrasonic
        self.ultrasonic= ultrasonic_sensor # UltrasonicSensor(cfg.CENTER_SENSOR_TRIGGER_PIN, cfg.CENTER_SENSOR_ECHO_PIN,cfg.CENTER_SENSOR_X_OFFSET,cfg.CENTER_SENSOR_Z_OFFSET)  #for one ultrasonic
        self._serial = None

        if self.enable_motion:
            self._open_serial()
        if self._serial is not None:
            self.home()

        # ====================================================
        # OUTPUT / LAST-FRAME DATA
        # ====================================================

        self.detection = None
        self.direction = None
        self.x_arm = None
        self.y_arm = None
        self.z_arm = None
        self.ik_result = None

        # This is the variable you will use from test.py -- now a
        # 3-tuple (base_angle, link1_angle, theta2) instead of a
        # 4-element list. If test.py reads self.angles[0..3], it
        # will need updating to expect 3 values, not 4.
        self.angles = None


    # ========================================================
    # CAMERA PIXEL -> LATERAL X (relative to arm)
    # ========================================================

    def pixel_to_lateral_x(self, px, z_arm):
        """
        Standard calibrated pinhole conversion, using the depth
        already determined by the ultrasonic sensor as Z_camera:

            X_camera = (px - cx) * Z / fx

        then shifted by the camera's own mounting offset from the
        arm to get X relative to the arm's J1 origin.

        Returns (camera_x, x_arm) -- both are printed in debug
        mode since your test-mode list asks for each separately.
        """

        camera_x = (px - cfg.CAMERA_CX) * z_arm / cfg.CAMERA_FX

        x_arm = camera_x + cfg.CAMERA_X_OFFSET

        return camera_x, x_arm

    def pixel_to_lateral_y(self, py,z_arm):
            """
            Standard calibrated pinhole conversion, using the depth
            already determined by the ultrasonic sensor as Z_camera:
    
                X_camera = (px - cx) * Z / fx
    
            then shifted by the camera's own mounting offset from the
            arm to get X relative to the arm's J1 origin.
    
            Returns (camera_x, x_arm) -- both are printed in debug
            mode since your test-mode list asks for each separately.
            """
    
            camera_y = (py - cfg.CAMERA_CY) * z_arm / cfg.CAMERA_FY
    
            y_arm = camera_y + cfg.CAMERA_Y_OFFSET
    
            return camera_y, y_arm
    

    # ========================================================
    # SERVO CALIBRATION
    # ========================================================

    def _clamp_and_warn(self, value, limits, name):

        lo, hi = limits

        if value < lo or value > hi:
            print(
                f"WARNING: {name} angle {value:.2f} deg is outside "
                f"its configured limit {limits} -- clamping."
            )

        return max(lo, min(value, hi))


    def _to_servo_command(self, base_angle, link1_angle, theta2):
        """
        Applies each joint's zero-offset + direction sign (from
        config.py), then clamps to that joint's safe servo range.
        This is where the pure geometric angles from ik.py become
        the actual values sent to the Arduino.
        """

        base_cmd = cfg.SERVO_SIGN_BASE * base_angle + cfg.SERVO_ZERO_OFFSET_BASE
        link1_cmd = cfg.SERVO_SIGN_LINK1 * link1_angle + cfg.SERVO_ZERO_OFFSET_LINK1
        theta2_cmd = cfg.SERVO_SIGN_THETA2 * theta2 + cfg.SERVO_ZERO_OFFSET_THETA2

        base_cmd = self._clamp_and_warn(base_cmd, cfg.SERVO_LIMIT_BASE, "base")
        link1_cmd = self._clamp_and_warn(link1_cmd, cfg.SERVO_LIMIT_LINK1, "link1")
        theta2_cmd = self._clamp_and_warn(theta2_cmd, cfg.SERVO_LIMIT_THETA2, "theta2")

        return base_cmd, link1_cmd, theta2_cmd


    # ========================================================
    # ARDUINO SERIAL
    # ========================================================
    #
    # NEW -- your original controller.py had no serial code at
    # all, so there was no existing communication method to
    # preserve. This opens a plain pyserial connection and writes
    # one comma-separated text line per update. Adjust the format
    # in send_to_arduino() below to match whatever your Arduino
    # sketch actually parses.
    #
    # Install with: pip install pyserial

    def _open_serial(self):

        if cfg.ARDUINO_SERIAL_PORT is None:
            print(
                "ARDUINO_SERIAL_PORT is not set in config.py -- "
                "cannot open a connection to the Arduino."
            )
            for i in range (0,10):
                try:
                    self._serial = serial.Serial(f'/dev/ttyACM{i}', cfg.ARDUINO_BAUD_RATE, timeout=cfg.ARDUINO_SERIAL_TIMEOUT_S)
                    print(f'arm is connected to /dev/ttyACM{i}\n')
                    break
                except serial.SerialException as e:
                    print(f"Error opening serial port: {e}")
                    time.sleep(1)
                    self._serial = None
            return

        try:
            self._serial = serial.Serial(
                cfg.ARDUINO_SERIAL_PORT,
                cfg.ARDUINO_BAUD_RATE,
                timeout=cfg.ARDUINO_SERIAL_TIMEOUT_S
            )
        except serial.SerialException as error:
            print(f"Could not open Arduino serial port: {error}")
            self._serial = None

    def home(self):
        try:
            self._serial.write(b'H60,60,20,180,0,73,20\n')
        except serial.SerialException as error:
            print(f"Failed to send to Arduino: {error}")
            

    def send_to_arduino(self, base_angle, link1_angle, link2_angle):
        """
        Sends the three servo angles to the Arduino as one
        comma-separated line, e.g.:

            "90.00,68.09,52.30\\n"
        """

        if self._serial is None:
            print("Arduino serial port not open -- skipping send.")
            return

        # message=f'"P"{base_angle:.2f},{link1_angle:.2f},90,0,{theta2:.2f}\n,200\n"'
        # message=f'"P"{int(base_angle)},{int(link1_angle)},{int(link2_angle)},0,{int(link1_angle)-int(link2_angle)+90},100,200\n"' ## int(link1_angle)-int(link2_angle)+90 for keeping the grabber downward always

        try:
            # self._serial.write(message.encode("utf-8"))
            # self.write_arduino([int(base_angle),int(link1_angle),int(link2_angle),0,int(link1_angle)-int(link2_angle)+90,100])
            self.write_arduino([int(base_angle),0,0,0,0,100])
        except serial.SerialException as error:
            print(f"Failed to send to Arduino: {error}")

    def write_arduino(self,angles):
        angle_string=','.join([str(elem) for elem in angles])  # join the list values togheter
        angle_string="P"+angle_string+",100\n"    
        print(angle_string)
        self._serial.write(angle_string.encode())          #.encode encodes the string to bytes
        self._serial.flush()

        reply = self._serial.readline().decode(errors="ignore").strip()
        print("Arduino:", reply)


    def dump_garbage_w1(self):
        self.write_arduino([60,80,100,0,90+80-100,100])
        time.sleep(2)
        self.write_arduino([60,80,100,0,90+80-100,10])
        time.sleep(.25)
        self.write_arduino([60,170,10,180,90,10])
        time.sleep(.25)
        self.write_arduino([60,170,30,180,90,100])
        time.sleep(.25)
        self.write_arduino([60,110,110,0,90+110-110,100])
        time.sleep(.25)

    def dump_garbage_w2(self):
            self.write_arduino([60,80,100,0,90+80-100,100])
            time.sleep(2)
            self.write_arduino([60,80,100,0,90+80-100,10])
            time.sleep(.25)
            self.write_arduino([180,170,100,0,90+120-100,10])
            time.sleep(.25)
            self.write_arduino([180,170,100,0,90+120-100,100])
            time.sleep(.25)
            self.write_arduino([60,110,110,0,90+110-110,100])
            time.sleep(.25)


    # ========================================================
    # DEBUG PRINTING
    # ========================================================
    # Prints exactly the fields your test-mode list asked for.

    def _print_debug(
        self,
        px, py,
        camera_x,
        direction,
        raw_mm,
        x_arm, z_arm,
        R,
        base_angle, link1_angle, theta2
    ):

        print(f"Pixel center:     ({px:.1f}, {py:.1f})")
        print(f"Camera X:         {camera_x:.2f} mm")
        print(f"Sensor selected:  {direction}")
        print(f"Ultrasonic (raw): {raw_mm:.2f} mm")
        print(f"X_arm:            {x_arm:.2f} mm")
        print(f"Z_arm:            {z_arm:.2f} mm")
        print(f"R (radial):       {R:.2f} mm")
        print(f"base_angle:       {base_angle:.2f} deg")
        print(f"link1_angle:      {link1_angle:.2f} deg")
        print(f"theta2:           {theta2:.2f} deg")


    # ========================================================
    # PROCESS ONE FRAME
    # ========================================================

    def update(self):
        """
        Capture one frame, detect the trash, select + read the
        matching ultrasonic sensor, run the 2D IK, and (only if
        enable_motion is True) send the result to the Arduino.

        Returns the final (base, link1, theta2) servo angles, or
        None if nothing valid was found this frame.
        """

        frame = self.detector.read_frame()

        if frame is None:

            print(
                "Camera frame failed."
            )

            self.angles = None

            return None


        # ====================================================
        # DETECTION
        # ====================================================

        detections = self.detector.detect(
            frame
        )


        # No bottle

        if not detections:

            self.detection = None
            self.direction = None
            self.x_arm = None
            self.z_arm = None
            self.ik_result = None
            self.angles = None


            if self.show_video:

                cv2.putText(
                    frame,
                    "Nothing found yet",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )


                cv2.imshow(
                    self.window_name,
                    frame
                )


                self._handle_keyboard()


            return None


        # ====================================================
        # FIRST Object
        # ====================================================

        self.detection = detections[0]

        px, py = (
            self.detection["center"]
        )

        confidence = (
            self.detection["confidence"]
        )


        # ====================================================
        # ULTRASONIC: select a sensor from the pixel position,
        # read it, and get Z relative to the arm.
        # ====================================================

        #direction, raw_mm, z_arm = self.ultrasonic.measure(px)   #for three ults
        #----------------------------for one ultrasonic----------------#
        direction="CENTER"
        raw_mm= self.ultrasonic.read_mm()
        z_arm= raw_mm+cfg.CENTER_SENSOR_Z_OFFSET if raw_mm is not None else None
        self.direction = direction

        #--------------------------------------------#

        if z_arm is None:

            print(
                f"Ultrasonic ({direction}) reading invalid -- skipping."
            )

            self.x_arm = None
            self.z_arm = None
            self.ik_result = None
            self.angles = None

            if self.show_video:

                frame = self.detector.draw(
                    frame,
                    detections
                )

                cv2.putText(
                    frame,
                    f"No valid range ({direction})",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                cv2.imshow(
                    self.window_name,
                    frame
                )

                self._handle_keyboard()

            return None


        # ====================================================
        # CAMERA X (using the now-known depth)
        # ====================================================

        camera_x, x_arm = self.pixel_to_lateral_x(
            px,
            z_arm
        )

        self.x_arm = x_arm
        self.z_arm = z_arm

        camera_y, y_arm = self.pixel_to_lateral_y(
            py,
            z_arm
        )

        self.y_arm = y_arm


        # ====================================================
        # IK
        # ====================================================

        try:

            self.ik_result = inverse_kinematics(
                x_arm,
                y_arm,
                z_arm
            )

        except ValueError as error:

            print(
                "IK ERROR:",
                error
            )

            self.ik_result = None
            self.angles = None

            return None

        base_angle = self.ik_result["base_angle"]
        link1_angle = self.ik_result["link1_angle"]
        theta2 = self.ik_result["theta2"]
        R = self.ik_result["R"]

        self.angles = self._to_servo_command(
            base_angle,
            link1_angle,
            theta2
        )


        # ====================================================
        # PRINT DEBUG INFORMATION
        # ====================================================

        if self.debug:

            print(
                f"Bottle: "
                f"{confidence:.2f}"
            )

            self._print_debug(
                px, py,
                camera_x,
                direction,
                raw_mm,
                x_arm, z_arm,
                R,
                base_angle, link1_angle, theta2
            )


        # ====================================================
        # SEND TO ARDUINO (only if motion is enabled)
        # ====================================================

        if self.enable_motion:

            self.send_to_arduino(
                *self.angles
            )


        # ====================================================
        # VIDEO
        # ====================================================

        if self.show_video:

            frame = self.detector.draw(
                frame,
                detections
            )


            if self.angles is not None:

                text = (
                    f"Base: {self.angles[0]:.1f}, "
                    f"L1: {self.angles[1]:.1f}, "
                    f"T2: {self.angles[2]:.1f}"
                )

                cv2.putText(
                    frame,
                    text,
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2
                )


            cv2.imshow(
                self.window_name,
                frame
            )


            self._handle_keyboard()


        return self.angles


    # ========================================================
    # KEYBOARD
    # ========================================================

    def _handle_keyboard(self):

        key = cv2.waitKey(1) & 0xFF


        if (
            key == ord("q") or
            key == ord("Q") or
            key == 27
        ):

            self.close()

            raise KeyboardInterrupt


    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        self.detector.release()

        #self.ultrasonic.close()  #for three ults

        if self._serial is not None:
            self._serial.close()

        cv2.destroyAllWindows()
