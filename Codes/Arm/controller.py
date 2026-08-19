import math
import cv2

from .detector import BottleDetector
from .ik import inverse_kinematics


class Controller:

    def __init__(
        self,
        show_video=True,
        camera_id=0,
        debug=True,
        detector=None
            ):
        
        self.show_video = show_video
        self.debug = debug

        # ====================================================
        # VIDEO
        # ====================================================

        self.show_video = show_video

        self.window_name = "Rover Controller"


        # ====================================================
        # CAMERA
        # ====================================================

        self.image_width = 1080 # 640
        self.image_height = 720 # 480

        self.horizontal_fov = 70.0
        self.vertical_fov = 55.0


        # ====================================================
        # CAMERA OFFSET FROM J1
        # ====================================================

        # mm

        self.xo = 0.0
        self.yo = 0.0
        self.zo = 0.0


        # ====================================================
        # GRIPPER
        # ====================================================

        self.gripper_angle = -90.0


        # ====================================================
        # TEMPORARY DISTANCE
        # ====================================================

        # Later replace this with ultrasonic reading.

        self.distance_mm = 100.0


        # ====================================================
        # OUTPUT
        # ====================================================

        # This is the variable you will use
        # from test.py.

        self.angles = None


        # ====================================================
        # OTHER DATA
        # ====================================================

        self.detection = None

        self.camera_position = None

        self.arm_position = None


        # ====================================================
        # DETECTOR
        # ====================================================

        self.detector = detector or BottleDetector(
            camera_id=camera_id
        )


    # ========================================================
    # CAMERA PIXEL -> CAMERA XYZ
    # ========================================================

    def pixel_to_camera(
        self,
        px,
        py,
        distance
    ):

        hfov = math.radians(
            self.horizontal_fov
        )

        vfov = math.radians(
            self.vertical_fov
        )

        fx = (
            self.image_width /
            (
                2 *
                math.tan(hfov / 2)
            )
        )

        fy = (
            self.image_height /
            (
                2 *
                math.tan(vfov / 2)
            )
        )

        cx = self.image_width / 2
        cy = self.image_height / 2


        # Temporary simplified model

        z = distance

        x = (
            (px - cx) *
            z /
            fx
        )

        y = (
            (py - cy) *
            z /
            fy
        )


        return [
            x,
            y,
            z
        ]


    # ========================================================
    # CAMERA XYZ -> ARM XYZ
    # ========================================================

    def camera_to_arm(
        self,
        camera_position
    ):

        xc, yc, zc = camera_position

        x = xc + self.xo
        y = yc + self.yo
        z = zc + self.zo

        return [
            x,
            y,
            z
        ]


    # ========================================================
    # PROCESS ONE FRAME
    # ========================================================

    def update(self, frame=None, detections=None):

        """
        Capture one frame, detect the bottle,
        calculate XYZ and calculate IK.

        Returns:

            self.angles

        Example:

            [45.2, 72.1, 83.4, 104.6]

        If no valid bottle is detected:

            None
        """


        if frame is None:
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

        if detections is None:
            detections = self.detector.detect(frame)


        # No bottle

        if not detections:

            self.detection = None
            self.camera_position = None
            self.arm_position = None
            self.angles = None


            if self.show_video:

                cv2.putText(
                    frame,
                    "No bottle",
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
        # FIRST BOTTLE
        # ====================================================

        self.detection = detections[0]


        px, py = (
            self.detection["center"]
        )


        confidence = (
            self.detection["confidence"]
        )


        # ====================================================
        # CAMERA XYZ
        # ====================================================

        self.camera_position = (
            self.pixel_to_camera(
                px,
                py,
                self.distance_mm
            )
        )


        # ====================================================
        # ARM XYZ
        # ====================================================

        self.arm_position = (
            self.camera_to_arm(
                self.camera_position
            )
        )


        x, y, z = self.arm_position


        # ====================================================
        # IK
        # ====================================================

        try:

            self.angles = (
                inverse_kinematics(
                    x,
                    y,
                    z,
                    self.gripper_angle
                )
            )

        except ValueError as error:

            print(
                "IK ERROR:",
                error
            )

            self.angles = None


        # ====================================================
        # PRINT DEBUG INFORMATION
        # ====================================================

        if self.debug:
            print(
                f"Bottle: "
                f"{confidence:.2f}"
            )

            print(
                f"Pixel: "
                f"({px:.1f}, {py:.1f})"
            )

            print(
                f"Camera XYZ: "
                f"{self.camera_position}"
            )

            print(
                f"Arm XYZ: "
                f"{self.arm_position}"
            )

            print(
                f"Angles: "
                f"{self.angles}"
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
                    f"J: "
                    f"{self.angles[0]:.1f}, "
                    f"{self.angles[1]:.1f}, "
                    f"{self.angles[2]:.1f}, "
                    f"{self.angles[3]:.1f}"
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

        cv2.destroyAllWindows()