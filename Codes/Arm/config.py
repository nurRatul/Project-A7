"""
Central configuration for the trash-collecting robot.

Every physical dimension, offset, and hardware setting the robot
needs lives here. Nothing else in detector.py, controller.py,
ultrasonic.py, or ik.py should hardcode a physical number -- if you
need to change a measurement, this should be the only file you have
to touch.

Values marked "??? MEASURE" are placeholders. I did not invent
numbers for these -- they were not present anywhere in your
original files, so you'll need to physically measure them (or, for
the Arduino port/baud rate, confirm them against your Arduino
sketch) before the robot can run. See the chat response for the
full list of what to fill in.
"""

# ========================================================
# USER CONFIGURATION — EDIT THESE VALUES
# ========================================================

# ------------------------------------------------------------
# CAMERA
# ------------------------------------------------------------

MODEL_PATH = "yolo26n.pt"

CAMERA_ID = 0

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

YOLO_CONFIDENCE = 0.50   # was named CONFIDENCE in your original detector.py

BOTTLE_CLASS_ID = 39     # COCO class id for "bottle"

# Camera calibration (pinhole model).
#
# CAMERA_FX / CAMERA_FY below are NOT a real calibration -- they
# are back-calculated from the horizontal_fov=70 deg /
# vertical_fov=55 deg assumption your old controller.py used to
# compute an approximate focal length at runtime. I carried that
# same approximation over here as a starting value so the code is
# runnable, but it is only as accurate as that original guess was.
# For real accuracy, run an actual camera calibration (e.g.
# OpenCV's checkerboard/chessboard calibration routine) and replace
# these four numbers with the real result.
CAMERA_FX = 627
CAMERA_FY = 627
CAMERA_CX = IMAGE_WIDTH / 2
CAMERA_CY = IMAGE_HEIGHT / 2
# NOTE: CAMERA_FY and CAMERA_CY are not currently used anywhere in
# the pipeline below -- only lateral X is computed from the camera;
# Y is now fixed by the known ground/gripper geometry instead (see
# ARM section). They're kept here because a calibrated camera
# normally needs the full fx/fy/cx/cy set, and you may want them
# later.

# How far the camera is mounted to the side of the arm's J1
# centerline, in mm. Positive = camera is to the right of J1.
# This replaces "xo" from your old controller.py.
CAMERA_X_OFFSET = -60   # ??? MEASURE if the camera isn't exactly on the arm's centerline


# ------------------------------------------------------------
# ARM
# ------------------------------------------------------------

LINK1 = 110.0   # mm, J1 -> J2   (was "L2" in your original ik.py)
LINK2 = 120.0   # mm, J2 -> gripper mount   (was "L3" in your original ik.py)

J1_HEIGHT = 95         # mm, height of J1 above the ground
GRIPPER_HEIGHT = 130.0   # mm, height of the gripper pickup point above the ground

# Derived -- don't edit this line directly, edit J1_HEIGHT /
# GRIPPER_HEIGHT above instead and this updates automatically.
TARGET_HEIGHT = GRIPPER_HEIGHT - J1_HEIGHT   # = 30 mm

# The gripper is assumed to be mechanically fixed pointing downward
# (this code does not compute or send an active wrist/gripper
# angle -- see the note in ik.py for why). If your hardware
# actually has a wrist servo that must be driven independently to
# keep the gripper level, tell me and I'll add that calculation
# back into ik.py.

# --- Servo calibration -------------------------------------------
# Applied AFTER the geometric IK math, right before sending values
# to the Arduino: final_command = sign * geometric_angle + offset,
# then clamped to the matching limit range.
#
# Per your notes, theta2 should need offset=0 / sign=1 because you
# physically re-zeroed that servo to match this math directly.
# base_angle and link1_angle are NOT confirmed the same way -- keep
# an eye on the printed values during testing and adjust these two
# if the numbers don't land inside your servos' real travel range.
SERVO_ZERO_OFFSET_BASE = 0.0     # degrees ??? verify during testing
SERVO_ZERO_OFFSET_LINK1 = 0.0    # degrees ??? verify during testing
SERVO_ZERO_OFFSET_THETA2 = 0.0   # degrees -- should need no change per your notes

SERVO_SIGN_BASE = 1     # set to -1 if the base servo needs to be reversed
SERVO_SIGN_LINK1 = 1    # set to -1 if the link1/shoulder servo needs to be reversed
SERVO_SIGN_THETA2 = 1   # set to -1 if the theta2/elbow servo needs to be reversed

# Safe travel range for each servo, in degrees, AFTER offset/sign
# is applied: (minimum, maximum). Default assumes a standard
# 0-180 deg hobby servo -- narrow these if your physical linkage
# can't safely reach the full range.
SERVO_LIMIT_BASE = (0.0, 180.0)
SERVO_LIMIT_LINK1 = (0.0, 180.0)
SERVO_LIMIT_THETA2 = (0.0, 180.0)


# ------------------------------------------------------------
# ULTRASONIC
# ------------------------------------------------------------

# Position of each sensor relative to the arm's J1 origin, in mm.
# X = left/right (left negative, same convention as the arm),
# Z = forward/back (0 if the sensor sits on the same front plane
# as J1).
#
# None of your original files contained ultrasonic hardware code,
# so I have no existing values to carry over -- please measure:
LEFT_SENSOR_X_OFFSET = None     # ??? MEASURE: mm, left of J1 centerline (negative)
CENTER_SENSOR_X_OFFSET = 0.0    # assumed ~0 (on centerline) -- confirm/adjust
RIGHT_SENSOR_X_OFFSET = None    # ??? MEASURE: mm, right of J1 centerline (positive)

LEFT_SENSOR_Z_OFFSET = 0.0      # ??? MEASURE if this sensor isn't on the same front plane as J1
CENTER_SENSOR_Z_OFFSET = 70    # ??? MEASURE if needed
RIGHT_SENSOR_Z_OFFSET = 0.0     # ??? MEASURE if needed

# GPIO pins (BCM numbering) for each HC-SR04-style sensor. I don't
# know how you've wired these -- please fill in:
LEFT_SENSOR_TRIGGER_PIN = None     # ??? BCM pin number
LEFT_SENSOR_ECHO_PIN = None        # ??? BCM pin number
CENTER_SENSOR_TRIGGER_PIN = 9   # ??? BCM pin number
CENTER_SENSOR_ECHO_PIN = 10  # ??? BCM pin number
RIGHT_SENSOR_TRIGGER_PIN = None    # ??? BCM pin number
RIGHT_SENSOR_ECHO_PIN = None       # ??? BCM pin number

# How far off-center (in pixels, measured from CAMERA_CX) the
# detected trash has to be before it counts as LEFT or RIGHT
# instead of CENTER. Starting guess -- tune by looking at real
# pixel coordinates for objects placed left/center/right of frame.
SENSOR_SELECT_THRESHOLD_PX = 80

# Ultrasonic readings outside this range (mm) are treated as
# invalid and discarded. Adjust MIN to your sensor's datasheet;
# MAX is set a bit beyond the arm's own maximum reach
# (LINK1 + LINK2 = 253 mm) as a sanity bound.
ULTRASONIC_MIN_VALID_MM = 20.0
ULTRASONIC_MAX_VALID_MM = 400.0


# ------------------------------------------------------------
# ROBOT / ARDUINO
# ------------------------------------------------------------

# Your original controller.py did not contain any existing
# Raspberry Pi <-> Arduino serial communication code (there was a
# placeholder distance value and a comment about replacing it, but
# no actual serial link) -- so there was nothing for me to preserve
# here. This is new. Confirm both values match your Arduino sketch.
ARDUINO_SERIAL_PORT = None    # ??? e.g. "/dev/ttyUSB0" or "/dev/ttyACM0" -- check with `ls /dev/tty*` on the Pi
ARDUINO_BAUD_RATE = 9600      # ??? must match Serial.begin(...) in your Arduino sketch
ARDUINO_SERIAL_TIMEOUT_S = 1.0
