"""
Inverse kinematics for the trash-collecting arm.

Calculates the THREE servo angles the arm needs to reach a target
on the ground:

    base_angle   -- rotates the whole arm to face the trash
                     (left/right). 0 deg = straight ahead.
    link1_angle  -- angle of Link 1 (the shoulder), measured from
                     +horizontal -- exactly as in your drawing.
    theta2       -- angle of Link 2 (the elbow), measured from the
                     DOWNWARD vertical reference -- exactly as your
                     physical servo is zeroed, so this value is
                     meant to be usable directly as the motor angle.

The trash is assumed to be on the ground, so the vertical target
height is fixed by your known arm geometry (J1_HEIGHT /
GRIPPER_HEIGHT in config.py) rather than measured by the camera.

WHAT CHANGED FROM YOUR ORIGINAL ik.py
--------------------------------------
- Fixed the coordinate-system bug: base rotation is now
  atan2(X_arm, Z_arm) in the horizontal plane. Your old theta1
  was atan2(y, x), which used the wrong pair of axes for a base
  rotation and would not have pointed the arm at the trash.
- Removed the LG = 35 mm "subtract gripper length from the wrist
  target" calculation. In the new geometry the gripper pickup
  point itself is the target (fixed 30 mm above J1), so there's no
  separate wrist point to solve for.
- Removed theta3 (elbow-relative angle) and theta4 (wrist/gripper
  compensation). Your 10-step simplified-IK procedure and pipeline
  only define base_angle, link1_angle, and theta2 -- no third arm
  angle is calculated anywhere in it, and you were explicit about
  not adding one back in unless the physical hardware has an
  independent joint that needs it. I'm assuming the gripper stays
  level through the fixed mechanical mount rather than an actively
  driven wrist. If that's wrong and you do have a wrist servo that
  needs driving, tell me and I'll add the calculation back in.
- Removed the "compute both elbow-up and elbow-down solutions,
  then filter for a valid one" logic that was in ik.py and still in
  ik_simplified (1).py. Your instructions say only ONE configuration
  matches your physical arm, so link1_angle is calculated directly
  with alpha + gamma (matching the "up/right" Link 1 posture from
  your drawing). If testing shows the arm should bend the other
  way, change the line marked below to alpha - gamma.
- Return value changed from a plain [j1, j2, j3, j4] list to a
  dict with named keys. With the angle count and meaning both
  changing (4 generic joint angles -> 3 specifically-named ones),
  a positional list risked exactly the kind of index confusion
  these instructions were trying to get away from. It also lets R
  (an intermediate value) be returned for debugging without
  recomputing it elsewhere -- see the "no duplicate distance
  calculations" cleanup note in the chat response.
"""

import math

from . import config as cfg


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def inverse_kinematics(x_arm, z_arm):
    """
    x_arm, z_arm: position of the trash relative to the arm's J1
    origin, in mm (X = left/right, Z = forward). Y is not a
    parameter -- it's fixed by config.TARGET_HEIGHT.

    Returns a dict:
        {
            "base_angle":  degrees,
            "link1_angle": degrees,
            "theta2":      degrees,
            "R":           mm (horizontal radial distance -- for
                           debugging only, not sent to the Arduino)
        }

    Raises ValueError if the target is outside the arm's reach.
    """

    # ========================================================
    # STEP 1 / 2 -- horizontal radial distance + base rotation
    # ========================================================

    R = math.sqrt(x_arm ** 2 + z_arm ** 2)

    base_angle = math.atan2(x_arm, z_arm)

    # ========================================================
    # STEP 3 -- target height relative to J1 (fixed, ground pickup)
    # ========================================================

    H = cfg.TARGET_HEIGHT

    # ========================================================
    # STEP 4 -- distance from J1 to the target point
    # ========================================================

    D_sq = R ** 2 + H ** 2
    D = math.sqrt(D_sq)

    max_reach = cfg.LINK1 + cfg.LINK2
    min_reach = abs(cfg.LINK1 - cfg.LINK2)

    if D > max_reach:
        raise ValueError(
            f"Target too far: {D:.2f} mm (max reach {max_reach:.2f} mm)"
        )

    if D < min_reach:
        raise ValueError(
            f"Target too close: {D:.2f} mm (min reach {min_reach:.2f} mm)"
        )

    # ========================================================
    # STEP 5 -- alpha: angle from J1 to target, from +horizontal
    # ========================================================

    alpha = math.atan2(H, R)

    # ========================================================
    # STEP 6 -- law of cosines for gamma
    # ========================================================

    cos_gamma = (
        D_sq + cfg.LINK1 ** 2 - cfg.LINK2 ** 2
    ) / (
        2 * D * cfg.LINK1
    )

    cos_gamma = clamp(cos_gamma, -1.0, 1.0)

    gamma = math.acos(cos_gamma)

    # ========================================================
    # STEP 7 -- link1_angle, ONE configuration only.
    #
    # This matches the "Link 1 -> up/right" posture from your
    # drawing. If your arm physically bends the other way, change
    # this to: link1_angle = alpha - gamma
    # ========================================================

    link1_angle = alpha + gamma

    # ========================================================
    # STEP 8 -- absolute direction of Link 2
    # ========================================================

    link1_x = cfg.LINK1 * math.cos(link1_angle)
    link1_y = cfg.LINK1 * math.sin(link1_angle)

    link2_x = R - link1_x
    link2_y = H - link1_y

    link2_angle = math.atan2(link2_y, link2_x)

    # ========================================================
    # STEP 9 -- theta2: convert to your downward-zero convention
    #
    # Your servo's physical zero points straight down. In this
    # math's convention, "straight down" is -90 deg from
    # +horizontal, so shifting by +90 deg maps that physical zero
    # to a mathematical theta2 of 0, matching how you re-zeroed
    # the motor.
    # ========================================================

    theta2 = link2_angle + math.pi / 2

    # ========================================================
    # STEP 10 -- return only what the physical motors need
    # ========================================================

    return {
        "base_angle": round(math.degrees(base_angle), 2),
        "link1_angle": round(math.degrees(link1_angle), 2),
        "theta2": round(math.degrees(theta2), 2),
        "R": round(R, 2),
    }


# ============================================================
# EXAMPLE
# ============================================================
#
# Trash centered, ultrasonic says 150 mm away:
#
#     result = inverse_kinematics(0, 150)
#     print(result)
#     # {'base_angle': 0.0, 'link1_angle': 68.13, 'theta2': 52.28, 'R': 150.0}
#
# Keep this commented out when using controller.py.
