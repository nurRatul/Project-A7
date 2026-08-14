import math


# ============================================================
# ARM DIMENSIONS
# ============================================================

L1 = 95.0       # J1 -> J2
L2 = 120.0      # J2 -> J3
L3 = 133.0      # J3 -> J4
LG = 35.0       # J4 -> gripper


# ============================================================
# HELPER
# ============================================================

def clamp(value, minimum, maximum):

    return max(
        minimum,
        min(value, maximum)
    )


# ============================================================
# INVERSE KINEMATICS
# ============================================================

def inverse_kinematics(
        x,
        y,
        z,
        phi=-90.0):

    """
    Input:

        x = forward distance from J1 (mm)
        y = sideways distance from J1 (mm)
        z = height above J1 (mm)

        phi = desired gripper orientation

              0°   = horizontal
              -90° = downward
              +90° = upward

    Output:

        theta1
        theta2
        theta3
        theta4

    These are MATHEMATICAL angles.

    They are NOT yet guaranteed to correspond
    directly to your physical servo angles.
    """


    # ========================================================
    # J1
    # ========================================================

    theta1 = math.atan2(y, x)


    # Horizontal distance

    r = math.sqrt(
        x * x +
        y * y
    )


    # ========================================================
    # J1 -> J2
    # ========================================================

    h = z - L1


    # ========================================================
    # Remove gripper length
    # ========================================================

    phi_rad = math.radians(phi)


    wrist_r = (
        r -
        LG * math.cos(phi_rad)
    )


    wrist_z = (
        h -
        LG * math.sin(phi_rad)
    )


    # ========================================================
    # Distance J2 -> J4
    # ========================================================

    d2 = (
        wrist_r * wrist_r +
        wrist_z * wrist_z
    )

    d = math.sqrt(d2)


    # Reachability

    maximum_reach = L2 + L3

    minimum_reach = abs(
        L2 - L3
    )


    if d > maximum_reach:

        raise ValueError(
            f"Target too far: "
            f"{d:.2f} mm "
            f"(max {maximum_reach:.2f} mm)"
        )


    if d < minimum_reach:

        raise ValueError(
            f"Target too close: "
            f"{d:.2f} mm "
            f"(min {minimum_reach:.2f} mm)"
        )


    # ========================================================
    # J3
    # ========================================================

    cos_theta3 = (
        d2 -
        L2 * L2 -
        L3 * L3
    ) / (
        2 * L2 * L3
    )


    cos_theta3 = clamp(
        cos_theta3,
        -1.0,
        1.0
    )


    theta3 = math.acos(
        cos_theta3
    )


    # ========================================================
    # J2
    # ========================================================

    alpha = math.atan2(
        wrist_z,
        wrist_r
    )


    beta = math.atan2(
        L3 * math.sin(theta3),
        L2 +
        L3 * math.cos(theta3)
    )


    theta2 = (
        alpha -
        beta
    )


    # ========================================================
    # J4
    # ========================================================

    theta4 = (
        phi_rad -
        theta2 -
        theta3
    )


    # ========================================================
    # Radians -> degrees
    # ========================================================

    theta1 = math.degrees(theta1)
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)
    theta4 = math.degrees(theta4)


    return (
        theta1,
        theta2,
        theta3,
        theta4
    )


# ============================================================
# SERVO CONVERSION
# ============================================================

def to_servo_angles(
        theta1,
        theta2,
        theta3,
        theta4):

    """
    Converts mathematical angles to servo angles.

    These offsets/signs MUST be calibrated against
    your physical arm.
    """


    J1_OFFSET = 90.0
    J2_OFFSET = 90.0
    J3_OFFSET = 90.0
    J4_OFFSET = 90.0


    J1_SIGN = 1
    J2_SIGN = 1
    J3_SIGN = 1
    J4_SIGN = 1


    j1 = (
        J1_OFFSET +
        J1_SIGN * theta1
    )

    j2 = (
        J2_OFFSET +
        J2_SIGN * theta2
    )

    j3 = (
        J3_OFFSET +
        J3_SIGN * theta3
    )

    j4 = (
        J4_OFFSET +
        J4_SIGN * theta4
    )


    angles = [
        j1,
        j2,
        j3,
        j4
    ]


    # Check servo limits

    for i, angle in enumerate(angles):

        if not 0 <= angle <= 180:

            raise ValueError(
                f"J{i + 1} = "
                f"{angle:.2f}° "
                f"is outside 0-180°"
            )


    return angles


# ============================================================
# DIRECT IK TEST
# ============================================================

if __name__ == "__main__":

    print()
    print("==============================")
    print("IK TEST")
    print("==============================")


    # Target in mm

    x = 220
    y = 50
    z = 150


    # Gripper downward

    phi = -90


    try:

        angles = inverse_kinematics(
            x,
            y,
            z,
            phi
        )


        print()
        print("Target:")
        print(
            f"X = {x} mm"
        )
        print(
            f"Y = {y} mm"
        )
        print(
            f"Z = {z} mm"
        )


        print()
        print("Mathematical angles:")

        print(
            f"J1 = {angles[0]:.2f}°"
        )

        print(
            f"J2 = {angles[1]:.2f}°"
        )

        print(
            f"J3 = {angles[2]:.2f}°"
        )

        print(
            f"J4 = {angles[3]:.2f}°"
        )


        print()
        print("Servo angles:")


        servo = to_servo_angles(
            *angles
        )


        for i, angle in enumerate(
            servo,
            start=1
        ):

            print(
                f"J{i} = "
                f"{angle:.2f}°"
            )


    except ValueError as error:

        print()
        print(
            "IK ERROR:",
            error
        )