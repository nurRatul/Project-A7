import math


L1 = 95.0
L2 = 120.0
L3 = 133.0
LG = 0 # 35.0


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def inverse_kinematics(
    x,
    y,
    z,
    phi=-90.0
):

    # -------------------------
    # J1
    # -------------------------

    theta1 = math.atan2(y, x)

    r = math.sqrt(
        x ** 2 +
        y ** 2
    )

    # -------------------------
    # J2 height
    # -------------------------

    h = z - L1

    # -------------------------
    # Remove gripper length
    # -------------------------

    phi_rad = math.radians(phi)

    wrist_r = (
        r -
        LG * math.cos(phi_rad)
    )

    wrist_z = (
        h -
        LG * math.sin(phi_rad)
    )

    # -------------------------
    # J2 -> wrist
    # -------------------------

    d2 = (
        wrist_r ** 2 +
        wrist_z ** 2
    )

    d = math.sqrt(d2)

    max_reach = L2 + L3
    min_reach = abs(L2 - L3)

    if d > max_reach:
        raise ValueError(
            f"Target too far: {d:.2f} mm"
        )

    if d < min_reach:
        raise ValueError(
            f"Target too close: {d:.2f} mm"
        )

    # -------------------------
    # J3
    # -------------------------

    cos_theta3 = (
        d2 -
        L2 ** 2 -
        L3 ** 2
    ) / (
        2 * L2 * L3
    )

    cos_theta3 = clamp(
        cos_theta3,
        -1.0,
        1.0
    )

    theta3_base = math.acos(
        cos_theta3
    )

    # -------------------------
    # J2
    # Try both configurations
    # -------------------------

    alpha = math.atan2(
        wrist_z,
        wrist_r
    )

    solutions = []

    for theta3 in [
        theta3_base,
        -theta3_base
    ]:

        beta = math.atan2(
            L3 * math.sin(theta3),
            L2 + L3 * math.cos(theta3)
        )

        theta2 = alpha - beta

        theta4 = (
            phi_rad -
            theta2 -
            theta3
        )

        # Convert to degrees
        j1 = math.degrees(theta1)
        j2 = math.degrees(theta2)
        j3 = math.degrees(theta3)
        j4 = math.degrees(theta4)

        solutions.append(
            [j1, j2, j3, j4]
        )

    # -------------------------
    # Convert mathematical
    # angles to servo angles
    # -------------------------

    # Pick solution where J2 is positive
    valid = [
        s for s in solutions
        if 0 <= s[1] <= 180
    ]

    if not valid:
        raise ValueError(
            "No valid J2 solution"
        )

    # Prefer solution with
    # positive J3 as well
    positive_j3 = [
        s for s in valid
        if 0 <= s[2] <= 180
    ]

    if positive_j3:
        solution = positive_j3[0]
    else:
        solution = valid[0]

    return [
        round(solution[0], 2),
        round(solution[1], 2),
        round(solution[2], 2),
        round(solution[3], 2)
    ]