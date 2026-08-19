# -*- coding: utf-8 -*-
"""
Created on Fri Aug 6:00 PM 14 2026

@author: nur
"""

from sympy import *
import math
import numpy as np
import os

'''
ch = 0 # Car Height from ground
bh= (60+35+ch) # base height
l1=120 # shoulder length
l2= (90+33) # forward-arm length or elbow length
l3= 0 #wrist length, don't know yet


def move_to_position_cart(x,y,z):
    r_compensation=1.02 #add 2 percent
    z=z+15  #compensation for backlash
    r_hor=sqrt(x**2+y**2)
    ## r=sqrt(r_hor**2+(z-71.5)**2)*r_compensation
    r=sqrt(r_hor**2+(z-l0)**2)*r_compensation
    
    if y==0:
        if x<=0:
            theta_base=180
        else:
            theta_base=0
    else:
        theta_base=90-degrees(atan(x/y))  #add 2 degrees for backlash compensation
    #print(theta_base)
    #theta_base=backlash_compensation_base(theta_base)  #check if compensation is needed
    
    #calulcate angles for level operation
    
    alpha1=acos(((r-l2)/(l1+l3)))
    theta_shoulder=degrees(alpha1)
    alpha3=asin((sin(alpha1)*l3-sin(alpha1)*l1)/l2)  #compensate for the difference in arm length
    theta_elbow=(90-degrees(alpha1))+degrees(alpha3)
    theta_wrist=(90-degrees(alpha1))-degrees(alpha3)
    
    if theta_wrist <=0: #when arm length compensation results in negative values
        alpha1=acos(((r-l2)/(l1+l3)))
        theta_shoulder=degrees(alpha1+asin((l3-l1)/r))
        theta_elbow=(90-degrees(alpha1))
        theta_wrist=(90-degrees(alpha1))
    
    #adjust shoulder angle to increase heigth
    if z!=l0:
        theta_shoulder=theta_shoulder+degrees(atan(((z-l0)/r)))
        #print(degrees(atan(((z-l0)/r))))
    
    #add compensation for bad line-up of servo with mount
    theta_elbow=theta_elbow+5  
    theta_wrist=theta_wrist+5  


    
    
    theta_array=[round(theta_base),round(theta_shoulder),round(theta_elbow),round(theta_wrist)]
    
    return theta_array


'''


# =========================
# ARM DIMENSIONS (mm)
# =========================

L1 = 95.0    # J1 -> J2, vertical
L2 = 120.0   # J2 -> J3
L3 = 133.0   # J3 -> J4
LG = 35.0    # J4 -> gripper tip


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def ik(x, y, z, phi=-90):
    """
    Inverse kinematics for the 4-DOF arm.

    x = forward distance from J1 (mm)
    y = sideways distance from J1 (mm)
    z = height from J1 (mm)

    phi = desired gripper orientation in degrees
          0   = horizontal forward
          -90 = pointing downward

    Returns:
        J1, J2, J3, J4
    """

    # ---------------------------------
    # J1: Base rotation
    # ---------------------------------

    theta1 = math.atan2(y, x)

    # Distance from J1 axis to target in horizontal plane
    r = math.sqrt(x**2 + y**2)

    # ---------------------------------
    # Move reference from J1 to J2
    # ---------------------------------

    h = z - L1

    # ---------------------------------
    # Remove gripper length.
    #
    # The wrist/gripper joint must be
    # LG distance behind the target.
    # ---------------------------------

    phi_rad = math.radians(phi)

    wrist_r = r - LG * math.cos(phi_rad)
    wrist_z = h - LG * math.sin(phi_rad)

    # ---------------------------------
    # Distance J2 -> wrist
    # ---------------------------------

    D2 = wrist_r**2 + wrist_z**2

    D = math.sqrt(D2)

    # ---------------------------------
    # Reachability check
    # ---------------------------------

    if D > L2 + L3:
        raise ValueError(
            f"Target too far: {D:.1f} mm "
            f"(maximum {L2 + L3:.1f} mm)"
        )

    if D < abs(L2 - L3):
        raise ValueError(
            f"Target too close: {D:.1f} mm "
            f"(minimum {abs(L2 - L3):.1f} mm)"
        )

    # ---------------------------------
    # J3 using law of cosines
    # ---------------------------------

    cos_theta3 = (
        D2 - L2**2 - L3**2
    ) / (2 * L2 * L3)

    cos_theta3 = clamp(cos_theta3, -1.0, 1.0)

    # Elbow-down configuration
    theta3 = math.acos(cos_theta3)

    # ---------------------------------
    # J2
    # ---------------------------------

    alpha = math.atan2(wrist_z, wrist_r)

    beta = math.atan2(
        L3 * math.sin(theta3),
        L2 + L3 * math.cos(theta3)
    )

    theta2 = alpha - beta

    # ---------------------------------
    # J4
    #
    # Absolute orientation:
    #
    # phi = J2 + J3 + J4
    # ---------------------------------

    theta4 = phi_rad - theta2 - theta3

    # Convert radians -> degrees
    theta1 = math.degrees(theta1)
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)
    theta4 = math.degrees(theta4)

    # ---------------------------------
    # Convert mathematical angles
    # to servo angles.
    #
    # IMPORTANT:
    # These offsets depend on how your
    # physical servos are mounted.
    # ---------------------------------

    servo1 = theta1
    servo2 = theta2
    servo3 = theta3
    servo4 = theta4

    # ---------------------------------
    # Servo range check
    # ---------------------------------

    angles = [servo1, servo2, servo3, servo4]
    print(angles)

    for i, angle in enumerate(angles, start=1):
        if not 0 <= angle <= 180:
            raise ValueError(
                f"J{i} requires {angle:.2f}°, "
                f"outside servo range 0–180°"
            )

    return angles


# =====================================
# TEST
# =====================================

try:

    # Target position in mm
    x = 220
    y = 50
    z = 80

    # Gripper pointing downward
    phi = -90

    angles = ik(x, y, z, phi)

    print("\nIK Result")
    print("----------------")
    print(f"J1 = {angles[0]:.2f}°")
    print(f"J2 = {angles[1]:.2f}°")
    print(f"J3 = {angles[2]:.2f}°")
    print(f"J4 = {angles[3]:.2f}°")

except ValueError as e:
    print("\nIK Error:")
    print(e)