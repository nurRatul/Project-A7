# Braccio Servo Wiring

Pin assignments come from the `BraccioRobot` library (`BraccioRobot.cpp` -> `init()`), which the sketch `arduino_python_script3.ino` uses.

## Servo signal (data) pins

| Joint | Arduino pin | Python variable name | Notes |
|-------|-------------|----------------------|-------|
| Base (rotation) | **11** | `base` (index 0) | |
| Shoulder | **10** | `shoulder` (index 1) | |
| Elbow | **9** | `elbow` (index 2) | |
| Wrist (vertical axis) | **6** | `wrist` (index 3) | |
| Wrist rotation | **5** | `wristRot` (index 4) | |
| Gripper | **3** | `gripper` (index 5) | |

## Power / enable pin

| Function | Arduino pin | Behavior |
|----------|-------------|----------|
| Servo power (soft-start / enable) | **12** | `LOW` = power ON, `HIGH` = power OFF (managed by `BraccioRobot.powerOn()` / `powerOff()` and `init()`) |

This is the enable line of the Braccio shield (V1.6 or later). If you wire the servos directly without the shield, connect the servo 5 V supply through this line as well, or leave pin 12 unused (then `powerOn()`/`powerOff()` do nothing).

## Servo connector colors (standard servo cable)

| Wire | Connect to |
|------|------------|
| Orange (signal) | Arduino digital pin from the table above |
| Red (+5 V) | Braccio shield servo power (or external 5 V supply) |
| Brown (GND) | Braccio shield GND (or external supply GND, shared with Arduino GND) |

## Important notess

- **Do NOT power 6 servos from the Arduino's own 5 V pin** — current draw can exceed 1 A. Use the Braccio power shield with its external power adapter (or a separate 5 V supply), and make sure it shares a common GND with the Arduino.
- This pin map matches the official TinkerKit Braccio shield header (D11, D10, D9, D6, D5, D3).
- The `H0,90,20,90,90,73,20` home command in `braccio_control_python.py` sends: base=90, shoulder=90, elbow=90, wrist=90, wristRot=90, gripper=73, speed=20.
