import time
import gpiod
from gpiod.line import Direction, Value

TRIG_PIN = 23  # Change to your Trigger GPIO pin number
ECHO_PIN = 24  # Change to your Echo GPIO pin number

chip = gpiod.request_lines(
    "/dev/gpiochip0",  # Use /dev/gpiochip0 on older Pi models / kernels
    consumer="ultrasonic",
    config={
        TRIG_PIN: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
        ECHO_PIN: gpiod.LineSettings(direction=Direction.INPUT),
    },
)

def get_distance():
    # Pulse Trigger pin high for 10us
    chip.set_value(TRIG_PIN, Value.ACTIVE)
    time.sleep(0.00001)
    chip.set_value(TRIG_PIN, Value.INACTIVE)

    # Wait for echo high
    timeout = time.time() + 1
    while chip.get_value(ECHO_PIN) == Value.INACTIVE:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None

    # Wait for echo low
    while chip.get_value(ECHO_PIN) == Value.ACTIVE:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    duration = pulse_end - pulse_start
    distance_cm = (duration * 34300) / 2
    return distance_cm

try:
    while True:
        dist = get_distance()
        if dist:
            print(f"Distance: {dist:.2f} cm")
        else:
            print("Measurement timeout")
        time.sleep(0.5)
except KeyboardInterrupt:
    chip.release()
