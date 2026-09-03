from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(
    echo=24,
    trigger=23,
    max_distance=2.0
)

try:
    while True:
        distance_mm = sensor.distance * 1000
        print(f"Distance: {distance_mm:.1f} mm")
        sleep(0.5)

except KeyboardInterrupt:
    sensor.close()
    print("Stopped")
