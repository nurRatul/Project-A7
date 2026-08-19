from Arm import Controller
from Arm.detector import BottleDetector
from Car.carController import CarController

detector = BottleDetector(camera_id=0)
arm = Controller(detector=detector, show_video=True)
car = CarController()

try:
    while True:
        frame = detector.read_frame()
        if frame is None:
            break

        detections = detector.detect(frame)

        car.update_vision(frame, detections)
        angles = arm.update(frame, detections)

        if detections:
            target = detections[0]
            print("Target:", target["center"])

            # Add car navigation logic here.
            # Example: car.turn_left(...) or car.move_forward(...)

finally:
    arm.close()