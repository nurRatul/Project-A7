import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "controller"))

from my_code import Controller
from controller.braccio_control_python import write_arduino


controller = Controller(
    show_video=True,
    debug=False
)


# for i in range(0,190,30):
#     write_arduino([i, 0, 71, 10,0,0])
#     time.sleep(2)



try:

    while True:

        angles = controller.update()

        if angles is not None:

            print("ARM ANGLES:", angles)

            # Your arm code here
            # arm.move(angles)
            arm_ang=angles
            arm_ang.append(0)
            arm_ang.append(0)
            arm_ang = [int(x) for x in arm_ang]

            print(f'arm_ang:{arm_ang}')
            write_arduino(arm_ang)



except KeyboardInterrupt:

    pass

finally:

    controller.close()