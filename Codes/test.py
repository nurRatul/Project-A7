import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "controller"))

from braccio_control_python import *

# backlash()

pos = [0, 0, 0, 0, 0, 0]

# for j in range(0,6,1):
#     for i in range(0,180,30):
#         pos[j] += i
#         write_arduino(pos)
#         time.sleep(2)
#         print(pos)

for i in range(0,190,180):
    pos[0] =  i
    print(pos)
    write_arduino(pos.copy())
    time.sleep(2)
