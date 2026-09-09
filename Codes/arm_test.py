import serial
import time
import os




arm = serial.Serial('COM6', 115200, timeout=5)
print("Initializing arm") 
time.sleep(2)
arm.write(b'H0,90,20,90,90,73,20\n')  #home the arm at low speeds
time.sleep(2)



def write_arduino(angles):
    
    # angles[0]=180-angles[0]  #invert degrees for base
    # angles[3]=180-angles[3]  #invert degrees for base
    angle_string=','.join([str(elem) for elem in angles])  # join the list values togheter
    angle_string="P"+angle_string+",200\n"    
    print(angle_string)
    arm.write(angle_string.encode())          #.encode encodes the string to bytes
            

write_arduino([150,15,0,0,90,73])  #home the arm at low speeds