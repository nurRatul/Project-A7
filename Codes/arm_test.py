import serial
import time
import os
import random


for i in range (0,10):
   try:
      arm = serial.Serial(f'/dev/ttyACM{i}', 115200, timeout=5)
      break
   except serial.SerialException as e:
      print(f"Error opening serial port: {e}")
      time.sleep(1)
if not arm:
   exit(1)


print("Initializing arm") 
time.sleep(2)
arm.write(b'H0,90,20,90,90,73,20\n')  #home the arm at low speeds
time.sleep(2)



def write_arduino(angles):
    angle_string=','.join([str(elem) for elem in angles])  # join the list values togheter
    angle_string="P"+angle_string+",100\n"    
    print(angle_string)
    arm.write(angle_string.encode())          #.encode encodes the string to bytes
    arm.flush()

    reply = arm.readline().decode(errors="ignore").strip()
    print("Arduino:", reply)
            

write_arduino([40,110,110,180,0,20])  #home the arm at low speeds


for i in range(0,181,20):
   write_arduino([i,i,i,i,i,i])
   time.sleep(.5)

   
for i in range(15,61,5):
   write_arduino([40,110,110,180,0,i])
   time.sleep(.5)


# Grabbing magic------->

write_arduino([60,110,110,0,90,100])
time.sleep(2)
write_arduino([60,110,110,0,90,10])
time.sleep(.25)
write_arduino([60,170,60,180,180,10])
time.sleep(.25)
write_arduino([60,170,60,180,180,100])
time.sleep(.25)
write_arduino([60,110,110,0,0,100])
time.sleep(.25)

#<---------------####

write_arduino([60,110,110,0,80,10])
time.sleep(2)
while True:
   x= random.randint(90,120)
   y= random.randint(10,90)

   t=x-y+90-15
   # t=t if t<=120 else 120
    
   write_arduino([60,x,y,0,t,10])
   time.sleep(2)
   