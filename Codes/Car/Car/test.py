from carController import CarController

car = CarController()

car.move_forward(deltaT=2, speed=0.5)
car.move_backward(deltaT=2, speed=0.5)

# from gpiozero import PWMOutputDevice, DigitalOutputDevice

# pin = PWMOutputDevice(23, frequency=1000)
# pin.value = 1;
