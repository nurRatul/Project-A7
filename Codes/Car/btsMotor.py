from time import sleep

# ----- Temporary, remove after setiing into Pi ------ #
# PWMOutputDevice = None
# DigitalOutputDevice = None
# ---------------------------------------------------- #

from gpiozero import PWMOutputDevice, DigitalOutputDevice

class BTSMotor:

    def __init__(self, rpwm, lpwm, ren, len, pwm_frequency=1000):
        self.rpwm = PWMOutputDevice(rpwm, frequency=pwm_frequency)
        self.lpwm = PWMOutputDevice(lpwm, frequency=pwm_frequency)
        self.ren = DigitalOutputDevice(ren)
        self.len = DigitalOutputDevice(len)

        self.ren.on()
        self.len.on()
        self.stop()

    def forward(self, speed):
        speed = max(0.0, min(1.0, speed))
        self.lpwm.value = 0
        self.rpwm.value = speed

    def backward(self, speed):
        speed = max(0.0, min(1.0, speed))
        self.rpwm.value = 0
        self.lpwm.value = speed

    def stop(self):
        self.rpwm.value = 0
        self.lpwm.value = 0

    def close(self):
        self.stop()
        self.ren.off()
        self.len.off()
        self.rpwm.close()
        self.lpwm.close()
        self.ren.close()
        self.len.close()
