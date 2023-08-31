# vibration motor
# last update: 23.06.25 / seohyeon

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import OutputDevice
import time

pigpio_factory__ = PiGPIOFactory()

class VibrateAlarm:
    def __init__(self, pinNum = 16):
        self.motor = OutputDevice(pin=pinNum, active_high=True, initial_value=False, pin_factory=pigpio_factory__)

# time_ms만큼 진동
    def vibrate(self, time_ms):
        self.motor.on()
        time.sleep(time_ms)
        self.motor.off()
        time.sleep(time_ms)


    def dangerRate(self, distance):
        if distance == None or distance > 2000:
            return 'NONE'
        elif distance > 1500:
            return 'LOW'
        elif distance > 1000:
            return 'MID'
        else:
            return 'HIGH'
        
    def alarm(self, distance):
        if self.dangerRate(distance) == 'NONE':
            time.sleep(1)
        elif self.dangerRate(distance) == 'LOW':
            self.vibrate(1)
        elif self.dangerRate(distance) == 'MID':
            self.vibrate(0.6)
        elif self.dangerRate(distance) == 'HIGH':
            self.vibrate(0.3)

    def turn_off(self):
        time.sleep(0.5)
        self.motor.off()
        time.sleep(0.5)

vibrator = VibrateAlarm(16)

if __name__ == "__main__":
    while True:
        vibrator.vibrate(1)
        print("a")

