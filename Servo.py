# servo motor v2 / pan & tilt

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
import time

pigpio_factory = PiGPIOFactory()
pigpio_factory_ = PiGPIOFactory()

class servo:
    def __init__(self, panPin = 12, tiltPin = 13):
        self.panPin = panPin
        self.tiltPin = tiltPin

        self.panServo = AngularServo(panPin, min_angle=-90, max_angle=90, pin_factory=pigpio_factory)
        self.tiltServo = AngularServo(tiltPin, min_angle=-40, max_angle=40, pin_factory=pigpio_factory_)


    def pwmActive(self, active = True):
        if active:
            self.panServo.angle = 0.0
            time.sleep(0.1)
            self.tiltServo.angle = 0.0
            time.sleep(0.1)
        else:
            self.panServo.detach()
            self.tiltServo.detach()

    def finish(self):
        self.panServo.angle = 0.0
        time.sleep(0.1)
        self.tiltServo.angle = 0.0
        time.sleep(0.1)
        self.pwmActive(False)
    

    def pan(self, angle):
        self.panServo.angle = angle
        time.sleep(0.02)


    def tilt(self, angle):
        self.tiltServo.angle = angle
        time.sleep(0.02)


servo = servo()

if __name__ == "__main__":
    servo.pwmActive()
    time.sleep(1)
    servo.pan(30)
    servo.tilt(20)
    time.sleep(5)
    servo.finish()
    time.sleep(1)
