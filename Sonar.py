from DFRobot_RaspberryPi_A02YYUW import DFRobot_A02_Distance as Board
import time, RPi.GPIO as GPIO

GPIO.setwarnings(False)

class Sonar_uart:
    def __init__(self):
        self.board = Board()

        dis_min = 0   #Minimum ranging threshold: 0mm
        dis_max = 4500 #Highest ranging threshold: 4500mm
        self.board.set_dis_range(dis_min, dis_max)

# distance 리턴
    def getDistance(self):
        distance = self.board.getDistance()
        time.sleep(0.3)

        if self.board.last_operate_status == self.board.STA_OK:
            return distance
        elif self.board.last_operate_status == self.board.STA_ERR_CHECKSUM: #error
            print("sonar error")
        
        return None


class Sonar_hc_sr04:
    def __init__(self, trig = 23, echo = 24):
        self.trig = trig
        self.echo = echo
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.5)

    def getDistance(self):
        GPIO.output(self.trig, False)
        time.sleep(0.3)
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        while GPIO.input(self.echo) == 0:
            start = time.time()
        
        while GPIO.input(self.echo) == 1:
            stop = time.time()

        time_interval = stop - start
        distance = time_interval * 17000
        distance = round(distance, 2)

        return distance * 10


sonar_uart = Sonar_uart()
sonar_hc_sr04 = Sonar_hc_sr04()


if __name__ == "__main__":
    while True:
        print(sonar_uart.getDistance())
        print(sonar_hc_sr04.getDistance())
