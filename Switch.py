import RPi.GPIO as GPIO
import os
import time

def power_off_callback(channel):
    GPIO.cleanup()
    print("call")
    os.system("sudo shutdown -h now")

GPIO.setmode(GPIO.BCM)

GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(23, GPIO.FALLING, callback=power_off_callback, bouncetime=300)

time.sleep(1000)
