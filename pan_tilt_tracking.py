from multiprocessing import Manager
from multiprocessing import Process
from objCenter import ObjCenter
from servo_v2 import servo
from ultrasonic_v2 import sonar_uart
from vibrator_v2 import vibrator

from voiceAlarm_v2 import *
import argparse
import signal
import time
import sys
import cv2
from ultralytics.yolo.utils.plotting import Annotator

# define the range for the motors
servoPanRange = (-90, 90)
servoTiltRange = (-35, 35)

# function to handle keyboard interrupt
def signal_handler(sig, frame):
	# print a status message
	#print("[INFO] You pressed `ctrl + c`! Exiting...")
	# disable the servos
    servo.pwmActive(False)
    vibrator.turn_off()
	# exit
    sys.exit()
	
def obj_center(objX, objY, centerX, centerY, clsId):
	# signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
	# start the video stream and wait for the camera to warm up
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    time.sleep(1.0)
	# initialize the object center finder
    obj = ObjCenter("/home/pi/wst/WST-main/ver_2/best_200.pt")
	# loop indefinitely
    while True:
        _, frame = cap.read()
        (H, W) = frame.shape[:2]
        centerX.value = H/2
        centerY.value = W/2
		# calculate the center of the frame as this is where we will
		# try to keep the object
		# find the object's location
        objectLoc = obj.update(frame, (centerX.value, centerY.value))
        ((objX.value, objY.value), clsId.value) = objectLoc
        cv2.waitKey(1)
		
def pid_process(output, objCoord, centerCoord, clsId):
	# signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
	# loop indefinitely
    output.value = 0.0
    error_ = 0.0
    while True:
        if clsId.value == None:
            output.value = 0.0
            error_ = 0.0
            time.sleep(0.5)
            continue
        objCenter = objCoord.value
        frameCenter = centerCoord.value
        error_ = objCenter - frameCenter
        error_ /= frameCenter
        print(error_)
        # update the value
        output.value += error_ * 2.5
        time.sleep(0.5)
		

def in_range(val, start, end):
	# determine the input value is in the supplied range
    return (val >= start and val <= end)


def set_servos(pan, tlt):
	# signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
	# loop indefinitely
    while True:
		# the pan and tilt angles are reversed
        panAngle = -1 * pan.value
        tiltAngle = tlt.value
		# if the pan angle is within the range, pan
        if in_range(panAngle, servoPanRange[0], servoPanRange[1]):
            #print(panAngle)
            servo.pan(panAngle)
		# if the tilt angle is within the range, tilt
        if in_range(tiltAngle, servoTiltRange[0], servoTiltRange[1]):
            #print(tiltAngle)
            servo.tilt(tiltAngle)


def distance(sonar, distance):
	signal.signal(signal.SIGINT, signal_handler)
	while True:
		distance.value = sonar.getDistance()


def vibrateAlarm(distance1, distance2):
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        #distance = min(distance1.value, distance2.value)
        distance = distance2.value
        vibrator.alarm(distance)
        print(distance)


def voiceAlarm(clsId, distance, angle):
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        if clsId != None:
            voiceAlarm(int(clsId.value), int(distance.value), int(-1 * angle.value))
            time.sleep(1)

			
# check to see if this is the main body of execution
if __name__ == "__main__":
	# construct the argument parser and parse the arguments
	#ap = argparse.ArgumentParser()
	#ap.add_argument("-c", "--cascade", type=str, required=True,
	#	help="path to input Haar cascade for detection")
	#args = vars(ap.parse_args())
	
    # start a manager for managing process-safe variables
    with Manager() as manager:
		# enable the servos
        servo.pwmActive(True)
		# set integer values for the object center (x, y)-coordinates
        centerX = manager.Value("i", 320)
        centerY = manager.Value("i", 240)
		# set integer values for the object's (x, y)-coordinates
        objX = manager.Value("i", 320)
        objY = manager.Value("i", 240)
		# pan and tilt values will be managed by independed PIDs
        pan = manager.Value("i", 0)
        tlt = manager.Value("i", 0)
		
        clsId = manager.Value("i", 8)
		
		# sonar sensor
        distance1 = manager.Value("f", 5000.0)
        distance2 = manager.Value("f", 5000.0)


        # we have 8 independent processes
		# 1. objectCenter  - finds/localizes the object
		# 2. panning       - PID control loop determines panning angle
		# 3. tilting       - PID control loop determines tilting angle
		# 4. setServos     - drives the servos to proper angles based
		#                    on PID feedback to keep object in center
		# 5. distanceForward - detect forward object (fixed location)
		# 6. distanceTarget - detect target object (same location with camera)
		# 7. vibrataeAlarm - vibrate with distance
		# 8. voiceAlarm
        processObjectCenter = Process(target=obj_center,
			args=(objX, objY, centerX, centerY, clsId))
        processPanning = Process(target=pid_process,
			args=(pan, objX, centerX, clsId))
        processTilting = Process(target=pid_process,
			args=(tlt, objY, centerY, clsId))
        processSetServos = Process(target=set_servos, args=(pan, tlt))

		#processDistanceForward = Process(target=distance,
		#	args=(sonar_uart, distance1))
        processDistanceTarget = Process(target=distance,
			args=(sonar_uart, distance2))
        processVibrateAlarm = Process(target=vibrateAlarm,
			args=(distance1, distance2))
        #processVoiceAlarm = Process(target=voiceAlarm,
		#	args=(clsId, distance2, pan))
		
		# start all 8 processes
        processObjectCenter.start()
        processPanning.start()
        processTilting.start()
        processSetServos.start()

		#processDistanceForward.start()
        processDistanceTarget.start()
        processVibrateAlarm.start()
        #processVoiceAlarm.start()

		# join all 4 processes
        processObjectCenter.join()
        processPanning.join()
        processTilting.join()
        processSetServos.join()

		#processDistanceForward.join()
        processDistanceTarget.join()
        processVibrateAlarm.join()
        #processVoiceAlarm.join()

		# disable the servos
        servo.pwmActive(False)
