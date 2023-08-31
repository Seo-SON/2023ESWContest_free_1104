from multiprocessing import Manager
from multiprocessing import Process
from servo_v2 import servo
from ultrasonic_v2 import sonar_uart
from vibrator_v2 import vibrator
from ultralytics import YOLO
from voiceAlarm_v2 import *
import signal, time, sys, cv2, queue, threading


#user defined capture class
#to reduce delay
class VideoCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.cap.set(3, 320)
        self.cap.set(4, 240)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.q = queue.Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()   # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.q.get()


# define the range for the motors
servoPanRange = (-90, 90)
servoTiltRange = (-35, 35)

priority_id = [0, 5, 1, 2, 6, 7, 3, 4]

# function to handle keyboard interrupt
def signal_handler(sig, frame):
	# print a status message
	#print("[INFO] You pressed `ctrl + c`! Exiting...")
	# disable the servos
    servo.pwmActive(False)
    vibrator.turn_off()
	# exit
    sys.exit()


def obj_center(clsId, angle):
	# signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    
    prev_time = 0
    angle.value = None
    
    cap = VideoCapture(0)

    pan = 0.0
    tlt = 0.0
    time.sleep(1.0)
	# initialize the object center finder
    model = YOLO("/home/pi/wst/WST-main/ver_2/best_200.pt")
	# loop indefinitely
    while True:
        frame = cap.read()
        current_time = time.time() - prev_time
        
        #cv2.imshow('image',frame)
        
        #cv2.waitKey(1)

        print(current_time)
        
        (H, W) = frame.shape[:2]
        centerX = H/2
        centerY = W/2

        objX = centerX
        objY = centerY

        prev_time = time.time()

		# find the object's location
        results = model.predict(frame)

        priority = 8
        id = 8

        for r in results:
            boxes = r.boxes
            for box in boxes:
                id_ = int(box.cls[0].item())
                coord = box.xywh[0].tolist()

                if priority > priority_id[id_]:
                    priority = priority_id[id_]
                    id = id_
                    center = (coord[0], coord[1])

        if id != 8:
            (objX, objY) = center
        else:
            pan = 0.0
            tlt = 0.0
        errorX = (objX - centerX) * 10 / centerX
        errorY = (objY - centerY) * 10 / centerY
        print("errorX: " + str(errorX))
        print("errorY: " + str(errorY))
        print("frameCenter: " + str(centerX) + ", " + str(centerY))
        print("objCenter: " + str(objX) + ", " + str(objY))
            
        if abs(errorX) > 1 and abs(errorX) <= 3 :
            pan += errorX

        if abs(errorX) > 3:
            pan += 2 * errorX

        if abs(errorY) > 1 and abs(errorY) <= 3 :
            tlt += errorY

        if abs(errorY) > 3:
            tlt += 2 * errorY
            
        clsId.value = id
        if abs(errorX) < 3:
            angle.value = int(-1 * pan)
        else:
            angle.value = None

        if in_range(pan, servoPanRange[0], servoPanRange[1]):
            servo.pan(-1 * pan)
        if in_range(tlt, servoTiltRange[0], servoTiltRange[1]):
            servo.tilt(tlt)


def in_range(val, start, end):
	# determine the input value is in the supplied range
    return (val >= start and val <= end)


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


def main_voiceAlarm(clsId, distance, angle):
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        if clsId.value != 8 and angle.value != None:
            voiceAlarm(clsId.value, distance.value, angle.value)
        time.sleep(1)

			
# check to see if this is the main body of execution
if __name__ == "__main__":
	
    # start a manager for managing process-safe variables
    with Manager() as manager:
		# enable the servos
        servo.pwmActive(True)
		
        clsId = manager.Value("i", 8)
        angle = manager.Value("i", 0)

		# sonar sensor
        distance1 = manager.Value("f", 5000.0)
        distance2 = manager.Value("f", 5000.0)


        # we have 5 independent processes
		# 1. objectCenter  - finds the object's coord, pan & tilt
		# 2. distanceForward - detect forward object with sonar(fixed direction)
		# 3. distanceTarget - detect target object with sonar(same direction with camera)
		# 4. vibrataeAlarm - vibrate with distance
		# 5. voiceAlarm - alarm with voice
        processObjectCenter = Process(target=obj_center,
                args = (clsId, angle))

		#processDistanceForward = Process(target=distance,
		#	args=(sonar_uart, distance1))
        processDistanceTarget = Process(target=distance,
			args=(sonar_uart, distance2))
        processVibrateAlarm = Process(target=vibrateAlarm,
			args=(distance1, distance2))
        processVoiceAlarm = Process(target=main_voiceAlarm, 
            args=(clsId, distance2, angle))
		
		# start all 8 processes
        processObjectCenter.start()

		#processDistanceForward.start()
        processDistanceTarget.start()
        processVibrateAlarm.start()
        processVoiceAlarm.start()

		# join all 4 processes
        processObjectCenter.join()

		#processDistanceForward.join()
        processDistanceTarget.join()
        processVibrateAlarm.join()
        processVoiceAlarm.join()

		# disable the servos
        servo.pwmActive(False)
