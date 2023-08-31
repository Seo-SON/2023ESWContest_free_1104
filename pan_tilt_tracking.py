from multiprocessing import Manager
from multiprocessing import Process
from Servo import servo
from Sonar import sonar_uart, sonar_hc_sr04
from Vibrator import vibrator
from VoiceAlarm import *
import signal, time, sys, cv2, queue, threading, os, torch, argparse

sys.path.append("/home/pi/wst/WST-main/ver_2/Yolo-FastestV2")
import model.detector
import utils.utils


parser = argparse.ArgumentParser()
parser.add_argument('--data', type=str, default='',
                        help='Specify training profile *.data')

opt = parser.parse_args()
cfg = utils.utils.load_datafile(opt.data)


#user defined capture class
#to reduce delay
class VideoCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.cap.set(3, 320)
        self.cap.set(4, 240)
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
priority_id_inverse = [0, 2, 3, 6, 7, 1, 4, 5]


# function to handle keyboard interrupt
def signal_handler(sig, frame):
	# print a status message
	#print("[INFO] You pressed `ctrl + c`! Exiting...")
	# disable the servos
    servo.pwmActive(False)
    vibrator.turn_off()
	# exit
    sys.exit()


def obj_center(clsId, angle, trafLight):
	# signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    
    prev_time = 0
    angle.value = None
    
    cap = VideoCapture(0)

    pan = 0.0
    tlt = 0.0
    
    # initialize the object center finder
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model_ = model.detector.Detector(cfg["classes"], cfg["anchor_num"], True).to(device)
    model_.load_state_dict(torch.load('/home/pi/wst/WST-main/ver_2/coco-400-epoch-0.538768ap-model.pth', map_location=device))

    #sets the module in eval node
    model_.eval()

    time.sleep(1.0)

    # loop indefinitely
    while True:
        frame = cap.read()

        (H, W) = frame.shape[:2]

        centerX = H/2
        centerY = W/2

        current_time = time.time() - prev_time

        print(current_time)
        
        res_img = cv2.resize(frame, (cfg["width"], cfg["height"]), interpolation = cv2.INTER_LINEAR)
        img = res_img.reshape(1, cfg["height"], cfg["width"], 3)
        img = torch.from_numpy(img.transpose(0,3, 1, 2))
        img = img.to(device).float() / 255.0

        #inference
        start = time.perf_counter()
        with torch.no_grad():
            preds = model_(img)
        end = time.perf_counter()
        time_ = (end - start) * 1000.
        print("forward time:%fms"%time_)

        #output
        output = utils.utils.handel_preds(preds, cfg, device)
        output_boxes = utils.utils.non_max_suppression(output, conf_thres = 0.3, iou_thres = 0.4)

        scale_h, scale_w = float(H / 352), float(W / 352)

        objX = centerX
        objY = centerY

        prev_time = time.time()


        array_wh = [0 for i in range(8)] # 바운딩 박스 크기 저장
        array_center = [(0,0) for i in range(8)] # 바운딩 박스 센터 저장
        trafLightCenter = [] #신호등 좌표 저장
        trafLightId = [] #신호등 레이블 저장

        for box in output_boxes[0]:
            box = box.tolist()
            
            id_ = int(box[5])
            print("id: " + str(id_))
            x1, y1 = float(box[0] * scale_w), float(box[1] * scale_h)
            x2, y2 = float(box[2] * scale_w), float(box[3] * scale_h)

            #center of bounding box
            coordX = (x1 + x2) / 2
            coordY = (y1 + y2) / 2

            if id_ == 8 or id_ == 9 or id_ == 10: #신호등 좌표는 따로 저장
                trafLightCenter.append((coordX, coordY))
                trafLightId.append(id_)
                continue

            arrayIdx = priority_id[id_]

            # 더 큰 바운딩 박스를 갖는 객체를 우선으로 한다
            if array_wh[arrayIdx] == 0:
                array_wh[arrayIdx] = abs((x1 - x2)) * abs((y1 - y2))
                array_center[arrayIdx] = (coordX, coordY)
            else:
                wh = abs((x1 - x2)) * abs((y1 - y2))
                if wh > array_wh[arrayIdx]:
                    array_wh[arrayIdx] = wh
                    array_center[arrayIdx] = (coordX, coordY)


        trafLightId_ = -1
        if array_wh[0] != 0 and len(trafLightCenter) > 0: #횡단보도 감지 & 신호등 감지
            
            distTrafLight = 1000000
            for i in range(len(trafLightCenter)):
                distTemp = (trafLightCenter[i][0] - centerX)^2 + (trafLightCenter[i][1] - centerY)^2
                if distTrafLight > distTemp:
                    trafLightId_ = trafLightId[i]

        trafLight.value = trafLightId_ # if trafLight.value = -1 신호등 없음
        

        #select highest priority
        priority = 8
        for i in range(8):
            if array_wh[i] != 0 and array_center != (0,0):
                priority = i
                break
        
        
        if priority != 8:
            clsId.value = priority_id_inverse[priority]
            (objX, objY) = array_center[priority]
        else:
            clsId.value = 8
            pan = 0.0
            tlt = 0.0


        # calculate error
        errorX = float((objX - centerX) * 5 / centerX)
        errorY = float((objY - centerY) * 5 / centerY)
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


        if abs(errorX) < 3:
            angle.value = int(-1 * pan)
        else:
            angle.value = None

        #panning and tilting
        if in_range(pan, servoPanRange[0], servoPanRange[1]):
            servo.pan(-1 * pan)
        if in_range(tlt, servoTiltRange[0], servoTiltRange[1]):
            servo.tilt(tlt)


def in_range(val, start, end):
    return (val >= start and val <= end)


def distance(sonar, distance):
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        distance.value = sonar.getDistance()


def vibrateAlarm(distance1, distance2):
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        distance = min(distance1.value, distance2.value)
        distance = distance2.value
        vibrator.alarm(distance)


def main_voiceAlarm(clsId, distance, angle, trafLight):
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        if clsId.value != 8 and angle.value != None:
            voiceAlarm(clsId.value, distance.value, angle.value, trafLight.value)
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

        trafLight = manager.Value("i", -1)

	# 1. objectCenter  - finds the object's coord, pan & tilt
	# 2. distanceForward - detect forward object with sonar_hc_sr04(fixed direction)
	# 3. distanceTarget - detect target object with sonar_uart(same direction with camera)
	# 4. vibrateAlarm - vibrate with distance
	# 5. voiceAlarm - alarm with voice
        processObjectCenter = Process(target=obj_center,
                args = (clsId, angle, trafLight))

        processDistanceForward = Process(target=distance,
			args=(sonar_hc_sr04, distance1))
        processDistanceTarget = Process(target=distance,
			args=(sonar_uart, distance2))
        processVibrateAlarm = Process(target=vibrateAlarm,
			args=(distance1, distance2))
        processVoiceAlarm = Process(target=main_voiceAlarm, 
            args=(clsId, distance2, angle, trafLight))
		
        # start all 8 processes
        processObjectCenter.start()

        processDistanceForward.start()
        processDistanceTarget.start()
        #processVibrateAlarm.start()
        processVoiceAlarm.start()

        # join all 4 processes
        processObjectCenter.join()

        processDistanceForward.join()
        processDistanceTarget.join()
        #processVibrateAlarm.join()
        processVoiceAlarm.join()

        # disable the servos
        servo.pwmActive(False)
