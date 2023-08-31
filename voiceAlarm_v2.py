# voice alarm
# last update: 23.06.26 / seohyeon

import os
import time


className = ("횡단보도", "킥보드", "계단", "공사표지판", "기둥", "물체", "오토바이", "자전거")
option = '-s 160 -p 95 -a 200 -v ko+f3'


def speak(option, msg) :
    os.system("espeak {} '{}'".format(option,msg))

def voiceAlarm(id, dist, angle, trafLight):
    if dist > 3000:
        time.sleep(1)
        return

    if angle >= -90 and angle < -70:
        direction = "세 시 방향 "
    elif angle >= -70 and angle < -50:
        direction = "두 시 방향 "
    elif angle >= -50 and angle <20:
        direction = "한 시 방향 "
    elif angle >= -20 and angle < 20:
        direction = "전방 "
    elif angle >= 20 and angle < 50:
        direction = "열한 시 방향 "
    elif angle >= 50 and angle < 70:
        direction = "열 시 방향 "
    else:
        direction = "아홉 시 방향 "

    msg = direction + str(dist // 1000) + " 미터 부근에 " + className[id]
    print(msg)
    speak(option, msg)
    time.sleep(1)

    if id == 0 and trafLight != -1:
        if trafLight == 8:
            color = '빨간 불'
        elif trafLight == 9:
            color = '노란 불'
        else:
            color = '초록 불'

        msg = '신호등은 '+color+' 입니다'

        print(msg)
        speak(option, msg)
        time.sleep(1)


if __name__ == "__main__":
    voiceAlarm(2, 100, 30)

