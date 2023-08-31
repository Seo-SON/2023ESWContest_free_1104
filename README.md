# team_WST
team - WST

## Setting
Please download DFRobot_RaspberryPi_A02YYUW.py and Yolo-fastestV2

```
sudo git clone https://github.com/DFRobot/DFRobot_RaspberryPi_A02YYUW
sudo git clone https://github.com/dog-qiuqiu/Yolo-FastestV2
```


Modify coco.name file path in coco.data 

```
names=path of coco.name
```


Modify weight file path in pan_tilt_tracking.py

```
model_.load_state_dict(torch.load('weight file path(.pth)', map_location=device))
```

## Run
```
python3 pan_tilt_tracking.py --data /home/pi/wst/WST-main/ver_2/coco.data
```

## Reference
* used model - Yolo-fastestV2 : https://github.com/dog-qiuqiu/Yolo-FastestV2

* pan_tilt_tracking.py - class VideoCapture : https://stackoverflow.com/a/54755738

* Sonar.py - from DFRobot_RaspberryPi_A02YYUW import DFRobot_A02_Distance : https://github.com/DFRobot/DFRobot_RaspberryPi_A02YYUW
