# ver2

import cv2
from ultralytics import YOLO
from ultralytics.yolo.utils.ops import xyxy2xywh

class ObjCenter:
    def __init__(self, modelPath):
        self.model = YOLO(modelPath)
        self.priority_id = [0, 5, 1, 2, 6, 7, 3, 4] # priority[0] = 0번 class의 priority

    def setPriority(self, priority_list):
        self.priority_id = priority_list

    def update(self, frame, frameCenter):

        results = self.model.predict(frame)

		# 우선순위 가장 높은 애 하나만 리턴
        priority = 8
        cls = 8
        center = frameCenter

        for r in results:
            boxes = r.boxes
            for box in boxes:
                id = int(box.cls[0].item())
                coord = box.xywh[0].tolist()  # get box coordinates in (top, left, bottom, right) format
                
                if priority > self.priority_id[id]:
                    priority = self.priority_id[id]
                    cls = id
                    center = (coord[0], coord[1])

        if cls != 8:
			# return the center (x, y), clsId
            return (center, int(cls))
        else:
		    # otherwise 
            return (frameCenter, None)
