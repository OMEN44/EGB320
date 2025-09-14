import cv2
import numpy as np
import time

from std_msgs.msg import String

from robot.vision.pipeline import proccess

WIDTH = 640
HEIGHT = 480
FPS = 60

def useVideo(self, framePath=None):

    frame = None

    ret, frame = self.cap.read()
    if not ret:
        print("Failed to grab frame")
        return

    now = time.time()

    frames = proccess(self, frame)

    for i in range(len(frames)):
        frames[i] = cv2.putText(frames[i], f'FPS: {int(1/(time.time()-now))}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        self.sink[i].schedule_frame(cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB))