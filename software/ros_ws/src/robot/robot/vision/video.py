import cv2
import pyfakewebcam
import numpy as np
import time

from std_msgs.msg import String

from robot.vision.pipeline import proccess
from robot.vision.image import setupFakeCam

WIDTH = 640
HEIGHT = 480
FPS = 60

def useVideo(self):
    sink = setupFakeCam()
    # cap = setupCameraWithDefaults(auto_exposure=False, exposure=1, gain=1, brightness=-64)
    # cap = setupCameraWithDefaults(auto_exposure=False, exposure=10, auto_wb=False)
    cap = setupCameraWithDefaults()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        now = time.time()

        frame = cv2.imread(f'/home/pi/EGB320/software/python/test_data2/picking1.jpg')

        frames = proccess(self, frame)

        for i in range(len(frames)):
            frames[i] = cv2.putText(frames[i], f'FPS: {int(1/(time.time()-now))}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            sink[i].schedule_frame(cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB))



def setupCamera():
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    # Disable auto exposure
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, 5)
    # Disable auto white balance
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4600)

    cap.set(cv2.CAP_PROP_BRIGHTNESS, -0)
    cap.set(cv2.CAP_PROP_CONTRAST, 3)

    return cap

# takes arguments for each camera setting all with default values
def setupCameraWithDefaults(auto_wb=True, auto_exposure=True, brightness=0, contrast=3, saturation=56, hue=0, white_balance=4600, gamma=84, gain=1, temperature=4600, sharpness=2, backlight=0, exposure=156):
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.CAP_PROP_SATURATION, saturation) # 69
    cap.set(cv2.CAP_PROP_HUE, hue)
    cap.set(cv2.CAP_PROP_IOS_DEVICE_WHITEBALANCE, white_balance)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1 if auto_wb else 0)
    cap.set(cv2.CAP_PROP_GAMMA, gamma)
    cap.set(cv2.CAP_PROP_GAIN, gain)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, white_balance)
    cap.set(cv2.CAP_PROP_TEMPERATURE, temperature)
    cap.set(cv2.CAP_PROP_SHARPNESS, sharpness)
    cap.set(cv2.CAP_PROP_BACKLIGHT, backlight) # 121
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3 if auto_exposure else 1) # 3
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure) # 10

    return cap