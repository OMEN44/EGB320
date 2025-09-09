import cv2
import pyfakewebcam
import numpy as np
import time

import robot.vision.isle_marker as isle_marker

WIDTH = 640
HEIGHT = 480
FPS = 15

def useVideo():
    cap = setupCamera()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        isle_marker.findMarkers(frame)

        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Resize frame to 640x480
        frame_resized = cv2.resize(frame_rgb, (640, 480))
        print('framed')

def setupCamera():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

    return cap