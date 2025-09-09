import cv2
import pyfakewebcam
import numpy as np
import time

cap = cv2.VideoCapture(0)



def useVideo():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Resize frame to 640x480
        frame_resized = cv2.resize(frame_rgb, (640, 480))
        print('framed')

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break