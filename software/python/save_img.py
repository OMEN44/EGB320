import cv2
import numpy as np
from time import sleep
from datetime import datetime

cap = cv2.VideoCapture(0)
width = 640
height = 480
fps = 15

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (width, height))


cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

count = 0

while count < 200:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    
    # filename = f'/home/pi/EGB320/software/python/test_data2/img_{datetime.now().timestamp()}.jpg'
    # cv2.imwrite(filename, frame)
    # print(f'Saved {filename}')
    out.write(frame)

    count += 1
    sleep(0.3)  # Sleep for a short duration to avoid overwhelming the camera

cap.release()
out.release()
    
