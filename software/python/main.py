import cv2
import pyfakewebcam
import numpy as np
from time import sleep

import isleMarkers
import stock
import colour_mask

cap = cv2.VideoCapture(0)
width = 640
height = 480
fps = 15

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

# Video sinks
camera = pyfakewebcam.FakeWebcam('/dev/video2', width, height)
camera2 = pyfakewebcam.FakeWebcam('/dev/video3', width, height)
camera3 = pyfakewebcam.FakeWebcam('/dev/video4', width, height)

def process(frame):
    processed_frame = frame.copy()
    cv2.GaussianBlur(processed_frame, (5, 5), 0, processed_frame)
    hsv = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    size = 5
    mask_red = cv2.erode(mask_red, np.ones((size, size)))  # Erode to remove noise
    mask_red = cv2.dilate(mask_red, np.ones((size, size)))  # Dilate to restore size

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_red:
        area = cv2.contourArea(cnt)
        if area > 2000:  # Ignore small noise
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(processed_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(processed_frame, str(area), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return processed_frame


def useVideo():
    count = 1
    while True:
        frame = cv2.imread(f'/home/huon/Programs/EGB320/software/python/test_data2/isle2.jpg')
        if frame is None:
            print("Failed to grab frame")
            break


        outputFrame = frame.copy()
        outputFrame[:, :, 0] = frame[:, :, 2]
        outputFrame[:, :, 2] = frame[:, :, 0]
        
        camera.schedule_frame(outputFrame)
        camera2.schedule_frame(colour_mask.proccess(
            frame, 
            # Blue
            # np.array([80, 0, 0]), 
            # np.array([130, 255, 180]), 
            # Green
            # (51,50,0),
            # (97,255,255),
            # Yellow
            (15,53,36), 
            (37,231,255),
            5,
            True
        ))
        # camera2.schedule_frame(stock.findStock(isleMarkers.findMarkers(frame)))

        sleep(1)
        count += 1
        if count >= 5:
            count = 1

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

useVideo()

frame = cv2.imread('/home/bird/EGB320/software/python/test_data/IsleView.jpg')
outputFrame = frame.copy()
outputFrame[:, :, 0] = frame[:, :, 2]
outputFrame[:, :, 2] = frame[:, :, 0]

camera.schedule_frame(stock.findStock(isleMarkers.findMarkers(frame)))

camera2.schedule_frame(isleMarkers.findMarkers(frame))

# camera.schedule_frame(stock.findStock(cv2.imread('/home/bird/EGB320/software/python/test_data/testA.jpg')))
# camera2.schedule_frame(stock.findStock(cv2.imread('/home/bird/EGB320/software/python/test_data/IsleView.jpg')))

