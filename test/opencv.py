# see red_blue.py in the examples dir
import time
import pyfakewebcam
import numpy as np
import cv2

cap = cv2.VideoCapture(0)
width = 640
height = 480
fps = 30

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)


blue = np.zeros((480,640,3), dtype=np.uint8)
blue[:,:,2] = 255

red = np.zeros((480,640,3), dtype=np.uint8)
red[:,:,0] = 255

camera = pyfakewebcam.FakeWebcam('/dev/video2', 640, 480)

while True:
    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define colour ranges in HSV
    upper_yellow = np.array([10, 181, 237])
    lower_yellow = np.array([21, 171, 155])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)


# Find contours for yellow
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_yellow:
        area = cv2.contourArea(cnt)
        if area > 500:  # Ignore small noise
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.putText(frame, "Yellow Can", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)




    outputFrame = frame.copy()
    outputFrame[:, :, 0] = frame[:, :, 2]
    outputFrame[:, :, 2] = frame[:, :, 0]
    camera.schedule_frame(outputFrame)

    # camera.schedule_frame(blue)
    # time.sleep(1/30.0)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()

