import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
width = 640
height = 480
fps = 30
fourcc = cv.VideoWriter_fourcc(*'YUYV') # YUYV format is often used with v4l2loopback
output_device = '/dev/video2' # Replace with your v4l2loopback device

# Create VideoWriter object
out = cv.VideoWriter(output_device, fourcc, fps, (width, height))

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([100,100,100])
    upper_blue = np.array([150,150,150])

    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)

    cv.imshow('frame',frame)
    cv.imshow('mask',mask)
    out.write(mask)
    cv.imshow('res',res)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break

cv.destroyAllWindows()
