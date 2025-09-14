import cv2
import numpy as np


def proccess(frame, lower_limit, upper_limit):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_limit, upper_limit)

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    outputFrame = frame.copy()
    outputFrame = cv2.drawContours(outputFrame, contours, -1, (0, 255, 0), -1)

    return outputFrame
    # mask = frame.copy()

    # # cv2.GaussianBlur(mask, (5, 5), 0, mask)
    # hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)

    # mask = cv2.inRange(hsv, lower_limit, upper_limit)
    # mask = cv2.erode(mask, np.ones((opening_value, opening_value)))  # Erode to remove noise
    # mask = cv2.dilate(mask, np.ones((opening_value, opening_value)))  # Dilate to restore size

    # # Overlay the mask on the original image in white
    # contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    # outputFrame = frame.copy()
    # for cnt in contours:
    #     cv2.drawContours(outputFrame, [cnt], -1, (0, 255, 0), -1)

    # return outputFrame

