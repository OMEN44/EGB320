import cv2
import numpy as np


def proccess(self, frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, self.colourMask[0], self.colourMask[1])

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    filteredContours = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > self.areaLimit[0] and area < self.areaLimit[1]:
            filteredContours.append(cnt)

    outputFrame = frame.copy()
    outputFrame = cv2.drawContours(outputFrame, filteredContours, -1, (0, 255, 0), -1)

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

