import cv2
import numpy as np

from robot.vision.utils import getPoi, getEmptyPoi

def findPickingStation(self, hsvFrame, outputFrame):
    # make a mask for the color black
    lower_black = np.array([10, 80, 100])
    upper_black = np.array([40, 255, 255])

    mask = cv2.inRange(hsvFrame, lower_black, upper_black)

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            # overlay bounding rect on output frame
            outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            
            return [outputFrame, [getPoi(60, w, x, self.calibration['new_k'][0,0])]]


    return [outputFrame, [getEmptyPoi()]]