import cv2
import numpy as np

from robot.vision.utils import getPoi

def findObstacles(self, hsvFrame, outputFrame):
    lower = np.array([35, 80, 25])
    upper = np.array([90, 255, 255])

    mask = cv2.inRange(hsvFrame, lower, upper)
    # mask = cv2.inRange(hsvFrame, self.colourMask[0], self.colourMask[1])

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    ramps = []
    people = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200 and area < 50000:
        # if area > self.areaLimit[0] and area < self.areaLimit[1]:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            if cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(approx)

                outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                if (w > h):
                    ramps.append(getPoi(12, w, x, self.calibration['new_k'][0,0]))
                    outputFrame = cv2.putText(outputFrame, f'R', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                else:
                    people.append(getPoi(5, w, x, self.calibration['new_k'][0,0]))
                    outputFrame = cv2.putText(outputFrame, f'P', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return [outputFrame, ramps, people]