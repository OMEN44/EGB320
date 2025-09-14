import cv2
import numpy as np

from robot.vision.utils import objectDistance, objectAngle

def findObstacles(self, hsvFrame, outputFrame):
    # make a mask for the color black
    lower_black = np.array([60, 100, 0])
    upper_black = np.array([95, 255, 255])

    mask = cv2.inRange(hsvFrame, lower_black, upper_black)
    # mask = cv2.inRange(hsvFrame, self.colourMask[0], self.colourMask[1])

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    data = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200 and area < 50000:
        # if area > self.areaLimit[0] and area < self.areaLimit[1]:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            if cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(approx)

                distance = 0
                angle = objectAngle(x + w / 2)
                person = False

                outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                if (w > h):
                    distance = objectDistance(5, w)
                    person = True
                    outputFrame = cv2.putText(outputFrame, f'R', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                else:
                    distance = objectDistance(12, w)
                    outputFrame = cv2.putText(outputFrame, f'P', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                data.append(f'{ "P" if person else "R"}: {distance}cm @ {angle}°')

    return [outputFrame, data]