import cv2
import numpy as np

from robot.vision.utils import objectDistance, objectDistanceWithCorrection, objectAngle

def findTestObject(self, hsvframe, outputFrame):

    # create mask using self.colourMask
    mask = cv2.inRange(hsvframe, self.colourMask[0], self.colourMask[1])

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > self.areaLimit[0] and area < self.areaLimit[1]:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            # outputFrame = cv2.drawContours(outputFrame, [approx], -1, (255, 0, 0), 2)
            x, y, w, h = cv2.boundingRect(approx)
            outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            distance = objectDistanceWithCorrection(self.number[0], w, x + w/2)
            angle = objectAngle(x + w/2)
            outputFrame = cv2.putText(outputFrame, f'{w}px -> {np.round(distance, 2)}cm @ {angle}d', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            return [outputFrame, [{
                'name': 'test',
                'type': 0,
                'distance': int(distance * 1000),
                'bearing': [int(objectAngle(x) * 1000), int(angle * 1000), int(objectAngle(x + w) * 1000)]
            }]]
        
    return [outputFrame, []]

