import cv2
import numpy as np

from robot.vision.utils import objectDistance, objectAngle

ITEM_WIDTHS = [
    3, # Bowl
    3, # Coffee Cup
    3, # Robot Oil Bottle
    3, # Rubiks Cube
    3, # Soccer Ball
    3, # Wheet Bots
]

# range
# [0, 90, 130] to [40, 255, 255]
# area [100, 2000]

def findItems(frame, outputFrame):
    data = []

    #  convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # create a mask using self.colourMask
    mask = cv2.inRange(hsv, np.array([0, 90, 130]), np.array([40, 255, 255]))

    # Find items
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 100 and area < 2000:
            x, y, w, h = cv2.boundingRect(contour)
            distance = objectDistance(ITEM_WIDTHS[0], w)
            angle = objectAngle(x + w / 2)
            outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (18, 111, 183), 2)
            outputFrame = cv2.putText(outputFrame, f'{int(distance)}cm, {int(angle)}deg', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (18, 111, 183), 2)
            data.append(f'item: {distance}cm @ {angle}°')
    return [outputFrame, data]