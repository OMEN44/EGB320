import cv2
import numpy as np

from robot.vision.utils import getPoi

ITEM_WIDTHS = [
    5.5, # Bowl
    4.5, # Coffee Cup
    2, # Robot Oil Bottle
    4, # Rubiks Cube
    4.5, # Soccer Ball
    6.5, # Wheet Bots
]

# range
# [0, 90, 130] to [40, 255, 255]
# area [100, 2000]

# [0, 5, 0] to [15, 255, 255]
# area [200, 40000]  

def findItems(hsvframe, outputFrame):
    data = []

    # create a mask using self.colourMask
    # mask = cv2.inRange(hsvframe, np.array([0, 5, 0]), np.array([15, 255, 255]))
    mask = cv2.inRange(hsvframe, np.array([0, 100, 100]), np.array([15, 255, 255]))

    # Find items
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 150 and area < 8000:
            x, y, w, h = cv2.boundingRect(contour)
            poi = getPoi('item', 0, ITEM_WIDTHS[1], w, x)
            outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (18, 111, 183), 2)
            # outputFrame = cv2.putText(outputFrame, f'{int(poi.distance)}cm, {int(poi.bearing[1])}deg', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (18, 111, 183), 2)
            data.append(poi)
            
    return [outputFrame, data]