import cv2
import numpy as np

from robot.vision.utils import getPoi

ITEM_WIDTHS = [
    5.5, # 0: Bowl
    4.5, # 1: Coffee Cup
    2,   # 2: Robot Oil Bottle
    4,   # 3: Rubiks Cube
    4.5, # 4: Soccer Ball
    6.5, # 5: Wheet Bots
]

# range
# [0, 90, 130] to [40, 255, 255]
# area [100, 2000]

# [0, 5, 0] to [15, 255, 255]
# area [200, 40000]  

def findItems(self, hsvframe, outputFrame):
    data = []

    # create a mask using self.colourMask
    # mask = cv2.inRange(hsvframe, np.array([5, 70, 30]), np.array([15, 255, 255]))
    mask = cv2.inRange(hsvframe, np.array([0, 100, 100]), np.array([15, 255, 255]))

    # Find items
    count = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 150 and area < 25000:
            x, y, w, h = cv2.boundingRect(contour)
            poi = getPoi('item', count, ITEM_WIDTHS[3], w, x, self.calibration['new_k'][0,0])
            outputFrame = cv2.putText(outputFrame, f'Item {count}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (18, 111, 183), 2)
            count += 1
            outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (18, 111, 183), 2)
            # outputFrame = cv2.putText(outputFrame, f'{int(poi.distance)}cm, {int(poi.bearing[1])}deg', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (18, 111, 183), 2)
            data.append(poi)
            
    return [outputFrame, data]