import cv2
import numpy as np

from robot.vision.utils import getPoi, HISTORY_LEN, filterFalsePositives

def findShelves(self, hsvFrame, outputFrame):

    data = []
    
    # lower_blue = np.array([100, 130, 50])
    lower_blue = np.array([100, 130, 50])
    upper_blue = np.array([150, 255, 255])

    mask = cv2.inRange(hsvFrame, lower_blue, upper_blue)

    openingValue = 50
    mask = cv2.dilate(mask, np.ones((openingValue, openingValue)))  # Dilate to restore size
    mask = cv2.erode(mask, np.ones((openingValue, openingValue)))  # Erode to remove noise

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    rects = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            approx = cv2.approxPolyDP(cnt, .008 * cv2.arcLength(cnt, True), True)
            
            x, y, w, h = cv2.boundingRect(approx)

            rects.append((x, y, w, h))

            outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            outputFrame = cv2.putText(outputFrame, f'Shelf', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            data.append(getPoi(-1, w, x, self.calibration['new_k'][0,0]))

    self.poiHistory['shelves'].append(data)
    if (len(self.poiHistory['shelves']) > HISTORY_LEN):
        self.poiHistory['shelves'].pop(0)

    # data = filterFalsePositives(self.poiHistory['shelves'])

    return [outputFrame, data]