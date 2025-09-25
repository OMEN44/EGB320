import cv2
import numpy as np

from robot.vision.utils import getPoi

def findShelves(self, hsvFrame, outputFrame):

    data = []
    
    lower_blue = np.array([100, 130, 50])
    upper_blue = np.array([150, 255, 255])

    mask = cv2.inRange(hsvFrame, lower_blue, upper_blue)

    openingValue = 50
    mask = cv2.dilate(mask, np.ones((openingValue, openingValue)))  # Dilate to restore size
    mask = cv2.erode(mask, np.ones((openingValue, openingValue)))  # Erode to remove noise

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            approx = cv2.approxPolyDP(cnt, .008 * cv2.arcLength(cnt, True), True)
            # hull = cv2.convexHull(approx)
            # defects = cv2.convexityDefects(approx, cv2.convexHull(approx, returnPoints=False))
            # if defects is not None and defects.shape[0] >= 2:
            #     farPoints = []
            #     for i in range(defects.shape[0]):
            #         s, e, f, d = defects[i, 0]
            #         farPoints.append(tuple(approx[f][0]))
            # outputFrame = cv2.drawContours(outputFrame, [approx], 0, (0, 0, 255), 2)
            # outputFrame = cv2.drawContours(outputFrame, [hull], 0, (0, 255, 0), 2)
            
            x, y, w, h = cv2.boundingRect(approx)
            # outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            outputFrame = cv2.putText(outputFrame, f'Shelf', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            data.append(getPoi('shelves', 1, -1, w, x, self.calibration['new_k'][0,0]))

    # outputFrame = cv2.Canny(cv2.bitwise_and(outputFrame, outputFrame, mask=mask), 100, 200)
    # outputFrame = cv2.bitwise_and(outputFrame, outputFrame, mask=mask)

    return [outputFrame, data]