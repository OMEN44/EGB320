import cv2
import numpy as np

def findShelves(self, hsvFrame, outputFrame):
    
    lower_blue = np.array([100, 100, 0])
    upper_blue = np.array([150, 255, 255])

    mask = cv2.inRange(hsvFrame, lower_blue, upper_blue)

    openingValue = 10
    mask = cv2.erode(mask, np.ones((openingValue, openingValue)))  # Erode to remove noise
    mask = cv2.dilate(mask, np.ones((openingValue, openingValue)))  # Dilate to restore size

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            
            x, y, w, h = cv2.boundingRect(approx)
            outputFrame = cv2.rectangle(outputFrame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            outputFrame = cv2.putText(outputFrame, f'Shelf', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # outputFrame = cv2.Canny(cv2.bitwise_and(outputFrame, outputFrame, mask=mask), 100, 200)
    # outputFrame = cv2.bitwise_and(outputFrame, outputFrame, mask=mask)

    return [outputFrame, []]