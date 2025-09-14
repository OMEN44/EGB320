import cv2
import numpy as np

def findShelves(self, hsvFrame, outputFrame):
    # make a mask for the color black
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 255, 50])

    mask = cv2.inRange(hsvFrame, self.colourMask[0], self.colourMask[1])
    
    openingValue = 10
    mask = cv2.erode(mask, np.ones((openingValue, openingValue)))  # Erode to remove noise
    mask = cv2.dilate(mask, np.ones((openingValue, openingValue)))  # Dilate to restore size

    # outputFrame = cv2.Canny(cv2.bitwise_and(outputFrame, outputFrame, mask=mask), 100, 200)
    outputFrame = cv2.bitwise_and(outputFrame, outputFrame, mask=mask)

    return [outputFrame, []]