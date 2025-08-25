import cv2
import numpy as np

# Return a frame with boxes arround orange stock items
def findStock(frame):
    mask = frame.copy()
    # cv2.cvtColor(mask, cv2.COLOR_BGR2RGB, mask)
    cv2.GaussianBlur(mask, (5, 5), 0, mask)
    hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([0, 80, 0])
    upper_orange = np.array([100, 255, 255])

    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    openingMagnitude = 5
    mask_orange = cv2.erode(mask_orange, np.ones((openingMagnitude, openingMagnitude)))  # Erode to remove noise
    mask_orange = cv2.dilate(mask_orange, np.ones((openingMagnitude, openingMagnitude)))  # Dilate to restore size

    contours, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    outputFrame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 10000:  # Ignore small noise
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int_(box)
            cv2.drawContours(outputFrame,[box],0,(0,0,255),2)
            # Add the width

            # disatance 230mm
            # width 118mm
            # width 300px
            # Focal length = 584.75
            cv2.putText(outputFrame, str(np.round((118 * 584.75) / rect[1][0]) / 10), (int(rect[0][0]), int(rect[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return outputFrame