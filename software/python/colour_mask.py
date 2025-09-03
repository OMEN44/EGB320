import cv2
import numpy as np


def proccess(frame, lower_limit, upper_limit, opening_value, overlay_mask = False):
    mask = frame.copy()

    cv2.GaussianBlur(mask, (5, 5), 0, mask)
    hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_limit, upper_limit)
    mask = cv2.erode(mask, np.ones((opening_value, opening_value)))  # Erode to remove noise
    mask = cv2.dilate(mask, np.ones((opening_value, opening_value)))  # Dilate to restore size

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    outputFrame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 10000:  # Ignore small noise
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int_(box)
            if overlay_mask:
                cv2.drawContours(outputFrame, [cnt], -1, (0, 255, 0), 2)
            cv2.drawContours(outputFrame,[box],0,(0,0,255),2)

    return cv2.cvtColor(cv2.bitwise_and(frame, frame, mask=mask), cv2.COLOR_BGR2RGB)
    return outputFrame

