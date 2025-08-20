import cv2
import numpy as np

def findMarkers(frame):
    mask = frame.copy()
    cv2.GaussianBlur(mask, (150, 150), 0, mask)
    hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)
    # lower_black = np.array([160, 0, 0])
    # upper_black = np.array([220, 0, 0])

    # mask = cv2.inRange(hsv, upper_black, lower_black)
    
    # make a mask for the color black
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([255, 128, 96])

    # blk1= 0
    # cv2.cvtColor(np.array([[0x15],[0x15],[0x13]]), cv2.COLOR_RGB2HSV, blk1)
    # blk2= 0
    # cv2.cvtColor(np.array([[0],[0],[0]]), cv2.COLOR_RGB2HSV, blk2)
    # print("Black1:", blk1, "Black2:", blk2)

    mask = cv2.inRange(hsv, lower_black, upper_black)
    mask = cv2.erode(mask, np.ones((5, 5)))  # Erode to remove noise
    mask = cv2.dilate(mask, np.ones((5, 5)))  # Dilate to restore size

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            print("Found marker with area:", area)

    test = np.zeros((480, 640, 3), dtype=np.uint8)
    test[:,:,0] = mask
    test[:,:,1] = mask
    test[:,:,2] = mask

    test = np.array(test) * 255

    outputFrame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)
    outputFrame[mask > 0] = [0, 255, 0]

    return outputFrame