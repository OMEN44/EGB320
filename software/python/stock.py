import cv2
import numpy as np

# Return a frame with boxes arround orange stock items
def findStock(frame):
    mask = frame.copy()
    cv2.cvtColor(frame, cv2.COLOR_BGR2RGB, frame)
    cv2.GaussianBlur(mask, (5, 5), 0, mask)
    hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([0, 128, 128])
    upper_orange = np.array([40, 255, 255])

    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_orange = cv2.erode(mask_orange, np.ones((5, 5)))  # Erode to remove noise
    mask_orange = cv2.dilate(mask_orange, np.ones((5, 5)))  # Dilate to restore size

    contours, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # Ignore small noise
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 165, 255), 2)  # Orange color for stock items
            cv2.putText(frame, 'grocery item (w: {}, h: {})'.format(w, h), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)


    # test = np.zeros((480, 640, 3), dtype=np.uint8)
    # test[:,:,0] = mask_orange
    # test[:,:,1] = mask_orange
    # test[:,:,2] = mask_orange

    # test = np.array(test) * 255

    # outputFrame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)
    # outputFrame[mask_orange > 0] = [0, 255, 0]

    return frame