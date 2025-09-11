import cv2
import numpy as np



from robot.vision.isle_marker import findIsleMarkers, findPickingStation
from robot.vision.colour_mask import proccess as mask

def proccess(self, frame):

    # Gaussian blur everything before proccessing to reduce noise
    # cv2.GaussianBlur(frame, (5, 5), 0, frame)

    img1 = findIsleMarkers(self, frame)

    img1 = findPickingStation(self, img1)

    img2 = mask(frame, np.array([0,0,0]), np.array([255, 255, 35]), 9, True)

    return [img1, img2, frame]