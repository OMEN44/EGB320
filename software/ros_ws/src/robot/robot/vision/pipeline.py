import cv2
import numpy as np

from std_msgs.msg import String

from robot.vision.isle_marker import findIsleMarkers, findPickingStation
from robot.vision.colour_mask import proccess as mask
from robot.vision.items import findItems
from robot.vision.shelves import findShelves
from robot.vision.obstacles import findObstacles

def proccess(self, frame):

    data = []
    outputFrame = frame.copy()
    outputFrame2 = frame.copy()
    outputFrame3 = frame.copy()

    hsvframe = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

    for filter in self.pipeline:
        if filter == "isleMarkers":
            [outputFrame, isleMarkers] = findIsleMarkers(self, hsvframe, outputFrame)
            data += isleMarkers
        elif filter == "pickingStation":
            [outputFrame, pickingStations] = findPickingStation(hsvframe, outputFrame)
            data += pickingStations
        elif filter == "items":
            [outputFrame, items] = findItems(hsvframe, outputFrame)
            data += items
        elif filter == "obstacles":
            [outputFrame, obstacles] = findObstacles(self, hsvframe, outputFrame)
            data += obstacles
        elif filter == "shelves":
            [outputFrame, shelves] = findShelves(self, hsvframe, outputFrame)
            data += shelves
        
        elif filter == "colourMask":
            outputFrame2 = mask(self, hsvframe, outputFrame2)

    # Draw a black cross in the center of the frame
    cv2.line(outputFrame, (int(frame.shape[1]/2) - 10, int(frame.shape[0]/2)), (int(frame.shape[1]/2) + 10, int(frame.shape[0]/2)), (0, 0, 0), 2)
    cv2.line(outputFrame, (int(frame.shape[1]/2), int(frame.shape[0]/2) - 10), (int(frame.shape[1]/2), int(frame.shape[0]/2) + 10), (0, 0, 0), 2)

    self.poi.publish(String(data=', '.join(data)))

    return [outputFrame, outputFrame2, outputFrame3]