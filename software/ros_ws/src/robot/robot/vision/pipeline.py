import cv2
import numpy as np

from std_msgs.msg import String
from robot_interfaces.msg import Poi, PoiGroup

from robot.vision.isle_marker import findIsleMarkers, findPickingStation
from robot.vision.colour_mask import proccess as mask
from robot.vision.items import findItems
from robot.vision.shelves import findShelves
from robot.vision.obstacles import findObstacles
from robot.vision.test import findTestObject

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
            [outputFrame, pickingStations] = findPickingStation(self, hsvframe, outputFrame)
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
        elif filter == "test":
            [outputFrame, tests] = findTestObject(self, hsvframe, outputFrame)
            data += tests
        
        elif filter == "colourMask":
            outputFrame2 = mask(self, hsvframe, outputFrame2)

    # Draw a black cross in the center of the frame
    cv2.line(outputFrame, (int(frame.shape[1]/2) - 10, int(frame.shape[0]/2)), (int(frame.shape[1]/2) + 10, int(frame.shape[0]/2)), (0, 0, 0), 2)
    cv2.line(outputFrame, (int(frame.shape[1]/2), int(frame.shape[0]/2) - 10), (int(frame.shape[1]/2), int(frame.shape[0]/2) + 10), (0, 0, 0), 2)

    poiMsg = PoiGroup()
    poiMsg.pois = []

    for poi in data:
        print()
        print('POI:', poi)
        print(poi['name'])
        poiMsg.pois.append(Poi(name=poi['name'], type=poi['type'], distance=poi['distance'], bearing=poi['bearing'].copy()))

    self.poi.publish(poiMsg)

    return [outputFrame, outputFrame2, outputFrame3]