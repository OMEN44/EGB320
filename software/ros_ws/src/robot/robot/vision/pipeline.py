import cv2
import numpy as np

from std_msgs.msg import String
from robot_interfaces.msg import Poi, PoiGroup

from robot.vision.utils import HISTORY_LEN, PERSISTENCE_THRESHOLD, filterFalsePositives, getEmptyPoi
from robot.vision.isle_marker import findIsleMarkers, findPickingStationMarkers
from robot.vision.colour_mask import proccess as mask
from robot.vision.items import findItems
from robot.vision.shelves import findShelves
from robot.vision.obstacles import findObstacles
from robot.vision.test import findTestObject
from robot.vision.pickingStation import findPickingStation

def proccess(self, frame):

    outputFrame = frame.copy()
    outputFrame2 = frame.copy()
    outputFrame3 = frame.copy()

    hsvframe = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

    poiMsg = PoiGroup()
    poiMsg.items = []
    poiMsg.ramps = []
    poiMsg.shelves = []
    poiMsg.obstacles = []
    poiMsg.aisle_markers = []
    poiMsg.picking_markers = []
    poiMsg.picking_station = []

    for filter in self.pipeline:
        if filter == "aisleMarkers":
            [outputFrame, isleMarkers] = findIsleMarkers(self, hsvframe, outputFrame)
            self.poiHistory['aisle_markers'].append(isleMarkers)
            if len(self.poiHistory['aisle_markers']) > HISTORY_LEN:
                self.poiHistory['aisle_markers'].pop(0)

            draft = []

            for i in range(len(isleMarkers)):
                count = sum(previous[i].exists and isleMarkers[i].distance < previous[i].distance for previous in self.poiHistory['aisle_markers'])
                if count > PERSISTENCE_THRESHOLD:
                    draft.append(isleMarkers[i])
                else:
                    draft.append(self.poiHistory['aisle_markers'][-1][i] if len(self.poiHistory['aisle_markers']) > 1 else getEmptyPoi())

            poiMsg.aisle_markers = draft
        elif filter == "pickingMarkers":
            [outputFrame, pickingStationMarkers] = findPickingStationMarkers(self, hsvframe, outputFrame)
            poiMsg.picking_markers = pickingStationMarkers
        elif filter == "pickingStation":
            [outputFrame, pickingStations] = findPickingStation(self, hsvframe, outputFrame)
            poiMsg.picking_station = pickingStations
        elif filter == "items":
            [outputFrame, items] = findItems(self, hsvframe, outputFrame)
            poiMsg.items = items
        elif filter == "obstacles":
            [outputFrame, ramps, people] = findObstacles(self, hsvframe, outputFrame)
            poiMsg.obstacles = people
            poiMsg.ramps = ramps
        elif filter == "shelves":
            [outputFrame, shelves] = findShelves(self, hsvframe, outputFrame)
            poiMsg.shelves = shelves
        elif filter == "test":
            [outputFrame, tests] = findTestObject(self, hsvframe, outputFrame)
        elif filter == "colourMask":
            outputFrame2 = mask(self, hsvframe, outputFrame2)

    # Draw a black cross in the center of the frame
    cv2.line(outputFrame, (int(frame.shape[1]/2) - 10, int(frame.shape[0]/2)), (int(frame.shape[1]/2) + 10, int(frame.shape[0]/2)), (0, 0, 0), 2)
    cv2.line(outputFrame, (int(frame.shape[1]/2), int(frame.shape[0]/2) - 10), (int(frame.shape[1]/2), int(frame.shape[0]/2) + 10), (0, 0, 0), 2)

    self.poi.publish(poiMsg)

    return [outputFrame, outputFrame2, outputFrame3]