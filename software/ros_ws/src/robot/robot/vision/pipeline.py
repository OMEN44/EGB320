import cv2
import numpy as np

from std_msgs.msg import String

from robot.vision.isle_marker import findIsleMarkers, findPickingStation
from robot.vision.colour_mask import proccess as mask
from robot.vision.items import findItems

def proccess(self, frame):

    data = []
    outputFrame = frame.copy()
    outputFrame2 = frame.copy()

    for filter in self.pipeline:
        if filter == "isleMarkers":
            [outputFrame, isleMarkers] = findIsleMarkers(self, frame, outputFrame)
            data += isleMarkers
        elif filter == "pickingStation":
            [outputFrame, pickingStations] = findPickingStation(frame, outputFrame)
            data += pickingStations
        elif filter == "items":
            [outputFrame, items] = findItems(frame, outputFrame)
            data += items
        elif filter == "colourMask":
            outputFrame2 = mask(self, frame)

    # Draw a black cross in the center of the frame
    cv2.line(outputFrame, (int(frame.shape[1]/2) - 10, int(frame.shape[0]/2)), (int(frame.shape[1]/2) + 10, int(frame.shape[0]/2)), (0, 0, 0), 2)
    cv2.line(outputFrame, (int(frame.shape[1]/2), int(frame.shape[0]/2) - 10), (int(frame.shape[1]/2), int(frame.shape[0]/2) + 10), (0, 0, 0), 2)

    self.poi.publish(String(data=', '.join(data)))

    return [outputFrame, outputFrame2, frame]