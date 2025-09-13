import cv2
import numpy as np

from std_msgs.msg import String

from robot.vision.isle_marker import findIsleMarkers, findPickingStation
from robot.vision.colour_mask import proccess as mask

def proccess(self, frame):

    data = []
    outputFrame = frame.copy()
    outputFrame2 = frame.copy()

    for filter in self.pipeline:
        if filter == "isleMarkers":
            [outputFrame, isleMarkers] = findIsleMarkers(frame, outputFrame)
            data += isleMarkers
        elif filter == "pickingStation":
            [outputFrame, pickingStations] = findPickingStation(frame, outputFrame)
            data += pickingStations
        elif filter == "colourMask":
            outputFrame2 = mask(frame, np.array([0,0,0]), np.array([255, 255, 50]))

    self.publisher.publish(String(data=', '.join(data)))

    return [outputFrame, outputFrame2, frame]