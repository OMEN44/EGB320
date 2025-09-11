import cv2
import numpy as np
import
from std_msgs.msg import String

import json
import random

isleMarkerCount = []

FOCAL_LENGTH = (32 / 100) * 7

def findIsleMarkers(self, frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # make a mask for the color black
    # lower_black = np.array([0, 0, 0])
    # upper_black = np.array([255, 140, 110])

    # mask = cv2.inRange(hsv, lower_black, upper_black)
    # openingValue = 10
    # # erode and dilate in a circle
    # circleMatrix = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (openingValue, openingValue))
    # mask = cv2.erode(mask, circleMatrix)
    # mask = cv2.dilate(mask, circleMatrix)

    # contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 1,
                               param1=100, param2=30,
                               minRadius=1, maxRadius=30)
    
    outputFrame = frame.copy()
    if len(isleMarkerCount) > 30:
        isleMarkerCount.pop(0)
    isleMarkerCount.append(len(circles[0]) if circles is not None else 0)
    
    avgRadius = 0

    if circles is not None:
        circles = np.uint16(circles)
        for i in circles[0, :]:
            avgRadius += i[2]
            # draw the outer circle
            outputFrame = cv2.circle(outputFrame, (i[0], i[1]), i[2], (255, 0, 255), 2)
            # draw the center of the circle
            outputFrame = cv2.circle(outputFrame, (i[0], i[1]), 2, (0, 0, 255), 3)
        avgRadius /= len(circles[0])

    count = np.round(np.mean(isleMarkerCount))
    distance = ((FOCAL_LENGTH * 7) / (2 * avgRadius)) if avgRadius != 0 else -1
    distance *= 2

    self.publisher.publish(String(data=f'isle_markers: {count}, width: {np.round(avgRadius * 2, 2)}px, distance: {distance} cm'))

    # isle = []


    # for cnt in contours:
    #     area = cv2.contourArea(cnt)
    #     if area > 100 and area < 1200:
    #         outputFrame = cv2.drawContours(outputFrame, [cnt], -1, (0, 255, 0), 2)

    return outputFrame

def findPickingStation(self, frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # make a mask for the color black
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([255, 140, 110])

    mask = cv2.inRange(hsv, lower_black, upper_black)
    openingValue = 10
    cv2.erode(mask, np.ones((openingValue, openingValue)))  # Erode to remove noise
    cv2.dilate(mask, np.ones((openingValue, openingValue)))  # Dilate to restore size

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100 and area < 1200:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                outputFrame = cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            else:
                outputFrame = cv2.drawContours(frame, [approx], -1, (255, 0, 0), 2)


    
            

    outputFrame = frame.copy()
    return outputFrame

# def findMarkers(self, frame):
#     mask = frame.copy()
#     cv2.GaussianBlur(mask, (5, 5), 0, mask)
#     hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)
#     # lower_black = np.array([160, 0, 0])
#     # upper_black = np.array([220, 0, 0])

#     # mask = cv2.inRange(hsv, upper_black, lower_black)
    
#     # make a mask for the color black
#     lower_black = np.array([0, 0, 0])
#     upper_black = np.array([255, 140, 110])

#     # limit = 30
#     # lower_black = np.array([0, 0, 0])
#     # upper_black = np.array([limit, limit, limit])

#     mask = cv2.inRange(hsv, lower_black, upper_black)
#     # openingValue = 2
#     # mask = cv2.erode(mask, np.ones((openingValue, openingValue)))  # Erode to remove noise
#     # mask = cv2.dilate(mask, np.ones((openingValue, openingValue)))  # Dilate to restore size

#     outputFrame = frame.copy()

#     contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#     markerClusters = []

#     for cnt in contours:
#         area = cv2.contourArea(cnt)
#         if area > 100 and area < 1200:
#             approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
#             # if len(approx) == 4 and cv2.isContourConvex(approx):
#             #     cv2.drawContours(outputFrame, [approx], 0, (0, 255, 0), 2)
#             # elif len(approx) >= 11 and len(approx) <= 15 and cv2.isContourConvex(approx):
#             #     cv2.drawContours(outputFrame, [approx], 0, (0, 0, 255), 2)
#             # else:
#             #     cv2.drawContours(outputFrame, [approx], 0, (255, 0, 0), 2)
#             rect = cv2.minAreaRect(cnt)
#             if abs(rect[1][0] - rect[1][1]) < 15:
#             # if True:
#                 box = cv2.boxPoints(rect)
#                 box = np.int_(box)
#                 # draw contours with random colour
#                 cv2.drawContours(outputFrame,[box],0,(random.randint(0,255), random.randint(0,255), random.randint(0,255)),2)

#                 # Cluster markers
#                 clusterIndex = -1
#                 if (len(markerClusters) == 0):
#                     markerClusters.append([rect[0][0], rect[0][1], 1, -1])
#                 else:
#                     for index, cluster in enumerate(markerClusters):
#                         if (abs(np.sqrt((cluster[0] - rect[0][0])**2 + (cluster[1] - rect[0][1])**2))) < (rect[1][0] * 3):
#                             cluster[0] = (cluster[0] + rect[0][0]) / 2
#                             cluster[1] = (cluster[1] + rect[0][1]) / 2
#                             cluster[2] += 1
#                             clusterIndex = index
#                             break
#                         else:
#                             clusterIndex = len(markerClusters)
#                             markerClusters.append([rect[0][0], rect[0][1], 1, -1])

#                 if len(approx) == 4:
#                     # 0 means picking station
#                     markerClusters[clusterIndex][3] = 0
#                 elif len(approx) >= 8 and cv2.isContourConvex(approx):
#                     # 1 means isle marker
#                     markerClusters[clusterIndex][3] = 1

#     # Write a label for each cluster
#     for cluster in markerClusters:
#         if cluster[3] != -1:
#             cv2.putText(outputFrame, f'{"P" if cluster[3] == 0 else "I"}: {cluster[2]}', (int(cluster[0]) - 12, int(cluster[1]) + 6), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)

#     # Publish ros2 message
#     self.publisher.publish(String(data=json.dumps(markerClusters)))

#     test = np.zeros((480, 640, 3), dtype=np.uint8)
#     test[:,:,0] = mask
#     test[:,:,1] = mask
#     test[:,:,2] = mask

#     test = np.array(test) * 255

#     # outputFrame[mask > 0] = [0, 255, 0]

#     return outputFrame