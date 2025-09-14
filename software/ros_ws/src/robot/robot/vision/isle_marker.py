import cv2
import numpy as np
from std_msgs.msg import String

from robot.vision.utils import objectDistance, objectAngle

import json
import random

isleMarkerCount = []

def findIsleMarkers(self, frame, outputFrame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # make a mask for the color black
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 140, 120])

    mask = cv2.inRange(hsv, lower_black, upper_black)

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    # 0: (x,y) position
    # 1: number of markers
    # 2: average width
    clusterCenter = [[0,0], 0, 0]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200 and area <  2000:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            # outputFrame = cv2.drawContours(outputFrame, [approx], -1, (0, 0, 255), 2)
            if cv2.isContourConvex(approx):
                
                x, y, w, h = cv2.boundingRect(approx)
                # outputFrame = cv2.circle(outputFrame, (int(x + w / 2), int(y + h / 2)), 5, (0, 255, 0), -1)
                clusterCenter[0][0] += (x + w / 2)
                clusterCenter[0][1] += (y + h / 2)
                clusterCenter[1] += 1
                clusterCenter[2] += w
                
    if (clusterCenter[1] > 0):
        clusterCenter[0][0] = clusterCenter[0][0] / clusterCenter[1]
        clusterCenter[0][1] = clusterCenter[0][1] / clusterCenter[1]
        clusterCenter[2] = clusterCenter[2] / clusterCenter[1]
        outputFrame = cv2.circle(outputFrame, (int(clusterCenter[0][0]), int(clusterCenter[0][1])), 5, (0, 0, 255), -1)
        outputFrame = cv2.putText(outputFrame, f'{clusterCenter[1]}', (int(clusterCenter[0][0]) - 5, int(clusterCenter[0][1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        distance = objectDistance(7, clusterCenter[2])
        return [outputFrame, [f'I{clusterCenter[1]}: {distance}cm @ {objectAngle(clusterCenter[0][0])}°']]
    return [outputFrame, []]
    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray = cv2.medianBlur(gray, 5)

    # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 5,
    #                            param1=100, param2=30,
    #                            minRadius=1, maxRadius=30)
    
    # if len(isleMarkerCount) > 30:
    #     isleMarkerCount.pop(0)
    # isleMarkerCount.append(len(circles[0]) if circles is not None else 0)
    
    # avgRadius = 0

    # if circles is not None:
    #     circles = np.uint16(circles)
    #     for i in circles[0, :]:
    #         avgRadius += i[2]
    #         # draw the outer circle
    #         outputFrame = cv2.circle(outputFrame, (i[0], i[1]), i[2], (255, 0, 255), 2)
    #         # draw the center of the circle
    #         outputFrame = cv2.circle(outputFrame, (i[0], i[1]), 2, (0, 0, 255), 3)
    #     avgRadius /= len(circles[0])

    # count = np.round(np.mean(isleMarkerCount))
    # distance = ((FOCAL_LENGTH * 7) / (2 * avgRadius)) if avgRadius != 0 else -1

    # # self.publisher.publish(String(data=f'isle_markers: {count}, width: {np.round(avgRadius * 2, 2)}px, distance: {distance} cm'))

    # return [outputFrame, [f'isle_markers: {count}, width: {np.round(avgRadius * 2, 2)}px, distance: {distance} cm']]

def findPickingStation(frame, outputFrame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # make a mask for the color black
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([255, 255, 50])

    mask = cv2.inRange(hsv, lower_black, upper_black)

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    # 0: (x,y) position
    # 1: number of markers
    # 2: average width
    # 3: list of marker rectangles
    isleClusters = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 70 and area < 1200:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            if cv2.isContourConvex(approx) and len(approx) == 4:
                outputFrame = cv2.drawContours(outputFrame, [approx], -1, (0, 255, 0), 2)
                
                rect = cv2.boundingRect(approx)
                if len(isleClusters) == 0:
                    isleClusters.append([[rect[0] + rect[2] / 2, rect[1] + rect[3] / 2], 1, rect[2], [rect]])
                else:
                    newCluster = True
                    for cluster in isleClusters:
                        if abs(cluster[0][0] - rect[0]) < (rect[2] * 2.5):
                            # check to see if this is a double up
                            duplicate = False
                            for r in cluster[3]:
                                if abs(r[0] - rect[0]) < (rect[2] * 0.5) and abs(r[1] - rect[1]) < (rect[3] * 0.5):
                                    duplicate = True
                                    break
                            if not duplicate:
                                cluster[0][0] = (cluster[0][0] + rect[0] + rect[2] / 2) / 2
                                cluster[0][1] = (cluster[0][1] + rect[1] + rect[3] / 2) / 2
                                cluster[1] += 1
                                cluster[2] = (cluster[2] + rect[2])
                                cluster[3].append(rect)
                            newCluster = False
                            break
                    if newCluster:
                        isleClusters.append([[rect[0] + rect[2] / 2, rect[1] + rect[3] / 2], 1, rect[2], [rect]])

    # If there are two clusters with the same marker count assume the one on the right should be 1 count higher
    if len(isleClusters) > 1:
        for i in range(len(isleClusters)):
            for j in range(i + 1, len(isleClusters)):
                if isleClusters[i][1] == isleClusters[j][1]:
                    if isleClusters[i][0][0] < isleClusters[j][0][0]:
                        isleClusters[j][1] += 1
                    else:
                        isleClusters[i][1] += 1

    message = []

    for cluster in isleClusters:
        cluster[2] = cluster[2] / cluster[1]
        # draw a dot in the middle of each cluster and label with number of markers
        cv2.circle(outputFrame, (int(cluster[0][0]), int(cluster[0][1])), 5, (0, 0, 255), -1)
        cv2.putText(outputFrame, f'{cluster[1]}', (int(cluster[0][0]) - 5, int(cluster[0][1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        distance = objectDistance(7, cluster[2])
        message.append(f'P{cluster[1]}: {distance}cm @ {objectAngle(cluster[0][0])}°')

    return [outputFrame, message]