import cv2
import numpy as np
from std_msgs.msg import String

from robot.vision.utils import getPoi

import json
import random

isleMarkerCount = []

def findIsleMarkers(self, hsvFrame, outputFrame):
    # make a mask for the color black
    # lower_black = np.array([0, 0, 0])
    # upper_black = np.array([100, 140, 120])
    lower_black = np.array([0, 0, 110])
    upper_black = np.array([180, 100, 200])

    mask = cv2.inRange(hsvFrame, lower_black, upper_black)

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    # 0: (x,y) position
    # 1: number of markers
    # 2: average width
    clusterCenter = [[0,0], 0, 0]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200 and area < 10000:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            if cv2.isContourConvex(approx) and len(approx) > 4:
                
                x, y, w, h = cv2.boundingRect(approx)
                if abs(w - h) < 10:
                    outputFrame = cv2.drawContours(outputFrame, [approx], -1, (255, 0, 0), 2)
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
        outputFrame = cv2.putText(outputFrame, f'{clusterCenter[1]}: {np.round(clusterCenter[2])}px', (int(clusterCenter[0][0]) - 5, int(clusterCenter[0][1]) + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        return [outputFrame, [getPoi('isleMarkers', clusterCenter[1], 7, clusterCenter[2], clusterCenter[0][0])]]
    return [outputFrame, []]

def findPickingStation(self, hsvFrame, outputFrame):
    # make a mask for the color black
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 255, 80])

    mask = cv2.inRange(hsvFrame, lower_black, upper_black)

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    # 0: (x,y) position
    # 1: number of markers
    # 2: average width
    # 3: list of marker rectangles
    isleClusters = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 70 and area < 10000:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            # If convex and has 4 sides and is roughly square
            if cv2.isContourConvex(approx) and len(approx) == 4 and abs(w - h) < 10:
                outputFrame = cv2.drawContours(outputFrame, [approx], -1, (0, 255, 0), 2)
                
                if len(isleClusters) == 0:
                    isleClusters.append([[x + w / 2, y + h / 2], 1, w, [(x, y, w, h)]])
                else:
                    newCluster = True
                    for cluster in isleClusters:
                        if abs(cluster[0][0] - x) < (w * 3): # Only check need to check in the x direction
                            # check to see if this is a double up
                            duplicate = False
                            for r in cluster[3]:
                                if abs(r[0] - x) < (w * 0.5) and abs(r[1] - y) < (h * 0.5):
                                    duplicate = True
                                    break
                            if not duplicate:
                                cluster[0][0] = (cluster[0][0] + x + w / 2) / 2
                                cluster[0][1] = (cluster[0][1] + y + h / 2) / 2
                                cluster[1] += 1
                                cluster[2] = (cluster[2] + w)
                                cluster[3].append((x, y, w, h))
                            newCluster = False
                            break
                    if newCluster:
                        isleClusters.append([[x + w / 2, y + h / 2], 1, w, [(x, y, w, h)]])


    # If there are two clusters with the same marker count assume the one on the right should be 1 count higher
    if len(isleClusters) > 1:
        done = False
        while not done:
            for i in range(len(isleClusters)):
                for j in range(i + 1, len(isleClusters)):
                    if isleClusters[i][1] == isleClusters[j][1]:
                        if isleClusters[i][0][0] < isleClusters[j][0][0]:
                            isleClusters[j][1] += 1
                        else:
                            isleClusters[i][1] += 1
                        done = False
                        break
                else:
                    continue
                break
            else:
                done = True

    message = []

    for cluster in isleClusters:
        cluster[2] = cluster[2] / cluster[1]
        # draw a dot in the middle of each cluster and label with number of markers
        cv2.circle(outputFrame, (int(cluster[0][0]), int(cluster[0][1])), 5, (0, 0, 255), -1)
        cv2.putText(outputFrame, f'{cluster[1]}', (int(cluster[0][0]) - 5, int(cluster[0][1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        message.append(getPoi('pickingStation', cluster[1], 5, cluster[2], cluster[0][0]))

    return [outputFrame, message]