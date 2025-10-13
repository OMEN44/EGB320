import cv2
import numpy as np
from std_msgs.msg import String

from robot.vision.utils import getPoi, getEmptyPoi, HISTORY_LEN, PERSISTENCE_THRESHOLD

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
    # 3: list of marker rectangles
    # clusterCenter = [[0,0], 0, 0, []]
    clusterCenter = {
        'position': [0,0],
        'count': 0,
        'average_width': 0,
        'rectangles': []
    }

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200 and area < 10000:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            if len(approx) > 6 and cv2.isContourConvex(approx):
                
                x, y, w, h = cv2.boundingRect(approx)
                if abs(w - h) < 10:
                    # If same positionn assume duplicate
                    duplicate = False
                    for prev in clusterCenter['rectangles']:
                        if abs(prev[0] - x) < (w * 0.5) and abs(prev[1] - y) < (h * 0.5):
                            duplicate = True
                            break

                    if not duplicate:
                        outputFrame = cv2.drawContours(outputFrame, [approx], -1, (255, 0, 0), 2)
                        # outputFrame = cv2.circle(outputFrame, (int(x + w / 2), int(y + h / 2)), 5, (0, 255, 0), -1)
                        clusterCenter['position'] = [clusterCenter['position'][0] + (x + w / 2), clusterCenter['position'][1] + (y + h / 2)]
                        clusterCenter['count'] += 1
                        clusterCenter['average_width'] += w
                        clusterCenter['rectangles'].append((x, y, w, h))

    if (clusterCenter['count'] > 0):
        if (clusterCenter['count'] > 1):
            clusterCenter['position'][0] = clusterCenter['position'][0] / clusterCenter['count']
            clusterCenter['position'][1] = clusterCenter['position'][1] / clusterCenter['count']
            clusterCenter['average_width'] = clusterCenter['average_width'] / clusterCenter['count']
        outputFrame = cv2.circle(outputFrame, (int(clusterCenter['position'][0]), int(clusterCenter['position'][1])), 5, (0, 0, 255), -1)
        outputFrame = cv2.putText(
            outputFrame, 
            '{}: {}px'.format(str(clusterCenter['count']), str(np.round(clusterCenter['average_width']))), 
            (int(clusterCenter['position'][0]) - 5, 
            int(clusterCenter['position'][1]) + 40), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 
            2
        )

        poi = getPoi(7, clusterCenter['average_width'], clusterCenter['position'][0], self.calibration['new_k'][0,0])

        return [outputFrame, [
            poi if clusterCenter['count'] == 1 else getEmptyPoi(),
            poi if clusterCenter['count'] == 2 else getEmptyPoi(),
            poi if clusterCenter['count'] == 3 else getEmptyPoi(),
        ]]
    return [outputFrame, [getEmptyPoi(), getEmptyPoi(), getEmptyPoi()]]

def findPickingStationMarkers(self, hsvFrame, outputFrame):
    # make a mask for the color black
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 255, 100])

    mask = cv2.inRange(hsvFrame, lower_black, upper_black)

    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    mask = cv2.erode(mask, np.ones((5, 5)))  # Erode to remove noise
    mask = cv2.dilate(mask, np.ones((5, 5)))  # Dilate to restore size

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
                    isleClusters.append({'position': [x + w / 2, y + h / 2], 'count': 1, 'average_width': w, 'rectangles': [(x, y, w, h)]})
                else:
                    newCluster = True
                    for cluster in isleClusters:
                        if abs(cluster['position'][0] - x) < (w * 3): # Only check need to check in the x direction
                            # check to see if this is a double up
                            duplicate = False
                            for r in cluster['rectangles']:
                                if abs(r[0] - x) < (w * 0.5) and abs(r[1] - y) < (h * 0.5):
                                    duplicate = True
                                    break
                            if not duplicate:
                                cluster['position'][0] = (cluster['position'][0] + x + w / 2) / 2
                                cluster['position'][1] = (cluster['position'][1] + y + h / 2) / 2
                                cluster['count'] += 1
                                cluster['average_width'] += w
                                # cluster['average_width'] = (cluster['average_width'] + w) / 2
                                cluster['rectangles'].append((x, y, w, h))
                            newCluster = False
                            break
                    if newCluster:
                        isleClusters.append({'position': [x + w / 2, y + h / 2], 'count': 1, 'average_width': w, 'rectangles': [(x, y, w, h)]})


    # If there are two clusters with the same marker count assume the one on the right should be 1 count higher
    if len(isleClusters) > 1:
        done = False
        while not done:
            for i in range(len(isleClusters)):
                for j in range(i + 1, len(isleClusters)):
                    if isleClusters[i]['count'] == isleClusters[j]['count']:
                        if isleClusters[i]['position'][0] < isleClusters[j]['position'][0]:
                            isleClusters[j]['count'] += 1
                        else:
                            isleClusters[i]['count'] += 1
                        done = False
                        break
                else:
                    continue
                break
            else:
                done = True

    pickingStationMarkers = [getEmptyPoi(), getEmptyPoi(), getEmptyPoi()]

    for cluster in isleClusters:
        if cluster['count'] <= 3:
            cluster['average_width'] = cluster['average_width'] / cluster['count']
            pickingStationMarkers[cluster['count'] - 1] = getPoi(5, cluster['average_width'], cluster['position'][0], self.calibration['new_k'][0,0])

    
    self.poiHistory['picking_markers'].append(pickingStationMarkers)
    if len(self.poiHistory['picking_markers']) > HISTORY_LEN:
        self.poiHistory['picking_markers'].pop(0)
    l = len(self.poiHistory['picking_markers'])

    draft = []
    if l > 1:
        for i in range(len(pickingStationMarkers)):
            shouldExist = sum(self.poiHistory['picking_markers'][j][i].exists for j in range(l - 1)) > PERSISTENCE_THRESHOLD
            print('marker', i, ', count', sum(self.poiHistory['picking_markers'][j][i].exists for j in range(l - 1)), shouldExist)
            if shouldExist:
                draft.append(pickingStationMarkers[i])
            else:
                draft.append(getEmptyPoi())
                

    for cluster in isleClusters:
        if cluster['count'] <= 3 and draft[cluster['count'] - 1].exists:
            # draw a dot in the middle of each cluster and label with number of markers
            cv2.circle(outputFrame, (int(cluster['position'][0]), int(cluster['position'][1])), 5, (0, 0, 255), -1)
            cv2.putText(outputFrame, '{}'.format(str(cluster['count'])), (int(cluster['position'][0]) - 5, int(cluster['position'][1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            

    return [outputFrame, draft]