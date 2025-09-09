import cv2
import numpy as np

def findMarkers(frame):
    mask = frame.copy()
    cv2.GaussianBlur(mask, (5, 5), 0, mask)
    hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)
    # lower_black = np.array([160, 0, 0])
    # upper_black = np.array([220, 0, 0])

    # mask = cv2.inRange(hsv, upper_black, lower_black)
    
    # make a mask for the color black
    # lower_black = np.array([0, 0, 0])
    # upper_black = np.array([255, 120, 80])

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([30, 30, 30])

    mask = cv2.inRange(hsv, lower_black, upper_black)
    openingValue = 8
    mask = cv2.erode(mask, np.ones((openingValue, openingValue)))  # Erode to remove noise
    mask = cv2.dilate(mask, np.ones((openingValue, openingValue)))  # Dilate to restore size

    outputFrame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    markerClusters = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 150:
            approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
            # print(approx)
            rect = cv2.minAreaRect(cnt)
            if abs(rect[1][0] - rect[1][1]) < 15:
                box = cv2.boxPoints(rect)
                box = np.int_(box)
                cv2.drawContours(outputFrame,[box],0,(0,0,255),2)

                # Cluster markers
                clusterIndex = -1
                if (len(markerClusters) == 0):
                    markerClusters.append([rect[0][0], rect[0][1], 1, -1])
                else:
                    for index, cluster in enumerate(markerClusters):
                        if abs(np.sqrt((cluster[0] - rect[0][0])**2 + (cluster[1] - rect[0][1]))) < rect[1][0] * 1.5:
                            cluster[0] = (cluster[0] + rect[0][0]) / 2
                            cluster[1] = (cluster[1] + rect[0][1]) / 2
                            cluster[2] += 1
                            clusterIndex = index
                            break
                        else:
                            clusterIndex = len(markerClusters)
                            markerClusters.append([rect[0][0], rect[0][1], 1, -1])

                if len(approx) == 4:
                    # 0 means picking station
                    markerClusters[clusterIndex][3] = 0
                elif len(approx) >= 8 and cv2.isContourConvex(approx):
                    # 1 means isle marker
                    markerClusters[clusterIndex][3] = 1

    # Write a label for each cluster
    for cluster in markerClusters:
        if cluster[3] != -1:
            print(f'{"Picking" if cluster[3] == 0 else "Isle"} marker at ({cluster[0]}, {cluster[1]}) with {cluster[2]} markers')
            print(cluster)
            cv2.putText(outputFrame, f'{"P" if cluster[3] == 0 else "I"}: {cluster[2]}', (int(cluster[0]) - 12, int(cluster[1]) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            
            

    test = np.zeros((480, 640, 3), dtype=np.uint8)
    test[:,:,0] = mask
    test[:,:,1] = mask
    test[:,:,2] = mask

    test = np.array(test) * 255

    # outputFrame[mask > 0] = [0, 255, 0]

    return outputFrame