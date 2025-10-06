#!/usr/bin/python

from warehousebot_lib import *
import os, math
import numpy as np
import matplotlib.pyplot as plt
import time
import random


obstacle_width = 0.5  # m
max_range = 2.5      # m

shelf_to_aisle = {
    "0": ["1", np.pi/2],
    "1": ["1", -np.pi/2],
    "2": ["2", np.pi/2],
    "3": ["2", -np.pi/2],
    "4": ["3", np.pi/2],
    "5": ["3", -np.pi/2]   
}

pickingbay_distance_wall = {"1":0.75, "2":0.45, "3":0.1}
shelf_distance_marker = {"1":0.93, "2":0.65, "3":0.42, "4":0.15}
aisle_distance_wall = {"1":[0.2, np.pi/2], "2":[0.8, -np.pi/2], "3":[0.2, -np.pi/2]}

picking_bay_marker_distance = 0.29  # m

deliveries = []
for picking_bay in [1, 2, 3]:
    shelf_first = random.randint(0, 5)   # first number 0–5
    shelf_second = random.randint(1, 4)  # second number 1–4
    shelf_id = float(f"{shelf_first}.{shelf_second}")  # combine into format like 1.4
    deliveries.append((picking_bay, shelf_id))

# Unpack into individual variables
delivery_one, delivery_two, delivery_three = deliveries

print(deliveries)

# Example deliveries: (picking bay number, shelf id)
# deliveries = [(3, 0.3), (2, 0.3), (1, 5.4)]

deliveryNo = 0

# --- place once, outside main loop ---
previous_best_bearing = None
previous_omega = 0.0
previous_e_theta = 0.0
previous_time = time.time()


def getMeasurements(deliveries, deliveryNo):
    pickingBay, shelf = deliveries[deliveryNo]
    pickingBayArrayIndex = pickingBay - 1
    # Split shelf into two parts
    first, second = str(shelf).split(".")

    # Look up picking bay wall distance
    pickingBayWallDistance = pickingbay_distance_wall[str(pickingBay)]

    # Shelf → Aisle mapping
    aisle_id, shelfOrientation = shelf_to_aisle[first]

    # Aisle wall distance and orientation
    aisleWallDistance, aisleOrientation = aisle_distance_wall[aisle_id]

    # Shelf marker distance (based on the second digit)
    shelfMarkerDistance = shelf_distance_marker[second]

    return pickingBayArrayIndex, aisle_id, pickingBayWallDistance, aisleWallDistance, shelfMarkerDistance, shelfOrientation, aisleOrientation

# ---------------------------
# Potential fields
# ---------------------------
def repulsiveField(obstacleList, phi=np.linspace(-np.pi, np.pi, 360)):
    U_rep = np.zeros_like(phi)
    if not obstacleList:
        return U_rep
    
    for obs in obstacleList:
        if obs is None or len(obs) != 2:
            continue
        obs_distance, obs_bearing = obs
        if obs_distance <= 0 or abs(obs_bearing) > np.pi/2:
            continue

        dphi = np.arcsin((obstacle_width / 2) / obs_distance) if obs_distance > (obstacle_width / 2) else np.pi/2
        mask = (phi >= (obs_bearing - dphi)) & (phi <= (obs_bearing + dphi))
        U_rep[mask] = np.maximum(U_rep[mask], (1.0 / obs_distance))
    return U_rep


# Attractive field towards target
def attractiveField(target, phi=np.linspace(-np.pi, np.pi, 360), max_bearing_deg=90):
    if target is None:
        return np.zeros_like(phi)
    target_distance, target_bearing = target
    slope = target_distance / np.radians(max_bearing_deg)
    U_att = np.maximum(0.0, target_distance - np.abs(phi - target_bearing) * slope)
    return U_att


# Pick the best bearing (Goal - Obstacles)
def bestBearing(U_att, U_rep, phi=np.linspace(-np.pi, np.pi, 360)):
    U_total = U_att - U_rep
    if not np.isfinite(U_total).all():
        return None
    if np.allclose(U_total, 0.0):
        return None
    best_index = np.argmax(U_total)
    return phi[best_index]  # radians



# ---------------------------
# Utilities
# ---------------------------
def angle_wrap(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2*np.pi) - np.pi

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

# ---------------------------
# Scene & Robot configuration
# ---------------------------
sceneParameters = SceneParameters()
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl
sceneParameters.pickingStationContents[1] = warehouseObjects.mug
sceneParameters.pickingStationContents[2] = warehouseObjects.bottle

sceneParameters.obstacle0_StartingPosition = -1
# sceneParameters.obstacle1_StartingPosition = [-0.35, -0.15]
# sceneParameters.obstacle2_StartingPosition = [-0.60, -0.20]

sceneParameters.obstacle1_StartingPosition = -1
sceneParameters.obstacle2_StartingPosition = -1

robot_starting_x = random.uniform(-0.8, -0.4)  
robot_starting_y = random.uniform(-0.85, -0.1)  
robot_starting_angle = random.uniform(0, 2*np.pi)  # Random float between 0 and 2π radians
# sceneParameters.robotStartingPosition = [robot_starting_x, robot_starting_y, robot_starting_angle]  # x, y, theta in radians
# sceneParameters.robotStartingPosition = [-0.5, -0.3, np.pi/2]  # x, y, theta in radians
sceneParameters.robotStartingPosition = [robot_starting_x, robot_starting_y, math.radians(225)]  # x, y, theta in radians

robotParameters = RobotParameters()
robotParameters.driveType = 'differential'
robotParameters.minimumLinearSpeed = 0.0
robotParameters.maximumLinearSpeed = 0.25
robotParameters.driveSystemQuality = 1
robotParameters.cameraOrientation = 'landscape'
robotParameters.cameraDistanceFromRobotCenter = 0.1
robotParameters.cameraHeightFromFloor = 0.15
robotParameters.cameraTilt = 0.0
robotParameters.maxItemDetectionDistance = 1.0
robotParameters.maxPackingBayDetectionDistance = 2.5
robotParameters.maxObstacleDetectionDistance = 1.5
robotParameters.maxRowMarkerDetectionDistance = 2.5
robotParameters.collectorQuality = 1
robotParameters.maxCollectDistance = 0.15
robotParameters.sync = False


# ---------------------------
# Main
# ---------------------------
if __name__ == '__main__':
    try:
        state = -0.1
        print("Press Ctrl+C to stop the simulation\n")
        print("Connecting to CoppeliaSim...")

        bot = COPPELIA_WarehouseRobot(
            robotParameters, sceneParameters,
            coppelia_server_ip='192.168.56.1', port=23000
        )

        bot.SetTargetVelocities(0.0, 0.0)
        bot.StartSimulator()
        bot.SetTargetVelocities(0.0, 0.0)

        print("Starting main control loop...")
        print("Robot is now ready for navigation commands.")

        phi = np.linspace(-np.pi, np.pi, 360)  # Shared angular grid for fields
        print("Yellow - Moving to picking station or transporting item to target bay")

        while True:
            # print(state)
            # print(state)
            # print(state)
            bot.UpdateObjectPositions()

            # Order here must match what GetDetectedObjects returns
            objectsRB = bot.GetDetectedObjects([
                warehouseObjects.items,
                warehouseObjects.shelves,
                warehouseObjects.row_markers,
                warehouseObjects.obstacles,
                warehouseObjects.pickingStation,
                warehouseObjects.PickingStationMarkers
            ])
            itemsRB, packingStationRB, obstaclesRB, rowMarkerRB, shelfRB, pickingStationRB = objectsRB
            res, distance, point, obj, n = bot.sim.readProximitySensor(bot.proximityHandle)
            # print(packingStationRB)
            # print(packingStationRB)
            if state == -0.1:
                startIMU = bot.robotPose[5]
                state = -1
            
            # ------------------ STATE -1: Calibration depending on what robot can see ------------------------------
            elif state == -1:
                bot.SetTargetVelocities(0.0, -0.2)
                currentIMU = bot.robotPose[5]
                if abs(angle_wrap(currentIMU - startIMU)) < np.radians(350):    
                    sees_rowMarker = rowMarkerRB is not None and len(rowMarkerRB) > 0
                    if sees_rowMarker:
                        for i, row in enumerate(rowMarkerRB):
                            if row is not None and len(row) > 0:
                                rowIndex = i
                                bot.SetTargetVelocities(0.0, 0.0)
                                state = -1.1

                else:      
                    sees_shelf = shelfRB and any(row is not None for row in shelfRB)
                    sees_picking_bay = pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0
                    if sees_picking_bay:
                        state = 0  # Go straight to picking station state
                        bot.SetTargetVelocities(0.0, 0.0)  # Stop turning immediately
                        # print("Picking bay spotted, skipping to state 0")

                    elif sees_shelf:
                        # Keep turning while shelf is visible
                        bot.SetTargetVelocities(0.0, -0.2)
                        # print("Turning... still see shelf")

                    else:
                        # No shelf, no picking bay → fallback
                        state = -1.5
                        bot.SetTargetVelocities(0.0, 0.0)
                        # print("No shelf spotted, moving to -1.5")

            # ------------------ STATE -1.1: Robot sees a row marker ------------# 
            elif state == -1.1:
                # bot.GetCameraImage()
                has_row = (rowMarkerRB and rowMarkerRB[rowIndex] is not None and len(rowMarkerRB[rowIndex]) > 0)
                if has_row:
                    rowBearing = math.degrees(rowMarkerRB[rowIndex][1])
                    kp = 0.01
                    rotation_velocity = kp * rowBearing
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(rowBearing) < 40:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = -1.2
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, -0.2)

            # ------------------ STATE -1.2: Drive toward Aisle Marker with fields ------------
            elif state == -1.2:
                has_row2 = (rowMarkerRB and rowMarkerRB[rowIndex] is not None and len(rowMarkerRB[rowIndex]) > 0)
                if has_row2:
                    aisleBearing = rowMarkerRB[rowIndex][1]
                    aisleDistance = rowMarkerRB[rowIndex][0]
                    all_obstacles = []
                    for group in (obstaclesRB, pickingStationRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)   # merge lists
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(rowMarkerRB[rowIndex], phi)
                    best_bearing = bestBearing(U_att, U_rep, phi)

                    if best_bearing is not None:
                        theta = 0.0
                        e_theta = angle_wrap(best_bearing - theta)

                        k_omega = 0.4
                        v_max = 0.1
                        sigma = np.radians(30)

                        omega = k_omega * e_theta
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        bot.SetTargetVelocities(v, omega)
                    else:
                        bot.SetTargetVelocities(0.0, 0.15)

                    if aisleDistance < 1.24:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 3.1
                else:
                    bot.SetTargetVelocities(0.0, 0.15)


            # ------------------ STATE -1.5: Turn left until shelf is seen ------------------------------
            elif state == -1.5:
                sees_picking_bay = pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0
                if sees_picking_bay:
                    state = 0  # Go straight to picking station state
                    bot.SetTargetVelocities(0.0, 0.0)  # Stop turning immediately
                    # print("Picking bay spotted, skipping to state 0")
                if shelfRB and any(row is not None for row in shelfRB):
                    for i, row in enumerate(shelfRB):
                        if row is not None and len(row) > 0:
                            rowBearing = row[1]
                            rowIndex = i
                            state = -2
                            break
                    bot.SetTargetVelocities(0.0, 0.0)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)

            # ------------------ STATE -2: Turn to most right shelf ------------------------------
            elif state == -2:
                has_row = (shelfRB and shelfRB[rowIndex] is not None and len(shelfRB[rowIndex]) > 0)
                if has_row:
                    rowBearing = math.degrees(shelfRB[rowIndex][1])
                    rowDistance = shelfRB[rowIndex][0]
                    if rowDistance < 0.65:
                        state = 0
                    kp = 0.01
                    rotation_velocity = kp * rowBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(rowBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = -3
                        forward_time = time.time()
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)

            # ------------------ STATE -3: Move forward a bit to get a better view of picking bay 1 ------------------------------
            elif state == -3:
                bot.SetTargetVelocities(0.1, 0.0)
                if (time.time() - forward_time >= 0.5):
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 0

            # ------------------ STATE 0: Turn to Picking Station 1 Marker ------------------------------
            elif state == 0:
                has_ps1 = (pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0)
                if has_ps1:
                    bearing = pickingStationRB[0][1]  # radians
                    kp = 0.01
                    rotation_velocity = kp * math.degrees(bearing)  # deg * gain -> rad/s proxy per lib
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(math.degrees(bearing)) < 2:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 1
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                    
            # ------------------ STATE 1: Drive toward Picking Station 1 with fields ----------
            elif state == 1:
                has_ps1 = (pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0)
                if has_ps1:
                    markerDistance = pickingStationRB[0][0]
                    # Gather all detected objects into one list
                    all_obstacles = []
                    for group in (obstaclesRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)   # merge lists
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(pickingStationRB[0], phi)
                    best_bearing = bestBearing(U_att, U_rep, phi)

                    if best_bearing is not None:
                        theta = 0.0  # if you have a robot heading API, replace this with it
                        e_theta = angle_wrap(best_bearing - theta)

                        k_omega = 0.3
                        v_max = 0.1
                        sigma = np.radians(30)

                        omega = k_omega * e_theta
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        bot.SetTargetVelocities(v, omega)
                    else:
                        # No valid direction -> gentle spin to search
                        bot.SetTargetVelocities(0.0, 0.15)

                    if markerDistance < 0.35:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 2
                else:
                    # Lost the picking station -> search
                    bot.SetTargetVelocities(0.0, 0.2)

            # ------------------ STATE 2: Turn to Aisle Marker 2 ------------------------------
            elif state == 2:
                # bot.GetCameraImage()
                has_row2 = (rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0)
                if has_row2:
                    aisleBearing = math.degrees(rowMarkerRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * aisleBearing
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(aisleBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 3
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, -0.2)

            # ------------------ STATE 3: Drive toward Aisle Marker 2 with fields ------------
            elif state == 3:
                has_row2 = (rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0)
                if has_row2:
                    aisleBearing = rowMarkerRB[1][1]
                    aisleDistance = rowMarkerRB[1][0]
                    all_obstacles = []
                    for group in (obstaclesRB, pickingStationRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)   # merge lists
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(rowMarkerRB[1], phi)
                    best_bearing = bestBearing(U_att, U_rep, phi)

                    if best_bearing is not None:
                        theta = 0.0
                        e_theta = angle_wrap(best_bearing - theta)

                        k_omega = 0.4
                        v_max = 0.1
                        sigma = np.radians(30)

                        omega = k_omega * e_theta
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        bot.SetTargetVelocities(v, omega)
                    else:
                        bot.SetTargetVelocities(0.0, 0.15)

                    if aisleDistance < 1.24:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 3.1
                else:
                    bot.SetTargetVelocities(0.0, 0.15)
            
            # ------------------ STATE 3.1: Get distance of picking bay 1 ------------

            elif state == 3.1:
                has_pickingbay1 = (pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0)
                if has_pickingbay1:
                    picking_bay1_bearing = math.degrees(pickingStationRB[0][1])
                    kp = 0.01
                    rotation_velocity = kp * picking_bay1_bearing
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(picking_bay1_bearing) < 0.4:
                        bot.SetTargetVelocities(0.0, 0.0)
                        distance_pickingbay1 = pickingStationRB[0][0]
                        state = 3.2
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)

            # ------------------ STATE 3.2: Get distance of picking bay 2 ------------
            elif state == 3.2:
                has_pickingbay2 = (pickingStationRB and pickingStationRB[1] is not None and len(pickingStationRB[1]) > 0)
                if has_pickingbay2:
                    picking_bay2_bearing = math.degrees(pickingStationRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * picking_bay2_bearing
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(picking_bay2_bearing) < 0.4:
                        bot.SetTargetVelocities(0.0, 0.0)
                        distance_pickingbay2 = pickingStationRB[1][0]
                        state = 3.3
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, -0.2)

            # ------------------ STATE 3.3: Calculate IMU value towards aisle markers ------------
            elif state == 3.3:
                cos_D = (picking_bay_marker_distance**2 + distance_pickingbay2**2 - distance_pickingbay1**2) / (2 * distance_pickingbay2 * picking_bay_marker_distance)
    
                # Numerical stability check (rounding errors may push value slightly out of [-1, 1])
                cos_D = max(-1, min(1, cos_D))
                
                # Return angle in radians
                angleD = math.acos(cos_D)
                currentIMU = bot.robotPose[5]
                aisleIMU = currentIMU - angleD - np.pi/2
                state = 5

            # ------------------ STATE 4: Turn to Aisle Orientation ------------------------------
            elif state == 4:
                has_row2 = (rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0)
                if has_row2:
                    aisleBearing = math.degrees(rowMarkerRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * aisleBearing
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(aisleBearing) < 0.4:
                        bot.SetTargetVelocities(0.0, 0.0)
                        aisleIMU = bot.robotPose[5] # radians
                        state = 5
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, -0.2)

            # ------------------ STATE 5: turn towards wall ready to drive towards front of picking bay ------------------------------
            elif state == 5:
                # Desired heading: 90 degrees left from current aisle orientation
                target_heading = aisleIMU + np.pi/2  # radians
                current_heading = bot.robotPose[5]  # radians

                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5  # Adjust gain as needed
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 5.5
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)
            
            # ------------------ STATE 5.5: Get next delivery instructions ------------------------------
            elif state == 5.5:
                pickingBayArrayIndex, aisle_id, pickingBayWallDistance, aisleWallDistance, shelfMarkerDistance, shelfOrientation, aisleOrientation = getMeasurements(deliveries, deliveryNo)
                # wall_distance = distance
                desired_offset = pickingBayWallDistance
                # target_x = abs(wall_distance - desired_offset)
                # print(target_x)

                # target_y = 0.0
                # goalRB = np.array([target_x, target_y])
                state = 6


            #------------------ STATE 6: Drive to wall until certain distance ------------------------------
            elif state == 6:
                # print(packingStationRB)
                target_distance = pickingBayWallDistance 
                error = distance - target_distance

                # proportional control
                kp = 0.5   # tune as needed
                v = kp * error

                # clamp velocity so it doesn’t crawl too slowly or rush too fast
                v = max(min(v, 0.12), 0.03)  

                bot.SetTargetVelocities(v, 0.0)
                # print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")

                # stop when within ±1 cm of 0.45 m
                if abs(error) <= 0.02:
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 7
                
            # ------------------ STATE 7: Turn to Picking Bay ------------------------------
            elif state == 7:
                # bot.GetCameraImage()
                has_pickingbay = (pickingStationRB and pickingStationRB[pickingBayArrayIndex] is not None and len(pickingStationRB[pickingBayArrayIndex]) > 0)
                if has_pickingbay:
                    pickingBayBearing = math.degrees(pickingStationRB[pickingBayArrayIndex][1])
                    kp = 0.01
                    rotation_velocity = kp * pickingBayBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(pickingBayBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        pickingBayDistance = pickingStationRB[pickingBayArrayIndex][0]
                        state = 8
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)

            # ------------------ STATE 8: Drive to Picking Bay with fields ------------------------------
            elif state == 8:
                # print(pickingStationRB[pickingBayArrayIndex])
                has_row2 = (pickingStationRB and pickingStationRB[pickingBayArrayIndex] is not None and len(pickingStationRB[pickingBayArrayIndex]) > 0)
                if has_row2:
                    all_obstacles = []
                    for group in (obstaclesRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)   # merge lists
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(pickingStationRB[pickingBayArrayIndex], phi)
                    best_bearing = bestBearing(U_att, U_rep, phi)

                    if best_bearing is not None:
                        theta = 0.0
                        e_theta = angle_wrap(best_bearing - theta)

                        k_omega = 0.4
                        v_max = 0.1
                        sigma = np.radians(30)

                        omega = k_omega * e_theta
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        bot.SetTargetVelocities(v, omega)
                    else:
                        bot.SetTargetVelocities(0.0, 0.15)

                    if pickingStationRB[pickingBayArrayIndex][0] < 0.185:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Green - Collecting an item at the picking station")
                        state = 9
                else:
                    bot.SetTargetVelocities(0.0, 0.15)

            # ------------------ STATE 9: Collect item ------------------------------
            elif state == 9:
                success, station = bot.CollectItem(closest_picking_station=True)
                if success:
                    print("Item collected successfully")
                    print("Yellow - Moving to picking station or transporting item to target bay")
                    state = 10
                else:
                    state = 100            

            # ------------------ STATE 10: Turn until Row Marker seen ------------------------------
            elif state == 10:
                if rowMarkerRB:
                    for i, row in enumerate(rowMarkerRB):
                        if row is not None and len(row) > 0:
                            rowBearing = row[1]  # or [0], depending on your data
                            rowIndex = i
                            state = 11
                        else:
                            bot.SetTargetVelocities(0.0, 0.2)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                                         
            # ------------------ STATE 11: Turn to Row Marker ------------------------------
            elif state == 11:
                has_row = (rowMarkerRB and rowMarkerRB[rowIndex] is not None and len(rowMarkerRB[rowIndex]) > 0)
                if has_row:
                    rowBearing = math.degrees(rowMarkerRB[rowIndex][1])
                    kp = 0.01
                    rotation_velocity = kp * rowBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(rowBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 12
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)

            # ------------------ STATE 12: Drive toward Row Marker with fields ------------------------------
            elif state == 12:
                has_row = (rowMarkerRB and rowMarkerRB[rowIndex] is not None and len(rowMarkerRB[rowIndex]) > 0)
                if has_row:
                    markerDistance = rowMarkerRB[rowIndex][0]
                    # Gather all detected objects into one list
                    all_obstacles = []
                    for group in (obstaclesRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)   # merge lists
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(rowMarkerRB[rowIndex], phi)
                    best_bearing = bestBearing(U_att, U_rep, phi)

                    if best_bearing is not None:
                        theta = 0.0  # if you have a robot heading API, replace this with it
                        e_theta = angle_wrap(best_bearing - theta)

                        k_omega = 0.3
                        v_max = 0.1
                        sigma = np.radians(30)

                        omega = k_omega * e_theta
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        bot.SetTargetVelocities(v, omega)
                    else:
                        # No valid direction -> gentle spin to search
                        bot.SetTargetVelocities(0.0, 0.15)
                    if markerDistance < 1.3:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 13
                else:
                    # Lost the picking station -> search
                    bot.SetTargetVelocities(0.0, 0.2)

            # ------------------ STATE 13: Turn to face wall to drive to aisle entranse ------------------------------
            elif state == 13:
                target_heading = aisleIMU + aisleOrientation # radians
                current_heading = bot.robotPose[5]  # radians

                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5  # Adjust gain as needed
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 14
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)
            
            # ------------------ STATE 14: Drive to wall until certain distance (front of desired aisle) ------------------------------
            elif state == 14:
                target_distance = aisleWallDistance
                error = distance - target_distance
                
                # proportional control
                kp = 0.5   # <-- tune this value
                v = kp * error
                
                # clamp velocity
                v = max(min(v, 0.15), 0.03)  # between 0.03 and 0.15 m/s
                
                bot.SetTargetVelocities(v, 0.0)
                # print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")
                
                # within tolerance
                if abs(error) <= 0.07:  # 1 cm offset tolerance
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 15
                if (aisle_id == "1"):
                    if distance < aisleWallDistance:
                        state = 14.5

            # ------------------ STATE 14.5: Reverse a bit if overshot ------------------------------
            elif state == 14.5:
                bot.SetTargetVelocities(-0.1, 0.0)
                if distance > aisleWallDistance:
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 15

            # ------------------ STATE 15: Turn to face aisle marker ------------------------------
            elif state == 15:
                target_heading = aisleIMU
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 16
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)

            # ------------------ STATE 16: Drive toward Row Marker with fields ------------------------------
            elif state == 16:
                aisleIndex = int(aisle_id) - 1
                has_row = (rowMarkerRB and rowMarkerRB[aisleIndex] is not None and len(rowMarkerRB[aisleIndex]) > 0)
                if has_row:
                    markerDistance = rowMarkerRB[aisleIndex][0]
                    # --- Potential fields toward row marker ---
                    all_obstacles = []
                    for group in (obstaclesRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(rowMarkerRB[aisleIndex], phi)
                    best_bearing = bestBearing(U_att, U_rep, phi)

                    if best_bearing is not None:
                        theta = 0.0
                        e_theta = angle_wrap(best_bearing - theta)

                        k_omega = 0.3
                        v_max = 0.1
                        sigma = np.radians(30)

                        omega = k_omega * e_theta
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        bot.SetTargetVelocities(v, omega)
                    else:
                        bot.SetTargetVelocities(0.0, 0.15)

                    if markerDistance < shelfMarkerDistance:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 17
                else:
                    state = 13
            
            # ------------------ STATE 17: Turn to face shelf bay to drop item ------------------------------
            elif state == 17:
                target_heading = aisleIMU + shelfOrientation
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 18
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)
            
            # ------------------ STATE 18: Drive to shelf bay to drop item ------------------------------
            elif state == 18:
                target_distance = 0.06   #  6m
                error = distance - target_distance

                kp = 0.2   # proportional gain

                if error-0.009 <= 0.01:  # within ±1 cm
                    v = 0.0
                    bot.SetTargetVelocities(0.0, 0.0)
                    # print(f"Stopping. Distance: {distance:.3f}, Error: {error:.3f}")
                    print("Red - Placing Item on a shelf")
                    state = 19
                else:
                    v = kp * error
                    v = max(min(v, 0.05), -0.05)  # clamp both directions
                    bot.SetTargetVelocities(v, 0.0)
                    # print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")

            # ------------------ STATE 19: Drop item ------------------------------
            elif state == 19:
                bot.SetTargetVelocities(0.0, 0.0)
                bot.DropItem()
                bot.DropItemInClosestShelfBay()
                # print("Item dropped successfully.")
                backTime = time.time()
                print("Yellow - Moving to picking station or transporting item to target bay")
                state = 20

            # ------------------ STATE 20: Reverse a bit ------------------------------
            elif state == 20:
                if (time.time() - backTime >= 0.1):
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 21
                bot.SetTargetVelocities(-0.1, 0.0)

            # ------------------ STATE 21: Turn to face away from aisle marker ------------------------------
            elif state == 21:
                target_heading = aisleIMU + math.pi
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 22
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)


            # ------------------ STATE 22: Drive to wall until certain distance (return to return zone) ------------------------------                 
            elif state == 22:
                if aisle_id == "1":
                    return_index = 2
                    state = 222
                elif aisle_id == "2":
                    return_index = 0
                    state = 222
                else:
                    state = 23

            # # ------------------ STATE 221: Turn to face picking station (if aisle 1 delivery) ------------------------------
            # elif state == 221:
            #     # bot.GetCameraImage()
            #     has_row2 = (pickingStationRB and pickingStationRB[return_index] is not None and len(pickingStationRB[return_index]) > 0)
            #     if has_row2:
            #         stationBearing = math.degrees(pickingStationRB[return_index][1])
            #         kp = 0.01
            #         rotation_velocity = kp * stationBearing
            #         rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
            #         if abs(stationBearing) < 10:
            #             bot.SetTargetVelocities(0.0, 0.0)
            #             state = 222
            #         else:
            #             bot.SetTargetVelocities(0.0, rotation_velocity)
            #     else:
            #         bot.SetTargetVelocities(0.0, -0.2)

            # ------------------ STATE 222: Drive toward Picking Station with fields ------------------------------
            elif state == 222:
                has_ps1 = (pickingStationRB and pickingStationRB[return_index] is not None and len(pickingStationRB[return_index]) > 0)
                if has_ps1:
                    markerDistance = pickingStationRB[return_index][0]
                    # Gather all detected objects into one list
                    all_obstacles = []
                    for group in (obstaclesRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)   # merge lists
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(pickingStationRB[return_index], phi)
                    best_bearing = bestBearing(U_att, U_rep, phi)

                    if best_bearing is not None:
                        theta = 0.0  # if you have a robot heading API, replace this with it
                        e_theta = angle_wrap(best_bearing - theta)

                        k_omega = 0.3
                        v_max = 0.1
                        sigma = np.radians(30)

                        omega = k_omega * e_theta
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        bot.SetTargetVelocities(v, omega)
                    else:
                        # No valid direction -> gentle spin to search
                        bot.SetTargetVelocities(0.0, 0.15)

                    if markerDistance < 0.55:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 123
                else:
                    # Lost the picking station -> search
                    bot.SetTargetVelocities(0.0, 0.2)

            # ------------------ STATE 23: Drive back to return zone ------------------------------
            elif state == 23:
                target_distance = 0.53
                error = distance - target_distance
                
                # proportional control
                kp = 0.5   # <-- tune this value
                v = kp * error
                
                # clamp velocity
                v = max(min(v, 0.15), 0.03)  # between 0.03 and 0.15 m/s
                
                bot.SetTargetVelocities(v, 0.0)
                # print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")
                
                # within tolerance
                if abs(error) <= 0.1:  # 1 cm offset tolerance
                    bot.SetTargetVelocities(0.0, 0.0)
                    print("Successful delivery!")
                    state = 24

            # ------------------ STATE 24: Turn to face aisle ------------------------------
            elif state == 24:
                target_heading = aisleIMU + math.pi/2
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                    if (distance < 0.5):
                        state = 22
                    else:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 25
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)

            # ------------------ STATE 123: Turn to wall (to drive to front of aisle 2 for reset)  ------------------------------
            elif state == 123:
                target_heading = aisleIMU - math.pi/2
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 124
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)

            # ------------------ STATE 124: Drive to front of aisle 2 for reset ------------------------------
            elif state == 124:
                target_distance = aisle_distance_wall["2"][0]   
                error = distance - target_distance

                kp = 0.3   # proportional gain

                if error-0.009 <= 0.01:  # within ±1 cm
                    v = 0.0
                    bot.SetTargetVelocities(0.0, 0.0)
                    # print(f"Stopping. Distance: {distance:.3f}, Error: {error:.3f}")
                    state = 25
                else:
                    v = kp * error
                    v = max(min(v, 0.05), -0.05)  # clamp both directions
                    bot.SetTargetVelocities(v, 0.0)
                    # print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")

            # ------------------ STATE 25: Get next delivery instructions or end ------------------------------
            elif state == 25:
                if deliveryNo < len(deliveries) - 1:
                    deliveryNo += 1
                    state = 5
                    print("Next delivery...")
                    print("Yellow - Moving to picking station or transporting item to target bay")
                else:
                    print("Finished deliveries!")
                    print(deliveries)
                    state = 100
                

            elif state == 100:
                bot.SetTargetVelocities(0.0, 0.0)
            

    except KeyboardInterrupt:
        print("\nStopping simulation...")
        bot.StopSimulator()
        print("Simulation stopped successfully. Goodbye!")

    except Exception as e:
        print(f"\nAn error occurred: {e}")
        print("Stopping simulation...")
        try:
            bot.StopSimulator()
        except:
            pass
        print("Please check your CoppeliaSim setup and try again.")
