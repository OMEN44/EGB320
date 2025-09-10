#!/usr/bin/python

from warehousebot_lib import *
import os, math
import numpy as np
import matplotlib.pyplot as plt
import time

obstacle_width = 0.5  # m
max_range = 2.5      # m
message = "HELLO"

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

        dphi = np.arcsin((obstacle_width/2) / obs_distance) if obs_distance > (obstacle_width/2) else np.pi/2
        mask = (phi >= (obs_bearing - dphi)) & (phi <= (obs_bearing + dphi))
        U_rep[mask] = np.maximum(U_rep[mask], (1.0 / obs_distance))
    return U_rep


def attractiveField(target, phi=np.linspace(-np.pi, np.pi, 360), max_bearing_deg=90):
    if target is None:
        return np.zeros_like(phi)
    target_distance, target_bearing = target
    slope = target_distance / np.radians(max_bearing_deg)
    U_att = np.maximum(0.0, target_distance - np.abs(phi - target_bearing) * slope)
    return U_att

def bestBearing(U_att, U_rep, phi=np.linspace(-np.pi, np.pi, 360)):
    # Sensor-view slide uses subtraction (Goal - Obstacles)
    U_total = U_att - U_rep
    if not np.isfinite(U_total).all():
        return None
    if np.allclose(U_total, 0.0):
        return None
    best_index = np.argmax(U_total)
    return phi[best_index]   # radians



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
        state = 0
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

        while True:
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
            print(math.degrees(bot.robotPose[5]))
            # ------------------ STATE 0: Find / center on Picking Station 1 ------------------
            if state == 0:
                
                has_ps1 = (pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0)
                if has_ps1:
                    bearing = pickingStationRB[0][1]  # radians
                    kp = 0.01
                    rotation_velocity = kp * math.degrees(bearing)  # deg * gain -> rad/s proxy per lib
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(math.degrees(bearing)) < 2:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Centred at Picking Bay 1")
                        state = 1
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Station 1... bearing: {math.degrees(bearing):.2f}°")
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                    print("Searching for Station 1")

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
                        print(f"Moving to picking bay marker 1 | v={v:.2f} m/s, ω={omega:.2f} rad/s")
                    else:
                        # No valid direction -> gentle spin to search
                        bot.SetTargetVelocities(0.0, 0.15)
                        print("No valid best bearing; spinning to search...")

                    if markerDistance < 0.3:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Stopped near picking bay 1")
                        state = 2
                else:
                    # Lost the picking station -> search
                    bot.SetTargetVelocities(0.0, 0.2)
                    print("Lost Station 1; searching...")

            # ------------------ STATE 2: Turn to Aisle Marker 2 ------------------------------
            elif state == 2:
                print("Turning to Aisle Marker 2")
                # bot.GetCameraImage()
                has_row2 = (rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0)
                if has_row2:
                    aisleBearing = math.degrees(rowMarkerRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * aisleBearing
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(aisleBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Centred at Aisle 2 Marker")
                        state = 3
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Aisle 2... bearing: {aisleBearing:.2f}°")
                else:
                    bot.SetTargetVelocities(0.0, -0.2)
                    print("Searching for Aisle Marker 2")

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
                        print(f"Moving to Aisle 2 Marker | v={v:.2f} m/s, ω={omega:.2f} rad/s")
                    else:
                        bot.SetTargetVelocities(0.0, 0.15)
                        print("No valid best bearing (Aisle 2); spinning to search...")

                    if aisleDistance < 1.20:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 4
                        # z = abs(1.968 - aisleDistance- 0.13) # EDIT THIS
                else:
                    bot.SetTargetVelocities(0.0, 0.15)
                    print("Lost Aisle 2 marker; spinning to reacquire...")
            
            elif state == 4:
                has_row2 = (rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0)
                if has_row2:
                    aisleBearing = math.degrees(rowMarkerRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * aisleBearing
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(aisleBearing) < 0.4:
                        bot.SetTargetVelocities(0.0, 0.0)
                        z = 2 - (aisleDistance + 0.575)
                        print("Stopped near Aisle 2 Entrance")
                        print(z)
                        state = 5
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Aisle 2... bearing: {aisleBearing:.2f}°")
                else:
                    bot.SetTargetVelocities(0.0, -0.2)
                    print("Searching for Aisle Marker 2")

            elif state == 5:
                # bot.GetCameraImage()
                has_pickingbay = (pickingStationRB and pickingStationRB[1] is not None and len(pickingStationRB[1]) > 0)
                if has_pickingbay:
                    pickingBayBearing = math.degrees(pickingStationRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * pickingBayBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(pickingBayBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Centred at Picking Bay Marker")
                        pickingBayDistance = pickingStationRB[1][0]
                        print(pickingBayDistance)
                        state = 6
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Picking Bay Marker... bearing: {aisleBearing:.2f}°")
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                    print("Searching for Picking Bay Marker")
            
            elif state == 6:
                # alpha = np.arcsin(z/pickingBayDistance)
                alpha = np.arctan2(z, np.sqrt(pickingBayDistance**2 - z**2))
                distanceForward = np.sqrt(pickingBayDistance**2 - z**2)
                # distanceForward = pickingBayDistance*np.cos(alpha)
                turn_time = time.time()
                print("ALPHA")
                print(alpha)
                state = 7

            elif state == 7:
                omega = -0.2
                duration_turn = alpha / abs(omega)  
                bot.SetTargetVelocities(0.0, omega)
                if time.time() - turn_time >= duration_turn:
                    bot.SetTargetVelocities(0.0, 0.0)
                    forward_time = time.time()
                    state = 8

            elif state == 8:
                forward_vela = 0.1
                duration_forward = distanceForward / abs(forward_vela) 
                if (time.time() - forward_time >= duration_forward * 0.22):
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 9
                bot.SetTargetVelocities(forward_vela, 0.0)
    
            elif state == 9:
                # bot.GetCameraImage()
                has_pickingbay = (pickingStationRB and pickingStationRB[1] is not None and len(pickingStationRB[1]) > 0)
                if has_pickingbay:
                    pickingBayBearing = math.degrees(pickingStationRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * pickingBayBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(pickingBayBearing ) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Centred at Picking Bay Marker")
                        pickingBayDistance = pickingStationRB[1][0]
                        state = 10
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Picking Bay Marker... bearing: {aisleBearing:.2f}°")
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                    print("Searching for Picking Bay Marker")

            # ------------------ STATE 3: Drive toward Aisle Marker 2 with fields ------------
            elif state == 10:
                has_row2 = (pickingStationRB and pickingStationRB[1] is not None and len(pickingStationRB[1]) > 0)
                if has_row2:
                    all_obstacles = []
                    for group in (obstaclesRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)   # merge lists
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(pickingStationRB[1], phi)
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
                        print(f"Moving to Aisle 2 Marker | v={v:.2f} m/s, ω={omega:.2f} rad/s")
                    else:
                        bot.SetTargetVelocities(0.0, 0.15)
                        print("No valid best bearing (Aisle 2); spinning to search...")

                    if pickingStationRB[1][0] < 0.185:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Stopped near Aisle 2 Entrance")
                        state = 11
                else:
                    bot.SetTargetVelocities(0.0, 0.15)
                    print("Lost Aisle 2 marker; spinning to reacquire...")

            elif state == 11:
                has_pickingbay = (pickingStationRB and pickingStationRB[1] is not None and len(pickingStationRB[1]) > 0)
                if has_pickingbay:
                    pickingBayBearing = math.degrees(pickingStationRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * pickingBayBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(pickingBayBearing ) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Centred at Picking Bay Marker")
                        state = 12
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Picking Bay Marker... bearing: {aisleBearing:.2f}°")
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                    print("Searching for Picking Bay Marker")

            elif state == 12:
                success, station = bot.CollectItem(closest_picking_station=True)
                if success:
                    print(f"Collected item from station {station}")
                    state = 13
                    # turn_time = time.time()
                    # flipTurnSpeed = 0.4
                    # flipTurnDuration = ((math.pi) / flipTurnSpeed) 
                else:
                    print("Failed to collect item")   
                    state = 11            


            elif state == 13:
                bot.GetCameraImage()
                # bot.SetTargetVelocities(0.0, flipTurnSpeed)
                # if (time.time() - turn_time >= flipTurnDuration):
                #     state = 14
                if rowMarkerRB:
                    for i, row in enumerate(rowMarkerRB):
                        if row is not None and len(row) > 0:
                            rowBearing = row[1]  # or [0], depending on your data
                            rowIndex = i
                            state = 14
                        else:
                            bot.SetTargetVelocities(0.0, 0.2)
                            print("Searching for Picking Bay Marker")       
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                    print("Searching for Picking Bay Marker")   
                                         
            elif state == 14:
                has_row = (rowMarkerRB and rowMarkerRB[rowIndex] is not None and len(rowMarkerRB[rowIndex]) > 0)
                if has_row:
                    rowBearing = math.degrees(rowMarkerRB[rowIndex][1])
                    kp = 0.01
                    rotation_velocity = kp * rowBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(rowBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        print("Centred at Picking Bay Marker")
                        state = 15
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Picking Bay Marker... bearing: {aisleBearing:.2f}°")
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                    print("Searching for Picking Bay Marker")

            elif state == 15:
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
                    if markerDistance < 1.2:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 16
                else:
                    # Lost the picking station -> search
                    bot.SetTargetVelocities(0.0, 0.2)

            elif state == 16:
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
