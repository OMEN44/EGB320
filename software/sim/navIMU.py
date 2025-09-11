#!/usr/bin/python

from warehousebot_lib import *
import os, math
import numpy as np
import matplotlib.pyplot as plt
import time

obstacle_width = 0.5  # m
max_range = 2.5      # m

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
            # print(math.degrees(bot.robotPose[5]))
            # ------------------ STATE 0: Find / center on Picking Station 1 ------------------
            res, distance, point, obj, n = bot.sim.readProximitySensor(bot.proximityHandle)
            if state == 0:
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

                    if markerDistance < 0.3:
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
                        state = 4
                else:
                    bot.SetTargetVelocities(0.0, 0.15)
            
            elif state == 4:
                has_row2 = (rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0)
                if has_row2:
                    aisleBearing = math.degrees(rowMarkerRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * aisleBearing
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(aisleBearing) < 0.4:
                        bot.SetTargetVelocities(0.0, 0.0)
                        aisleIMU = bot.robotPose[5]
                        state = 5
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, -0.2)

            elif state == 5:
                # has_pickingbay = (pickingStationRB and pickingStationRB[1] is not None and len(pickingStationRB[1]) > 0)
                # if has_pickingbay:
                #     pickingBayBearing = math.degrees(pickingStationRB[1][1])
                #     kp = 0.01
                #     rotation_velocity = kp * pickingBayBearing 
                #     rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                #     if abs(pickingBayBearing) < 1:
                #         bot.SetTargetVelocities(0.0, 0.0)
                #         pickingBayDistance = pickingStationRB[1][0]
                #         bearingPickingBay = bot.robotPose[5]
                #         state = 6
                #     else:
                #         bot.SetTargetVelocities(0.0, rotation_velocity)
                # else:
                #     bot.SetTargetVelocities(0.0, 0.2)

                state = 7
            
            elif state == 6: # FIX THIS SECTION
                # Print for debugging
                # print("Aisle heading (deg):", math.degrees(aisleIMU))
                # print("Bearing to picking bay (deg):", math.degrees(bearingPickingBay))
                # print("Relative bearing to perpendicular (deg):", math.degrees(bearingPickingBay - aisleIMU - np.pi/2))

                # # Perpendicular heading (90° from aisle)
                # perp_heading = aisleIMU + np.pi/2  

                # # Angle between perpendicular and the line to picking bay
                # delta = angle_wrap(bearingPickingBay - perp_heading)

                # # Projected distance along perpendicular
                # distanceForward = pickingBayDistance * np.sin(delta)  # Use sin, not cos

                # print("Forward distance along perpendicular (m):", distanceForward)


                state = 7

            elif state == 7:
                # Desired heading: 90 degrees left from current aisle orientation
                target_heading = aisleIMU + np.pi/2  # radians
                current_heading = bot.robotPose[5]  # radians

                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5  # Adjust gain as needed
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(e_theta) < np.radians(1):  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    forward_time = time.time()
                    state = 8
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)



            elif state == 8:
                target_distance = 0.45
                error = distance - target_distance

                # proportional control
                kp = 0.5   # tune as needed
                v = kp * error

                # clamp velocity so it doesn’t crawl too slowly or rush too fast
                v = max(min(v, 0.12), 0.03)  

                bot.SetTargetVelocities(v, 0.0)
                print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")

                # stop when within ±1 cm of 0.45 m
                if abs(error) <= 0.01:
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 9

                
    
            elif state == 9:
                # bot.GetCameraImage()
                has_pickingbay = (pickingStationRB and pickingStationRB[1] is not None and len(pickingStationRB[1]) > 0)
                if has_pickingbay:
                    pickingBayBearing = math.degrees(pickingStationRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * pickingBayBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(pickingBayBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        pickingBayDistance = pickingStationRB[1][0]
                        state = 10
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)

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
                    else:
                        bot.SetTargetVelocities(0.0, 0.15)

                    if pickingStationRB[1][0] < 0.185:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 11
                else:
                    bot.SetTargetVelocities(0.0, 0.15)

            elif state == 11:
                has_pickingbay = (pickingStationRB and pickingStationRB[1] is not None and len(pickingStationRB[1]) > 0)
                if has_pickingbay:
                    pickingBayBearing = math.degrees(pickingStationRB[1][1])
                    kp = 0.01
                    rotation_velocity = kp * pickingBayBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(pickingBayBearing ) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 12
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)

            elif state == 12:
                success, station = bot.CollectItem(closest_picking_station=True)
                if success:
                    state = 13
                else:
                    state = 11            


            elif state == 13:
                if rowMarkerRB:
                    for i, row in enumerate(rowMarkerRB):
                        if row is not None and len(row) > 0:
                            rowBearing = row[1]  # or [0], depending on your data
                            rowIndex = i
                            state = 14
                        else:
                            bot.SetTargetVelocities(0.0, 0.2)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)
                                         
            elif state == 14:
                has_row = (rowMarkerRB and rowMarkerRB[rowIndex] is not None and len(rowMarkerRB[rowIndex]) > 0)
                if has_row:
                    rowBearing = math.degrees(rowMarkerRB[rowIndex][1])
                    kp = 0.01
                    rotation_velocity = kp * rowBearing 
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                    if abs(rowBearing) < 1:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 15
                    else:
                        bot.SetTargetVelocities(0.0, rotation_velocity)
                else:
                    bot.SetTargetVelocities(0.0, 0.2)

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
                    if markerDistance < 1.24:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 17
                else:
                    # Lost the picking station -> search
                    bot.SetTargetVelocities(0.0, 0.2)

            elif state == 16:
                
                target_heading = aisleIMU# radians
                current_heading = bot.robotPose[5]  # radians

                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5  # Adjust gain as needed
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(e_theta) < np.radians(1):  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    forward_time = time.time()
                    state = 17
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)

            elif state == 17:
                target_heading = aisleIMU  - np.pi/2# radians
                current_heading = bot.robotPose[5]  # radians

                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5  # Adjust gain as needed
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(e_theta) < np.radians(1):  # close enough to target
                    bot.SetTargetVelocities(0.0, 0.0)
                    forward_time = time.time()
                    state = 18
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)
            
            elif state == 18:
                bot.SetTargetVelocities(0.1, 0.0)
                if (time.time() - forward_time >= 0.5):
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 22
            
            elif state == 19:
                # Align with aisle heading
                target_heading = aisleIMU
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(e_theta) < np.radians(1):  
                    bot.SetTargetVelocities(0.0, 0.0)

                    # Now check if marker is visible and within acceptable bearing
                    has_row2 = (rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0)
                    if has_row2:
                        aisleBearing = math.degrees(rowMarkerRB[1][1])
                        if abs(aisleBearing) < 5:   # acceptable entry window
                            state = 20
                        else:
                            state = 21
                    else:
                        state = 21
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)

            elif state == 20:
                has_row = (rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0)

                if has_row:
                    markerDistance = rowMarkerRB[1][0]
                    # --- Potential fields toward row marker ---
                    all_obstacles = []
                    for group in (obstaclesRB, shelfRB):
                        if group:
                            all_obstacles.extend(group)
                    U_rep = repulsiveField(all_obstacles, phi)
                    U_att = attractiveField(rowMarkerRB[1], phi)
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

                    if markerDistance < 0.42:
                        bot.SetTargetVelocities(0.0, 0.0)
                        state = 24

                else:
                    state = 21


            elif state == 21:
                # Overshoot correction
                target_heading = aisleIMU - np.pi/2
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(e_theta) < np.radians(1):
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 18
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)
            
            elif state == 22:
                target_distance = 0.8
                error = distance - target_distance
                
                # proportional control
                kp = 0.5   # <-- tune this value
                v = kp * error
                
                # clamp velocity
                v = max(min(v, 0.15), 0.03)  # between 0.03 and 0.15 m/s
                
                bot.SetTargetVelocities(v, 0.0)
                print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")
                
                # within tolerance
                if abs(error) <= 0.07:  # 1 cm offset tolerance
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 23

            elif state == 23:
                target_heading = aisleIMU
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(e_theta) < np.radians(1):
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 20
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)
            
            elif state == 24:
                target_heading = aisleIMU + math.pi/2
                current_heading = bot.robotPose[5]
                e_theta = angle_wrap(target_heading - current_heading)

                kp = 0.5
                rotation_velocity = kp * e_theta
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(e_theta) < np.radians(1):
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 25
                else:
                    bot.SetTargetVelocities(0.0, rotation_velocity)
            
            elif state == 25:
                target_distance = 0.1   # 11 cm
                error = distance - target_distance

                # proportional control
                kp = 0.5   # tune lower here since it's a very small range
                v = kp * error

                # clamp velocity: slower because we're so close
                v = max(min(v, 0.05), 0.01)

                bot.SetTargetVelocities(v, 0.0)
                print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")

                # stop when within ±0.005 m (0.5 cm)
                if abs(error) <= 0.05:
                    bot.SetTargetVelocities(0.0, 0.0)
                    state = 26


            elif state == 26:
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
