#!/usr/bin/python

"""
EGB320 CoppeliaSim Warehouse Robot Example

GETTING STARTED:
===============
1. Open CoppeliaSim
2. Load the warehouse robot scene file
3. Run this Python script
4. The robot will connect to CoppeliaSim and start detecting objects

WHAT THIS EXAMPLE DOES:
======================
- Connects to CoppeliaSim simulation
- Sets up a warehouse scene with items at picking stations
- Continuously detects objects using the robot's camera
- Shows how to control robot movement
- Demonstrates item collection and dropping

STUDENT TASKS:
=============
- Modify the robot movement commands in the main loop
- Implement navigation algorithms using object detection data
- Add logic to collect items and deliver them to shelves
- Experiment with different robot parameters

For more information, see the documentation in warehousebot_lib.py
"""

# Import the warehouse bot library
from warehousebot_lib import *

# Import additional modules
import os
import time
import math
import numpy as np

def clear_screen():
    """Clear the terminal screen for better output readability"""
    os.system('cls' if os.name == 'nt' else 'clear')

# CONFIGURE SCENE PARAMETERS
sceneParameters = SceneParameters()

# Set which items appear at which picking stations (0=station 1, 1=station 2, 2=station 3)
# Set to -1 to leave a station empty
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl    # Bowl at picking station 1
sceneParameters.pickingStationContents[1] = warehouseObjects.mug     # Mug at picking station 2  
sceneParameters.pickingStationContents[2] = warehouseObjects.bottle  # Bottle at picking station 3

# Set obstacle starting positions [x, y] in meters
# Use -1 to keep current CoppeliaSim position, None to disable obstacle
# sceneParameters.obstacle0_StartingPosition = [-0.2, -0.25]
sceneParameters.obstacle0_StartingPosition = -1
sceneParameters.obstacle1_StartingPosition = -1  # Use current position
sceneParameters.obstacle2_StartingPosition = -1  # Use current position

# CONFIGURE ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive system settings
robotParameters.driveType = 'differential'        # Type of drive system
robotParameters.minimumLinearSpeed = 0.0          # Minimum forward speed (m/s)
robotParameters.maximumLinearSpeed = 0.25         # Maximum forward speed (m/s)
robotParameters.driveSystemQuality = 1            # Drive quality (0-1, 1=perfect)

# Camera settings
robotParameters.cameraOrientation = 'landscape'   # Camera orientation
robotParameters.cameraDistanceFromRobotCenter = 0.1  # Distance from robot center (m)
robotParameters.cameraHeightFromFloor = 0.15      # Height above floor (m)
robotParameters.cameraTilt = 0.0                # Camera tilt angle (radians)

# Object detection ranges (in meters)
robotParameters.maxItemDetectionDistance = 1.0         # Items
robotParameters.maxPackingBayDetectionDistance = 2.5   # Picking stations
robotParameters.maxObstacleDetectionDistance = 1.5     # Obstacles
robotParameters.maxRowMarkerDetectionDistance = 2.5    # Row markers

# Item collection settings
robotParameters.collectorQuality = 1              # Collector reliability (0-1)
robotParameters.maxCollectDistance = 0.15         # Maximum collection distance (m)

# Simulation settings
robotParameters.sync = False  # Use asynchronous mode (recommended)

# MAIN PROGRAM
if __name__ == '__main__':
    # Use try-except to handle Ctrl+C gracefully
    try:
        state = 0        # 0 = idle/rotate, 1 = rotating, 2 = moving forward
        shelf_index = None
        pickingBays = [0, 1, 2]
        rightAngleTurn = (math.pi / 2) / 0.2   # ≈ 7.85 seconds        print("EGB320 CoppeliaSim Warehouse Robot Example")
        desired_station_standoff = 0.4   # meters; “between shelf and bay” stop
        print("Press Ctrl+C to stop the simulation\n")
        
        # Enable/disable debug output
        show_debug_info = False

        # Create and initialize the warehouse robot
        print("Connecting to CoppeliaSim...")
        # warehouseBotSim = COPPELIA_WarehouseRobot(robotParameters, sceneParameters, 
        #                                           coppelia_server_ip='127.0.0.1', port=23000)
        warehouseBotSim = COPPELIA_WarehouseRobot(
            robotParameters, sceneParameters, 
            coppelia_server_ip='192.168.56.1', port=23000
        )

        # Start the simulation
        warehouseBotSim.SetTargetVelocities(0.0, 0.0)  # Ensure robot starts stopped
        warehouseBotSim.StartSimulator()
        warehouseBotSim.SetTargetVelocities(0.0, 0.0)  # Ensure robot starts stopped

        # Main control loop
        print("Starting main control loop...")
        print("Robot is now ready for navigation commands.")
        
        while True:
            warehouseBotSim.UpdateObjectPositions()
            # Set robot movement (forward_velocity, rotation_velocity)
            # Students: Replace these values with your navigation algorithm
            # warehouseBotSim.SetTargetVelocities(0.0, 0.0)  # Stop the robot

            # Get all detected objects in camera field of view
            objectsRB = warehouseBotSim.GetDetectedObjects([
                warehouseObjects.items,                # Items (bowls, mugs, etc.)
                warehouseObjects.shelves,              # Storage shelves
                warehouseObjects.row_markers,          # Navigation markers
                warehouseObjects.obstacles,            # Obstacles to avoid
                warehouseObjects.pickingStation,       # Main picking station
                warehouseObjects.PickingStationMarkers # Individual picking stations
            ])

            # Unpack the detection results
            itemsRB, packingStationRB, obstaclesRB, rowMarkerRB, shelfRB, pickingStationRB = objectsRB
            warehouseBotSim.GetCameraImage()

            # --- Control logic ---
            # Assuming state variable: 0 = rotating, 1 = moving forward, 2 = stopped
            # if state == 0:  # Turning to look for picking station 1
            #     warehouseBotSim.SetTargetVelocities(0.0, 0.3)  # Rotate right
            #     print("ROTATING")
                
            #     # Check only picking station 1
            #     if pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0:
            #         bearing = pickingStationRB[0][1]  # bearing of station 1
            #         if abs(math.degrees(bearing)) < 4:
            #             pickingStation_index = 0
            #             state = 1  # switch to moving forward
            #             print("Station 1 spotted, moving forward")

            if state == 0:  # Turning to look for picking station 1
                if pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0:
                    bearing = math.degrees(pickingStationRB[0][1])  # in degrees
                    kp = 0.01  # proportional gain (tune as needed)
                    rotation_velocity = kp * bearing

                    # Clamp rotation speed
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                    if abs(bearing) < 1:  # within tolerance
                        pickingStation_index = 0
                        warehouseBotSim.SetTargetVelocities(0.0, 0.0)
                        print("Station 1 centred, moving forward")
                        state = 1
                    else:
                        warehouseBotSim.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Station 1... bearing: {bearing:.2f}°")
                else:
                    warehouseBotSim.SetTargetVelocities(0.0, 0.2)  # slow scan if not visible
                    print("Searching for Station 1...")

            elif state == 1:  # Move forward toward picking station 1
                warehouseBotSim.SetTargetVelocities(0.1, 0.0)
                print("MOVING FORWARD")
                
                # Check distance to picking station 1
                if pickingStationRB and pickingStationRB[0] is not None and len(pickingStationRB[0]) > 0:
                    distance = pickingStationRB[0][0]  # distance of station 1
                    print(f"Distance to station 1: {distance:.2f} m")
                    if distance < 0.45:  # stopping threshold
                        state = 2


            elif state == 2:  # Stop at station 1
                warehouseBotSim.SetTargetVelocities(0.0, 0.0)
                print("STOPPED at station 1")
                state = 3

            # elif state == 3:
            #     warehouseBotSim.SetTargetVelocities(0.0, -0.2)
            #     if rowMarkerRB:
            #         for i, rowMarker in enumerate(rowMarkerRB):
            #             if rowMarker is not None and len(rowMarker) > 0:
            #                 bearing = rowMarker[1]  # or [0], depending on your data
            #                 if abs(math.degrees(bearing)) < 4:
            #                     rowMarker_index = i
            #                     state = 4
            #                     break

            elif state == 3:
                if rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0:
                    bearing = math.degrees(rowMarkerRB[1][1])  # in degrees
                    kp = 0.01  # proportional gain (tune as needed)
                    rotation_velocity = kp * bearing

                    # Clamp rotation speed
                    rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                    if abs(bearing) < 1:  # within tolerance
                        warehouseBotSim.SetTargetVelocities(0.0, 0.0)
                        print("Station 1 centred, moving forward")
                        state = 4
                    else:
                        warehouseBotSim.SetTargetVelocities(0.0, rotation_velocity)
                        print(f"Aligning with Station 1... bearing: {bearing:.2f}°")
                else:
                    warehouseBotSim.SetTargetVelocities(0.0, -0.2)  # slow scan if not visible
                    print("Searching for Station 1...")



            elif state == 4: 
                if rowMarkerRB and rowMarkerRB[1] is not None and len(rowMarkerRB[1]) > 0:
                    rowMarkerDistance = rowMarkerRB[1][0]
                    forward_velocity = (rowMarkerDistance - 1.2)/7
                    state = 5
            
            elif state == 5:
                rowMarkerDistance = rowMarkerRB[1][0]
                if rowMarkerDistance < 1.2:   # distance stopping condition
                    state = 6
                    print("Robot has stopped at Return Zone")
                else:
                    warehouseBotSim.SetTargetVelocities(forward_velocity, 0.0)
                    print(f"Moving towards row marker... distance: {rowMarkerDistance:.2f}m")

            
            elif state == 6:
                warehouseBotSim.SetTargetVelocities(0.0, 0.0)
            


    #         elif state == 5:
    #             warehouseBotSim.SetTargetVelocities(0.0, 0.0)
    #             print("STOPPED at Corridor Zone")
    #             forward_start_time = time.time()

    #             state = 6
            
    #         elif state == 6:
    #             warehouseBotSim.SetTargetVelocities(0.0, 0.2)
    #             print("Turning left 90 degrees...")
    #             if forward_start_time is not None and (time.time() - forward_start_time) >= rightAngleTurn:                    
    #                 state = 7  # done turning left
    #                 warehouseBotSim.SetTargetVelocities(0.0, 0.0)  # stop 

    #         elif state == 7:
    #             warehouseBotSim.SetTargetVelocities(0.0, 0.0)
    #             print("STOPPED facing WALL")
    #             forward_start_time = time.time()
    #             # state = 8
            
    #         elif state == 8:
    # # Step 1: Turn left 90°
    #             target = pickingBays[0]
    #             warehouseBotSim.SetTargetVelocities(0.0, 0.2)  # rotate left
    #             print("Turning left 90 degrees...")
    #             if forward_start_time is not None and (time.time() - forward_start_time) >= rightAngleTurn:                    
    #                 state = 9  # done turning left
    #                 warehouseBotSim.SetTargetVelocities(0.0, 0.0)  # stop

    #         elif state == 9:
    #             # Step 2: Check if target is centred
    #             if pickingStationRB and pickingStationRB[target] is not None and len(pickingStationRB[target]) > 0:
    #                 bearing = math.degrees(pickingStationRB[target][1])  # in degrees
    #                 print(f"Target bearing: {bearing:.2f}°")
    #                 if abs(math.degrees(bearing)) < 4:  # centred
    #                     print("Target centred, moving forward...")
    #                     warehouseBotSim.SetTargetVelocities(0.1, 0.0)
    #                     state = 11  # final approach
    #                 else:
    #                     print("Target not centred, preparing to turn back...")
    #                     forward_start_time = time.time()
    #                     state = 10  # go turn back
    #             else:
    #                 print("Cannot see target picking station, turning back...")
    #                 forward_start_time = time.time()
    #                 state = 10

    #         elif state == 10:
    #             # Step 3: Turn right 90° back to original orientation
    #             warehouseBotSim.SetTargetVelocities(0.0, -0.2)  # rotate right
    #             print("Turning right 90 degrees (back to corridor)...")
    #             if forward_start_time is not None and (time.time() - forward_start_time) >= rightAngleTurn:                    
    #                 warehouseBotSim.SetTargetVelocities(0.0, 0.0)
    #                 print("Back to original heading, moving forward slightly...")
    #                 warehouseBotSim.SetTargetVelocities(0.1, 0.0)
    #                 state = 12  # move forward a bit

    #         elif state == 11:
    #             # Driving straight to the station (already centred)
    #             # Add stopping condition as needed
    #             pass

    #         elif state == 12:
    #             # Step 4: Short forward move after returning
    #             # Use a timer or distance check here
    #             # Example: move forward for 2 seconds then stop
    #             if forward_start_time is None:
    #                 forward_start_time = time.time()
    #             if (time.time() - forward_start_time) >= 2.0:  # 2 second move
    #                 warehouseBotSim.SetTargetVelocities(0.0, 0.0)
    #                 print("Forward move complete")
    #                 state = 13


    except KeyboardInterrupt:
        print("\nStopping simulation...")
        warehouseBotSim.StopSimulator()
        print("Simulation stopped successfully. Goodbye!")

    except Exception as e:
        print(f"\nAn error occurred: {e}")
        print("Stopping simulation...")
        try:
            warehouseBotSim.StopSimulator()
        except:
            pass
        print("Please check your CoppeliaSim setup and try again.")
