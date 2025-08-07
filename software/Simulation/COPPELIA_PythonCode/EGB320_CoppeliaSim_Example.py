#!/usr/bin/python


# import the packing bot module - this will include math, time, numpy (as np) and CoppeliaSim python modules
from warehousebot_lib import *
import time
#import any other required python modules


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()

# Starting contents of the bays [shelf,X,Y]. Set to -1 to leave empty.
sceneParameters.bayContents = np.random.random_integers(0,5,sceneParameters.bayContents.shape)
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl
sceneParameters.bayContents[1,1,2] = warehouseObjects.mug
sceneParameters.bayContents[2,3,1] = warehouseObjects.bottle
sceneParameters.bayContents[3,1,2] = warehouseObjects.soccer
sceneParameters.bayContents[4,2,0] = warehouseObjects.rubiks
sceneParameters.bayContents[5,0,1] = warehouseObjects.cereal


sceneParameters.obstacle0_StartingPosition = [-0.5,0]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
# sceneParameters.obstacle0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = -1   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = -1   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene


# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'	# specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1		# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the collector in metres
robotParameters.cameraHeightFromFloor = 0.15 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.0 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxItemDetectionDistance = 1 # the maximum distance away that you can detect the items in metres
robotParameters.maxPackingBayDetectionDistance = 2.5 # the maximum distance away that you can detect the packing bay in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres
robotParameters.maxRowMarkerDetectionDistance = 2.5 # the maximum distance away that you can detect the row markers in metres

# Collector Parameters
robotParameters.collectorQuality = 1 # specifies how good your item collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
robotParameters.maxCollectDistance = 0.15 #specificies the operating distance of the automatic collector function. Item needs to be less than this distance to the collector

robotParameters.sync = False # This parameter forces the simulation into syncronous mode when True; requiring you to call stepSim() to manually step the simulator in your loop


# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the CoppeliaSim so don't have to Stop it manually when pressing CTRL+C
	try:

		# Create CoppeliaSim PackerBot object - this will attempt to open a connection to CoppeliaSim. Make sure CoppeliaSim is running.
		warehouseBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
		# warehouseBotSim.SetScene()
		warehouseBotSim.StartSimulator()

		#We recommended changing this to a controlled rate loop (fixed frequency) to get more reliable control behaviour
		while True:
			# move the robot at a forward velocity of 0.05m/s with a rotational velocity of 0.1 rad/s.
			warehouseBotSim.SetTargetVelocities(0.00, 0.1)

			# Get Detected Objects
			objectsRB = warehouseBotSim.GetDetectedObjects(
				[
					warehouseObjects.items,
     				warehouseObjects.shelves,
					warehouseObjects.row_markers,
					warehouseObjects.obstacles,
					warehouseObjects.packingBay,
				]
			)
			itemsRB, packingBayRB, obstaclesRB, rowMarkerRangeBearing, shelfRangeBearing = objectsRB

			dist = warehouseBotSim.readProximity()

			#Check to see if an item is within the camera's FOV
			for itemClass in itemsRB:
				if itemClass != None:
					# loop through each item detected using Pythonian way
					for itemRB in itemClass:
						itemRange = itemRB[0]
						itemBearing = itemRB[1]

						warehouseBotSim.CollectItem(1)

			# warehouseBotSim.Dropitem()

			# Check to see if any obstacles are within the camera's FOV
			if obstaclesRB != None:
				# loop through each obstacle detected using Pythonian way
				for obstacle in obstaclesRB:
					obstacleRange = obstacle[0]
					obstacleBearing = obstacle[1]

			# Get Detected Wall Points
			wallPoints = warehouseBotSim.GetDetectedWallPoints()
			if wallPoints == None:
				print("To close to the wall")
			else:
				print("\nDetected Wall Points")
				# print the range and bearing to each wall point in the list
				for point in wallPoints:
					print("\tWall Point (range, bearing): %0.4f, %0.4f"%(point[0], point[1]))


			# Update object Positions
			warehouseBotSim.UpdateObjectPositions()

			if robotParameters.sync:
				warehouseBotSim.stepSim()

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in CoppeliaSim 
		warehouseBotSim.StopSimulator()



