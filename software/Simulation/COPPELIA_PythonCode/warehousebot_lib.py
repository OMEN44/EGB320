#import any required python modules
import coppelia
import time
import math
import numpy as np
import sys
from enum import IntEnum




# This class wraps COPPELIA api functions to allow 
# users to start testing Navigation/AI systems
class warehouseObjects(IntEnum):
	
	# TODO: figure out what the enums need to be.
	bowl = 0
	mug = 1
	bottle = 2
	soccer = 3
	rubiks = 4
	cereal = 5

	obstacle0 = 6
	obstacle1 = 7
	obstacle2 = 8
	
	packingBay = 9

	row_marker_1 = 10
	row_marker_2 = 11
	row_marker_3 = 12

	shelf_0 = 13
	shelf_1 = 14
	shelf_2 = 15
	shelf_3 = 16
	shelf_4 = 17
	shelf_5 = 18

	# Enums for turning off detection of certain objects
	items = 101
	obstacles = 102
	row_markers = 103
	shelves = 104
	#packingBay = 105 # There's only one so it is already defined.





################################
###### WAREHOUSE BOT CLASS #####
################################



class COPPELIA_WarehouseRobot(object):
	"""docstring for COPPELIA_WarehouseRobot"""
	
	####################################
	#### COPPELIA WAREHOUSE BOT INIT ###
	####################################

	def __init__(self, coppelia_server_ip, robotParameters, sceneParameters):
		# Robot Parameters
		self.robotParameters = robotParameters
		self.leftWheelBias = 0
		self.rightWheelBias = 0

		# Scene Paramaters
		self.sceneParameters = sceneParameters

		# COPPELIA Simulator Client ID
		self.clientID = None

		# COPPELIA Object Handle Variables
		self.robotHandle = None
		self.cameraHandle = None
		self.leftMotorHandle = None 		# left and right used for differential drive
		self.rightMotorHandle = None
		self.v60MotorHandle = None 			# 60, 180, 300 used for omni drive
		self.v180MotorHandle = None
		self.v300MotorHandle = None
		self.itemTemplateHandles = [None] * 6
		self.itemHandles = np.zeros((6,4,3),dtype=np.int16)
		self.obstacleHandles = [None, None, None]
		self.packingBayHandle = None
		self.rowMarkerHandles = [None,None,None]
		self.shelfHandles = [None]*6

		

		# Wheel Bias
		if self.robotParameters.driveSystemQuality != 1:
			# randomly generate a left and right wheel bias
			self.leftWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)
			self.rightWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)

		# Obstacle Parameters
		self.obstacleSize = 0.18 # diameter of obstacle

		# item Parameters
		self.itemSize = 0.05 # diameter of item

		# Variables to hold object positions
		self.robotPose = None
		self.cameraPose = None
		self.itemPositions = np.full((6,4,3,3),np.nan,dtype=np.float32)
		self.packingBayPosition = None
		self.obstaclePositions = [None, None, None]
		self.rowMarkerPositions = [None, None, None]

		# Variable to hold whether the item has been joined to the robot
		self.itemConnectedToRobot = False

		# Attempt to Open Connection to COPPELIA API Server
		self.OpenConnectionToCOPPELIA(coppelia_server_ip)

		# Attempt To Get COPPELIA Object Handles
		self.GetCOPPELIAObjectHandles()

		# Send Robot Parameters to COPPELIA
		self.UpdateCOPPELIARobot()

		



	########################################
	##### WAREHOUSE BOT API FUNCTIONS ######
	########################################
	# THESE ARE THE FUNCTIONS YOU SHOULD CALL.
	# ALL OTHER FUNCTIONS ARE HELPER FUNCTIONS.

	# Starts the COPPELIA Simulator. 
	# The COPPELIA Simulator can also be started manually by pressing the Play Button in COPPELIA.
	def StartSimulator(self):
		print('Attempting to Start the Simulator')
		
		if coppelia.simxStartSimulation(self.clientID, coppelia.simx_opmode_oneshot_wait) != 0:
			print('An error occurred while trying to start the simulator via the Python API. Terminating Program!')
			print('Comment out calls to StartSimulator() and start the simulator manully by pressing the Play button in COPPELIA.')
			sys.exit(-1)
		else:
			print('Successfully started the COPPELIA Simulator.')
		
		print('Setting scene')
		self.SetScene()
		
		# Setup streaming modes to each object
		coppelia.simxGetObjectPosition(self.clientID, self.robotHandle, -1, coppelia.simx_opmode_streaming)
		coppelia.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, coppelia.simx_opmode_streaming)
		coppelia.simxGetObjectPosition(self.clientID, self.cameraHandle, -1, coppelia.simx_opmode_streaming)
		coppelia.simxGetObjectPosition(self.clientID, self.packingBayHandle, -1, coppelia.simx_opmode_streaming)
		coppelia.simxReadProximitySensor(self.clientID,self.proximityHandle,coppelia.simx_opmode_streaming)

		
		res,resolution,image=coppelia.simxGetVisionSensorImage(self.clientID,self.cameraHandle,0,coppelia.simx_opmode_streaming)


		for handle in self.obstacleHandles:
			coppelia.simxGetObjectPosition(self.clientID, handle, -1, coppelia.simx_opmode_streaming)

		for handle in self.itemHandles.flatten():
			coppelia.simxGetObjectPosition(self.clientID, handle, -1, coppelia.simx_opmode_streaming)

		for handle in self.rowMarkerHandles:
			coppelia.simxGetObjectPosition(self.clientID, handle, -1, coppelia.simx_opmode_streaming)

		time.sleep(1)

		#initialise local copy of object positions
		self.GetObjectPositions()


	# Stops the COPPELIA Simulator. 
	# The COPPELIA Simulator can also be stopped manually by pressing the Stop Button in COPPELIA.
	def StopSimulator(self):
		print('Attempting to Stop the Simulator')
		if coppelia.simxStopSimulation(self.clientID, coppelia.simx_opmode_oneshot_wait) != 0:
			print('Could not stop the simulator. You can stop the simulator manually by pressing the Stop button in COPPELIA.')
		else:
			print('Successfully stoped the COPPELIA Simulator.')

		# Stop streaming modes to each object
		coppelia.simxGetObjectPosition(self.clientID, self.robotHandle, -1, coppelia.simx_opmode_discontinue)
		coppelia.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, coppelia.simx_opmode_discontinue)
		coppelia.simxGetObjectPosition(self.clientID, self.cameraHandle, -1, coppelia.simx_opmode_discontinue)
		coppelia.simxGetObjectPosition(self.clientID, self.packingBayHandle, -1, coppelia.simx_opmode_discontinue)
		coppelia.simxReadProximitySensor(self.clientID,self.proximityHandle,coppelia.simx_opmode_discontinue)

		for handle in self.obstacleHandles:
			coppelia.simxGetObjectPosition(self.clientID, handle, -1, coppelia.simx_opmode_discontinue)

		for handle in self.itemHandles.flatten():
			coppelia.simxGetObjectPosition(self.clientID, handle, -1, coppelia.simx_opmode_discontinue)

	# Gets the Range and Bearing to All Detected Objects.
	# returns:
	#	itemRangeBearing - range and bearing to the items with respect to the camera, will return None if the object is not detected
	#	packingBayRangeBearing - range and bearing to the packingBaywith respect to the camera, will return None if the object is not detected
	#	obstaclesRangeBearing - range and bearing to the obstacles with respect to the camera, will return None if the object is not detected
	def GetDetectedObjects(self,objects = None):
		# Variables used to return range and bearing to the objects
		itemRangeBearing = [None]*6
		packingBayRangeBearing = None
		obstaclesRangeBearing = None
		rowMarkerRangeBearing = [None,None,None]
		shelfRangeBearing = [None]*6

		# if objects variable is None, detect all objects.
		objects= objects or [warehouseObjects.items,warehouseObjects.shelves,warehouseObjects.row_markers,warehouseObjects.obstacles,warehouseObjects.packingBay]

		# Make sure the camera's pose is not none
		if self.cameraPose != None:

			#check which objects are currently in FOV using object detection sensor within COPPELIA sim
			retCode,objectsDetected,_,_,_ = coppelia.simxCallScriptFunction(self.clientID, 'Robot', coppelia.sim_scripttype_childscript, 'getObjectsInView',[],[],[],bytearray(),coppelia.simx_opmode_oneshot_wait)
			if objectsDetected != []:

				# check to see if blue shelves are in field of view

				if warehouseObjects.shelves in objects:
					shelfRB = self.GetShelfRangeBearing()
					for index,rb in enumerate(shelfRB):
						if objectsDetected[warehouseObjects.shelf_0 + index] == True:
							if rb[0] < self.robotParameters.maxShelfDetectionDistance:
								shelfRangeBearing[index] = rb

				# check to see if item is in field of view
				if warehouseObjects.items in objects:
					for shelf in range(6):
						if shelfRangeBearing[shelf] is None and warehouseObjects.shelves in objects:
							continue
						for x,y in [(x,y) for x in range(4) for y in range(3)]:
							item_type = self.sceneParameters.bayContents[shelf,x,y]
							if item_type == -1: 
								continue
							
							itemPosition = self.itemPositions[shelf,x,y]
							#if detected calculate range and bearing from camera pose location
							if objectsDetected[warehouseObjects.bowl + item_type] == True\
							and shelfRangeBearing[shelf] is not None\
							and self.PointInsideArena(itemPosition):
								_valid, _range, _bearing = self.GetRBInCameraFOV(itemPosition)
														
								# check range is not too far away
								if _range < self.robotParameters.maxItemDetectionDistance \
									and abs(_bearing) < self.robotParameters.cameraPerspectiveAngle/2:
									# make itemRangeBearing into empty lists, if currently set to None
									if itemRangeBearing[item_type] == None:
										itemRangeBearing[item_type] = []
									itemRangeBearing[item_type].append([_range, _bearing])
							
					
				# check to see which obstacles are within the field of view
				if warehouseObjects.obstacles in objects:
					for index, obstaclePosition in enumerate(self.obstaclePositions):
						if obstaclePosition != None:

							# check to see if the current obstacle is in the FOV and within the field. If so add to detected obstacle range bearing list
							if objectsDetected[warehouseObjects.obstacle0 + index] == True and self.PointInsideArena(obstaclePosition):
								_valid, _range, _bearing = self.GetRBInCameraFOV(obstaclePosition)
								if _valid:

									# make obstaclesRangeBearing into empty lists, if currently set to None
									if obstaclesRangeBearing == None:
										obstaclesRangeBearing = []

									if _range <  self.robotParameters.maxObstacleDetectionDistance:
										obstaclesRangeBearing.append([_range, _bearing])


				# check to see if yellow packingBay is in field of view
				if warehouseObjects.packingBay in objects:
					if self.packingBayPosition != None:
						
						#if detected calculate range and bearing from camera pose location
						if objectsDetected[warehouseObjects.packingBay] == True:
							_valid, _range, _bearing = self.GetRBInCameraFOV(self.packingBayPosition)
							if _valid:
								# check range is not to far away
								if _range < self.robotParameters.maxPackingBayDetectionDistance:
									packingBayRangeBearing = [_range, _bearing]


				# check to see if black row markers are in field of view
				if warehouseObjects.row_markers in objects:
					for index, rowMarkerPosition in enumerate(self.rowMarkerPositions):
						if rowMarkerPosition != None:
							
							#if detected calculate range and bearing from camera pose location
							if objectsDetected[warehouseObjects.row_marker_1 + index] == True:
								_valid, _range, _bearing = self.GetRBInCameraFOV(rowMarkerPosition)
								if _valid:
									# check range is not to far away
									if _range < self.robotParameters.maxRowMarkerDetectionDistance:
										rowMarkerRangeBearing[index] = [_range, _bearing]


		return itemRangeBearing, packingBayRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing


	def GetCameraImage(self):

		if self.cameraHandle == None:
			None, None
	
		res,resolution,image=coppelia.simxGetVisionSensorImage(self.clientID,self.cameraHandle,0,coppelia.simx_opmode_buffer)
		
		if res==coppelia.simx_return_ok:
			return resolution, image    
		else:
			return None, None
			# res=coppelia.simxSetVisionSensorImage(clientID,v1,image,0,coppelia.simx_opmode_oneshot)
	
	# Gets the Range and Bearing to the wall(s)
	# returns:
	#	None - if there are no valid wall points (i.e. the robot is right up against a wall and facing it)
	#	A list of [range, bearing] arrays. There will either be 1, 2, or 3 [range, bearing] arrays depending on the situation
	#		will return 1 range-bearing array if the robot is close to a wall but not directly facing it and one edge of the camera's view limit is up against the wall, while the other can see part of the field
	#		will return 2 range-bearing array if the robot can see the wall but is not facing a corner
	#		will return 3 range-bearing array if the robot can see the wall and is facing into a corner
	def GetDetectedWallPoints(self):
		wallPoints = None

		if self.cameraPose == None:
			return None
		
		cameraPose2D = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]

		# Get range and bearing to the valid points at limit of camera's view (no occlusions)
		wallPoints = self.CameraViewLimitsRangeAndBearing(cameraPose2D)
		if wallPoints == None:
			# return None to indicate to close to wall or because some maths error and didn't get 1 or 2 valid intersection points 
			# (hopefully a maths error doesn't occur and believe all cases have been taken care of)
			return None

		# See if a corner is within the field of view (no occlusions)
		cornerRangeBearing = self.FieldCornerRangeBearing(cameraPose2D)
		if cornerRangeBearing == []:
			return wallPoints

		wallPoints.append(cornerRangeBearing)
		return wallPoints
		

	# Set Target Velocities
	# inputs:
	#	x - the velocity of the robot in the forward direction (in m/s)
	#	theta_dt - the rotational velocity of the robot (in rad/s)
	def SetTargetVelocities(self, x_dot, theta_dot):
		
		# Need to set based on drive system type
		if self.robotParameters.driveType == 'differential':
			# ensure wheel base and wheel radius are set as these are not allowed to be changed
			self.robotParameters.wheelBase = 0.15
			self.robotParameters.wheelRadius = 0.04
		
			# determine minimum wheel speed based on minimumLinear and maximumLinear speed
			minWheelSpeed = self.robotParameters.minimumLinearSpeed / self.robotParameters.wheelRadius
			maxWheelSpeed = self.robotParameters.maximumLinearSpeed / self.robotParameters.wheelRadius

			# calculate left and right wheel speeds in rad/s
			leftWheelSpeed = (x_dot - 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.leftWheelBias
			rightWheelSpeed = (x_dot + 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.rightWheelBias

			# add gaussian noise to the wheel speed
			if self.robotParameters.driveSystemQuality != 1:
				leftWheelSpeed = np.random.normal(leftWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)
				rightWheelSpeed = np.random.normal(rightWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)

			# ensure wheel speeds are not greater than maximum wheel speed
			leftWheelSpeed = min(leftWheelSpeed, maxWheelSpeed)
			rightWheelSpeed = min(rightWheelSpeed, maxWheelSpeed)

			# set wheel speeds to 0 if less than the minimum wheel speed
			if abs(leftWheelSpeed) < minWheelSpeed:
				leftWheelSpeed = 0
			if abs(rightWheelSpeed) < minWheelSpeed:
				rightWheelSpeed = 0

			# set motor speeds
			# errorCode = coppelia.simxSetJointTargetVelocity(self.clientID, self.leftMotorHandle, leftWheelSpeed, coppelia.simx_opmode_oneshot)
			# errorCode = coppelia.simxSetJointTargetVelocity(self.clientID, self.rightMotorHandle, rightWheelSpeed, coppelia.simx_opmode_oneshot) 
			# errorCode2 = coppelia.simxSetJointTargetVelocity(self.clientID, self.leftRearMotorHandle, leftWheelSpeed, coppelia.simx_opmode_oneshot)
			# errorCode2 = coppelia.simxSetJointTargetVelocity(self.clientID, self.rightRearMotorHandle, rightWheelSpeed, coppelia.simx_opmode_oneshot) 
			errorCode = coppelia.simxSetJointTargetVelocity(self.clientID, self.leftMotorHandle, leftWheelSpeed, coppelia.simx_opmode_blocking)
			errorCode = coppelia.simxSetJointTargetVelocity(self.clientID, self.rightMotorHandle, rightWheelSpeed, coppelia.simx_opmode_blocking) 
			errorCode2 = coppelia.simxSetJointTargetVelocity(self.clientID, self.leftRearMotorHandle, leftWheelSpeed, coppelia.simx_opmode_blocking)
			errorCode2 = coppelia.simxSetJointTargetVelocity(self.clientID, self.rightRearMotorHandle, rightWheelSpeed, coppelia.simx_opmode_blocking)
			if errorCode != 0:
				print('Failed to set left and/or right motor speed. Error code %d'%errorCode)

		

	# Returns true if the item is within the collector
	# returns:
	#	true - if item is in the collector
	def itemCollected(self):
		return self.itemConnectedToRobot

	
	def Dropitem(self):
		if self.itemConnectedToRobot:
				coppelia.simxCallScriptFunction(self.clientID, 'Robot', coppelia.sim_scripttype_childscript, 'RobotReleaseItem',[],[],[],bytearray(),coppelia.simx_opmode_blocking)
				self.itemConnectedToRobot = False

	# Use this to force a physical connection between item and rover
	# Ideally use this if no collector has been added to your robot model
	# if two items are within the distance it will attempt to collect both
	# Outputs:
	#		Success - returns True if a item was collected or false if not
	def CollectItem(self,shelf_height):

		# for _, itemPosition in enumerate(self.itemPositions):
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			handle = self.itemHandles[shelf,x,y]
			itemPosition = self.itemPositions[shelf,x,y]

			itemDist = self.CollectorToItemDistance(itemPosition)


			if itemDist != None and itemDist < self.robotParameters.maxCollectDistance and self.itemConnectedToRobot == False and self.GetItemBayHeight(itemPosition) == shelf_height:
				# make physical connection between item and robot to simulate collector
				coppelia.simxCallScriptFunction(self.clientID, 'Robot', coppelia.sim_scripttype_childscript, 'JoinRobotAndItem',[handle],[],[],bytearray(),coppelia.simx_opmode_blocking)
				self.itemConnectedToRobot = True
				
		
		return self.itemConnectedToRobot

	def GetItemBayHeight(self,itemPosition):
		if itemPosition[2] < 0.1:
			return 0
		elif itemPosition[2] < 0.2:
			return 1
		else:
			return 2

	


	# Update Object Positions - call this in every loop of your navigation code (or at the frequency your vision system runs at). 
	# This is required to get correct range and bearings to objects.
	# This function also emulates the collector. The function returns the global pose/position of the robot and the objects too. 
	# However, you should not use these return values in your nagivation code, they are there to help you debug if you wish.
	# returns: 
	#		robotPose - a 6 element array representing the robot's pose (x,y,z,roll,pitch,yaw), or None if was not successfully updated from COPPELIA
	#		itemPosition - a 3 element array representing the item's position (x,y,z), or None if was not successfully updated from COPPELIA
	#		obstaclePositions - a 3 element list, with each index in the list containing a 3 element array representing the item's position (x,y,z), or None if was not successfully updated from COPPELIA
	def UpdateObjectPositions(self):
		# attempt to get object positions from COPPELIA
		self.GetObjectPositions()

		# update item
		self.UpdateItem()

		# return object positions		
		return self.robotPose, self.itemPositions, self.obstaclePositions


	def readProximity(self):
		error_code,objectDetected,detected_point,objectHandle,surfaceNormal= coppelia.simxReadProximitySensor(self.clientID,self.proximityHandle,coppelia.simx_opmode_buffer)
		if error_code != 0:
			print(f"Failed to read proximity sensor. Error code {error_code}")
		return np.linalg.norm(detected_point)

	#########################################
	####### COPPELIA API SERVER FUNCTIONS #######
	#########################################
	# These functions are called within the init function

	# Open connection to COPPELIA API Server
	def OpenConnectionToCOPPELIA(self, coppelia_server_ip):
		# Close any open connections to coppelia in case any are still running in the background
		print('Closing any existing COPPELIA connections.')
		coppelia.simxFinish(-1)

		# Attempt to connect to coppelia API server
		print('Attempting connection to COPPELIA API Server.')
		self.clientID = coppelia.simxStart(coppelia_server_ip, 19997, True, True, 5000, 5)
		if self.clientID != -1:
			print('Connected to COPPELIA API Server.')
		else:
			print('Failed to connect to COPPELIA API Server. Terminating Program')
			sys.exit(-1)

		if self.robotParameters.sync:
			coppelia.simxSynchronous(self.clientID, True)

	def stepSim(self):
		coppelia.simxSynchronousTrigger(self.clientID)

	# Get COPPELIA Object Handles
	def GetCOPPELIAObjectHandles(self):
		# attempt to get coppelia object handles
		errorCode = self.GetRobotHandle()
		if errorCode != 0:
			print('Failed to get Robot object handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode = self.GetCameraHandle()
		if errorCode != 0:
			print('Failed to get Vision Sensor object handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3,errorCode4 = self.GetMotorHandles()
		if errorCode1 != 0 or errorCode2 != 0:
			print('Failed to get Motor object handles. Terminating Program. Error Codes %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)
		elif errorCode3 != 0 or errorCode4 != 0:
			print("Failed to get rear wheel motor handles. Disregard if using the \"Differential\" scene.")

		packingBayErrorCode = self.GetPackingBayHandle()
		if packingBayErrorCode != 0:
			print('Failed to get packingBay object handles. Terminating Program. Error Codes %d'%(packingBayErrorCode))
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3 = self.GetObstacleHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get Obstacle object handles. Terminating Program. Error Codes %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)
		

		errorCode1, errorCode2, errorCode3 = self.GetRowMarkerHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get Row Marker object handles. Terminating Program. Error Codes %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)

		errorCodes = self.GetItemTemplateHandles()
		if any([code != 0 for code in errorCodes]):
			print(f'Failed to get Item object handles. Terminating Program. Error Codes {errorCodes}')
			sys.exit(-1)

		errorCodes = self.getShelfHandles()
		if any([code != 0 for code in errorCodes]):
			print(f'Failed to get Shelf object handles. Terminating Program. Error Codes {errorCodes}')
			sys.exit(-1) 
	
		errorCode = self.getProximityhandle()
		if errorCode != 0:
			print(f'Failed to get Proximity sensor handle. Terminating Program. Error Codes {errorCodes}')
			sys.exit(-1) 
	
	############################################
	####### COPPELIA OBJECT HANDLE FUNCTIONS #######
	############################################
	# These functions are called by the GetCOPPELIAObjectHandles function

	# Get COPPELIA Robot Handle
	def GetRobotHandle(self):
		errorCode, self.robotHandle = coppelia.simxGetObjectHandle(self.clientID, 'Robot', coppelia.simx_opmode_oneshot_wait)
		return errorCode


	# Get COPPELIA Camera Handle
	def GetCameraHandle(self):
		errorCode, self.cameraHandle = coppelia.simxGetObjectHandle(self.clientID, 'VisionSensor', coppelia.simx_opmode_oneshot_wait)
		return errorCode

			
	# Get COPPELIA Motor Handles
	def GetMotorHandles(self):
		errorCode1 = 0
		errorCode2 = 0
		errorCode3 = 0

		if self.robotParameters.driveType == 'differential':
			errorCode1, self.leftMotorHandle = coppelia.simxGetObjectHandle(self.clientID, 'LeftMotor', coppelia.simx_opmode_oneshot_wait)
			errorCode2, self.rightMotorHandle = coppelia.simxGetObjectHandle(self.clientID, 'RightMotor', coppelia.simx_opmode_oneshot_wait)
			errorCode3, self.leftRearMotorHandle = coppelia.simxGetObjectHandle(self.clientID, 'LeftRearMotor', coppelia.simx_opmode_oneshot_wait)
			errorCode4, self.rightRearMotorHandle = coppelia.simxGetObjectHandle(self.clientID, 'RightRearMotor', coppelia.simx_opmode_oneshot_wait)
		
		return errorCode1, errorCode2, errorCode3,errorCode4

	# Get COPPELIA PackingBay Handles
	def GetPackingBayHandle(self):
		packingBayErrorCode, self.packingBayHandle = coppelia.simxGetObjectHandle(self.clientID, 'Packing_Bay', coppelia.simx_opmode_oneshot_wait)	
		return packingBayErrorCode

	# Get COPPELIA item Template Handles
	def GetItemTemplateHandles(self):
		error_codes = []
		for index,name in enumerate(["BOWL","MUG","BOTTLE","SOCCER_BALL","RUBIKS_CUBE","CEREAL_BOX",]):
			code,handle = coppelia.simxGetObjectHandle(self.clientID, name, coppelia.simx_opmode_oneshot_wait)
			error_codes.append(code)
			self.itemTemplateHandles[index] = handle
			
		return error_codes

	# Get COPPELIA Obstacle Handles
	def GetObstacleHandles(self):
		obs0ErrorCode, self.obstacleHandles[0] = coppelia.simxGetObjectHandle(self.clientID, 'Obstacle_0', coppelia.simx_opmode_oneshot_wait)
		obs1ErrorCode, self.obstacleHandles[1] = coppelia.simxGetObjectHandle(self.clientID, 'Obstacle_1', coppelia.simx_opmode_oneshot_wait)
		obs2ErrorCode, self.obstacleHandles[2] = coppelia.simxGetObjectHandle(self.clientID, 'Obstacle_2', coppelia.simx_opmode_oneshot_wait)
		return obs0ErrorCode, obs1ErrorCode, obs2ErrorCode
	
	# Get COPPELIA Row marker handles
	def GetRowMarkerHandles(self):
		rowMarker1ErrorCode, self.rowMarkerHandles[0] = coppelia.simxGetObjectHandle(self.clientID, 'row_marker1', coppelia.simx_opmode_oneshot_wait)
		rowMarker2ErrorCode, self.rowMarkerHandles[1] = coppelia.simxGetObjectHandle(self.clientID, 'row_marker2', coppelia.simx_opmode_oneshot_wait)
		rowMarker3ErrorCode, self.rowMarkerHandles[2] = coppelia.simxGetObjectHandle(self.clientID, 'row_marker3', coppelia.simx_opmode_oneshot_wait)
		return rowMarker1ErrorCode, rowMarker2ErrorCode, rowMarker3ErrorCode


	# Get COPPELIA shelf handles
	def getShelfHandles(self):
		errorCodes = [None]*6
		for i in range(6):
			errorCodes[i],self.shelfHandles[i] = coppelia.simxGetObjectHandle(self.clientID, f'Shelf{i}', coppelia.simx_opmode_oneshot_wait)
		return tuple(errorCodes)

	# Get COPPELIA proximity sensor handle.
	def getProximityhandle(self):
		error_code,self.proximityHandle = coppelia.simxGetObjectHandle(self.clientID, 'Proximity_sensor', coppelia.simx_opmode_oneshot_wait)
		return error_code


	def GetShelfRangeBearing(self):
		ec,rets,data,_,_ = coppelia.simxCallScriptFunction(
			self.clientID,
			'Robot',
			coppelia.sim_scripttype_childscript,
			'getDistanceToObject',
			self.shelfHandles,[self.robotParameters.maxShelfDetectionDistance]*len(self.shelfHandles),[],bytearray(),
			coppelia.simx_opmode_oneshot_wait
		)
		
		rb = [None]*6
		if ec == 0:
			data = np.array(data).reshape(len(self.shelfHandles),-1)
			for i,vec in enumerate(data):
				if rets[i] != 0:
					continue
				pA = vec[:3]
				pB = vec[3:6]
				_,range,bearing = self.GetRBInCameraFOV(pB)
				# range = vec[-1]
				# bearing = np.arctan2(pB[1]-pA[1],pB[0]-pA[0])
				rb[i] = (range,bearing)
		return rb
	###############################################
	####### ROBOT AND SCENE SETUP FUNCTIONS #######
	###############################################
	# These functions are called within the init function

	# Updates the robot within COPPELIA based on the robot paramters
	def UpdateCOPPELIARobot(self):
		# Set Camera Pose and Orientation
		self.SetCameraPose(self.robotParameters.cameraDistanceFromRobotCenter, self.robotParameters.cameraHeightFromFloor, self.robotParameters.cameraTilt)
		self.SetCameraOrientation(self.robotParameters.cameraOrientation)

	# Sets the position of the item, robot and obstacles based on parameters
	def SetScene(self):
		
		# Start streaming the bay positions. Avoids slow oneshot_wait commands.
		bayHandles = np.zeros_like(self.sceneParameters.bayContents)
		for shelf in range(6):
			for x in range(4):
				for y in range(3):
					errorCode, bayHandles[shelf,x,y] = coppelia.simxGetObjectHandle(self.clientID,f"/Shelf{shelf}/Bay{x}{y}",coppelia.simx_opmode_oneshot_wait)
					coppelia.simxGetObjectPosition(self.clientID,bayHandles[shelf,x,y],-1,coppelia.simx_opmode_streaming)
		
		if np.sum(self.sceneParameters.bayContents >= 0) == 0:
			print("\033[93mWarning: Bay contents has not been initialised to contain any "+\
					"values. Please use sceneParameters.bayContents[SHELF,X,Y] = <0..5> "+\
					"to set bay contents.\nAny items dragged into the environment "+\
					"will not be interactable with by the robot.\033[0m"
			)
			time.sleep(2)
		else:
			for shelf in range(6):
				for x in range(4):
					for y in range(3):
						itemType = self.sceneParameters.bayContents[shelf,x,y]
						if itemType == -1:
							continue
						itemHandle = self.itemTemplateHandles[itemType]
						errorCode2,position = coppelia.simxGetObjectPosition(self.clientID,bayHandles[shelf,x,y],-1,coppelia.simx_opmode_buffer)
						errorCode, clonedHandles = coppelia.simxCopyPasteObjects(
							self.clientID,
							[itemHandle],
							coppelia.simx_opmode_oneshot_wait
						)
						coppelia.simxSetObjectPosition(self.clientID,clonedHandles[0],-1,position,coppelia.simx_opmode_oneshot)
						self.itemHandles[shelf,x,y] = clonedHandles[0]
						coppelia.simxGetObjectHandle(self.clientID,f"/Shelf{shelf}/Bay{x}{y}",coppelia.simx_opmode_discontinue)
						coppelia.simxGetObjectPosition(self.clientID,bayHandles[shelf,x,y],-1,coppelia.simx_opmode_discontinue)

		obstacleHeight = 0.15
		for index, obstaclePosition in enumerate([self.sceneParameters.obstacle0_StartingPosition, self.sceneParameters.obstacle1_StartingPosition, self.sceneParameters.obstacle2_StartingPosition]):
			if obstaclePosition != -1:
				if obstaclePosition != None:
					coppeliaStartingPosition = [obstaclePosition[0], obstaclePosition[1], obstacleHeight/2]
					coppelia.simxSetObjectPosition(self.clientID, self.obstacleHandles[index], -1, coppeliaStartingPosition, coppelia.simx_opmode_oneshot_wait)
				else:
					coppelia.simxSetObjectPosition(self.clientID, self.obstacleHandles[index], -1, [2,  -0.3 + (-0.175*index), 0.8125], coppelia.simx_opmode_oneshot_wait)
		
		

	### CAMERA FUNCTIONS ###

	# Sets the camera's pose
	# Inputs:
	#		x - distance between the camera and the center of the robot in the direction of the front of the robot
	#		z - height of the camera relative to the floor in metres
	#		pitch - tilt of the camera in radians
	def SetCameraPose(self, x, z, pitch):
		# assume the students want the camera in the center of the robot (so no y)
		# assume the student only wants to rotate the camera to point towards the ground or sky (so no roll or yaw)

		# update robot parameters
		self.robotParameters.cameraDistanceFromRobotCenter = x
		self.robotParameters.cameraHeightFromFloor = z
		self.robotParameters.cameraTilt = pitch

		# Need to change Z as in COPPELIA the robot frame is in the center of the Cylinder
		# z in COPPELIA robot frame = z - (cylinder height)/2 - wheel diameter
		z = z - 0.075 - 2*self.robotParameters.wheelRadius

		# Need to change the pitch by adding pi/2 (90 degrees) as pitch of 0 points up
		pitch = pitch + math.pi/2.0

		# set camera pose
		coppelia.simxSetObjectPosition(self.clientID, self.cameraHandle, coppelia.sim_handle_parent, [x,0,z], coppelia.simx_opmode_oneshot_wait)
		coppelia.simxSetObjectOrientation(self.clientID, self.cameraHandle, coppelia.sim_handle_parent, [0,pitch,math.pi/2.0], coppelia.simx_opmode_oneshot_wait)

	
	# Sets the camera's height relative to the floor in metres
	def SetCameraHeight(self, z):
		self.SetCameraPose(0, z, 0)


	# Sets the distance between the camera and the center of the robot in the direction of front of the robot
	def SetCameraOffsetFromRobotCentre(self, x):
		self.SetCameraPose(x, 0, 0)


	# Sets the tilt of the camera in radians
	def SetCameraTilt(self, pitch):
		self.SetCameraPose(0, 0, pitch)


	# Set Camera Orientation to either portrait or landscape
	def SetCameraOrientation(self, orientation):
		# get resolution based on orientation
		if orientation == 'portrait':
			x_res = 480
			y_res = 640
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle * x_res / y_res
		elif orientation == 'landscape':
			x_res = 640
			y_res = 480
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle * y_res / x_res
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle
		else:
			print('The camera orientation %s is not known. You must specify either portrait or landscape')
			return


		# update robot parameters
		self.robotParameters.cameraOrientation = orientation

		# set resolution of camera (vision sensor object) - resolution parameters are int32 parameters
		coppelia.simxSetObjectIntParameter(self.clientID, self.cameraHandle, coppelia.sim_visionintparam_resolution_x, x_res, coppelia.simx_opmode_oneshot_wait)
		coppelia.simxSetObjectIntParameter(self.clientID, self.cameraHandle, coppelia.sim_visionintparam_resolution_y, y_res, coppelia.simx_opmode_oneshot_wait)
		

	####################################
	####### API HELPER FUNCTIONS #######
	####################################	

	# Prints the pose/position of the objects in the scene
	def PrintObjectPositions(self):
		print("\n\n***** OBJECT POSITIONS *****")
		if self.robotPose != None:
			print("Robot 2D Pose (x,y,theta): %0.4f, %0.4f, %0.4f"%(self.robotPose[0], self.robotPose[1], self.robotPose[2]))

		if self.cameraPose != None:
			print("Camera 3D Pose (x,y,z,roll,pitch,yaw): %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f"%(self.cameraPose[0], self.cameraPose[1], self.cameraPose[2], self.cameraPose[3], self.cameraPose[4], self.cameraPose[5]))
		
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			itemPosition = self.itemPositions[shelf,x,y]
			if np.all(np.isnan(itemPosition)) == False:
				print("item from bay [%d,%d,%d] Position (x,y,z): %0.4f, %0.4f, %0.4f"%(shelf,x,y, itemPosition[0], itemPosition[1], itemPosition[2]))
			
		if self.packingBayPosition != None:
			print("PackingBay Position (x,y,z): %0.4f, %0.4f, %0.4f"%(self.packingBayPosition[0], self.packingBayPosition[1], self.packingBayPosition[2]))
			
		for index, obstacle in enumerate(self.obstaclePositions):
			if obstacle != None:
				print("Obstacle %d Position (x,y,z): %0.4f, %0.4f, %0.4f"%(index, obstacle[0], obstacle[1], obstacle[2]))

	# Gets the pose/position in the global coordinate frame of all the objects in the scene.
	# Stores them in class variables. Variables will be set to none if could not be updated
	def GetObjectPositions(self):
		# Set camera pose and object position to None so can check in an error occurred
		self.robotPose = None
		self.cameraPose = None
		# self.itemPositions = [None]*len(self.itemHandles)
		self.packingBayPosition = None
		self.obstaclePositions = [None, None, None]

		# GET 2D ROBOT POSE
		errorCode, robotPosition = coppelia.simxGetObjectPosition(self.clientID, self.robotHandle, -1, coppelia.simx_opmode_buffer)
		errorCode, robotOrientation = coppelia.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, coppelia.simx_opmode_buffer)
		if errorCode == 0:
			self.robotPose = [robotPosition[0], robotPosition[1], robotPosition[1], robotOrientation[0], robotOrientation[1], robotOrientation[2]]

		# GET 3D CAMERA POSE
		errorCode, cameraPosition = coppelia.simxGetObjectPosition(self.clientID, self.cameraHandle, -1, coppelia.simx_opmode_buffer)
		if errorCode == 0:
			self.cameraPose = [cameraPosition[0], cameraPosition[1], cameraPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]]
		

		# GET POSITION OF EACH OBJECT
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			handle = self.itemHandles[shelf,x,y]
			errorCode, itemPosition = coppelia.simxGetObjectPosition(self.clientID, handle, -1, coppelia.simx_opmode_buffer)
			if errorCode == 0:
				self.itemPositions[shelf,x,y] = itemPosition

		# packingBay position
		errorCode, packingBayPosition = coppelia.simxGetObjectPosition(self.clientID, self.packingBayHandle, -1, coppelia.simx_opmode_buffer)
		if errorCode == 0:
			self.packingBayPosition = packingBayPosition


		# obstacle positions
		obstaclePositions = [None, None, None]
		for index, obs in enumerate(self.obstaclePositions):
			errorCode, obstaclePositions[index] = coppelia.simxGetObjectPosition(self.clientID, self.obstacleHandles[index], -1, coppelia.simx_opmode_buffer)
			if errorCode == 0:
				self.obstaclePositions[index] = obstaclePositions[index]

		# row marker positions
		rowMarkerPositions = [None,None,None]
		for index, rowMarker in enumerate(self.rowMarkerPositions):
			errorCode,rowMarkerPositions[index] = coppelia.simxGetObjectPosition(self.clientID,self.rowMarkerHandles[index],-1,coppelia.simx_opmode_buffer)
			if errorCode == 0:
				self.rowMarkerPositions[index] = rowMarkerPositions[index]

	# Checks to see if an Object is within the field of view of the camera
	def GetRBInCameraFOV(self, objectPosition):
		# calculate range and bearing on 2D plane - relative to the camera
		cameraPose2d = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose2d, objectPosition)

		# vertical_test_cam_pose = [0,self.cameraPose[2],0]
		# vertical_test_pos = [_range,objectPosition[2]]
		# _vert_range, _vert_bearing = self.GetRangeAndBearingFromPoseAndPoint(vertical_test_cam_pose, vertical_test_pos)
		_valid = abs(_bearing) < self.robotParameters.cameraPerspectiveAngle/2 \
		# 	and abs(_vert_bearing) < self.robotParameters.cameraPerspectiveAngle/4

		# angle from camera's axis to the object's position
		# verticalAngle = math.atan2(objectPosition[2]-self.cameraPose[2], _range)

		#OLD code needs removing

		# # check to see if in field of view
		# if abs(_bearing) > (self.horizontalViewAngle/2.0):
		# 	# return False to indicate object outside camera's FOV and range and bearing
		# 	return False, _range, _bearing

		# if abs(verticalAngle) > (self.verticalViewAngle/2.0):
		# 	# return False to indicate object outside camera's FOV and range and bearing
		# 	return False, _range, _bearing

		# return True to indicate is in FOV and range and bearing
		return _valid, _range, _bearing

	def ObjectInCameraFOV(self,objectPosition):
		_,_bearing = self.GetRBInCameraFOV(objectPosition)
		return np.abs(_bearing) <= self.robotParameters.cameraPerspectiveAngle / 2
			
	
	# Determines if a 2D point is inside the arena, returns true if that is the case
	def PointInsideArena(self, position):
		if position[0] > -1 and position[0] < 1 and position[1] > -1 and position[1] < 1:
			return True

		return False


	# Update the item
	def UpdateItem(self):
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			itemPosition = self.itemPositions[shelf,x,y]
		
			if np.all(np.isnan(itemPosition)) == False:

				itemDist = self.CollectorToItemDistance(itemPosition)

				# DEPRACATED 
				# # See if need to connect/disconnect item from robot
				# if self.robotParameters.autoCollectItem == True and itemDist != None and itemDist < self.robotParameters.maxCollectDistance and self.itemConnectedToRobot == False:
				# 	# make physical connection between item and robot to simulate collector
				# 	coppelia.simxCallScriptFunction(self.clientID, 'Robot', coppelia.sim_scripttype_childscript, 'JoinRobotAndItem',[1],[self.robotParameters.maxCollectDistance],[],bytearray(),coppelia.simx_opmode_blocking)
				# 	self.itemConnectedToRobot = True

				if self.itemConnectedToRobot == True:
					# random chance to disconnect
					if np.random.rand() > self.robotParameters.collectorQuality:
						# terminate connection between item and robot to simulate collector
						coppelia.simxCallScriptFunction(self.clientID, 'Robot', coppelia.sim_scripttype_childscript, 'RobotReleaseItem',[],[],[],bytearray(),coppelia.simx_opmode_blocking)
						self.itemConnectedToRobot = False

				elif itemDist != None and itemDist > 0.03:
					self.itemConnectedToRobot = False

	
	# Gets the range and bearing to a corner that is within the camera's field of view.
	# Will only return a single corner, as only one corner can be in the field of view with the current setup.
	# returns:
	#	a list containing a [range, bearing] or an empty list if no corner is within the field of view
	def FieldCornerRangeBearing(self, cameraPose):
		rangeAndBearing = []

		# Get range and bearing from camera's pose to each corner
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [1, 1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [-1, 1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [-1, -1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [1, -1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		return rangeAndBearing


	# Gets the range and bearing to where the edge of camera's field of view intersects with the arena walls.
	# returns:
	#	None - if there are no valid wall points (i.e. the robot is right up against a wall and facing it)
	#	A list of [range, bearing] arrays. There will either be 1 or 2 [range, bearing] arrays depending on the situation
	#		will return 1 if the robot is close to a wall but not directly facing it and one edge of the camera's view limit is up against the wall, while the other can see part of the field
	#		will return 2 if the robot can see the wall but is not facing a corner
	def CameraViewLimitsRangeAndBearing(self, cameraPose):
		viewLimitIntersectionPoints = []
		rangeAndBearings = []

		# Get valid camera view limit points along the east wall
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'east')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the north wall (wall in positive y-direction)
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'north')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the west wall
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'west')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the south wall (wall in negative y-direction)
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'south')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Calculate range and bearing to the valid view limit wall intersection points and store in a list
		for point in viewLimitIntersectionPoints:
			_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, point)
			rangeAndBearings.append([_range, _bearing])

		# return None if rangeAndBearings list is empty
		if rangeAndBearings == []:
			return None
		else:
			return rangeAndBearings

	
	# Gets the points where the edges of the camera's field of view intersects with the specified wall.
	# inputs:
	#	cameraPose - pose of the camera [x, y, theta] in the global coordinate frame
	# 	wall - wall want to get the camera view limit points of ('east', 'west', 'north', 'south').
	# returns:
	#	p1 - will be [x,y] point if it is a valid wall point (i.e. lies on the arena's walls and is within the field of view) or None if it is not valid
	#	p2 - will be [x,y] point if it is a valid wall point (i.e. lies on the arena's walls and is within the field of view) or None if it is not valid
	def CameraViewLimitWallIntersectionPoints(self, cameraPose, wall):
		
		# calculate range to wall along camera's axis using the point where the camera's axis intersects with the specified wall
		x, y = self.CameraViewAxisWallIntersectionPoint(cameraPose, wall)
		centreRange = math.sqrt(math.pow(cameraPose[0]-x, 2) + math.pow(cameraPose[1]-y, 2))


		# determine camera view limit intersection points on wall
		if wall == 'east' or wall == 'west':
			d1 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi/2.0 - self.horizontalViewAngle/2.0 - cameraPose[2])
			d2 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi/2.0 - self.horizontalViewAngle/2.0 + cameraPose[2])
		elif wall == 'north' or wall == 'south':
			d1 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi - self.horizontalViewAngle/2.0 - cameraPose[2])
			d2 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(cameraPose[2] - self.horizontalViewAngle/2.0)


		# add d1 and d2 (or subtract) to the camera's axis wall intersection point (add/subtract and x/y depends on wall)
		if wall == 'east' or wall == 'west':
			p1 = [x, y+d1]
			p2 = [x, y-d2]
		elif wall == 'north' or wall == 'south':
			p1 = [x-d1, y]
			p2 = [x+d2, y]

		# determine camera view limit intersection point range and bearings relative to camera
		range1, bearing1 = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, p1)
		range2, bearing2 = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, p2)

		# Check that the two view limit intersection points are valid (i.e. occur on the arena boundary and not outside, that the bearing is within view and the range is greater than a minimum distance)
		# Need to add small percentage to the angle due to the numerical evaluation of COPPELIA this is to ensure that after checking against all walls that 2 points are returned this is where the *1.05 comes from
		# make sure p1 is within bounds and that bearing is valid
		if (p1[0] < -1 or p1[0] > 1 or p1[1] < -1 or p1[1] > 1):
			p1 = None
		elif abs(bearing1) > (self.horizontalViewAngle/2.0)*1.05:
			p1 = None
		elif range1 < self.robotParameters.minWallDetectionDistance:
			p1 = None
		
		# make sure p2 is within bounds
		if (p2[0] < -1 or p2[0] > 1 or p2[1] < -1 or p2[1] > 1):
			p2 = None
		elif abs(bearing2) > (self.horizontalViewAngle/2.0)*1.05:
			p2 = None
		elif range2 < self.robotParameters.minWallDetectionDistance:
			p2 = None

		return p1, p2


	# Gets the point where the camera's view axis (centre of image) intersects with the specified wall.
	# inputs:
	#	cameraPose - pose of the camera [x, y, theta] in the global coordinate frame
	# 	wall - wall want to get the camera view limit points of ('east', 'west', 'north', 'south').
	# returns:
	#	x - the x coordinate where the camera's axis intersects with the specified wall
	#	y - the y coordinate where the camera's axis intersects with the specified wall
	def CameraViewAxisWallIntersectionPoint(self, cameraPose, wall):
		if wall == 'east':
			x = 1
			y = (x - cameraPose[0]) * math.tan(cameraPose[2]) + cameraPose[1]
		
		elif wall == 'north':
			y = 1
			x = (y - cameraPose[1]) / math.tan(cameraPose[2]) + cameraPose[0]

		elif wall == 'west':
			x = -1
			y = (x - cameraPose[0]) * math.tan(cameraPose[2]) + cameraPose[1]

		elif wall == 'south':
			y = -1
			x = (y - cameraPose[1]) / math.tan(cameraPose[2]) + cameraPose[0]

		return x, y
	

	# Wraps input value to be between -pi and pi
	def WrapToPi(self, radians):
		return ((radians + math.pi) % (2* math.pi) - math.pi)

	# Gets the range and bearing given a 2D pose (x,y,theta) and a point(x,y). 
	# The bearing will be relative to the pose's angle
	def GetRangeAndBearingFromPoseAndPoint(self, pose, point):
		_range = math.sqrt(math.pow(pose[0] - point[0], 2) + math.pow(pose[1] - point[1], 2))
		_bearing = self.WrapToPi(math.atan2((point[1]-pose[1]), (point[0]-pose[0])) - pose[2])

		return _range, _bearing


	# Gets the orthogonal distance (in metres) from the collector to the item. 
	# Assuming the the item's centroid is within 70 degrees of the collector's centroid
	def CollectorToItemDistance(self, itemPosition):
		# get the position of the item relative to the collector motor
		if self.robotPose != None and np.all(np.isnan(itemPosition)) == False:
			# get the pose of the collector in the x-y plane using the robot's pose with some offsets
			collectorPose = [self.robotPose[0]+0.1*math.cos(self.robotPose[5]), self.robotPose[1]+0.1*math.sin(self.robotPose[5]), self.robotPose[5]]

			# get range and bearing from collector to item position
			_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(collectorPose, itemPosition)

			# check to see if the bearing to the item is larger than 70 degrees. If so return None
			if abs(_bearing) > math.radians(70):
				return None

			# return distance to item from collector orthogonal to collector's rotational axis
			return abs(_range * math.cos(_bearing))

		return None


####################################
###### SCENE PARAMETERS CLASS ######
####################################

# This class is a helper class to simply 
# group COPPELIA scene parameters together

class SceneParameters(object):
	"""docstring for SceneParameters"""
	def __init__(self):
		# item Starting Position

		# Starting contents of the items [shelf,X,Y]. Set to -1 to leave the bay empty.
		self.bayContents = -np.ones((6,4,3),dtype=np.int16)


		# Obstacles Starting Positions - set to none if you do not want a specific obstacle in the scene
		self.obstacle0_StartingPosition = [-0.45, 0.5]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current coppelia position, or none if not wanted in the scene
		self.obstacle1_StartingPosition = [-0.25,-0.675]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current coppelia position, or none if not wanted in the scene
		self.obstacle2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current coppelia position, or none if not wanted in the scene



####################################
###### ROBOT PARAMETERS CLASS ######
####################################

# This class is a helper class to simply 
# group robot parameters together

class RobotParameters(object):
	"""docstring for RobotParameters"""
	def __init__(self):

		# Body Paramaters
		self.robotSize = 0.15 # This parameter cannot be changed
		
		# Drive/Wheel Parameters
		self.driveType = 'differential'	# specifies the drive type ('differential' is the only type currently)
		self.wheelBase = 0.150 # This parameter should not be changed
		self.wheelRadius = 0.04 # This parameter should not  be changed
		self.minimumLinearSpeed = 0.0 	# minimum speed at which your robot can move forward in m/s
		self.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
		self.driveSystemQuality = 1.0 # specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

		# Camera Parameters
		self.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
		self.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the front of the robot
		self.cameraHeightFromFloor = 0.1 # height of the camera relative to the floor in metres
		self.cameraTilt = 0.0 # tilt of the camera in radians
		self.cameraPerspectiveAngle = math.radians(60) # do not change this parameter

		# Vision Processing Parameters
		self.maxItemDetectionDistance = 1 # the maximum distance away that you can detect the item in metres
		self.maxPackingBayDetectionDistance = 2.5 # the maximum distance away that you can detect the packingBay in metres
		self.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres
		self.maxRowMarkerDetectionDistance = 2.5 # the maximum distance away that you can detect the obstacles in metres.
		self.maxShelfDetectionDistance = 2.5 # the maximum distance away that you can detect the shelves in metres.
		self.minWallDetectionDistance = 0.1 # the minimum distance away from a wall that you have to be to be able to detect it

		# collector Parameters
		self.collectorQuality = 1.0 # specifies how good your item collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
		self.autoCollectItem = True #specifies whether the simulator automatically collects items if near the collector 
		self.maxCollectDistance = 0.03 #specificies the operating distance of the automatic collector function. Item needs to be less than this distance to the collector

		self.sync = False

