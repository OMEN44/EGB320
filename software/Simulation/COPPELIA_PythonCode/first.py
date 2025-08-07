#!/usr/bin/python

# import the  bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *

# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.bayContents= np.random.random_integers(0,5,(6,4,3)) # Random item in each bay
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl # specify a bowl in the bay in shelf 0 
# with x,y coords (3,1) (zero-indexed). Items are {bowl,mug,bottle,soccer,rubiks,cereal}.

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'	# specify if using differential (currently omni is not supported)

# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C
	try:
		# packerBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
		packerBotSim = COPPELIA_WarehouseRobot('192.168.56.1', robotParameters, sceneParameters)
		packerBotSim.StartSimulator()

		while True:
			packerBotSim.SetTargetVelocities(100, 0)  # forward 0.1 m/s and 0 rotational velocity
			
			packerBotSim.UpdateObjectPositions() # needs to be called once at the end of the main code loop

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		packerBotSim.StopSimulator()