# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
"""
Created on Mon Dec 13 12:07:31 2021

@author: denni
"""

import math
import sim

import time

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')
    
    errorcode, joint1 = sim.simxGetObjectHandle(clientID,'Revolute_joint1_test',sim.simx_opmode_blocking)
    errorcode1, link1 = sim.simxGetObjectHandle(clientID, 'link1_test', sim.simx_opmode_blocking)
    
    PID_P = float(input("What is the proportional gain? "))
    PID_I = float(input("What is the integral gain? "))
    PID_D = float(input("What is the derivative gain? "))
    pidCumulativeErrorForIntegralParam = 0
    pidLastErrorDerivativeParam = 0
    
    sim.simxSetJointTargetPosition(clientID, joint1, 0, sim.simx_opmode_blocking)
    currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
    print(currentPosition)
    targetPosition = float(input("What is the target position of Joint: ")) * math.pi/180
    
        
    currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
    errorValue = targetPosition - currentPosition[1]
        
        
    while abs(errorValue) > 0.01:
        i = 0
        currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
        errorValue = targetPosition - currentPosition[1]
        print(errorValue)

        #Proportional control
        ctrl = errorValue*PID_P

        #Integral control
        if PID_I != 0:
            pidCumulativeErrorForIntegralParam = pidCumulativeErrorForIntegralParam + errorValue * 0.005
        else:
            pidCumulativeErrorForIntegralParam = 0
        
        ctrl = ctrl + pidCumulativeErrorForIntegralParam*PID_I

        #Derivative Control
        if i != 0:
            ctrl = ctrl + (errorValue - pidLastErrorDerivativeParam)*PID_D/0.005

        pidLastErrorDerivativeParam = errorValue
         
        #Calculate Control
        theta = (ctrl/0.03083)*0.005
               
        sim.simxSetJointTargetPosition(clientID, joint1, theta, sim.simx_opmode_oneshot)
        currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
        i = i + 1
    
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
    