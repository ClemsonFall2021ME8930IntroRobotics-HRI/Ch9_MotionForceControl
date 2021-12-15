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
Created on Sun Dec 12 23:56:19 2021

@author: denni
"""

import math
import sim
import sys

import time

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')
    
    errorcode, joint1 = sim.simxGetObjectHandle(clientID,'Revolute_joint1_test',sim.simx_opmode_blocking)
    errorcode1, link1 = sim.simxGetObjectHandle(clientID, 'link1_test', sim.simx_opmode_blocking)
    errorcode2, joint2 = sim.simxGetObjectHandle(clientID,'Revolute_joint2_test',sim.simx_opmode_blocking)
    errorcode3, link2 = sim.simxGetObjectHandle(clientID, 'link2_test', sim.simx_opmode_blocking)
    
    PID_P = float(input("What is the proportional gain? "))
    PID_I = float(input("What is the integral gain? "))
    PID_D = float(input("What is the derivative gain? "))   
    pidCumulativeErrorForIntegralParam1 = 0
    pidCumulativeErrorForIntegralParam2 = 0
    pidLastErrorDerivativeParam1 = 0
    pidLastErrorDerivativeParam2 = 0
    
    sim.simxSetJointTargetPosition(clientID, joint1, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetPosition(clientID, joint2, 0, sim.simx_opmode_blocking)
    currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
    currentPosition = sim.simxGetJointPosition(clientID, joint2, sim.simx_opmode_streaming)
    print(currentPosition)
    #targetPosition1 = float(input("What is the target position of Joint 1: ")) * math.pi/180
    #targetPosition2 = float(input("What is the target position of Joint 2: ")) * math.pi/180
    
    #Lengths of Link 1  
    objectMin = sim.simxGetObjectFloatParameter(clientID, link1, 17, sim.simx_opmode_blocking)
    objectMax = sim.simxGetObjectFloatParameter(clientID, link1, 20, sim.simx_opmode_blocking)
    zLength1 = objectMax[1] - objectMin[1]
    
    
    #Lengths of Link 2    
    objectMin = sim.simxGetObjectFloatParameter(clientID, link2, 17, sim.simx_opmode_blocking)
    objectMax = sim.simxGetObjectFloatParameter(clientID, link2, 20, sim.simx_opmode_blocking)
    zLength2 = objectMax[1] - objectMin[1]
    
    
    xTargetPosition = float(input("What is the x coordinate for target position? "))
    yTargetPosition = float(input("What is the y coordinate for target position? "))
    r = xTargetPosition**2 + yTargetPosition**2
    
    if math.sqrt(r) > (zLength1 + zLength2):
        print("Error, target position is out of range")
        sys.exit()
        
    theta1 = (math.acos((zLength2**2 - zLength1**2 - r)/(-2*zLength1*math.sqrt(r))) - math.atan2(yTargetPosition, xTargetPosition))
    theta2 = math.pi - math.acos((r - zLength1**2 - zLength2**2)/(-2*zLength1*zLength2))
    
    if theta1 > math.pi:
        theta1 = theta1 - (2*math.pi)
    if theta2 > math.pi:
        theta2 = theta2 - (2*math.pi)
    
    
    currentPosition2 = sim.simxGetJointPosition(clientID, joint2, sim.simx_opmode_streaming)
    #errorValue1 = theta1 - currentPosition1[1]
    errorValue2 = theta2 - currentPosition2[1]
    while abs(errorValue2) > 0.01:
        i = 0
        #currentPosition1 = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
        currentPosition2 = sim.simxGetJointPosition(clientID, joint2, sim.simx_opmode_streaming)
        #errorValue1 = theta1 - currentPosition1[1]
        errorValue2 = theta2 - currentPosition2[1]
        print(errorValue2)

        #Proportional control
        #ctrl1 = errorValue1*PID_P
        ctrl2 = errorValue2*PID_P

        #Integral control
        if PID_I != 0:
            #pidCumulativeErrorForIntegralParam1 = pidCumulativeErrorForIntegralParam1 + errorValue1 * 0.005
            pidCumulativeErrorForIntegralParam2 = pidCumulativeErrorForIntegralParam2 + errorValue2 * 0.005
        else:
            pidCumulativeErrorForIntegralParam1 = 0
            pidCumulativeErrorForIntegralParam2 = 0
    
        #ctrl1 = ctrl1 + pidCumulativeErrorForIntegralParam1*PID_I
        ctrl2 = ctrl2 + pidCumulativeErrorForIntegralParam2*PID_I
    

        #Derivative Control
        if i != 0:
            #ctrl1 = ctrl1 + (errorValue1 - pidLastErrorDerivativeParam1)*PID_D/0.005
            ctrl2 = ctrl2 + (errorValue2 - pidLastErrorDerivativeParam2)*PID_D/0.005

        #pidLastErrorDerivativeParam1 = errorValue1
        pidLastErrorDerivativeParam2 = errorValue2
    
        #Calculate Control
        #thetaJoint1 = (ctrl1/0.03083)*0.005
        thetaJoint2 = (ctrl2/0.03083)*0.005
           
        #sim.simxSetJointTargetPosition(clientID, joint1, thetaJoint1, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID, joint2, thetaJoint2, sim.simx_opmode_oneshot)
        currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
        currentPosition = sim.simxGetJointPosition(clientID, joint2, sim.simx_opmode_streaming)
        i = i+1
    
    currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
    errorValue = theta1 - currentPosition[1]
    while abs(errorValue) > 0.01:
        i = 0
        currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
        errorValue = theta1 - currentPosition[1]
        print(errorValue)

        #Proportional control
        ctrl = errorValue*PID_P

        #Integral control
        if PID_I != 0:
            pidCumulativeErrorForIntegralParam1 = pidCumulativeErrorForIntegralParam1 + errorValue * 0.005
        else:
            pidCumulativeErrorForIntegralParam1 = 0
        
        ctrl = ctrl + pidCumulativeErrorForIntegralParam1*PID_I

        #Derivative Control
        if i != 0:
            ctrl = ctrl + (errorValue - pidLastErrorDerivativeParam1)*PID_D/0.005

        pidLastErrorDerivativeParam1 = errorValue
        
        #Calculate Control
        theta = (ctrl/0.03083)*0.005
               
        sim.simxSetJointTargetPosition(clientID, joint1, theta, sim.simx_opmode_oneshot)
        currentPosition = sim.simxGetJointPosition(clientID, joint1, sim.simx_opmode_streaming)
        i = i+1
        
else:
    print ('Failed connecting to remote API server')
print ('Program ended')