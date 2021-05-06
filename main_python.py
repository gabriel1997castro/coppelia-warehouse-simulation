#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  3 00:42:53 2021

@author: gabriel
"""

import sim
import sys

sim.simxFinish(-1) # just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Connection could not be established')
    sys.exit("Could not connect")
    
errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

errorCode=sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.1, sim.simx_opmode_streaming)
errorCode=sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.1, sim.simx_opmode_streaming)

errorCode, sensor1 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', sim.simx_opmode_oneshot_wait)
errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, sensor1, sim.simx_opmode_streaming)
errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, sensor1, sim.simx_opmode_buffer)