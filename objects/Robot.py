#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May  8 11:31:06 2021

@author: gabriel
"""
import sim

max_speed = 1.2
wheel_radius = 0.195/2
speed_f = 0.1
speed = speed_f*max_speed/wheel_radius
s_mode = sim.simx_opmode_streaming #stream mode

class Robot:
    def __init__(self, clientID):
        self.clientID = clientID
        self.error_l_motor, self.l_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
        self.error_r_motor, self.r_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)    

    def moveForward(self):
        sim.simxSetJointTargetVelocity(self.clientID, self.r_motor_handle,speed, s_mode)
        sim.simxSetJointTargetVelocity(self.clientID, self.l_motor_handle,speed, s_mode)
     
    def moveBackward(self):
        sim.simxSetJointTargetVelocity(self.clientID, self.r_motor_handle,-speed, s_mode)
        sim.simxSetJointTargetVelocity(self.clientID, self.l_motor_handle,-speed, s_mode)
     
    def turnLeft(self):
        sim.simxSetJointTargetVelocity(self.clientID, self.r_motor_handle,speed, s_mode)
        sim.simxSetJointTargetVelocity(self.clientID, self.l_motor_handle,-speed, s_mode)
     
    def turnRight(self):
        sim.simxSetJointTargetVelocity(self.clientID, self.r_motor_handle,-speed, s_mode)
        sim.simxSetJointTargetVelocity(self.clientID, self.l_motor_handle,speed, s_mode)