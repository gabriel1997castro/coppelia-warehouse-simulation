#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 10 23:37:13 2021

@author: gabriel
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  3 00:42:53 2021

@author: gabriel
"""

import sim
import sys
import time
sys.path.append("./objects")
from Robot import Robot

cam_name = 'cam1'
sim.simxFinish(-1) # just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

# ultrasonic sensors
front_range = [3, 4, 5, 6]
front_left_range = [1, 2, 3]
front_right_range = [6, 7, 8]
back_right_range = [9, 10, 11]
back_range = [11, 12, 13, 14]
back_left_range = [14, 15, 16]
    
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Connection could not be established')
    sys.exit("Could not connect")
    
def isNear(distances):
    print(distances)
    for distance in distances:
        if(distance < 10):
            return True
    return False

def main():
    Pioneer = Robot(clientID)

    distances = [40 for i in front_range]
    sensor = [False for i in range(16)]
    
    # dp - detection point
    # doh - detection object handle
    # dsnv - detectedSurfaceNormalVector

    for i in range(1,17):
        s_str = 'Pioneer_p3dx_ultrasonicSensor{}'.format(i) #sensor str
        error, sensor[i-1] = sim.simxGetDistanceHandle(clientID, s_str , sim.simx_opmode_oneshot_wait)
        error, distances[i-1] = sim.simxReadDistance(clientID, sensor[i-1], sim.simx_opmode_streaming)

    while(not isNear(distances)):
        for i in range(1,17):
            errorCode, distances[i-1] = sim.simxReadDistance(
                clientID, sensor[i-1], sim.simx_opmode_buffer)
#            print('detectionState', detectionState)
    # Pioneer.turnLeft()
        Pioneer.moveForward()
        time.sleep(1)
        Pioneer.stop()
    # time.sleep(2)
    # Pioneer.turnRight()
    # time.sleep(2)
    # Pioneer.moveBackward()
    # time.sleep(2)
    # Pioneer.moveForward()
    # time.sleep(6)
    
#    errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
#    errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
    
#    errorCode=sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.8, sim.simx_opmode_streaming)
#    errorCode=sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.8, sim.simx_opmode_streaming)
    
#    
    
#    errorCam0, camera_handle = sim.simxGetObjectHandle(clientID, cam_name, sim.simx_opmode_oneshot_wait)
            # Start the Stream
#    errorCam1, res, image = sim.simxGetVisionSensorImage(clientID, camera_handle, 0, sim.simx_opmode_streaming)
 
if __name__ == "__main__":
    main()

#errorCode, cam_handle = sim.simxGetObjectHandle(clientID, 'cam1', sim.simx_opmode_oneshot_wait)
#errorCode, resolution, image = sim.simxGetVisionSensorImage(clientID, cam_handle, 0, sim.simx_opmode_streaming)
#errorCode, resolution, image = sim.simxGetVisionSensorImage(clientID, cam_handle, 0, sim.simx_opmode_buffer)
