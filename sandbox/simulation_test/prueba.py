# -*- coding: utf-8 -*-
"""
Test para probar V-Rep con python

"""
import time
import vrep
import numpy as np

print ('Program started')
vrep.simxFinish(-1) 
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')

else:
    print ('Connection failed')

error,motorFL=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
error,motorFR=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
error,robot=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)

x=0
#while x<2:
#    time.sleep(1)
#    vrep.simxSetJointTargetVelocity(clientID,motorFL,1,vrep.simx_opmode_streaming)
#    vrep.simxSetJointTargetVelocity(clientID,motorFR,-1,vrep.simx_opmode_streaming)
#    time.sleep(1)
#    vrep.simxSetJointTargetVelocity(clientID,motorFL,-1,vrep.simx_opmode_streaming)
#    vrep.simxSetJointTargetVelocity(clientID,motorFR,1,vrep.simx_opmode_streaming)
#    x+=1

vrep.simxSetJointTargetVelocity(clientID,motorFL,1,vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID,motorFR,1,vrep.simx_opmode_streaming)

x=0
while x<400:
    errorCode, angle=vrep.simxGetObjectOrientation(clientID,robot,-1, vrep.simx_opmode_streaming)
    errorCode, position=vrep.simxGetObjectPosition(clientID,robot,-1, vrep.simx_opmode_streaming)
    print angle.index(0)
    
    #heading = np.atan2(y * sin(angle)- x * z * (1 - cos(angle)) , 1 - (y2 + z2 ) * (1 - cos(angle)))    
    
    
    print position
    time.sleep(0.1)
    x+=1

    