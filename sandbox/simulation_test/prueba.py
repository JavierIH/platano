# -*- coding: utf-8 -*-
"""
Test para probar V-Rep con python

"""

import vrep

print ('Program started')
vrep.simxFinish(-1) 
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')

else:
    print ('Connection failed')

error,motorFL=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_oneshot_wait)


vrep.simxSetJointTargetVelocity(clientID,motorFL,1,vrep.simx_opmode_streaming)