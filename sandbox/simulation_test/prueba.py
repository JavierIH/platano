# -*- coding: utf-8 -*-
"""
Test para probar V-Rep con python

"""
import time
import vrep
import numpy as np
import matplotlib.pyplot as mlp
   
print ('Program started')
vrep.simxFinish(-1) 
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print "OK"
else:
    print "FAIL"

error,motorFL=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
error,motorFR=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
error,camera=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_oneshot_wait)
error,robot=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)

def moveRobot (left, right):
    "move the left and right motors of the robot"
    vrep.simxSetJointTargetVelocity(clientID,motorFL,left,vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID,motorFR,right,vrep.simx_opmode_streaming)

def getRobotPosition():
    "get the position and orientation of the robot"
    errorCode, position=vrep.simxGetObjectPosition(clientID,robot,-1, vrep.simx_opmode_streaming)
    return position[0], position[1]

def getRobotOrientation():
    "get the position and orientation of the robot"
    errorCode, angle=vrep.simxGetObjectOrientation(clientID,robot,-1, vrep.simx_opmode_streaming)    
    return angle[2]*180/3.14159265

def printRobotLocation():
    "print the position and orientation of the robot (x, y, yaw)"
    errorCode, angle=vrep.simxGetObjectOrientation(clientID,robot,-1, vrep.simx_opmode_streaming)
    errorCode, position=vrep.simxGetObjectPosition(clientID,robot,-1, vrep.simx_opmode_streaming)

    yaw='%.2f'%(angle[2]*180/3.14159265)
    x='%.2f'%(position[0])
    y='%.2f'%(position[1])
    
    print "Position: (", x,",", y, ")", "\nYaw:", yaw
    return 0
    
def getDistanceToTarget (x_target, y_target):
    "gets the distance from the robot to a given target"
    x_robot, y_robot = getRobotPosition()
    return np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2)
    
def getDifferenceToTargetAngle (angle):
    "Gets the difference between the actual orientation and a given orientation"
    difference=angle-getRobotOrientation()
    if difference < -180:
        difference+=360
    elif difference > 180:
        difference-=360
    return(difference)
    
def getTargetAngle (x_target, y_target):
    "Gets the angle to a point target"
    x_robot, y_robot = getRobotPosition()
    
    x_gap=x_target-x_robot
    y_gap=y_target-y_robot    
    l_gap=np.sqrt(x_gap**2+y_gap**2)    
        
    angle=np.arccos(x_gap/l_gap)*180/3.14159265
    if y_gap < 0:
        angle=-angle  
    return angle
    
def turnToTargetAngle (target_angle):
    "Orientates robot to a given angle in the range from -180 to 180"
    while np.abs(getDifferenceToTargetAngle(target_angle)) > 1:    
        amp = getDifferenceToTargetAngle(target_angle)
        left_motor=amp/20
        right_motor=-amp/20
        moveRobot(left_motor,right_motor)
        #print "Distancia al angulo objetivo: ", getRobotOrientation()
    moveRobot(0,0)
       
def goToTarget (target_x,target_y):
    while(getDistanceToTarget(target_x, target_y)>0.10):
        turnToTargetAngle (getTargetAngle(target_x,target_y))
        moveRobot(1,1)
        time.sleep(0.2)
        print getDistanceToTarget(target_x, target_y)
        
    print "punto!"
    moveRobot(0,0)


#moveRobot(-0.5,0.5);4

printRobotLocation()
#print getAngleToTarget(5,5)


errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_streaming)
time.sleep(1)

errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)
errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)
errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)
errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)
errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)

print "hola"

im=np.array(image, dtype=np.uint8)
im.resize([resolution[1], resolution[0],3])

#mlp.imshow(im)

im=np.flipud(im)

mlp.imshow(im)



print "sacabao"

    