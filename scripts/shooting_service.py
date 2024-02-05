#!/usr/bin/env python

#This file contains the shooting routines of the NAO. It is implemented as a ros service which is called from the main engine
#The routine consists of two modes:
    #The prep_Shoot mode raises both arms and puts the NAO in a shooting position
    #The Shoot mode releases the 3d printed piece which stores the energy of the bow and fires the arrow.
    
#The motion of the arms are defined in functions at the bottom of the file and they are called from within the service callback
    
import rospy
from naoqi import ALProxy
import motion
import time
import numpy as np
from parameters import *
from PROJECT.srv import ShootBow
robotIp = "10.152.246.218"
motionProxy = ALProxy("ALMotion", robotIp, 9559)

def StiffnessOn(proxy):
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

                          
def service_callback(req):
    try:
        motionProxy = ALProxy("ALMotion", robotIp, 9559)
    except Exception:
        print("ALMotion Exception")
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIp, 9559)
    except Exception:
        print("AlRobotPosture Exception")

    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)

    # Send NAO to Pose Init
    
    frame = motion.FRAME_TORSO
    if req.Mode == "prep_Shoot":
        #Prep Shoot mode which raises both arms
        right_arm_part(motionProxy=motionProxy)
        left_arm_part_Shoot(motionProxy=motionProxy)
        print(req.height)
        y_adjust_angle = int(req.height)
        print(y_adjust_angle)
        motionProxy.angleInterpolation("RElbowRoll", np.deg2rad(0 + y_adjust_angle ), 3.0, True)
        time.sleep(3)
        return True
        
    elif req.Mode == "Shoot":
        #Shoot mode to release the piece and fire the arrow
        left_arm_part_release(motionProxy=motionProxy)
        
        return True
    
#Function that raises the right arm holding the bow
def right_arm_part(motionProxy):
    motionProxy.angleInterpolation("HeadYaw", np.deg2rad(-90), 3.0, True)
    motionProxy.angleInterpolation("RShoulderRoll", np.deg2rad(-76), 3.0, True)

    motionProxy.angleInterpolation("RElbowRoll", np.deg2rad(0), 3.0, True)
    motionProxy.angleInterpolation("RElbowYaw", np.deg2rad(100), 3, True)
    motionProxy.angleInterpolation("RShoulderPitch", np.deg2rad(-10), 5, True)

    motionProxy.angleInterpolation("RElbowRoll", np.deg2rad(0), 3.0, True)

#Function that raises the left arm and puts it close to the bow
def left_arm_part_Shoot(motionProxy):
    motionProxy.angleInterpolation("LShoulderRoll", np.deg2rad(76), 3.0, True)
    
    motionProxy.angleInterpolation("LShoulderPitch", np.deg2rad(-18), 3, True)

    motionProxy.angleInterpolation("LElbowYaw", np.deg2rad(0), 3, True)

    motionProxy.angleInterpolation("LElbowRoll", np.deg2rad(-40.5), 3.0, True)

    motionProxy.angleInterpolation("LShoulderRoll", np.deg2rad(-18), 3.0, True)

#Function to pull using the left arm and shoot the arrow
def left_arm_part_release(motionProxy):
    motionProxy.angleInterpolation("LElbowRoll", np.deg2rad(0), 0.5, True)
    motionProxy.angleInterpolation("LShoulderRoll", np.deg2rad(76), 0.5, True)  


#Main function to initialize the shooting node and service
if __name__ == '__main__':
    rospy.init_node('ShootBow')
    rospy.Service("ShootBow", ShootBow, service_callback)
    rospy.spin()