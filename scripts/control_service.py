#!/usr/bin/env python

#This module controls the movement of the NAO.
#It's implemented as a service and called from the main engine in order to align the robot with the detected targets for hitting them in the center.

#When called using the 'positioning' mode, the Robot will turn in place so that the chosen target among the detected ones during scanning is within the robot's sight in its shooting position. 
#The 'aiming' mode runs a control loop to exactly align the target horizontally with the robot given a pixel threshold.


import rospy
import time
from naoqi import ALProxy
from parameters import *
from PROJECT.srv import Control, Vision

class NAO_Control():
    def __init__(self):
        #Initialize the control node and service
        rospy.init_node('NAO_CONTROL', anonymous=True)
        rospy.Service("Control_Service", Control, self.service_callback)
        
        #Initialize proxies for movement control
        self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
        self.motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]]) #enable foot contact protection
        self.motionProxy.setWalkArmsEnabled(True, True) #enable arms control for walking
        
        #Clients to call the vision service for aiming
        self.vision_service = rospy.ServiceProxy('Vision_Service', Vision)
    
        #Control signals for aiming algorithm
        self.stopMoving = False    
        self.vision_response = None
        
        
    def service_callback(self, req):
        #This is the control service callback. It's request contains a string to choose either mode 'positioning' or 'aiming'
        #The request also contains an angle used in positioning mode to tell the NAO how much to turn
        
        if(req.mode=='positioning'):
            #Positioning mode turns NAO to target by calling turn_to_targets and returns True
            self.turn_to_targets(self.motionProxy, req.angle)
            print("EXITING CONTROL SERVICE")
            return True
        
        elif(req.mode=='aiming'):
            #Aiming mode of control service
            
            #Starts a timer to ensure stability of algorithm
            timeout = 60
            start_time = time.time()
            
            #If timer hasn't run out or stopMoving is False, we call vision service to get target's coordinates and use aiming function to turn the robot
            while not self.stopMoving and time.time() - start_time < timeout:
                req = Vision._request_class()
                req.x = 0.0
                req.y = 0.0
                req.r = 0.0
                req.mode = "detect_targets"
                self.vision_response = self.vision_service(req)
                
                if(self.vision_response.success == True):
                    self.nao_aiming(self.vision_response)
                else:
                    continue
            
            #If timer runs out we exit service by returning False
            if time.time() - start_time >= timeout:
                rospy.loginfo("Timeout reached. Stopping the robot")
                return False
            
            #If stopMoving was set to True, this means we are succesfullly aligned with target.
            #We set it to False for the next time we call the service and exit by returning True
            self.stopMoving = False 
            return True 
        
        else:
            rospy.loginfo('Wrong mode for the control service')
            return False 


    #Positioning Function
    #input : angle of target (request parameter of service)
    def turn_to_targets(self,motionProxy,angle):
        motionProxy.angleInterpolation("HeadYaw", np.deg2rad(-90), 6, True)
        motionProxy.setWalkArmsEnabled(True,True)
        motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
        while True:
            X = 0.0
            Y = 0.0
            Theta = np.deg2rad(angle)
            print(Theta)
            #Use moveTo function of the API to turn exactly Theta degrees which is the parameter of our service request call
            motionProxy.post.moveTo(X, Y, Theta)
            motionProxy.waitUntilMoveIsFinished()
            break
        
    
    #Functions used by aiming function to start walking and stop walking respectively with the angle it should turn as input
    def start_walking(self, aiming_direction):
        x = 0
        y = 0
        theta = aiming_direction
        frequency = 0.8
        self.motionProxy.setWalkTargetVelocity(x, y, theta, frequency)
    
    def stop_walking(self):
        self.motionProxy.setWalkTargetVelocity(0, 0, 0, 0)
    
    
    #Aiming Function
    def nao_aiming(self, resp):
        print("-------------------------------------------------------------------------------------")
        print("ENTERING AIMING FUNCTION")
        #Define control variables for the loop (error term, threshold)
        #This function uses the response from the vision_service (the coordinates of the target) to calculate the error term
        error = 0
        threshold = 15
        detected_offset = shooting_pixel - resp.x
        error = detected_offset
        print(error)
        if(abs(error)<=threshold):
            #If error is within a threshold -> NAO is aligned with target. It stops walking, sets stopMoving to True to return from service
            print('ERROR WITHIN TRESHOLD. STOP WALKING')
            self.stop_walking()
            self.stopMoving = True
            time.sleep(3)
            return
        else:
            if(abs(error))>threshold:
                #If error is outside of threhsold -> Keep Turning towards target and stop before reevaluating position
                direction = np.sign(error)*0.03
                print('NOT REACHED THRESHOLD RANGE. AIMING SLOWLY TOWARDS TARGET')
                self.start_walking(aiming_direction=direction)
                time.sleep(1)
                self.stop_walking()
                time.sleep(2)
                print('STOP WALKING TO REEVALUATE')
        print("-----------------------------------------------------------------------------------------")


if __name__ == '__main__':
    #Start Control Node and keep it running
    control = NAO_Control()
    rospy.spin()
