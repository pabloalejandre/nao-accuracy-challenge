#! /usr/bin/env python

#This file contains the tactile node and service. We use it to check whether the head of the NAO has been touched which signals that the loading has finished and the robot can shoot.

#It contains one subscriber to the "/tactile_touch" topic which tells us whether the head has been touched
#It also has a service which is called from the main engine to retrieve the information whether the was touched or not

import rospy
from naoqi_bridge_msgs.msg import HeadTouch
from PROJECT.srv import SpeakWords, SpeakWordsRequest
from PROJECT.srv import CheckHeadTouchResponse
from naoqi import ALProxy
from parameters import *
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty, EmptyResponse


class NaoTactileTouch:
    def __init__(self):
        # Iniitalize node and subscriber for checking head touch
        rospy.init_node("nao_tactile_touch")
        self.head_touch_sub = rospy.Subscriber("/tactile_touch", HeadTouch, self.check_head_touch_callback)
        
        # Create a service server for transmitting whether head has been touched
        self.head_touch_service = rospy.Service('check_head_touch', CheckHeadTouchResponse, self.check_head_touch_callback1)
        
        #Control signal
        self.head_touched = False

        rospy.spin()
        
    def check_head_touch_callback(self, data):
        #This is the callback of the subscriber
        # Modify the response to use the CheckHeadTouchResponse service message
        if data.state == 1:
            self.head_touched= True
            rospy.loginfo("Head has been touched!")
        else:
            rospy.loginfo("Head has NOT touched!")
            self.head_touched = False
        
        print(data)
        print(self.head_touched)
        return 
        
    
    def check_head_touch_callback1(self, req):
        #This is the callback of the service.
        print(self.head_touched)
        # Modify the response to use the CheckHeadTouchResponse service message
        if self.head_touched:
            #Set head_touched to False once we exit the service call
            self.head_touched = False
            return True
        else:
            return False
        
        
                             
if __name__ == "__main__":
    NaoTactileTouch()
