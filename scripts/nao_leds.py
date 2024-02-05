#! /usr/bin/env python

#This file contains the services which allows to set the leds of the nao to either green or red


import rospy
from naoqi_bridge_msgs.msg import HeadTouch
from PROJECT.srv import SpeakWords, SpeakWordsRequest
from std_srvs.srv import Empty, EmptyResponse
from naoqi import ALProxy
from parameters import *


#----------2 Services: set_led_red && set_led_green -----------

class NaoLeds:
    def __init__(self):
        rospy.init_node("nao_leds")
        
        # Initialize NAOqi LED proxy
        self.nao_leds = ALProxy("ALLeds", robotIP, PORT)

        # Create service servers for setting LEDs to red
        self.set_led_red_service = rospy.Service('set_led_red', Empty, self.set_led_red_callback)
        # Create service servers for setting LEDs to green
        self.set_led_green_service = rospy.Service('set_led_green', Empty, self.set_led_green_callback)

        rospy.spin()

    def set_led_red_callback(self, req):
        # Set all LEDs to red
        led_names = self.nao_leds.listLEDs()
        for led_name in led_names:
            self.nao_leds.fadeRGB(led_name, 1, 0, 0, 0)  # Set all LED colors to red
        return EmptyResponse()

    def set_led_green_callback(self, req):
        # Set all LEDs to green
        led_names = self.nao_leds.listLEDs()
        for led_name in led_names:
            self.nao_leds.fadeRGB(led_name, 0, 1, 0, 0)  # Set all LED colors to green
        return EmptyResponse()

                     
if __name__ == "__main__":
    NaoLeds()
    
