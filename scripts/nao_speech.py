#!/usr/bin/env python

#This file contains the speech service. The request parameter is a string, which the NAO processes and speaks out 

from __future__ import print_function
import rospy
import actionlib
from naoqi_bridge_msgs.msg import SpeechWithFeedbackActionGoal
from std_msgs.msg import String 
from PROJECT.srv import SpeakWords, SpeakWordsResponse 


class Speech(object):

    def __init__(self):
        rospy.init_node('nao_speech_service')
        self.speak_pub = rospy.Publisher('/speech_action/goal', SpeechWithFeedbackActionGoal, queue_size=10)


    # "speak_nao" method to say words
    def speak_nao(self, req):
        goal = SpeechWithFeedbackActionGoal()
        goal.goal.say = req.text_to_speak
        self.speak_pub.publish(goal)
        rospy.loginfo("NAO is speaking: " + req.text_to_speak)
        return SpeakWordsResponse(True)

    # "run_service" creates an active server called "speak_words"
    def run_service(self):
        service = rospy.Service('speak_words', SpeakWords, self.speak_nao)
        rospy.loginfo("NAO Speech Words service is ready.")
        rospy.spin()


if __name__ == '__main__':
    speech_handle = Speech()
    speech_handle.run_service()

