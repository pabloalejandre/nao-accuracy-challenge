#!/usr/bin/env python

#This is the main engine, where the client for all the services are defined. 
#Upon starting the script runs the main sequence which defines the structure and logic flow of the Shooting Game
#At the bottom of the file there are function definitions used throughout the sequence to call the various services such as using text_to_speech or detecting the targets in the area

from __future__ import print_function
import rospy
import cv2
import numpy as np
from PROJECT.srv import Vision, Control, SpeakWords, ShootBow,CheckHeadTouchResponse
import time
from naoqi import ALProxy
from parameters import *
from std_srvs.srv import Empty, EmptyResponse


#Function to turn on the stiffness of the robot 
def StiffnessOn(proxy):
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

class mainEngine():
    def __init__(self):
        
        #Start main engine node
        rospy.init_node('masternode', anonymous=True)
        
        ## Attributes for calculations
        self.vision_coords = [] 
        self.actual_targets = []
        self.angle_values = [] 
        
        #Service Servers 
        self.vision_service = rospy.ServiceProxy('Vision_Service', Vision)
        self.control_service = rospy.ServiceProxy('Control_Service', Control)
        self.shooting_service = rospy.ServiceProxy('Shooting_Service', ShootBow)
        self.speaking_service = rospy.ServiceProxy('speak_words', SpeakWords)
        self.tactile_service = rospy.ServiceProxy('check_head_touch', CheckHeadTouchResponse)
        self.led_red_service = rospy.ServiceProxy('set_led_red', Empty)
        self.led_green_service = rospy.ServiceProxy('set_led_green', Empty)
        
        #Service request objects
        control_request = Control._request_class()
        check_head_touch_request = CheckHeadTouchResponse._request_class()
        shoot_bow_request = ShootBow._request_class()
        
        ####
        #### Start Shooting Game
        ####
        
        self.speak_client("Starting the NAO SHOOTING GAME i will TRY HIT All Targets Then it is your Turn")
        time.sleep(1)
        self.speak_client("Start kill all Targets")
        
        #Get motion and posture proxies for movement control
        try:
            self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        except Exception:
            print("ALMotion Exception")
        try:
            self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
        except Exception:
            print("AlRobotPosture Exception")
            
        #Set NAO in "green" mode
        self.call_leds_green()
        #Set NAO in Stiffness On
        StiffnessOn(self.motionProxy)
        #Send NAO to Pose Init
        self.postureProxy.goToPosture("StandInit", 0.2)
        
        #Get all targets in gamespace and save them into self.actual_targets using scanning mode of control service
        self.speak_client("Sweeping get All Targets")
        self.get_targets(self.motionProxy)
        print("targets::",self.actual_targets)
        number_targets = len(self.actual_targets)
        my_string = "I Have detected:  " + str(number_targets) + " Targets"
        self.speak_client(my_string)
        time.sleep(3)
        
        if number_targets == 0:
            #If no targets detected end game
            self.speak_client("Ending Game. Try again placing targets in front of me and restarting game.")
            time.sleep(5)
            return
        
        for target in self.actual_targets:
            string_mas = "I have a detected an Target at " + str(target[2]) + " Degrees"
            self.speak_client(string_mas)
            time.sleep(3)
        
        #For every target detected:
        #
        #Position robot so that target is in sight when robot is in shooting position (facing to his right side) / Target should be to the right side of robot's body
        #Aim so that target is actually aligned with shooting pixel of robot
        #Shoot bow (raise bow -> release -> lower bow)
        #
        
        #Add 90 degree displacement at the beginning so that robot faces targets with rightside of the body
        displacement = 90
        self.speak_client("Start  Moving to the targets")
        control_request = Control._request_class()
        control_request.mode = 'positioning'
        control_request.angle = displacement        
        self.control_service(control_request)
        
        #Positioning, aiming and hitting loop for each target detected
        last_angle = 0
        i=0
        number_targets_hit = 0
        for target in self.actual_targets:
            #Position robot with respect to target
            target_angle = target[2] - last_angle
            control_request.mode = 'positioning'
            control_request.angle = target_angle
            self.control_service(control_request)
            
            last_angle = target[2]
            target_angle = 0
            
            #Aim towards target center
            control_request.mode = 'aiming'
            aiming_response = self.control_service(control_request)
       
            #Shoot bow
            #Put NAO into shooting position with both arms raised
            self.speak_client("Going into shooting motion")
            #Set NAO in "red" mode
            angle_y = self.estimate_angle_y()
            print("Angle Y:", angle_y)
            self.call_leds_red()
            self.start_shooting_motion(angle_y)

            self.speak_client("I need help loading my bow.")
            time.sleep(2)
            self.speak_client("Touch my head when you are done.")
            #Check tactile feedback
            while True:
                tactile_feedback = self.tactile_service(check_head_touch_request).success
                if(tactile_feedback):
                    #Shoot bow
                    print('TACTILE FEEBACK WAS TRUE: GOING TO SHOOT')
                    self.start_release_motion()
                    time.sleep(3)
                    break
                else:
                    #Wait for it to be touched 
                    print("TACTILE FEEDBACK FALSE. I'M WAITING")
                    time.sleep(1)
            #Lower bow to verify target hit or miss
            self.motionProxy.angleInterpolation("RShoulderPitch", np.deg2rad(60), 3, True)
            time.sleep(3)
            if self.verify_targets():
                time.sleep(1)
                self.speak_client("Target Hit")
                number_targets_hit = number_targets_hit +1
                print("target Hit")
            else:
                self.speak_client("Target Missed")
                print("target missed")
            time.sleep(1)
            i = i+1
            if number_targets == i:
                
                self.speak_client("Done hitting all Targets")
                my_string = "I Have Hit :  " + str(number_targets_hit) + "Targets"
                time.sleep(1)
                self.speak_client(my_string)
                time.sleep(1)
            else:
                print("Done with this target, onto the next one")
                self.speak_client("Done with this target, onto the next one")
            
            self.postureProxy.goToPosture("StandInit", 6)
            
        
        self.speak_client("Finished")
        time.sleep(1)
        
        self.speak_client("Now it is your turn to Shoot Try to Best Me  ")
        return


    #Calls the vision service first with detect targets mode to get target coordinates
    #Then, call with detect hits mode passing the coordinates as request parameter to verify whether the target was hit or missed
    def verify_targets(self):
        number_run = 0
        diff_list = []
        for _ in range(200):
            result = self.get_vision()  # Capture the result of self.get_vision() (detect targets) to get coordinates of target the NAO shot at
            
            if result:
                x = result.x
                y = result.y
                r = result.r
                number_run = number_run +1
            else:
                continue
            
            vision_request = Vision._request_class()    #Call vision service in detect hit mode by passing the coordinates of target
            vision_request.mode = 'detect_hits'
            vision_request.x = x
            vision_request.y = y
            vision_request.r = r
            response = self.vision_service(vision_request) #uses contour value of vision service response to decide whether the target was hit or not
            if response.contour_value >= 1000:
                response.contour_value = 999
            print(round(response.contour_value, -2))
            diff_list.append(round(response.contour_value, -2))
            median = max(set(diff_list), key=diff_list.count)
        try:
            if median < 10:
                return False
            else: 
                return True
        except:
            return False
       
    #Calls the shooting service with the Prep Shoot mode to put robot in shooting position 
    def start_shooting_motion(self,y):
        print("starting shooting motion")
        
        try:
            Shoot_service = rospy.ServiceProxy('ShootBow', ShootBow)
            req = ShootBow._request_class()
            req.Mode = "prep_Shoot"
            req.height = y
            result  = Shoot_service(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            
    #Calls the shooting service
    def start_release_motion(self):
        print("starting shooting")
        try:
            Shoot_service = rospy.ServiceProxy('ShootBow', ShootBow)
            req = ShootBow._request_class()
            req.Mode = "Shoot"
            result  = Shoot_service(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    #Estimates the angle for vertical aiming by calling the vision service
    def estimate_angle_y(self):
        count = 0
        while True:
            count = count +1
            result = self.get_vision()  # Capture the result of self.get_vision()
            if count > 1000:
                return 0
            if result:
                x = result.x
                y = result.y
                r = result.r
                r_a = r / 18
                if y > 104:
                    angle = 0
                else:
                    y_a = 104 - y
                    y_perc = y_a / 104
                    angle = 28 * y_perc * r_a
                return angle
                # Append the angle to the list and keep it at a maximum of 50 values
                self.angle_values.append(angle)
                # Calculate the average of the last 50 angles
                
                if len(self.angle_values) > 50:
                    self.angle_values.pop(0)
                    average_angle = sum(self.angle_values) / len(self.angle_values)
                    return average_angle
            else:
                continue
        
    #Text to speech by calling the speech service
    def speak_client(self,text_to_speak):
        rospy.wait_for_service('speak_words')
        
        try:
            speak_service = rospy.ServiceProxy('speak_words', SpeakWords)
            resp = speak_service(text_to_speak)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        
    #Calls the tactile service and returns whether the head was touched or not for entering Shoot mode
    def call_tactile_touch(self):
        rospy.wait_for_service('check_head_touch')
        try:
            tactile_touch = rospy.ServiceProxy('check_head_touch', CheckHeadTouchResponse)
            results = tactile_touch()
            return results
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    #Turns leds red by calling leds service
    def call_leds_red(self):
        rospy.wait_for_service('set_led_red')
        try:
            led_red_service = rospy.ServiceProxy('set_led_red', Empty)
            led_red_service()
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    
    #turns leds green by calling leds service
    def call_leds_green(self):
        rospy.wait_for_service('set_led_green')
        try:
            led_green_service = rospy.ServiceProxy('set_led_green', Empty)
            led_green_service()
            return False
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    
    #gets all targets in the area by moving head from side to side and calling the vision service
    def get_targets(self, motionProxy):
        motionProxy.angleInterpolation("RElbowRoll", np.deg2rad(0), 3.0, True)
        motionProxy.angleInterpolation("HeadYaw", np.deg2rad(119), 6, True)
        motionProxy.angleInterpolation("HeadPitch", np.deg2rad(-10), 3, True)
        max_angle = 119
        min_angle = -60#-119
        step = 2#1  # Step size in degrees
        isnewTarget= True

        # Start the loop from 119 degrees and decrease by 1 degree in each iteration
        for angle in range(max_angle, min_angle - 1, -step):
            angle_in_radians = np.deg2rad(angle)
            motionProxy.angleInterpolation("HeadYaw", angle_in_radians, 0.2, True)
            result = self.get_vision()  # Capture the result of self.get_vision()
            if result:
                # Append coordinates (x, y), head angle, and resp.r to the list
                self.vision_coords.append((result.x, result.y, angle, result.r))
                if 120 <= result.x <= 140:
                    if isnewTarget:
                        self.actual_targets.append((result.x, result.y, angle, result.r))
                        isnewTarget = False
                if 230 <= result.x <= 340:
                    isnewTarget = True 
                
        motionProxy.angleInterpolation("HeadPitch", np.deg2rad(0), 3, True)
        return True
    
    #Call the vision service in detect targets mode to get the coordinates of the targets in sight
    def get_vision(self):
        req = Vision._request_class()
        
        req.mode = 'detect_targets'
        resp = self.vision_service(req) 
        if resp.success:
            return resp  # Return the entire response object
        else:
            return None  # Return None if not successful



if __name__ == "__main__":
    engine = mainEngine()
    
    