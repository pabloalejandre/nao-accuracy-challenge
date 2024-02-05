#!/usr/bin/env python

#This module contains the perception/vision functionalities of the NAO:

#Upon starting it, the vision node subscribes to the top camera of the NAO and displays it. Additionally, it masks the color red and displays that mask as well
#It uses the information on both cameras for the target detection and hit detection

#In addition to the camera subscriber, it also has a service with two modes:

#Target detection mode uses the hough circles function to detect targets in sight of the robot and returns the coordinates of the target and radius
#Hit detection mode gets coordinates as parameter and returns a contour value which can be used to signal whether the target at those coords was hit or not

#This module uses the functions in nao_extract_target

import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
from PROJECT.msg import Target, Target_list
import cv2
from cv_bridge import CvBridge
from naoqi import ALProxy
import motion
import tf
from nao_extract_target import extract_target, hough_circles, hsv_process, triangulate
from parameters import *
from PROJECT.srv import Vision
import traceback
import copy

#ROS Node that process sensory input from top camera of the NAO: Monitors environment and extracts targets using functions in nao_extract_target.py
class NaoVision():
    
    #Class with the necessary implementations of vision tasks, target detection, target distance estimator
    def __init__(self):
        #Initialize the class. Initialize the class and the bridge message
        rospy.init_node('NAO_VISION', anonymous=True)
        rospy.Service("Vision_Service", Vision, self.service_callback)
        self.bridge = CvBridge()
        
        # ------------- Parameters for Transformation and rviz --------------------
        
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
        frame = motion.FRAME_TORSO
        transform = motionProxy.getTransform("CameraTop", frame, True)
        self.camera2torso = np.reshape(transform, (4, 4))
        self.optical2camera = optical2camera
        self.optical2torso = np.matmul(self.camera2torso, self.optical2camera)
        self.trans_bcaster = tf.TransformBroadcaster()
        
        # ------------- Parameters for correct detection algorithm -----------------
        
        self.track_window = track_window
        self.target_redcircle = True
        self.diameter = 0.18 #m
        self.bool_extract_roi = False
        
        # ----------- Camera Subscriber -----------------------------------------------
        
        self.camera_subscriber = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.camera_callback)
        
        
        # ----------- Variable to save masked image between functions ------------------
        
        self.mask = None
        
    
    #Subscribes to the top camera and processes the raw image (data)
    def camera_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv_image_copy = copy.deepcopy(cv_image)
         # Get image dimensions
        height, width, _ = cv_image.shape
        # Calculate the center coordinates
        center_x = 128
        center_y = 104

        # Draw a crosshair at the center of the image
        # Draw a black crosshair at the center of the image
        cv2.line(cv_image_copy, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 0), 2)
        cv2.line(cv_image_copy, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 0), 2)


        cv2.imshow('image', cv_image_copy)
        cv2.waitKey(2)
        # For ROI extraction
        if self.bool_extract_roi:
            cv2.imshow('Top Camera',cv_image)
            cv2.waitKey(0)
            try:
                extract_target()
            except:
                pass

        # Calculate the HSV and the mask for the image flow, we dont need the histogram therefore _
        # mask is either red color or high saturated pixels for back_proj
        hsv_image, self.mask = hsv_process(cv_image)

        # If we know that the object is red and circular.
        if self.target_redcircle:
            # Mask the red color
            masked_image = cv2.bitwise_and(hsv_image, hsv_image, mask=self.mask)
            cv2.imshow('Masked Image', masked_image)
            cv2.waitKey(2)
            # Blurring for Hough Circles
            self.image_ready = cv2.GaussianBlur(self.mask, (7,7), 1, 1) # Use mask instead of masked_image because masked_image has 3 channels


    def service_callback(self, req):
        #request = req
        if req.mode == 'detect_targets':
            try:
                circles = hough_circles(self.image_ready)
            except Exception as e:
                traceback.print_exc()
            if circles is not None:
                    try:
                        for i in range(circles.shape[0]):
                            x, y, r = float(circles[i,0]),float(circles[i,1]),float(circles[i,2])
                            print("Detected target with pixel coords: ", x, y)
                            X, Y, Z = triangulate(self.diameter, x, y, r)
                            print("3D coords: ", X, Y, Z)
                            room_coords = X, Y, Z
                        return True, x, y, r, None

                    except Exception as e:
                        traceback.print_exc()
                        return False, None, None, None, None
                    
        elif req.mode == 'detect_hits':
            x = req.x
            y = req.y
            r = req.r
            inv_mask = cv2.bitwise_not(self.mask)
            r_eff = (r-2)*np.sin(np.pi/4)
            extr_circle = inv_mask[y-r_eff:y+r_eff, x-r_eff:x+r_eff]
            im2, contours, hierarchy = cv2.findContours(extr_circle, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #cv2.imshow('Contours', im2)
            #cv2.waitKey(2)
            diffs = []
            diff = 0
            if contours:
                for contour in contours:
                    for i in range(contour.shape[0]):
                        cont = contour[i, :]
                        diff += abs(cont[0,0] - cont[0,1])
                        diffs.append(diff)
            return True, None, None, None, diff
                

#Main function to initialize the shooting node and service
if __name__ == '__main__':
    vision = NaoVision()
    rospy.spin()