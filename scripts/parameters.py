#! /usr/bin/env python

#This file holds parameters used throughout the project modules improving readibility in main scripts.

import numpy as np
import cv2

#Communication
robotIP = "10.152.246.218"
PORT = 9559
YOUR_IP = "10.152.246.41"

#Camera
shooting_pixel = 128
angle_z = np.deg2rad(-90)
angle_x = np.deg2rad(-90)
Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0],
            [np.sin(angle_z), np.cos(angle_z), 0],
            [0, 0, 1]])
Rx = np.array([[1, 0, 0],
            [0, np.cos(angle_x), -np.sin(angle_x)],
            [0, np.sin(angle_x), np.cos(angle_x)]])
optical2cameraR = np.matmul(Rz, Rx)
optical2camera = np.eye(4)
optical2camera[:3, :3] = optical2cameraR
term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
track_window = (141, 64, 60, 53)  # Initial tracking window