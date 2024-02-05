'''This file contains necessary functions for ROI extraction, image processing, circle detection and triangulation. 
They are kept in another file to allow readability in the main script. Only those functions are chosen, which does not depend on self of the class and whose
byproducts are not necessary.'''

import cv2
import numpy as np

camera_matrix_top = np.array([[551.543059, 0, 327.382898],
                                       [0, 553.736023, 225.026380],
                                       [0, 0, 1]], dtype=np.float32)

def extract_target():
    '''Extract the ROI out of the saved image.'''
    
    image_path = "/workspaces/hrs_ws/src/PROJECT/images/image4.png"
    image = cv2.imread(image_path)
    r=cv2.selectROI("Image with ROI", image, showCrosshair=True)
    # Crop image 
    cropped_image = image[int(r[1]):int(r[1]+r[3]),  
                        int(r[0]):int(r[0]+r[2])] 
# Display cropped image 
    cv2.imwrite("/workspaces/hrs_ws/src/PROJECT/images/croppedImg4.png", cropped_image)
    return

def hough_circles(ready_image):
    '''Compute the circular shaped objects through hough circles.
    Args:
        ready_image: The processed image, It is essential that it is 1D. For red circle it is the mask of red color; otherwise it is back projection mask.'''
    rows = ready_image.shape[0]
    circles = cv2.HoughCircles(ready_image, cv2.HOUGH_GRADIENT, 1, rows/8,
                            param1=100, param2=30,
                            minRadius=5, maxRadius=50)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        # print('Circle Image Coordinates: ')
        # print(circles[0, :])
        # print('\n')
        for i in circles[0, :]:
            center = (i[0], i[1])
            cv2.circle(ready_image, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(ready_image, center, radius, (255, 0, 255), 3)
            #cv2.imshow('Image with circles', ready_image)
            #cv2.waitKey(2)
    else:
        return False
    
    #cv2.imshow("Circular shapes top", ready_image)
    #cv2.waitKey(3)
    if circles is not None:
        return circles[0, :]
    else:
        return False
    
def hsv_process(image, color='red'):
    """Process the images either for red colour extraction or for the mean/cam shift algorithms.
    
    Args:
        image: The image which needs to be processed.
        target_redcircle: The boolean variable determines the masking algorithm. if True, then the red color is extracted if False, then low satured pixels are masked.
    Return:
        hsv_image: Image converted to hsv.
        mask: The mask.
        s_hist: Histogram of the image given the mask."""

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    if color == 'red':
        # Mask the red colors
        mask1 = cv2.inRange(hsv_image, (170, 190, 190), (179, 255, 255))
        mask2 = cv2.inRange(hsv_image, (0, 190, 190), (15, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        # cv2.imshow('Masked Image R', mask)
        # cv2.waitKey(2)
    elif color == 'green':
        # Mask the low saturated and low valued pixels with threshold
        mask = cv2.inRange(hsv_image, (35, 50, 50), (80, 255, 255))
        # cv2.imshow('Masked Image G', mask)
        # cv2.waitKey(2)

    return hsv_image, mask

def triangulate(diameter, x, y, r):
    '''Triangulates the 3D Coordinates of the Target.
    
    Args:
        diameter: The diameter of the target.
        circle: The houghCirlce return.
    Return:
        (X,Y,Z): The 3D Roomcoordinates.'''

    radius = diameter/2
    # circle[0,2] is the radius in ppixels.
    Z1 = camera_matrix_top[0,0] * radius / r / 2
    Z2 = camera_matrix_top[1,1] * radius / r / 2
    Z = min([Z1, Z2])
    X = Z / camera_matrix_top[0,0] * (x - camera_matrix_top[1,2])
    Y = Z / camera_matrix_top[1,1] * (y - camera_matrix_top[1,2])
    return X, Y, Z