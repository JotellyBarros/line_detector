#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import skimage, skimage.morphology
import glob
import imutils
import numpy as np

class lineDetector:
    def __init__(self):
        
        rospy.Subscriber("/image", Image, self.imageCallback)
        self.bridge = CvBridge()

    def imageCallback(self, image_msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            rotated, binary, opening = self.imageProcess(cv2_img)
            color_mask = self.colorFilter(cv2.cvtColor(rotated.copy(), cv2.COLOR_BGR2YUV), binary)

            kernel = np.ones((5,5),np.uint8)
            opening = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)

            #--------------------------------------------------------
            color_gray = cv2.cvtColor(opening, cv2.COLOR_BGR2GRAY)
            color_gray = cv2.bilateralFilter(color_gray, 1, 1, 1)
           
            # Otsu's thresholding after Gaussian filtering
            blur = cv2.GaussianBlur(color_gray,(9,9),0)
            ret, gaussian = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            image_contour = cv2.Canny(gaussian, 0, 225)
            
            # find contours
            im2, contours, hierarchy = cv2.findContours(image_contour,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            
            kernel = np.ones((5,5),np.uint8)
            dilate = cv2.dilate(im2,kernel,iterations = 1)
            trail = cv2.morphologyEx(dilate, cv2.MORPH_OPEN, kernel)
          
            #----------------------------------------------------
            # BlobDetector
            # Setup SimpleBlobDetector parameters.
            params = cv2.SimpleBlobDetector_Params()

            # Change thresholds
            params.minThreshold = 10
            params.maxThreshold = 200

            # Filter by Area.
            params.filterByArea = True
            params.minArea = 1500

            # Filter by Circularity
            params.filterByCircularity = True
            params.minCircularity = 0.1

            # Filter by Convexity
            params.filterByConvexity = True
            params.minConvexity = 0.87

            # Filter by Inertia
            params.filterByInertia = True
            params.minInertiaRatio = 0.01

            # Create a detector with the parameters
            ver = (cv2.__version__).split('.')
            if int(ver[0]) < 3:
                detector = cv2.SimpleBlobDetector(params)
            else:
                detector = cv2.SimpleBlobDetector_create(params)

            # Detect blobs.
            keypoints = detector.detect(trail)
            im2, contours, hierarchy = cv2.findContours(trail,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            rect = self.contoursAnalysis(rotated, trail, contours)
            
    
    def imageProcess(self, src):

        rotated = imutils.rotate_bound(cv2.flip(src, 1), -180)
        orig = cv2.cvtColor(rotated.copy(), cv2.COLOR_BGR2YUV)

        image = orig.copy() 
        image[:] = np.where((orig == 0) | (orig >= 255) , 0,255)
        kernel = np.ones((5,5),np.uint8)
        
        #Morphological Transformations Opening
        opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        dilation = cv2.dilate(opening,kernel,iterations = 1)

        return rotated, image, dilation

    def colorFilter(self, src, binary):

        # create NumPy arrays from the boundaries following the BGR covention
        color_lower = np.array([1, 120, 120])
        color_upper = np.array([128, 128, 128])
    
        # find the colors within the specified boundaries and apply the mask
        color_mask = cv2.inRange(src, color_lower, color_upper)
        color = cv2.bitwise_and(src, src, mask = color_mask)

        return color

    def findContours(self,src):
        (im2, contours, hierarchy) = cv2.findContours(src.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        img_bounded = np.zeros(src.shape,np.uint8)
        contours_img = cv2.drawContours(img_bounded, contours, -1, 255, 5)
        return contours,contours_img

    def findPosition(self, x, y):
        print "X = " + str(x) + " Y = " + str(y)
        post = 0
        
        if ((x > 0) & (x < 300)):
            post = "Left"

        elif ((x > 300) & (x < 500)):
            post = "Center"

        elif (x > 500):
            post = "Right"

        else:
            print("ERROR!")

        return post


    def contoursAnalysis(self, bgr, trail, contours):

        height = bgr.shape[0]
        width = bgr.shape[1]
        min_x, min_y = width, height
        max_x = max_y = 0

        rect = bgr.copy()

        for contour in contours:
            (x,y,w,h) = cv2.boundingRect(contour)
            aspect_ratio = w/float(h)
            perimeter = cv2.arcLength(contour,True)

            # print "\n\nAspect ratio = " + str(aspect_ratio)
            # print "W = " + str(w) + " H = " +str(h)
            # print "X = " +str(x) + " Y = " +str(y)
            # print "Perimeter = " + str(perimeter)

            position = self.findPosition((x+h/2), (y+h/2))
            print "\nPosition = " + str(position)

            if (w < 50 or h < 50):
                continue

            min_x, max_x = min(x, min_x), max(x+w, max_x)
            min_y, max_y = min(y, min_y), max(y+h, max_y)
            #----------------------------------------------------
            # get a blank canvas for drawing contour
            canvas = np.zeros(bgr.shape, np.uint8)
            epsilon = 0.1*cv2.arcLength(contour,True)
            approx = cv2.approxPolyDP(contour,epsilon,True)

            cv2.drawContours(canvas, contour, -5, (0, 255, 0), 5)
            cv2.drawContours(canvas, [approx], -1, (0, 0, 255), 3)

            #----------------------------------------------------
            # rows,cols = canvas.shape[:2]
            # [vx,vy,x,y] = cv2.fitLine(contour, cv2.DIST_L12,0,0.01,0.01)
            # lefty = int((-x*vy/vx) + y)
            # righty = int(((cols-x)*vy/vx)+y)
            # #cv2.circle(canvas,(cols-1,righty),(0,lefty),(255,0,0), 2)
            cv2.circle(canvas,((x+w/2),(y+h/2)), 10, (255,0,255), -1)

            #----------------------------------------------------
            rect = cv2.rectangle(bgr.copy(), (x,y), (x+w,y+h), (255, 0, 0), 2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(rect, str(position) ,((x+h/2),(y+h/2)), font, 1, (0,0,255) , 5, cv2.LINE_AA)
            cv2.putText(canvas, str("X: " + str(int((x+w/2))) + " " + "Y: " + str(int((y+h/2)))) + " : " + str(position),((x+w/2) + 20,(y+h/2) + 20), font, 1, (180,80,200) , 5, cv2.LINE_AA)
            
            #frames_view = np.hstack([bgr,cv2.cvtColor(trail.copy(), cv2.COLOR_GRAY2BGR), rect, canvas])            
            #cv2.namedWindow('keypoints', cv2.WINDOW_NORMAL)
            #cv2.imshow("keypoints", frames_view)

            Frames_rect = np.hstack([rect])
            cv2.imshow("Frames_rect", Frames_rect)

            frames_view_canvas = np.hstack([canvas])
            cv2.imshow("Frames_canvas", frames_view_canvas)
           
            k = cv2.waitKey(1)
            # print "Key = " + str(int(k))
            if k == 27:
                cv2.destroyAllWindows()
            
        return rect 

        