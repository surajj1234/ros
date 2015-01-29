#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import time
import numpy as np
import sys
import math

DEMO_ML_ENABLE = "demo_ml_enable" 
DEMO_ML_DISABLE = "demo_ml_disable" 

WINDOW_NAME = "output image"

WIDTH = 640
HEIGHT = 480

MOTION_THRESHOLD = 30

BABY_SLEEPING_STATE = "0" 
BABY_AWAKE_STATE = "1"

MAMAROO_SHUTOFF_TIME_S = 16         # Shutoff mamaRoo is no motion detected by baby for this time
STARTUP_DELAY_S = 5                 # Delay processing until this time after program start
PUBLISH_RATE_S = .2                 # Publish every 1 second
SHUTOFF_WAIT_TIME = 5               # After mamaRoo stops, wait for this period to allow toy bar to stabilize

MAMAROO_CMD_START_MOVING = "START MOVING" 
MAMAROO_CMD_STOP_MOVING = "STOP MOVING"
MAMAROO_CMD_WAKEUP = "POWER ON"
MAMAROO_CMD_POWER_OFF = "POWER OFF"

class MotionDetector():

    def __init__(self):

        self.imageDisplay = False 
        self.enableDemo = False
        self.firstRun = True
        self.motionThreshold = MOTION_THRESHOLD
        self.babyState = BABY_SLEEPING_STATE
        self.lastMotionDetectionTime = time.time()
        self.lastPublishTime = time.time()
        self.lastStopTime = time.time()

        self.result = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        self.frame1gray = np.zeros((HEIGHT, WIDTH, 1), np.uint8)
        self.frame2gray = np.zeros((HEIGHT, WIDTH, 1), np.uint8)
   
        self.width = WIDTH
        self.height = HEIGHT
        self.nb_pixels = self.width * self.height
        self.programStartTime = time.time()    #Hold timestamp of the last detection
    
        if self.imageDisplay == True:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
            cv2.startWindowThread()
       
        self.init_ros_stuff()
         
    def init_ros_stuff(self):

        rospy.init_node('motion_detector')
        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        
        self.imageSub = rospy.Subscriber("baby_monitor/baby_monitor_image", Image, self.image_callback)
        self.guiConfigSub = rospy.Subscriber("gui/ml_config", String, self.gui_config_callback)
        
        self.motionStatusPub = rospy.Publisher('motion_detector/motion_status', String, queue_size = 3)
        self.mamaRooPub = rospy.Publisher('motion_detector/mamaRoo_commands', String, queue_size = 3)

    def gui_config_callback(self, msg):

        if msg.data == DEMO_ML_ENABLE:
            self.enableDemo = True
            # Power on mamaRoo
            self.mamaRooPub.publish(MAMAROO_CMD_WAKEUP)

        elif msg.data == DEMO_ML_DISABLE:
            # Power off mamaRoo
            self.mamaRooPub.publish(MAMAROO_CMD_POWER_OFF)
            self.enableDemo = False
   
    def image_callback(self, data):
        '''Receive RGB images published from the baby detector node'''
        try:
            readFrame = self.bridge.imgmsg_to_cv2(data, "bgr8")
          
            timeSinceStart = time.time() - self.programStartTime

            # Don't process for a few seconds to let camera stabilize
            if timeSinceStart < STARTUP_DELAY_S:
                pass
 
            elif self.firstRun == True:
                self.frame1gray = cv2.cvtColor(readFrame, cv2.COLOR_RGB2GRAY)
                self.firstRun = False
            
            else:
                self.update(readFrame)

        except CvBridgeError, e:
            print(e)

    def update(self, newFrame):
       
        # Pre-processing of image : grayscale conversion, filtering, thresholding and frame differencing  
        self.process_image(newFrame)
      
        currentTime = time.time() 

        # Check resulting image for motion activity
        motionDetected =  self.somethingHasMoved() 
       
        if motionDetected == True:
            self.lastMotionDetectionTime = currentTime
            
        timeSinceLastMotion = currentTime - self.lastMotionDetectionTime
        timeSinceLastPublish = currentTime - self.lastPublishTime
        timeSinceLastShutoff = currentTime - self.lastStopTime

        self.mamaRooTimeoutLeft = MAMAROO_SHUTOFF_TIME_S - timeSinceLastMotion 
     
        # State machine to control the mamaRoo
        if self.babyState == BABY_AWAKE_STATE:
            if timeSinceLastMotion > MAMAROO_SHUTOFF_TIME_S:
                
                if self.enableDemo == True:
                    # Stop mamaRoo
                    self.mamaRooPub.publish(MAMAROO_CMD_STOP_MOVING)
                    self.lastStopTime = time.time()

                self.babyState = BABY_SLEEPING_STATE

        if self.babyState == BABY_SLEEPING_STATE:

            self.mamaRooTimeoutLeft = 0
           
            # Wait for some time after shutoff to allow camera to stabilize after mamaRoo stops
            if timeSinceLastShutoff > SHUTOFF_WAIT_TIME:

                if motionDetected == True:

                    if self.enableDemo == True:
                        # Activate mamaRo
                        self.mamaRooPub.publish(MAMAROO_CMD_START_MOVING)

                    # Publish new state
                    self.babyState = BABY_AWAKE_STATE

        if timeSinceLastPublish > PUBLISH_RATE_S:
            self.lastPublishTime = currentTime
            motionStatus = self.babyState + "," + str(int(self.mamaRooTimeoutLeft))
            self.motionStatusPub.publish(motionStatus)

    def process_image(self, frame):
        self.frame2gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
 
        #Absdiff to get the difference between to the frames
        self.result = cv2.absdiff(self.frame1gray, self.frame2gray)
     
        #Remove the noise and do the threshold
        self.result = cv2.blur(self.result, (5,5))
        self.result = cv2.morphologyEx(self.result, cv2.MORPH_OPEN, None)
        self.result = cv2.morphologyEx(self.result, cv2.MORPH_CLOSE, None)
        retVal, self.result  = cv2.threshold(self.result, 10, 255, cv2.THRESH_BINARY_INV)

        if self.imageDisplay == True:
            cv2.imshow(WINDOW_NAME, self.result)
            cv2.waitKey(1) & 0xFF       # 0xFF needed for 64-bit systems 
        
        # Save current frame for next update
        self.frame1gray = self.frame2gray.copy()

    def somethingHasMoved(self):
        
        nb = self.nb_pixels - cv2.countNonZero(self.result)    #Will hold the number of black pixels

        avg = (nb * 100.0) / self.nb_pixels     #Calculate the average of black pixel in the image
        
        if avg > self.motionThreshold:           #If over the ceiling trigger the alarm
            return True
        else:
            return False

    def shutdown(self):
        if self.imageDisplay == True:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    
    detector = MotionDetector()

    while not rospy.is_shutdown():
        rospy.spin()
