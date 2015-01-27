#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import time
import numpy as np

DEMO_ML_ENABLE = "demo_ml_enable" 
DEMO_ML_DISABLE = "demo_ml_disable" 

TOGGLE_DEBUG_IMAGE = "toggle_debug_image"

WINDOW_NAME = "threshold image"

WIDTH = 640
HEIGHT = 480

class MotionDetector():

    def __init__(self):

        self.imageDisplay = False
        self.enableDemo = False

        self.init_ros_stuff()
        

        self.result = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        self.frame1gray = np.zeros((HEIGHT, WIDTH, 1), np.uint8)
        self.frame2gray = np.zeros((HEIGHT, WIDTH, 1), np.uint8)
    
    def init_ros_stuff(self):

        rospy.init_node('motion_detector')
        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        
        self.imageSub = rospy.Subscriber("baby_monitor/baby_monitor_image", Image, self.image_callback)
        self.guiConfigSub = rospy.Subscriber("gui/ml_config", String, self.gui_config_callback)
        
        self.motionStatusPub = rospy.Publisher('motion_detector/motion_status', String, queue_size = 3)

    def gui_config_callback(self, msg):

        if msg.data == TOGGLE_DEBUG_IMAGE:
            
            self.imageDisplay = not self.imageDisplay
           
            if self.imageDisplay == True:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
                cv2.startWindowThread()
       
            elif self.imageDisplay == False:
                cv2.destroyAllWindows()

        elif msg.data == DEMO_ML_ENABLE:
            self.enableDemo = True

        elif msg.data == DEMO_ML_DISABLE:
            self.enableDemo = False

    def image_callback(self, data):
        '''Receive RGB images published from the baby detector node'''
        try:
            readFrame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            self.frame1gray = cv2.cvtColor(readFrame, cv2.COLOR_BGR2GRAY)

            if self.imageDisplay == True:
                cv2.imshow(WINDOW_NAME, self.frame1gray)
                cv2.waitKey(1) & 0xFF       # 0xFF needed for 64-bit systems 
        
        except CvBridgeError, e:
            print(e)

    def shutdown(self):
        cv2.destroyAllWindows()
        cv2.waitKey(1) & 0xFF       # 0xFF needed for 64-bit systems 

if __name__ == '__main__':
    
    detector = MotionDetector()

    while not rospy.is_shutdown():
        rospy.spin()
