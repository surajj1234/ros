#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
from PyQt4 import QtGui, QtCore

import cv2
import numpy as np

# Note : This is generated using the pyuic4 tool from our form designed in QT Designer
# The form can be found in the lib directory
# To regenerate, run pyuic4 gui_form.ui -o gui_form.py
from gui_form import *

DEMO_VR_ENABLE = "demo_vr_enable" 
DEMO_VR_DISABLE = "demo_vr_disable" 

DEMO_ML_ENABLE = "demo_ml_enable" 
DEMO_ML_DISABLE = "demo_ml_disable" 

ENABLE_DEBUG_IMAGE = "toggle_debug_image"

STATUS_VR_WORKING = "vr_working"
STATUS_MAMAROO_CONNECTED = "mamaRoo_connected"
STATUS_MAMAROO_DISCONNECTED = "mamaRoo_disconnected"
STATUS_ORIGAMI_CONNECTED = "origami_connected"
STATUS_ORIGAMI_DISCONNECTED = "origami_disconnected"

BABY_SLEEPING_STATE = "0" 
BABY_AWAKE_STATE = "1"

class DemoGUI(QtGui.QMainWindow):

    # Create a signal to alert QT thread that a new image is available from ROS
    newImageSignal = QtCore.pyqtSignal() 
    newStatusSignal = QtCore.pyqtSignal()

    def __init__(self):
        super(DemoGUI, self).__init__()
        
        self.init_gui()
        self.pixmap = None

        self.init_ros_stuff()
        self.select_demo("Monitor")

        self.vrStatus = "Not Working"
        self.origamiStatus = "Disconnected"
        self.mamaRooStatus = "Disconnected"

        self.update_status_indicators()

    def init_gui(self):

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
      
        # Connect signals to slots 
        self.newImageSignal.connect(self.update_gui_image)
        self.newStatusSignal.connect(self.update_status_indicators)

        self.ui.demoComboBox.activated[str].connect(self.select_demo)
        
        self.setWindowTitle('4moms Baby Monitor')
        self.show()

    def init_ros_stuff(self):
        rospy.init_node('gui')
        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        
        self.imageSub = rospy.Subscriber("baby_monitor/baby_monitor_image", Image, self.image_callback)
        self.vrStatusSub = rospy.Subscriber("voice_recognition/status", String, self.vr_status_callback)
        self.mamaRooStatusSub = rospy.Subscriber("mamaRoo_bt_controller/status", String, self.mamaRoo_status_callback)
        self.origamiStatusSub = rospy.Subscriber("origami_network_controller/status", String, self.origami_status_callback)
        self.motionDetectorSub = rospy.Subscriber("motion_detector/motion_status", String, self.motion_detector_callback)

        self.vrPub = rospy.Publisher('gui/vr_config', String, queue_size = 3)
        self.mlPub = rospy.Publisher('gui/ml_config', String, queue_size = 3)
    
    
    def image_callback(self, data):
        '''Receive RGB images published from the baby detector node'''
        try:
            readFrame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.pixmap = self.convert_frame(readFrame)
            self.newImageSignal.emit()

        except CvBridgeError, e:
            print(e)

    def convert_frame(self, readFrame):
        '''converts frame to format suitable for QtGui'''
        try:
            self.currentFrame = cv2.cvtColor(readFrame, cv2.COLOR_BGR2RGB)
            
            height, width = self.currentFrame.shape[:2]
            img = QtGui.QImage(self.currentFrame, width, height, QtGui.QImage.Format_RGB888)
            img = QtGui.QPixmap.fromImage(img)
            return img
        except:
            return None

    def update_gui_image(self):

        try:
            if self.pixmap is not None:
                self.ui.babyImageLabel.setPixmap(self.pixmap)
                self.ui.babyImageLabel.setScaledContents(True)
        except TypeError:
            print("No frame")
    
    def select_demo(self, text):
        
        if text == "Monitor":
            self.mlPub.publish(DEMO_ML_DISABLE)
            self.vrPub.publish(DEMO_VR_DISABLE)
        if text == "VR":
            self.mlPub.publish(DEMO_ML_DISABLE)
            self.vrPub.publish(DEMO_VR_ENABLE)
        elif text == "ML":
            self.vrPub.publish(DEMO_VR_DISABLE)
            self.mlPub.publish(DEMO_ML_ENABLE)

    def vr_status_callback(self, msg):

        if msg.data == STATUS_VR_WORKING:
            self.vrStatus = "Working"
            self.newStatusSignal.emit()

    def mamaRoo_status_callback(self, msg):

        if msg.data == STATUS_MAMAROO_CONNECTED:
            self.mamaRooStatus = "Connected"
            self.newStatusSignal.emit()
        elif msg.data == STATUS_MAMAROO_DISCONNECTED:
            self.mamaRooStatus = "Disconnected"
            self.newStatusSignal.emit()
        
    def origami_status_callback(self, msg):

        if msg.data == STATUS_ORIGAMI_CONNECTED:
            self.origamiStatus = "Connected"
            self.newStatusSignal.emit()
        elif msg.data == STATUS_ORIGAMI_DISCONNECTED:
            self.origamiStatus = "Disconnected"
            self.newStatusSignal.emit()

    def motion_detector_callback(self, msg):
       
        motionStatus = msg.data.split(",")
        
        if motionStatus[0] == BABY_AWAKE_STATE:
            self.ui.babyStateLabel.setText("BABY AWAKE")
        elif motionStatus[0] == BABY_SLEEPING_STATE:
            self.ui.babyStateLabel.setText("BABY SLEEPING")

        self.ui.timeoutLabel.setText(motionStatus[1])

    def update_status_indicators(self):
            
            if self.vrStatus == "Working":
                self.ui.vrStatusLabel.setText(self.vrStatus)
                self.ui.vrStatusLabel.setStyleSheet('color: green')

            elif self.vrStatus == "Not Working":
                self.ui.vrStatusLabel.setText(self.vrStatus)
                self.ui.vrStatusLabel.setStyleSheet('color: red')
            
            if self.mamaRooStatus == "Connected":
                self.ui.mamaRooStatusLabel.setText(self.mamaRooStatus)
                self.ui.mamaRooStatusLabel.setStyleSheet('color: green')

            elif self.mamaRooStatus == "Disconnected":
                self.ui.mamaRooStatusLabel.setText(self.mamaRooStatus)
                self.ui.mamaRooStatusLabel.setStyleSheet('color: red')
            
            if self.origamiStatus == "Connected":
                self.ui.origamiStatusLabel.setText(self.origamiStatus)
                self.ui.origamiStatusLabel.setStyleSheet('color: green')

            elif self.origamiStatus == "Disconnected":
                self.ui.origamiStatusLabel.setText(self.origamiStatus)
                self.ui.origamiStatusLabel.setStyleSheet('color: red')

    def shutdown(self):
        
        print("Exiting Demo GUI application")

def main():

    app = QtGui.QApplication(sys.argv)

    demoGui = DemoGUI()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
