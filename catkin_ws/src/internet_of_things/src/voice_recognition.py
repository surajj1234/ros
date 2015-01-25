#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

import pygtk
pygtk.require('2.0')
import gtk

import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()      # This is very important !
import gst

STATUS_VR_WORKING = "vr_working"

class Voice_Recognition(object):
    """GStreamer/PocketSphinx Demo Application"""

    def __init__(self):
        """Initialize a Voice Recognition object which publishes voice commands as ROS messages"""
       
        rospy.init_node('voice_recognition')
        
        rospy.on_shutdown(self.shutdown)
        
        self.statusPub = rospy.Publisher('voice_recognition/status', String, queue_size = 3)
        self.pub = rospy.Publisher('voice_recognition/voice_commands', String, queue_size = 3)
        
        self.guiSub = rospy.Subscriber("gui/vr_config", String, self.gui_config)
     
        self.init_gst()
        self.firstResult = True

    def init_gst(self):
        """Initialize the speech components"""
        
        self.pipeline = gst.parse_launch('gconfaudiosrc ! audioconvert ! audioresample '
                                         + '! vader name=vad auto-threshold=true '
                                         + '! pocketsphinx name=asr ! fakesink')
        asr = self.pipeline.get_by_name('asr')
        asr.connect('partial_result', self.asr_partial_result)
        asr.connect('result', self.asr_result)
        asr.set_property('configured', True)

        # Set languagel model and dictionary to be used
        asr.set_property('lm', "/home/suraj/ros/catkin_ws/src/internet_of_things/sphinx_config/1702.lm")
        asr.set_property('dict', "/home/suraj/ros/catkin_ws/src/internet_of_things/sphinx_config/1702.dic")
        asr.set_property("fsg", "/home/suraj/ros/catkin_ws/src/internet_of_things/sphinx_config/grammar.fsg")
        
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::application', self.application_message)

        self.pipeline.set_state(gst.STATE_PAUSED)
        rospy.sleep(1)

    def gui_config(self, msg):

        if msg.data == "demo_vr_enable":
            self.enable_VR(True)
        
        elif msg.data == "demo_vr_disable":
            self.enable_VR(False)

    def asr_partial_result(self, asr, text, uttid):
        """Forward partial result signals on the bus to the main thread."""
        
        struct = gst.Structure('partial_result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def asr_result(self, asr, text, uttid):
        """Forward result signals on the bus to the main thread."""
       
        struct = gst.Structure('result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def application_message(self, bus, msg):
        """Receive application messages from the bus."""
        
        msgtype = msg.structure.get_name()
        if msgtype == 'partial_result':
            self.partial_result(msg.structure['hyp'], msg.structure['uttid'])
        elif msgtype == 'result':
            self.final_result(msg.structure['hyp'], msg.structure['uttid'])

    def partial_result(self, hyp, uttid):
        """ """
        
        pass        

    def final_result(self, hyp, uttid):
        """Insert the final result."""
       
        print >> sys.stderr, hyp
        
        if self.firstResult == True:
            self.statusPub.publish(STATUS_VR_WORKING)
            self.firstResult = False

        self.pub.publish(hyp)

    def enable_VR(self, enable_VR):
        """ Enable or disable VR """
        
        if enable_VR == True:
            self.pipeline.set_state(gst.STATE_PLAYING)
        else:
            self.pipeline.set_state(gst.STATE_PAUSED)
            vader = self.pipeline.get_by_name('vad')
            vader.set_property('silent', True)

    def shutdown(self):
        """ Clean up """

        # Shutdown the GTK thread. 
        gtk.main_quit()

if __name__ == '__main__':

    vr = Voice_Recognition()
    gtk.main()
