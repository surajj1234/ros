#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import socket
import sys
import time

SERVER_IP = "10.10.10.181"
#SERVER_IP = "localhost"

SERVER_PORT = 10000
MESSAGE_SIZE = 1 

TURN_LIGHTS_ON = 'L'
TURN_LIGHTS_OFF = 'O'
STROLLER_ACTUATE = 'A'

STATUS_ORIGAMI_CONNECTED = "origami_connected"
STATUS_ORIGAMI_DISCONNECTED = "origami_disconnected"


class Origami_Network_Controller():

    def __init__(self):
        """Control an origami via a TCP/IP connection over a LAN"""

        rospy.init_node('origami_network_controller')
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('voice_recognition/voice_commands', String, self.speech_callback)
        self.statusPub = rospy.Publisher('origami_network_controller/status', String, queue_size = 3)
        
        self.runProgram = True
        self.connStatus = False

    def create_connection(self):

        if self.runProgram == True:
                
            # Create a TCP/IP socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Connect the socket to the port where the server is listening
            server_address = (SERVER_IP, SERVER_PORT)
            print >> sys.stderr, "Connecting to %s port %s" % server_address

            result = self.sock.connect_ex(server_address)

            if result > 0:
                print >> sys.stderr, "Unable to connect, retrying..."
            else:
                print >> sys.stderr, "Connected to Origami server"
                time.sleep(3)
                self.statusPub.publish(STATUS_ORIGAMI_CONNECTED)
                self.connStatus = True
     
    def send_command(self, command):

        try:
            if self.connStatus == True:
                print >> sys.stderr, "Sending command to origami %c " % command
                self.sock.sendall(command)
        except Exception as e:
            print >> sys.stderr, 'Unable to send data to origami %s' % e.value
            self.close_socket()

    def speech_callback(self, msg):

        if msg.data == "ORIGAMI":
            self.send_command(TURN_LIGHTS_ON)
        if msg.data == "MAGICALLY OPEN":
            self.send_command(STROLLER_ACTUATE)
        if msg.data == "CLOSE STROLLER":
            self.send_command(STROLLER_ACTUATE)

    def close_socket(self):

        self.statusPub.publish(STATUS_ORIGAMI_DISCONNECTED)
        self.connStatus = False
        print >> sys.stderr, "Closing socket"
        self.sock.close()
        
    def shutdown(self):
        '''Clean up'''

        self.runProgram = False
        print >> sys.stderr, "Exiting Origami network controller"
        self.close_socket()

if __name__ == '__main__':
    
    controller = Origami_Network_Controller()

    while not rospy.is_shutdown():
        if controller.connStatus == False:
            controller.create_connection()
        


