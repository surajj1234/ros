#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import BTDongle
import MySerial
import time

DEFAULT_COM_PORT_LINUX = "/dev/ttyACM0"
DEFAULT_COM_PORT_WIN = 25

HANDLE_MAMAROO_WRITE = 0x1F

# Mamoroo control packet : Command type identifier
APP_CMD_BYTE = 0x43             # 'C' in hex, is the first first byte of a mamaRoo control packet			
# Mamaroo control packet : Command IDs 
APP_CMD_POWER = 0x01		# Turn power on/off (1/0)	
APP_CMD_MOTION_ENABLE = 0x02	# Motion enable/disable command (1/0)
APP_CMD_SOUND_ENABLE = 0x03	# Mute/unmute command (0/1)
APP_CMD_MOTION = 0x04		# Set motion type (1-5)
APP_CMD_SOUND = 0x05		# Set sound type (1-5)
APP_CMD_SPEED = 0x06		# Set speed (1-5)
APP_CMD_VOLUME = 0x07		# Set volume (1-5)
APP_CMD_DEBUG = 0x08		# Enable/disable debug packets (1/0) 
APP_CMD_GET_BT_ID = 0x09	# Get BT ID 
APP_CMD_CDTIMER = 0x0A		# Set countdown timer (time in minutes)

# mamaRoo Tester State machine states
DEVICE_INIT = 1
DISCOVER = 2
CONNECT = 3
DO_NOTHING = 4

STATUS_MAMAROO_CONNECTED = "mamaRoo_connected"
STATUS_MAMAROO_DISCONNECTED = "mamaRoo_disconnected"


class MamaRoo_BTController():

    def __init__(self):
        """Control a mamaRoo via Bluetooth using a TI CC2540 USB dongle"""

        rospy.init_node('mamaRoo_bt_controller')
        rospy.on_shutdown(self.shutdown)
        
        rospy.Subscriber('voice_recognition/voice_commands', String, self.speech_callback)
        self.statusPub = rospy.Publisher('mamaRoo_bt_controller/status', String, queue_size = 3)

        self.init_BT()

    def init_BT(self):

        self.serial = MySerial.MySerial(DEFAULT_COM_PORT_LINUX)
        self.dongle = BTDongle.BTDongle(self.serial.comms)

        self.dongle.setup_device_init_callback(self.device_init_callback)
        self.dongle.setup_device_info_callback(self.device_info_callback)
        self.dongle.setup_discovery_done_callback(self.discovery_done_callback)
        self.dongle.setup_link_established_callback(self.link_established_callback)
        self.dongle.setup_link_terminated_callback(self.link_terminated_callback)
        self.dongle.setup_notification_received_callback(self.notify_callback)

        self.runProgram = True
        self.state = DEVICE_INIT
        self.testUnitFound = False
        self.testUnitConnected = False

    # BT Dongle callbacks-------------------------------------
    
    def device_init_callback(self, status):
        
        if status == 0:     # Success
           
            print("Device Init done")
            self.state = DISCOVER
 
            # start scanning for devices
            self.dongle.do_discovery()
        
        else:
            print("Device Init failure !")
            self.shutdown()

    def device_info_callback(self, status):
        
        # Check if device discovered is a debug mamaRoo unit
        for i in range(0, len(self.dongle.peripheral_list)):
            if self.dongle.peripheral_list[i].localName == "mamaRoo":
                self.testUnitFound = True
                self.testUnitID = i
                # Cancel scan since unit has been found
                self.dongle.do_cancel_discovery()

    def discovery_done_callback(self, status):
        
        if self.testUnitFound == True:
            print("Connecting...")
            self.dongle.do_establish_link(self.testUnitID)
            self.state = CONNECT

        else:
            print("Device Discovery Failure : No \"mamaRoo\" found! Retrying...")
            
            self.state = DISCOVER
            self.dongle.do_discovery()
 
    def link_established_callback(self, status):

        if status == 0:     # Success
            
            # Enable notifications from the custom mamaRoo data characteristic
            self.set_notify(True)

            self.statusPub.publish(STATUS_MAMAROO_CONNECTED)
            self.testUnitConnected = True
            self.state = DO_NOTHING
        else:
            print("Link Establish Failure : Unable to connect to \"mamaRoo\". Retrying...")
            self.state = DISCOVER
            self.dongle.do_discovery()

    def link_terminated_callback(self, status):
        print("Link terminated")
        self.testUnitConnected = False
        self.statusPub.publish(STATUS_MAMAROO_DISCONNECTED)

        self.state = DISCOVER
        self.dongle.do_discovery()

    def notify_callback(self, handle, value, valueStr):
        
        #print("Handle = %d, Value = %s") % (handle, valueStr)
        if handle == 31:        # Handle of the custom mamaRoo data characterisitc
            self.parse_received_packet(value)
    
    # Test app internals--------------------------------------

    def run_state_machine(self):

        while self.runProgram == True:
            
            # State machine
            if self.state == DEVICE_INIT:       # Wait until the BT dongle initialization is complete
                time.sleep(0.1) 
            if self.state == DISCOVER:          # Wait until the BT dongle completes a scan
                time.sleep(0.1)
            if self.state == CONNECT:           # wait until a link is established with the peripheral
                time.sleep(0.1)
            if self.state == DO_NOTHING:        # do nothing, wait here until the program is terminated
                time.sleep(1)

    def shutdown(self):
        self.runProgram = False
        
        self.dongle.do_terminate_link()
        time.sleep(0.1)
            
        # Shutdown serial and BT dongle modules
        self.serial.close()
        self.dongle.close()

        print("Exiting mamaRoo control program...")
    
    # Higher level functionality-------------------------------
    
    def parse_received_packet(self, value):
        
        pass        

    def set_notify(self, enableFlag):
        handle = 32     
        notifyData = bytearray()
 
        if enableFlag == True:
           notifyData.append(0x01)
        elif enableFlag == False:
           notifyData.append(0x00)
        
        notifyData.append(0x00)
        self.dongle.do_write_char_value(handle, notifyData)     # "01:00 to enable, 00:00 to disable"

    def speech_callback(self, msg):

        if msg.data == "MAMAROO":
            self.mamaRoo_send_command(APP_CMD_POWER, 1)
        if msg.data == "START MOVING":
            self.mamaRoo_send_command(APP_CMD_MOTION_ENABLE, 1)
        if msg.data == "TERMINATE MOTION":
            self.mamaRoo_send_command(APP_CMD_MOTION_ENABLE, 0)
        if msg.data == "SPEED FIVE":
            self.mamaRoo_send_command(APP_CMD_SPEED, 5)
        if msg.data == "MOTION KANGAROO":
            self.mamaRoo_send_command(APP_CMD_MOTION, 2)

    def mamaRoo_send_command(self, command, value):

        handle = HANDLE_MAMAROO_WRITE       # Handle for "data pipeline" characteristic on the mamaRoo BT device

        valuePacket = bytearray()
        valuePacket.append(APP_CMD_BYTE)
        valuePacket.append(command)
        valuePacket.append(value)
        
        if self.testUnitConnected == True:
            self.dongle.do_write_char_value(handle, valuePacket)

if __name__ == '__main__':
    
    controller = MamaRoo_BTController()

    while not rospy.is_shutdown():
        controller.run_state_machine()
