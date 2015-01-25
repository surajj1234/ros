from globalDefs import rxQueue
import serial 
import threading 
import BTPeripheral
import time

# RX state machine parameters

PKT_START_BYTE = 0x04       # Indicates that this is an HCI event
PKT_EVENT_CODE = 0xFF       # Indicates that this HCI Event is in a vendor specific format

RX_MAX_SIZE = 256 

RX_GET_SOF = 1
RX_GET_CMD_TYPE = 2
RX_GET_LEN = 3
RX_GET_DATA = 4

class BTDongle():

    def __init__(self, serialPort):
        
        self.comms = serialPort
        
        self.terminate = False
        self.serial_debug = False 
        self.hci_debug = False 

        self.rxPacket = [] 
        self.rxState = RX_GET_SOF

        self.deviceReady = False 
        self.peripheral_list = [] 
        self.connHandle = 0x0000         
        self.connectionActive = False 
        
        self.device_init_callback = None
        self.device_info_callback = None
        self.discovery_done_callback = None
        self.link_established_callback = None
        self.link_terminated_callback = None        
        self.notification_callback = None

        # Disconnect from previously existing connection, if present
        self.do_terminate_link()
 
        # Initialize the dongle
        self.do_device_init()

        self.start_threads()

    # HCI Commands--------------------------------------------

    def do_device_init(self):

        if self.hci_debug == True:
            print("Initializing TI dongle")
        
        packet = bytearray()
        packet.append(0x01) # HCI Command
        packet.append(0x00) # 0xFE00 GAP_DeviceInit
        packet.append(0xFE)
        packet.append(0x26) # Data Length
        packet.append(0x08) # Profile Role - Central
        packet.append(0x0F) # Max scan responses

        # IRK, CSRK, SignCounter
        config = bytearray('\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00')
        
        packet = packet + config
       
        if self.hci_debug == True:
            print("-" * 30)
            print("Type: 0x01 (Command)")
            print("OpCode : 0xFE00 (GAP_DeviceInit)")
            print("Data Length : 0x26 bytes")
            print("Profile Role : 0x08 (Central)")
            print("Max Scan Resps : 0x0F")
            print("-" * 30)

        self.send_bytes(packet)

    def do_discovery(self):

        if self.connectionActive == False:

            # Clear list of peripheral devices
            self.peripheral_list = []

            packet = bytearray()
            packet.append(0x01) # HCI command
            packet.append(0x04) # 0xFE04 GAP_DeviceDiscoveryRequest
            packet.append(0xFE)
            packet.append(0x03) # Datalength, well static
            packet.append(0x03) # Mode
            packet.append(0x01) # Enable Name Mode
            packet.append(0x00) # Disable Whitelist
        
            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x01 (Command)")
                print("OpCode : 0xFE04 (GAP_DeviceDiscoveryRequest)")
                print("Data Length : 0x03 bytes")
                print("Mode : 0x03 (All)")
                print("Active Scan : 0x01 (Enable)")
                print("White List : 0x00 (Disable)") 
                print("-" * 30)

            self.send_bytes(packet) 

        else:
            print("Warning : Cannot discover new devices until existing connection is terminated")

    def do_cancel_discovery(self):
        
        packet = bytearray()
	packet.append(0x01) # HCI command
	packet.append(0x05) # 0xFE05 GAP_DeviceDiscoveryRequest
        packet.append(0xFE)
        packet.append(0x00) # Datalength 
    
        if self.hci_debug == True:
            print("-" * 30)
            print("Type: 0x01 (Command)")
            print("OpCode : 0xFE05 (GAP_DeviceDiscoveryCancel)")
            print("Data Length : 0x00 bytes")
            print("-" * 30)

        
        self.display_devices_found()    
        self.send_bytes(packet) 
        
    def do_establish_link(self, device = 0):
        
        if device >= 0 and device < len(self.peripheral_list):
            
            bt_address = self.peripheral_list[device].btID

            packet = bytearray()
	    packet.append(0x01) # HCI command
	    packet.append(0x09) # 0x09FE GAP_EstablishLinkRequest
            packet.append(0xFE)
            packet.append(0x09) # Datalength
            packet.append(0x00) # HighDutyCycle
            packet.append(0x00) # WhiteList
            packet.append(0x00) # AddrTypePeer (Public)
	    
            # Split the BT address into bytes
            strings = bt_address.split(":")
            strings.reverse()

            for item in strings:
                packet.append(int(item, 16))
            
            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x01 (Command)")
                print("OpCode : 0xFE09 (GAP_EstablishLinkRequest)")
                print("Data Length : 0x09 bytes")
                print("HighDutyCycle : 0x00 (Disable)")
                print("WhiteList : 0x00 (Disable)")
                print("AddrTypePeer : 0x00 (Public)") 
                print("PeerAddr : %s") % strings 
                print("-" * 30)

            self.send_bytes(packet) 

        else:
            print("Warning: Connect error - argument must be a valid discovered device")

    def do_terminate_link(self):

            packet = bytearray()
            packet.append(0x01) # HCI command
            packet.append(0x0A) # 0xFE0A GAP_TerminateLinkRequest
            packet.append(0xFE)
            packet.append(0x02) # Datalength
            packet.append(self.connHandle & 0xFF) # Conn Handle low byte
            packet.append((self.connHandle >> 8) & 0xFF) # Conn Handle high byte

            self.send_bytes(packet)

    def do_set_gap_params(self, minConnIntvl=24, maxConnIntvl=32, slaveLatency=0, connTimeout=500):
        '''minConnIntvl=24 maxConnIntvl=32 slaveLatency=0 connTimeout=500'''
                
        # Verify given paramters
        paramsValid = True

        if minConnIntvl < 6 or minConnIntvl > 3200:
            paramsValid = False
        if maxConnIntvl < 6 or maxConnIntvl > 3200:
            paramsValid = False
        if slaveLatency < 0 or slaveLatency > 499:
            paramsValid = False
        if connTimeout < 10 or connTimeout > 3200:
            paramsValid = False
        if minConnIntvl > maxConnIntvl:
            paramsValid = False

        if paramsValid == True:
            
            packet = bytearray()
	    packet.append(0x01) # HCI command
	    packet.append(0x30) # 0xFE30 GAP_SetParam
            packet.append(0xFE)
            packet.append(0x03) # Datalength
            packet.append(0x15) # Minimum link layer connection interval
            packet.append(minConnIntvl & 0xFF) # min conn interval low byte
            packet.append((minConnIntvl >> 8) & 0xFF) # min conn interval high byte
            
	    packet.append(0x01) # HCI command
	    packet.append(0x30) # 0xFE30 GAP_SetParam
            packet.append(0xFE)
            packet.append(0x03) # Datalength
            packet.append(0x16) # Maximum link layer connection interval
            packet.append(maxConnIntvl & 0xFF) # max conn interval low byte
            packet.append((maxConnIntvl >> 8) & 0xFF) # max conn interval high byte
 	    
            packet.append(0x01) # HCI command
	    packet.append(0x30) # 0xFE30 GAP_SetParam
            packet.append(0xFE)
            packet.append(0x03) # Datalenigth
            packet.append(0x1A) # Slave Latency
            packet.append(slaveLatency & 0xFF) # slave latency low byte
            packet.append((slaveLatency >> 8) & 0xFF) # slave latency high byte
 	    
            packet.append(0x01) # HCI command
	    packet.append(0x30) # 0xFE30 GAP_SetParam
            packet.append(0xFE)
            packet.append(0x03) # Datalength
            packet.append(0x19) # Link layer connection supervision timeout
            packet.append(connTimeout & 0xFF) # Conn timeout low byte
            packet.append((connTimeout >> 8) & 0xFF) # Conn timeout high byte
 
            self.send_bytes(packet)
    
    def do_write_char_value(self, handle, value):
        if self.connectionActive == True:
            
            packet = bytearray()
            packet.append(0x01) # HCI Command
            packet.append(0x92) # 0xFD92 GATT_WriteCharValue
            packet.append(0xFD)

            pduLen = 4 + len(value)
            packet.append(pduLen) # Data Length
            packet.append(self.connHandle & 0xFF) # Conn Handle low byte
            packet.append((self.connHandle >> 8) & 0xFF) # Conn Handle high byte
            packet.append(handle & 0xFF) # Handle low byte
            packet.append((handle >> 8) & 0xFF) # Handle high byte

            for i in range(0, len(value)):
                packet.append(value[i])

            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x01 (Command)")
                print("OpCode : 0xFD92 (GATT_WriteCharValue)")
                print("Data Length : 0x%02x bytes") % pduLen
                print("ConnHandle : 0x%04x") % self.connHandle
                print("Handle : 0x%04x (%d)") % (handle, handle)
                
                valueStr = ":".join("%02x" % x for x in value)
                print("Value : %s") % valueStr
                print("-" * 30)

            self.send_bytes(packet)
        else:
            print("Warning : Write failure, there is no active connection")

    def do_read_char_value(self, handle):
        
        if self.connectionActive == True:
        
            packet = bytearray()
            packet.append(0x01) # HCI Command
            packet.append(0x8A) # 0xFD8A GATT_ReadCharValue
            packet.append(0xFD)
            packet.append(0x04) # Data Length
            packet.append(self.connHandle & 0xFF) # Conn Handle low byte
            packet.append((self.connHandle >> 8) & 0xFF) # Conn Handle high byte
            packet.append(handle & 0xFF) # Handle low byte
            packet.append((handle >> 8) & 0xFF) # Handle high byte

            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x01 (Command)")
                print("OpCode : 0xFD8A (GATT_ReadCharValue)")
                print("Data Length : 0x04 bytes")
                print("ConnHandle : 0x%04x") % self.connHandle
                print("Handle : 0x%04x (%d)") % (handle, handle)
                print("-" * 30)

            self.send_bytes(packet)
        else:
            print("Warning : Read failure, there is no active connection")

    # HCI Events-----------------------------------------------

    def process_event(self, packet):

        opCode = (packet[1] << 8) | packet[0]

        if opCode == 1536:              #0600 GAP_DeviceInitDone
            self.process_gap_device_init_done_event(packet)
        elif opCode == 1291:            #0x050B (ATT_ReadRsp)
            self.process_att_readResponse_event(packet)
        elif opCode == 1299:            #0x0513 (ATT_WriteRsp)
            self.process_att_writeResponse_event(packet)
        elif opCode == 1663:            #0x067F (GAP HCI Extension Command Status)
            self.process_gap_hci_ext_command_status_event(packet)
        elif opCode == 1537:            #0x0601 (GAP_DeviceDiscoveryDone)
            self.process_gap_discovery_done_event(packet)
        elif opCode == 1549:            #0x060D (GAP_DeviceInformationEvent)
            self.process_gap_device_information_event(packet) 
        elif opCode == 1541:            #0x0605 (GAP_EstablishLink)
            self.process_gap_establish_link_event(packet)
        elif opCode == 1542:            #0x0606 (GAP_TerminateLink)
            self.process_gap_terminate_link_event(packet)
        elif opCode == 1307:            #0x051b (ATT HandleValueNotification)
            self.process_att_handleValueNotification_event(packet) 
        else:
            print("Warning: Received HCI event with unknown opcode")
    
    def process_gap_device_init_done_event(self, args):
        
        status = args[2]
        
        if status == 0: # Success
            bt_address = args[3:9]
            bt_address.reverse()
            self.dongleAddress = ":".join("%02x" % x for x in bt_address)
            dataLength = args[9]
            numDataPackets = args[10] << 8 | args[11]
            self.IRK = ":".join("%02x" % x for x in args[12:28])
            self.CSRK = ":".join("%02x" % x for x in args[28:44])
            
            self.deviceReady = True

            # Send GAP connection parameters to the dongle
            self.do_set_gap_params()
            
            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x04 (Event)")
                print("EventCode : 0xFF (HCI_LE_Ext_Event)")
                print("Event : 0x0600 (GAP_DeviceInitDone)")
                print("Status : 0x00 (Success)")
                print("Device Address : %s") % self.dongleAddress
                print("Data Packet Length : %d bytes") % dataLength
                print("Num data packets : %d") % numDataPackets
                print("IRK : %s") % self.IRK
                print("CSRK : %s") % self.CSRK
                print("-" * 30)
        else:
            print("Warning: Unable to initialize dongle. Exit the program and retry...")

        if self.device_init_callback is not None:
            self.device_init_callback(status)

    def process_gap_discovery_done_event(self, args):
        status = args[2]

        if status == 0: # Success

            numDevicesFound = args[3]
            
            # Get BT address of each device
            for i in range(0, numDevicesFound):
                startByte = 4 + ((i * 8) + 2)       # Format is EventType(1), AddrType(1), BT Address(6)
                bt_address = args[startByte:startByte + 6]
                bt_address.reverse()
                btAddress = ":".join("%02x" % x for x in bt_address)
            
            # Print devices found to screen
            self.display_devices_found()
            
        else:
            if self.hci_debug == True:
                print("Warning: Received HCI event Device discovery Done failure")
        
        if self.discovery_done_callback is not None:
            self.discovery_done_callback(status)

    def process_gap_device_information_event(self, args):
        status = args[2]

        if status == 0:     # Success
            
            eventType = args[3]
            if eventType == 0x00:      # Connectable Undirected Advertisement
                eventTypeString = "Connectable Undirected Advertisement"
            elif eventType == 0x01:    # Connectable Directed Advertisement 
                eventTypeString = "Connectable Directed Advertisement"
            elif eventType == 0x02:    # Non connectable Undirected Advertisement 
                eventTypeString = "Non connectable Undirected Advertisement"
            elif eventType == 0x03:    # Scan Request 
                eventTypeString = "Scan Request"
            elif eventType == 0x04:     # Scan response packet
                eventTypeString = "Scan Response"
            elif eventType == 0x05:     # Connection Request
                eventTypeString = "Connect_Request"
            elif eventType == 0x06:    # Scannable Undirected Advertising
                eventTypeString = "Scannable Undirected Advertising"
            else:
                eventTypeString = ""
            
            addrType = args[4]
            
            bt_address = args[5:11]
            bt_address.reverse()
            deviceAddress = ":".join("%02x" % x for x in bt_address)
            
            # Check if this is a new device; if yes then add to list of devices
            newDevice = True

            for i in range(0, len(self.peripheral_list)):
                
                if self.peripheral_list[i].btID == deviceAddress:
                    newDevice = False
                    break

            if newDevice == True:   # Add device 
                peripheral = BTPeripheral.BTPeripheral()
                peripheral.btID = deviceAddress
                peripheral.btAddrType = addrType

                # Set advertisement properties of the peripheral device
                if eventType == 0x00:
                    peripheral.connectable = True
                    peripheral.undirected = True
                    peripheral.scannable = False
                elif eventType == 0x01:
                    peripheral.connectable = True
                    peripheral.undirected = False
                    peripheral.scannable = False
                elif eventType == 0x02:
                    peripheral.connectable = False
                    peripheral.undirected = True
                    peripheral.scannable = False
                elif eventType == 0x06:
                    peripheral.connectable = False
                    peripheral.undirected = True
                    peripheral.scannable = True

                self.peripheral_list.append(peripheral)
                print(".")

            # The other 2 packet types have a different format, so we ignore them from this point on
            if eventType == 0x00 or eventType == 0x02 or eventType == 0x04 or eventType == 0x06:  
                rssi = args[11]
                dataLength = args[12]

                deviceData = args[13:len(args)]
                deviceDataStr =  ":".join("%02x" % x for x in deviceData) 
            
                # Handle scan response packet 
                if eventType == 0x04:       # Scan Response event
                    nameLength = deviceData[0]  # length of local name, can be max 15 bytes long
                    nameBytes = deviceData[2:2 + nameLength - 1]
                    nameStr = "".join(chr(i) for i in nameBytes)
                    
                    for i in range(0, len(self.peripheral_list)):
                        if self.peripheral_list[i].btID == deviceAddress:
                            self.peripheral_list[i].scanRspData = deviceData
                            self.peripheral_list[i].localName = nameStr

                # Handle advertising packet
                if eventType == 0x00:       # Connectable Undirected Advertisement
                    for i in range(0, len(self.peripheral_list)):
                        if self.peripheral_list[i].btID == deviceAddress:
                            self.peripheral_list[i].advData = deviceData


            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x04 (Event)")
                print("EventCode : 0xFF (HCI_LE_Ext_Event)")
                print("Event : 0x060D (GAP_Device_Information)")
                print("Status : 0x00 (Success)")
                print("Event Type : 0x%x (%s)") % (eventType, eventTypeString)
                print("AddrType : 0x%x") % addrType
                print("Address : %s") % deviceAddress
                
                if eventType == 0x00 or eventType == 0x02 or eventType == 0x04 or eventType == 0x06:  
                    rssi = args[11]
                    dataLength = args[12]

                    deviceData = args[13:len(args)]
                    deviceDataStr =  ":".join("%02x" % x for x in deviceData) 

                    print("RSSI : %d") % rssi
                    print("Data Length : %d") % dataLength
                    print("Data : %s") % deviceDataStr
                
                print("-" * 30)
        else:
            if self.hci_debug == True:
                print("Warning: Received unsuccessful device information HCI event")

        if self.device_info_callback is not None:
            self.device_info_callback(status)

    def process_gap_establish_link_event(self, args):
        
        status = args[2]

        if status == 0:     # Success
            addrType = args[3]

            bt_address = args[4:10]
            bt_address.reverse()
            deviceAddress = ":".join("%02x" % x for x in bt_address)
            
            self.connHandle = ((args[11] | 0x0000) << 8) | args[10] 

            connInterval = ((args[13] | 0x0000) << 8) | args[12] 
            connLatency = ((args[15] | 0x0000) << 8) | args[14]
            connTimeout = ((args[17] | 0x0000) << 8) | args[16]

            clockAccuracy = args[18]

            self.connectionActive = True
            print("Connected")

            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x04 (Event)")
                print("EventCode : 0xFF (HCI_LE_Ext_Event)")
                print("Event : 0x0605 (GAP_Establish_Link)")
                print("Status : 0x00 (Success)")
                print("AddrType : 0x%x") % addrType
                print("Address : %s") % deviceAddress
                print("ConnHandle : 0x%04x") % self.connHandle
                print("ConnInterval : 0x%04x (%d)") % (connInterval, connInterval)
                print("ConnLatency : 0x%04x (%d)") % (connLatency, connLatency)
                print("ConnTimeout : 0x%04x (%d)") % (connTimeout, connTimeout)
                print("ClockAccuracy : 0x%02x") % clockAccuracy
                print("-" * 30)
        else:
            print("Warning: Received HCI establish link failure event: Unable to connect to device")

        if self.link_established_callback is not None:
            self.link_established_callback(status)

    def process_gap_terminate_link_event(self, args):
        
        status = args[2]

        if status == 0:     # Success
            self.connectionActive = False
            print("Disconnected")

        if self.link_terminated_callback is not None:
            self.link_terminated_callback(status)

    def process_att_handleValueNotification_event(self, args):
        
        status = args[2]

        if status == 0:     # Success
            
            connHandle = ((args[4] | 0x0000) << 8) | args[3]    
            pduLen = args[5] # Length of data payload
            
            handle = ((args[7] | 0x0000) << 8) | args[6]
            
            value = bytearray()
            for i in range(0, pduLen - 2):
                value.append(args[8 + i])
 
            valueStr = ":".join("%02x" % x for x in value)

            if self.notification_callback is not None:
                self.notification_callback(handle, value, valueStr)

            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x04 (Event)")
                print("EventCode : 0xFF (HCI_LE_Ext_Event)")
                print("Event : 0x051B (ATT_HandleValueNotification)")
                print("Status : 0x00 (Success)")
                print("ConnHandle : 0x%04x") % connHandle
                print("PduLen : 0x%02x") % pduLen
                print("Handle : 0x%04x (%d)") % (handle, handle)
                print("Value : %s") % valueStr
                print("-" * 30)
        else:
            print("Warning : ATT_handle_value_notify failure HCI event received")


    def process_att_readResponse_event(self, args):
        
        status = args[2]

        if status == 0:     # Success
            connHandle = ((args[4] | 0x0000) << 8) | args[3]    
            pduLen = args[5] # Length of data payload

            value = bytearray()
            for i in range(0, pduLen):
                value.append(args[6 + i])
            
            valueStr = ":".join("%02x" % x for x in value)
            print("Char Value is: %s") % valueStr

            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x04 (Event)")
                print("EventCode : 0xFF (HCI_LE_Ext_Event)")
                print("Event : 0x050B (ATT_ReadRsp)")
                print("Status : 0x00 (Success)")
                print("ConnHandle : 0x%04x") % connHandle
                print("PduLen : 0x%02x") % pduLen
                print("Value : %s") % valueStr
                print("-" * 30)

        else:
            print("Warning : Unable to read characteristic from peripheral GATT server")

    def process_att_writeResponse_event(self, args):
        
        status = args[2]

        if status == 0:     # Success
            
            connHandle = ((args[4] | 0x0000) << 8) | args[3]    
            pduLen = args[5] # Length of data payload
           
            if self.hci_debug == True:
                print("-" * 30)
                print("Type: 0x04 (Event)")
                print("EventCode : 0xFF (HCI_LE_Ext_Event)")
                print("Event : 0x0513 (ATT_WriteRsp)")
                print("Status : 0x00 (Success)")
                print("ConnHandle : 0x%04x") % connHandle
                print("PduLen : 0x%02x") % pduLen
                print("-" * 30)
        else:
            print("Warning : Unable to write to characteristic of peripheral GATT server")

    def process_gap_hci_ext_command_status_event(self, args):
        pass

    # Callbacks---------------------------------------------------

    def setup_device_init_callback(self, callback):
        '''Add a callback to be executed after a device init event is received from the dongle'''

        self.device_init_callback = callback

    def setup_device_info_callback(self, callback):
        '''Add a callback to be executed after a device information event is received from the dongle'''

        self.device_info_callback = callback

    def setup_discovery_done_callback(self, callback):
        '''Add a callback to be executed after a discovery done event is received from the dongle'''

        self.discovery_done_callback = callback

    def setup_link_established_callback(self, callback):
        '''Add a callback to be executed after a link established event is received from the dongle'''

        self.link_established_callback = callback

    def setup_link_terminated_callback(self, callback):
        '''Add a callback to be executed after a link terminated event is received from the dongle'''

        self.link_terminated_callback = callback


    def setup_notification_received_callback(self, callback):
        '''Add a callback to be executed if a notification is received from a \
           a GATT characteristic corresponding to given handle'''

        self.notification_callback = callback


    # Thread handling--------------------------------------------
    
    def start_threads(self):
        
        self.thread_list = []
        
        # Create threads
        self.thread_list.append(threading.Thread(target = self.rx_state_machine_thread))
        
        # Set threads as daemons
        for thread in self.thread_list:
            thread.daemon = True

        # Start threads
        for thread in self.thread_list:
            thread.start()

        # This blocks the calling thread until the thread whose join() method is called is terminated
        #for thread in thread_list:
            #thread.join()
    
    # Serial port functions---------------------------------------

    def set_hci_debug(self, args = False):
        self.hci_debug = args

    def set_serial_debug(self, args = False):
        self.serial_debug = args

    def send_string(self, writePacket):
        
        if self.serial_debug == True:
            print("TX  " + writePacket)
        
        self.comms.write(str(writePacket).encode())

    def send_bytes(self, writePacket):
        
        if self.serial_debug == True:
            
            hexString = ":".join("%02x" % x for x in writePacket)
            print("TX  " + hexString)

        self.comms.write(writePacket)
 
    def rx_state_machine_thread(self):

        while self.terminate == False:
            

            # Check if there is data to send, blocking if empty
            rxByte = ord(rxQueue.get()[0])
            
            # RX state machine

            if self.rxState == RX_GET_SOF:

                if rxByte == PKT_START_BYTE:                         # Stay in this state until a valid frame start byte is received
                    self.rxState = RX_GET_CMD_TYPE

            elif self.rxState == RX_GET_CMD_TYPE:                   

                if rxByte == PKT_EVENT_CODE:                    # Make sure that only TI's vendor specific HCI event packets are accepted
                    self.rxState = RX_GET_LEN
                else:
                    self.rxState = RX_GET_SOF
            
            elif self.rxState == RX_GET_LEN:

                if rxByte > 0 and rxByte <= RX_MAX_SIZE:     # Make sure that length byte is valid

                    self.rxLen = rxByte                             # This decrements to zero on receipt of successive bytes until the last byte of frame is received
                    self.rxState = RX_GET_DATA

                else:
                    self.rxState = RX_GET_SOF

            elif self.rxState == RX_GET_DATA:

                self.rxPacket.append(rxByte)
                self.rxLen = self.rxLen - 1 
                
                # Check for end of packet
                if self.rxLen == 0:
                    
                    if self.serial_debug == True:
            
                        hexString = ":".join("%02x" % x for x in self.rxPacket)
                        print("RX  " + hexString)
                    
                    self.process_event(self.rxPacket)

                    # Empty buffer for next frame
                    self.rxPacket = []
                    self.rxState = RX_GET_SOF

    # Helper functions-----------------------------------------

    def display_devices_found(self):
        '''Display the peripheral devices found'''

        print ("Devices found : %d") % len(self.peripheral_list) 

        for i in range (0, len(self.peripheral_list)):
            print("%d %s %s") % (i, self.peripheral_list[i].btID, self.peripheral_list[i].localName)

    # Clean up-------------------------------------------------

    def close(self):
        ''' Shutdown BT module'''

        self.terminate = True
        time.sleep(0.1)        
 
