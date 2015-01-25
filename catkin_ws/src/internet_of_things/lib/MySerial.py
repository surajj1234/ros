#Imports
import serial
import threading
from Queue import Queue
import os
from globalDefs import rxQueue
import time

BAUDRATE = 115200
TIMEOUT = 0                 # Timeout for serial read, in seconds, 0 for non-blocking reads

class MySerial():

    def __init__(self, comPort):
        '''Initialize serial module'''

        self.terminate = False
        self.baudRate = BAUDRATE

        self.comms = serial.Serial(comPort, self.baudRate, timeout = TIMEOUT, rtscts = 0)

        self.start_threads()

    def start_threads(self):
        
        self.thread_list = []

        # Create threads
        self.thread_list.append(threading.Thread(target = self.com_rx_thread))
        
        # Set threads as daemons
        for thread in self.thread_list:
            thread.daemon = True

        # Start threads
        for thread in self.thread_list:
            thread.start()

        # This blocks the calling thread until the thread whose join() method is called is terminated
        #for thread in thread_list:
            #thread.join()


    def close(self):
        ''' Shutdown serial module'''

        self.terminate = True
        time.sleep(0.1)
        self.comms.close()
        
    def com_rx_thread(self):

        while self.terminate == False:

            # Do a non-blocking read on the serial port
            byteChar = self.comms.read(1)
            
            # Check if a byte was read
            if(byteChar != b''):

                # Add to receive queue
                rxQueue.put(byteChar)
