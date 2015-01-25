/*
 * SerialComms.h
 *
 *  Created on: 05-Mar-2012
 *      Author: suraj
 */

#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#define IMU_BAUDRATE BAUDRATE_9600
#define IMU_SERIAL_PORT "/dev/ttyACM0"               // COM1(RX/TX)  :Connected to Arduino

#define BUFFER_RX_SIZE 32

#include <string.h>

class SerialComms
{

public:
	SerialComms();

	int findSerialPort();
	int openSerialPort();
	void closeSerialPort();

	int sendData(const char *buffer, int count);
	int getData();

private:

	int fileDescriptor;
	char serialPort[10];
	char bufferRx[BUFFER_RX_SIZE];

    	void parsePacket();
};

#endif /* SERIALCOMMS_H */




