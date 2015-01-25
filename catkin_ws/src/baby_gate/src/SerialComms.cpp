/*
 * CommsIMU.cpp
 *
 *  Created on: 05-Mar-2012
 *      Author: suraj
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <dirent.h>

#include <SerialComms.h>
#include <Serial.h>

#define TIMEOUT 10            // poll timeout, in seconds

SerialComms::SerialComms()
{

	for(int i = 0; i < BUFFER_RX_SIZE; i++)
	{
		bufferRx[i] = '\0';
	}
}

int SerialComms::findSerialPort()
{
	int ret;
	DIR *dir;
	struct dirent *entry;

	if ((dir = opendir("/dev/")) == NULL)
		return -1;

	while ((entry = readdir(dir)))				//runs till all Serial ports are found and appends them in list
	{
		ret = -1;
		//if (strncmp("ttyUSB", entry->d_name, 6) == 0)
		if(strncmp("ttyACM", entry->d_name, 6) == 0)
		{
			sprintf(serialPort ,"/dev/%s", entry->d_name);
			ret = 1;
			break;
		}
	}
	closedir(dir);
	return ret;
}

int SerialComms::openSerialPort()
{
	int ret = findSerialPort();

	if(ret == 1)
	{
		if((fileDescriptor = openSerial(serialPort, IMU_BAUDRATE)) == -1)
			return -1;
	}
	else if(ret == -1)
	{
		if((fileDescriptor = openSerial(IMU_SERIAL_PORT, IMU_BAUDRATE)) == -1)
			return -1;
	}
	return 1;
}

int SerialComms::sendData(const char *buffer, int count)
{
    int ret = 0;		
    ret = writeSerial(fileDescriptor, buffer, count);
    return ret;
}

void SerialComms::closeSerialPort()
{
	close(fileDescriptor);
}

/* Reads data received over serial port from the Arduino
 * Returns 1 on success, -1 if error, 0 if there is a timeout */

int SerialComms::getData()
{
/*	struct pollfd fds;

	int retValue;
	fds.fd = fileDescriptor;
	fds.events = POLLIN;

	retValue = poll(&fds, 1, TIMEOUT * 1000);

	if(retValue == -1)
	{
		perror("Poll serial port: IMU");
		return -1;
	}

	if(!retValue)
	{
		printf("Serial port read Timeout: IMU");
		return 0;
	}

	if(fds.revents & POLLIN)
	{
		char check[2];
		do
		{
		if(readSerial(fileDescriptor, check, 1) == -1)
		{
			perror("Unable to read serial port : IMU packet header");
			return -1;
		}
		}while(check[0] != '!');

		if(readSerial(fileDescriptor, bufferRx, BUFFER_RX_SIZE - 1) == -1)
		{
			perror("Unable to read serial port: IMU packet");
			return -1;
		}
	}
	if(bufferRx[BUFFER_RX_SIZE - 2] != '*')
	{
		//printf("Invalid IMU packet\n");
		return -1;
	}

	parsePacket();
	return 1;
*/
}

void SerialComms::parsePacket()
{

}

