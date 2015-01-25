/*
 * Serial.cpp
 *
 *  Created on: 20-Nov-2011
 *      Author: suraj
 */

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Serial.h"

int openSerial(char* device,int baudrate)
{
	struct termios oldtio,newtio;
	int fd;

	fd=open(device, O_RDWR | O_NOCTTY);
	if(fd<0)
	{
		perror("serial port device\n");
		printf("%s\n",device);
		return -1;
	}

	tcgetattr(fd,&oldtio);

	memset(&newtio,'\0',sizeof(newtio));           //clear newtio for new settings

	newtio.c_cflag=baudrate|CS8|CLOCAL|CREAD;

	newtio.c_iflag=IGNPAR;
	newtio.c_oflag &= ~OPOST;

	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);


	newtio.c_cc[VTIME]=0;
	newtio.c_cc[VMIN]=1;

	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);

	return fd;
}

int writeSerial(int fd, const char *buffer, int count)
{
	int bytesLeft = count;
	int bytesWritten = 0;
    int ret;

	while(bytesLeft!=0 && (ret = write(fd, buffer + (unsigned char)bytesWritten, bytesLeft))!=0)
	{
		if(ret == -1)
			return -1;

		bytesLeft -= ret;
        bytesWritten += ret;
	}
	return bytesWritten;
}

int readSerial(int fd, char *buffer, int count)
{
	int bytesLeft = count;
	int bytesRead = 0;
    int ret;

	while(bytesLeft!=0 && (ret = read(fd, buffer + (unsigned char)bytesRead, bytesLeft))!=0)
	{
		if(ret == -1)
			return -1;

		bytesLeft -= ret;
		bytesRead += ret;
	}

	return bytesRead;
}
