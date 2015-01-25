/*
 * Serial.h
 *
 *  Created on: 20-Nov-2011
 *      Author: suraj
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <termios.h>

/* Serial port information for talking to components */

#define	BAUDRATE_9600	B9600
#define BAUDRATE_19200	B19200
#define BAUDRATE_38400	B38400
#define BAUDRATE_57600  B57600
#define BAUDRATE_76800	B76800
#define	BAUDRATE_115200	B115200

/* Serial data structure */
#define s8N1	0		/* 8 data bits,   no parity, 1 stop bit */
#define s8E1	1		/* 8 data bits, even parity, 1 stop bit */
#define	s8O1	2		/* 8 data bits,  odd parity, 1 stop bit */


int openSerial(char* device,int baudrate);
int writeSerial(int fd, const char *buffer, int count);
int readSerial(int fd, char *buffer, int count);

#endif /* SERIAL_H_ */
