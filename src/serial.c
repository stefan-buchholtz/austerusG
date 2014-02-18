/* Port configuration partially based on great example "Arduino-serial" by
 * Tod E. Kurt.
 * http://todbot.com/blog/2006/12/06/arduino-serial-c-code-to-talk-to-arduino/
 *
 * Header from Tod's source file:
 *
 * Created 5 December 2006
 * Copyleft (c) 2006, Tod E. Kurt, tod@todbot.com
 * http://todbot.com/blog/
 */


#include <stdio.h>
#include <stdlib.h> 
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#ifdef __linux__
#include <linux/termios.h>
#else
#include <termios.h>
#include <sys/ioctl.h>
#endif

#ifdef __MACH__
#include <IOKit/serial/ioss.h>
#endif 

#include "serial.h"

int set_custom_baudrate(int serial, int baud);

// Open serial port from name and baud rate and return file desciptor
int serial_init(const char* serialport, int baud) {
	int serial;
	
	// fprintf(stderr, "opening serial port\n");
	serial = open(serialport, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (serial == -1) {
		return -1;
	}

	// fprintf(stderr, "setting serial port exclusive\n");
    if (ioctl(serial, TIOCEXCL) == -1) {
		return -1;
    }
    
    // Now that the device is open, clear the O_NONBLOCK flag so subsequent I/O will block.
    // See fcntl(2) <x-man-page//2/fcntl> for details.
	// fprintf(stderr, "setting serial port to blocking mode\n");
    if (fcntl(serial, F_SETFL, 0) == -1) {
		return -1;
    }

	// set the baud rate
	speed_t brate;
	bool isCustomBaudRate = false;

	switch (baud) {
#ifdef B4800
		case 4800:
			brate=B4800;
			break;
#endif
#ifdef B9600
		case 9600:
			brate=B9600;
			break;
#endif
#ifdef B14400
		case 14400:
			brate=B14400;
			break;
#endif
#ifdef B19200
		case 19200:
			brate=B19200;
			break;
#endif
#ifdef B28800
		case 28800:
			brate=B28800;
			break;
#endif
#ifdef B38400
		case 38400:
			brate=B38400;
			break;
#endif
#ifdef B57600
		case 57600:
			brate=B57600;
			break;
#endif
#ifdef B115200
		case 115200:
			brate=B115200;
			break;
#endif
#ifdef B250000
		case 250000:
			brate=B250000;
			break;
#endif
#ifdef B230400
		case 230400:
			brate=B230400;
			break;
#endif
#ifdef B460800
		case 460800:
			brate=B460800;
			break;
#endif
#ifdef B500000
		case 500000:
			brate=B460800;
			break;
#endif
#ifdef B576000
		case 576000:
			brate=B576000;
			break;
#endif
		default:
			isCustomBaudRate = true;
	}

#ifdef __linux__
	struct termios2 toptions;
	if (ioctl(serial, TCGETS2, &toptions) < 0) {
		return -1;
	}
	if ( isCustomBaudRate ) {
		toptions.c_cflag &= ~CBAUD;
		toptions.c_cflag |= BOTHER;
		toptions.c_ispeed = baud;
		toptions.c_ospeed = baud;		
	} else {
		toptions.c_cflag &= ~CBAUD;
		toptions.c_cflag |= brate;
	}
#else
	struct termios toptions;
	// fprintf(stderr, "tcgetattr\n");
	if ( tcgetattr(serial, &toptions) < 0 ) {
		perror("tcgetattr returned error");
		return -1;
	}
	if ( !isCustomBaudRate ) {
		cfsetispeed(&toptions, brate);
		cfsetospeed(&toptions, brate);
	}
#endif

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;

	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	// turn on READ & ignore ctrl lines
	toptions.c_cflag |= CREAD | CLOCAL;

	// turn off s/w flow ctrl
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY);

	// make raw
	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	toptions.c_oflag &= ~OPOST;

	// set blocking method
	toptions.c_cc[VMIN] = SERIAL_VMIN;
	toptions.c_cc[VTIME] = SERIAL_VTIME;

#ifdef __linux__
	if (ioctl(serial, TCSETS2, &toptions) < 0) {
		perror("ioctl TCSETS2 returned error");
		return -1;
	}
#else
	// fprintf(stderr, "tcgetattr\n");
	if (tcsetattr(serial, TCSANOW, &toptions) < 0) {
		perror("tcsetattr returned error");
		return -1;
	}
#endif

#ifdef __MACH__
	if ( isCustomBaudRate ) {
		speed_t speed = baud;
	    if (ioctl(serial, IOSSIOSPEED, &speed) == -1) {
			perror("Error setting custom baud rate");
			return -1;
	    }
	}
#endif

	return serial;
}

// Read a complete line from the serial port
int serial_getline(int serial, char *buffer, int timeout) {
	int nbytes, i = 0;
	int retries = (timeout * 10) / SERIAL_VTIME;
	char byte[1];

	do {
		// Block until one character received or timeout
		nbytes = read(serial, byte, 1);

		// Return if in error state
		if (nbytes == -1)
			return -1;

		// Try again if timed out
		if (nbytes == 0) {
			retries--;
			continue;
		}

		// Otherwise add the new byte to the buffer
		buffer[i] = byte[0];
		i++;

	} while (byte[0] != '\n' && retries > 0);

	// Remove end of line characters and null terminate the string
	if (buffer[i - 2] == '\r') {
		i -= 2;
	} else {
		i -= 1;
	}

	buffer[i] = 0;

	return i;
}


