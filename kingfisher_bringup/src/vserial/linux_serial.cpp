 /**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™ 
 *
 *  File: linux_serial.cpp
 *  Desc: Linux-compatible serial commands for linking with generic functions
 *        defined in serial.h 
 *  Auth: M. Hansen, R. Gariepy
 *
 *  Copyright (c) 2010, Clearpath Robotics, Inc. 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com 
 *
 */

#include "serial.h"  /* Std. function protos */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>  /* Malloc */


int OpenSerial(int* fd, const char* port_name)
{
	*fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (*fd == -1) {
		fprintf(stderr, "Unable to open %s\n\r", port_name);
		return -3;
	} 

    // Verify it is a serial port
    if (!isatty(*fd)) {
        close(*fd);
	    fprintf(stderr, "%s is not a serial port\n", port_name);
        return -3;
    }

	return *fd;
}

int SetupSerial(int fd, int baudrate)
{
    struct termios options;

    // Get the current options for the port...
    tcgetattr(fd, &options);

    // 8 bits, 1 stop, no parity
    options.c_cflag = 0;
    options.c_cflag |= CS8;         // 8-bit input

    // Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);

    /* Set baud rate (default is 115200.
     * There isn't a clear mapping from the integral baud rate
     * to the termios speed_t.  Cases will need to be added
     * for each baud we want to support. */
    if( (baudrate == 0) || (baudrate==115200) ) {
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
    } else if( (baudrate==57600) ) {
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
    } else if( (baudrate==38400) ) {
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
    } else if( (baudrate==19200) ) {
        cfsetispeed(&options, B19200);
        cfsetospeed(&options, B19200);
    } else if( (baudrate==9600) ) {
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
    } else {
        // Requested baud not supported
        return -1;
    }

    // No input processing
    options.c_iflag = 0;

    // No output processing
    options.c_oflag = 0;

    // No line processing
    options.c_lflag = 0;

    // read timeout
    options.c_cc[VMIN]  = 0;    // non-blocking
    options.c_cc[VTIME] = 1;    // always return after 0.1 seconds

    // Set the new options for the port...
    tcsetattr(fd, TCSAFLUSH, &options);

    return 0;
}
