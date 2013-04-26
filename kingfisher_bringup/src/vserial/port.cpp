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
 *  File: port.cpp
 *  Desc: Provides the Port class, which acts as one of the two endpoints
 *        of the vserial utility.  Currently, a port may be either a 
 *        pseudo terminal or a real serial port.
 *  Auth: Iain Peet
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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <termios.h>

#include <iostream>
#include <sstream>
using namespace std;

#include "port.h"
#include "serial.h"

PortException::PortException(const char* con, const char* msg) :
    conn_str(con), message(msg)
{
    fprintf(stderr,"PortException: %s: %s\n", conn_str, message);
    fflush(stderr);
}


Port::Port(const char* conn_string, struct pollfd* pollfd_ptr)
    : m_conn_string(conn_string), m_pollfd_ptr(pollfd_ptr)
{
    if (m_pollfd_ptr == 0) {
        throw new PortException(m_conn_string, "pollfd object not specified.");
    } else {
        m_pollfd_ptr->fd = -1;
        m_pollfd_ptr->events = POLLIN;
    }
}

Port::~Port() {
    /* Trivial destructor.  But if you inline this in the class definition,
     * you'll find that gcc spits a bunch of undefined vtable references
     * in your face. */
}

/**
 * Create a Port instance appropriate to a given connection string.
 * @param conn_string   The connection string.  Known forms:
 *                      "serial:devname,baud" - describes a real serial endpoint
 *                      "pty:path" - describe a pty endpoint
 *                      "path" - fallback if none of the above match, a pty endpoint 
 * @return An appropriate Port instance
 */
Port* Port::create(const char *conn_string, struct pollfd* pollfd_ptr) {
    if( ! strncmp(conn_string, "serial:", 7) ) {
        return new SerialPort(conn_string, pollfd_ptr);
    } else if( !strncmp(conn_string, "pty:", 4) ) {
        return new PtyPort(conn_string + 4, pollfd_ptr);
    } else {
        return new PtyPort(conn_string, pollfd_ptr);
    }
}
    
short Port::poll_revents() {
    return m_pollfd_ptr->revents;
}

/**
 * Create a pseudo terminal enpoint.
 * @param path  Path where a symlink to the pty device should be created.
 */
PtyPort::PtyPort(const char* path, struct pollfd* pollfd_ptr) : Port(0, pollfd_ptr), m_path(path), m_slavefd(-1)
{
    /* Make sure no file already exists at path */
    if( ! access(m_path, F_OK) ) {
        throw new PortException(m_path, "File already exists");
    }

    _create_pty();
}

/**
 * Close this pseudo terminal endpoint.
 */
PtyPort::~PtyPort()
{
    unlink(m_path);
    if(m_pollfd_ptr->fd >= -1) close(m_pollfd_ptr->fd);
    if(m_slavefd >= -1) close(m_slavefd);
}

/**
 * Creates a pseudo terminal device and sets the symlink
 * at m_path to point to it
 */
void PtyPort::_create_pty()
{
    /* Create the PTY */
    m_pollfd_ptr->fd = posix_openpt( O_RDWR | O_NOCTTY | O_NDELAY);
    if( m_pollfd_ptr->fd == -1 ) {
        throw new PortException(m_path, strerror(errno));
    }

    if( grantpt(m_pollfd_ptr->fd) ) {
        throw new PortException(m_path, strerror(errno));
	}
	if( unlockpt(m_pollfd_ptr->fd) ) {
        throw new PortException(m_path, strerror(errno));
	}

    /* Disable echoing */
    struct termios pty_attr;
    if( tcgetattr(m_pollfd_ptr->fd, &pty_attr) ) {
        throw new PortException(m_path, strerror(errno));
    }
    cfmakeraw(&pty_attr);
    if( tcsetattr(m_pollfd_ptr->fd, TCSANOW, &pty_attr) ) {
        throw new PortException(m_path, strerror(errno));
    }

    /* Obtain the name of the slave device file */
	const char *slave_name = (const char*)ptsname(m_pollfd_ptr->fd);
    if( !slave_name ) {
        throw new PortException(m_path, strerror(errno));
	}

    /* Open the slave device 
     * We never ever actually do anything with this.  This is a hack; it 
     * ensures that there is always an open file descriptor to the pty, so
     * it doesn't get closed when a client process closes the slave */
    m_slavefd = open(slave_name, O_RDWR | O_NOCTTY);

    /* Create a symlink to the slave device file */
    // Remove any old symlink
    int retval = unlink( m_path );
    if( (retval) && (errno!=ENOENT) ) {
        throw new PortException(m_path, strerror(errno));
    }
    retval = symlink(slave_name, m_path);
    if(retval) {
        throw new PortException(m_path, strerror(errno));
    }

    printf("\nCreated pty: %s --> %s\n", m_path, slave_name);
}

/**
 * Re-creates the pty, if a user process has closed it
 */
void PtyPort::_respawn_pty()
{
    if( m_pollfd_ptr->fd >= 0 ) close(m_pollfd_ptr->fd);
    _create_pty();
}

/**
 * Reads a single char from this PTY.  May respawn the pty if it has been
 * closed by a user process. 
 * @param ch    A pointer to a single char.  If a char is available, it will
 *              be written here.
 */
ssize_t PtyPort::read(void *buf, size_t count)
{
    if(m_pollfd_ptr->fd == -1 ) throw new PortException(m_path, "Attempted read from invalid pty");

    int retval = ::read(m_pollfd_ptr->fd, buf, count);
    if( (retval < 0) && (errno!=EAGAIN) ) {
        /* I/O error, attempt to respawn pty */ 
        _respawn_pty();
        return 0;
    } 
    if( retval <= 0 ) {
        // No data
        return 0;
    }
    return 1;
}

/**
 * Writes a single char to this PTY.  May respawn the pty if it has been
 * closed by a user process.
 * @param ch    The char to write
 */
ssize_t PtyPort::write(void *buf, size_t count)
{
    if(m_pollfd_ptr->fd == -1) throw new PortException(m_path, "Attempted write to invalid pty");

    int retval = ::write(m_pollfd_ptr->fd, buf, count);
    if( (retval < 0) && (errno!=EAGAIN) ) {
        /* I/O error, attempt to respawn pty */
        _respawn_pty();
    }
}

/** 
 * Open a serial port using a given connection strin.
 * @param conn_str  The connection string, of the form "serial:devname,baudrate".
 *                  The baudrate element is optional; baud rate defaults to 115200
 */
SerialPort::SerialPort(const char* conn_str, struct pollfd* pollfd_ptr)
    : Port(conn_str, pollfd_ptr)
{
    if( strncmp(conn_str, "serial:", 7) ) 
        throw new PortException(conn_str, "Bad serial connection string");

    /* Parse out the device name */
    const int devname_ofst = 7;
    char portname[128];
    // Dev name ends with the comma, or end of string
    int i;
    for(i=devname_ofst; conn_str[i]!='\0' && conn_str[i]!=','; ++i) {
        if( i>= 128 )
            throw new PortException(conn_str, "Buffer overrun reading device name");

        portname[i-devname_ofst] = conn_str[i];
    }
    portname[i-devname_ofst] = '\0';

    /* Parse out the baud rate */
    int baudrate = 115200;
    // If nothing followed the dev name, we just use default baud
    if( (conn_str[i] != '\0') || (conn_str[i+1]=='\0') ) {
        i++;  // at beginning of baud rate
        baudrate = atoi(conn_str+i);
    }

    printf("Opening serial port: %s, b%d\n", portname, baudrate);
    if( OpenSerial(&m_pollfd_ptr->fd, portname) < 0) {
        perror("OpenSerial");
        throw new PortException(conn_str, "Failed to open device.");
    }

    if( SetupSerial(m_pollfd_ptr->fd, baudrate) < 0) {
        perror("SetupSerial");
        throw new PortException(conn_str, "Failed to configure serial port.");
    }
}

/**
 * Closes the serial port.
 */
SerialPort::~SerialPort()
{
    if(m_pollfd_ptr->fd != -1) close(m_pollfd_ptr->fd);
}

ssize_t SerialPort::read(void *buf, size_t count)
{
    return ::read(m_pollfd_ptr->fd, buf, count);
}

ssize_t SerialPort::write(void *buf, size_t count)
{
    int n = 0;
    do {
        errno = 0;
	    n = ::write(m_pollfd_ptr->fd, buf, count);
    } while( errno == EAGAIN);
    return n;
}

