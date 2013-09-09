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
 *  File: main.c
 *  Desc: Main entry point for the vserial program.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <poll.h>

#include <vector>

#include "port.h"

using namespace std;

// Global for term signal handler:
vector<Port*> service_ports;
vector<Port*> client_ports;
vector<struct pollfd> pollfds;

// Need to execute atexit() when a fatal signal is received
void handle_term(int signum) {
    vector<Port*>::iterator iter;
    for(iter = service_ports.begin(); iter != service_ports.end(); ++iter) {
        delete *iter;
    }
    for(iter = client_ports.begin(); iter != client_ports.end(); ++iter) {
        delete *iter;
    }
   
    fflush(stderr);
    fflush(stdout);

    /* If we abort, we still need to do the above cleanup, but we should still
     * exit on an abort to signal that horrible things happened.
     * So, we restore default abort() behaviour and then re-raise the abort */
    if( signum == SIGABRT ) {
        signal(SIGABRT, SIG_DFL);
        abort();
    }

    printf("\nExiting virtual-serial\n");
    exit(0);
}


void print_help(const char* executable_name) {
    printf("Usage: \n"
           "1: %s [service] [client] [options]\n"
           "2: %s --service [ports] --client [ports] [options]\n"
           "\n"
           "1: Creates a pair of terminals and pipes them to each other\n"
           "[service] and [client] are port specification strings.\n"
           "\n"
           "2: Similar to 1, but it is possible to have multiple ports at either end\n"
           "of the connection.  [ports] is a list of port specification strings.\n"
           "\n"
           "Port specification: \n"
           "serial:devname,baudrate - Use a real serial port\n"
           "pty:path                - Create a pty, with symlink at given path\n"
           "path                    - Equivalent to pty:path\n"
           "\n"
           "With multiple ports, a char received on a given service port will be\n"
           "transmitted to all client ports, but not to other service ports. The\n"
           "converse holds for chars received on client ports.\n"
           "\n"
           "Options:\n"
           "--service [ports]   Specify additional service ports.  Each of [ports]\n"
           "                    is a port specification string.  If this option is\n"
           "                    specified multiple times, port lists will be added\n"
           "--client [ports]    Same as --service, but for client ports.\n"
           ,
           executable_name, executable_name);
}


void pass_data(vector<Port*>& from_ports, vector<Port*>& to_ports) {
    vector<Port*>::iterator from_iter, to_iter;
    for( from_iter = from_ports.begin(); from_iter < from_ports.end(); ++from_iter)
    {
        if ((*from_iter)->poll_revents() == POLLIN) {
            char buff[1024];
            int count = (*from_iter)->read(buff, sizeof(buff));
            for( to_iter = to_ports.begin();
                 to_iter < to_ports.end(); ++to_iter )
            {
                (*to_iter)->write(buff, count);
            }
        }
    }
}


void update_blocking() 
{
    poll(&pollfds[0], pollfds.size(), 1000);
    pass_data(service_ports, client_ports); 
    pass_data(client_ports, service_ports); 
}


int main(int argc, char *argv[]) {
    /* This program is often used as a daemon, with stdout piped to a log. Bash
     * seems to set pretty aggressive buffering on redirected stdout, which is
     * really annoying, since recent output ends up not being shown.  So, we
     * want to make sure stdout is using a relatively reasonable buffering 
     * policy (linebuffered, max 64 chars) */
    setvbuf(stdout, 0, _IOLBF, 64);

    if( argc < 3 ) {
        print_help(argv[0]);
        return 1;
    }
   
    /* Program parameters, to be parsed from options */
    vector<const char*> service_strs;
    vector<const char*> client_strs;

    // TODO: We've probably hit critical mass for optarg
    // (or making this a ROS node and using parameters...)
    int cur_arg = 1;

    /* If the first argument doesn't start with --, we assume that the first two
     * arguments are the service and client ports, respectively */
    if( strncmp(argv[1], "--", 2) ) {
        service_strs.push_back(argv[1]);
        client_strs.push_back(argv[2]);
        cur_arg = 3;
    }

    /* Now, handle any additional options */
    for(; cur_arg<argc; ++cur_arg) {
        if( !strcmp(argv[cur_arg], "--service") ) {
            /* Add all following args not starting with -- to the service list */
            while( (cur_arg+1 < argc) && strncmp(argv[cur_arg+1],"--",2) ) {
                ++cur_arg;
                if (argv[cur_arg][0] != '_') {
                    service_strs.push_back(argv[cur_arg]);
                }
            }
        } else if( !strcmp(argv[cur_arg], "--client") ) {
            /* Add all following args not starting with -- to the client list */
            while( (cur_arg+1 < argc) && strncmp(argv[cur_arg+1],"--",2) ) {
                ++cur_arg;
                if (argv[cur_arg][0] != '_') {
                     client_strs.push_back(argv[cur_arg]);
                }
            }
        } else {
            fprintf(stderr,"Unrecognized option: %s\n", argv[cur_arg]);
            exit(1);
        }
    }

    /* Check to make sure we have enough ports */
    if( service_strs.empty() ) {
        fprintf(stderr, "At least one service port must be specified!\n");
        exit(1);
    }
    if( client_strs.empty() ) {
        fprintf(stderr, "At least one client port must be specified!\n");
        exit(1);
    }


    /******* Begin actual initialization ********/

    // want to clean up and exit() on fatal signal:
    signal(SIGINT, handle_term);
    signal(SIGTERM, handle_term);
    signal(SIGABRT, handle_term);

    /* Open the ports, build pollfd array. */
    {
        pollfds.resize(service_strs.size() + client_strs.size());
        vector<struct pollfd>::iterator pollfd_iter = pollfds.begin();
        vector<const char*>::iterator str_iter;

        for(str_iter = service_strs.begin(); str_iter != service_strs.end(); ++str_iter, ++pollfd_iter) {
            service_ports.push_back(Port::create(*str_iter, &*pollfd_iter));
        }
        for(str_iter = client_strs.begin(); str_iter != client_strs.end(); ++str_iter, ++pollfd_iter) {
            client_ports.push_back(Port::create(*str_iter, &*pollfd_iter));
        }
    }

    while(1) {
        update_blocking();
    }
}

