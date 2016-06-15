#!/usr/bin/env python

# TCP client example
import socket
from time import sleep
from sys import stdout

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(("localhost", 29500))

speed = 0.0
speed_chg = 1.0
while 1:
    speed += speed_chg
    if abs(speed) > 90.0: speed_chg = -speed_chg
    sentence = "PYDIR,%f,%f" % (speed, 0.0 - speed)
    client_socket.send("$%s*\r\n" % sentence)
    stdout.write(".")
    stdout.flush()
    sleep(0.1)
    


    """data = client_socket.recv(512)
    if ( data == 'q' or data == 'Q'):
        client_socket.close()
        break;
    else:
        print "RECIEVED:" , data
        data = raw_input ( "SEND( TYPE q or Q to Quit):" )
        if (data <> 'Q' and data <> 'q'):
            client_socket.send(data)
        else:
            client_socket.send(data)
            client_socket.close()
            break;"""
