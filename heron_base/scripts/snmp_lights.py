#!/usr/bin/env python

import rospy
import threading
import time
from std_msgs.msg import Bool
from pysnmp.hlapi import *

status = False
ip_addr = "192.168.1.9"

<<<<<<< HEAD
#function to ping SNMP device manager
def ping():
    global status

    #fetch sysObjectID from Device Manager
=======
def ping():
    global status
>>>>>>> 7eba38b... Added WiFi watcher back in commit
    errorIndication, errorStatus, errorIndex, varBinds = next(getCmd(SnmpEngine(),
      CommunityData('public', mpModel=0),
      UdpTransportTarget((ip_addr, 161)),
      ContextData(),
      ObjectType(ObjectIdentity('SNMPv2-MIB', 'sysObjectID', 0)))
    )

<<<<<<< HEAD
    # if error in connection, log the error and send "false" to /has_wifi topic
    if errorIndication or errorStatus:
    	status = False
    	rospy.logerr("Ping to base station failed")

    # if sysObjectID begins with 1.3.6.1.4.1.21703, this is a microhard device
    # so we must be connected
    elif str(varBinds[0][1]).startswith("1.3.6.1.4.1.21703"):
    	status = True
    	rospy.loginfo("Ping to base station succeeded")

    # if objectID isn't what we want, this isn't a microhard device
    # and something's wrong with WiFi
    else:
    	status = False
    	rospy.logerr("Ping to base station failed")

# task to continuously ping SNMP device manager
def ping_loop():
    while not rospy.is_shutdown():
        ping()

# main node function
=======
    if errorIndication or errorStatus:
	status = False
	rospy.logerr("Ping to base station failed")
    elif str(varBinds[0][1]).startswith("1.3.6.1.4.1.21703"):
	status = True
	rospy.loginfo("Ping to base station succeeded")
    else:
	status = False
	rospy.logerr("Ping to base station failed")

def ping_loop():
        while not rospy.is_shutdown():
                ping()


>>>>>>> 7eba38b... Added WiFi watcher back in commit
def lights():
	global ip_addr

	rospy.init_node("wifi_check", anonymous=True)

<<<<<<< HEAD
    # depending on the wireless setup (i.e. base station or lack of), the IP address to ping changes
	if rospy.has_param("~ip_addr"):
		ip_addr = rospy.get_param("~ip_addr")

    # the lights controller expect a "True" signal on the /has_wifi topic at least 2Hz
    # the ping time is slower than this, so the SNMP pinging is placed on a separate thread
    # and it updates a global variable
=======
	if rospy.has_param("~ip_addr"):
		ip_addr = rospy.get_param("~ip_addr")

>>>>>>> 7eba38b... Added WiFi watcher back in commit
	ping_task = threading.Thread(target=ping_loop)
	ping_task.start()

	pub = rospy.Publisher("has_wifi", Bool, queue_size=1)
	rate = rospy.Rate(2)

<<<<<<< HEAD
    # loop to constantly publish SNMP status on /has_wifi topic
=======
>>>>>>> 7eba38b... Added WiFi watcher back in commit
	while not rospy.is_shutdown():
		pub.publish(Bool(status))
		rate.sleep()

if __name__ == '__main__':
	try:
		lights()
	except rospy.ROSInterruptException:
		pass
