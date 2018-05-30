#!/usr/bin/env python

import rospy
import threading
import time
from std_msgs.msg import Bool
from pysnmp.hlapi import *

status = False
ip_addr = "192.168.1.9"

def ping():
    global status
    errorIndication, errorStatus, errorIndex, varBinds = next(getCmd(SnmpEngine(),
      CommunityData('public', mpModel=0),
      UdpTransportTarget((ip_addr, 161)),
      ContextData(),
      ObjectType(ObjectIdentity('SNMPv2-MIB', 'sysObjectID', 0)))
    )

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


def lights():
	global ip_addr

	rospy.init_node("wifi_check", anonymous=True)

	if rospy.has_param("~ip_addr"):
		ip_addr = rospy.get_param("~ip_addr")

	ping_task = threading.Thread(target=ping_loop)
	ping_task.start()

	pub = rospy.Publisher("has_wifi", Bool, queue_size=1)
	rate = rospy.Rate(2)

	while not rospy.is_shutdown():
		pub.publish(Bool(status))
		rate.sleep()

if __name__ == '__main__':
	try:
		lights()
	except rospy.ROSInterruptException:
		pass
