#!/usr/bin/env python
# Software License Agreement (BSD)
#
# @author    Guy Stoppi <gstoppi@clearpathrobotics.com>
# @copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import threading
import time
from std_msgs.msg import Bool
from pysnmp.hlapi import *

status = False
ip_addr = "192.168.1.9"


#function to ping SNMP device manager
def ping():
    global status

    #fetch sysObjectID from Device Manager
    errorIndication, errorStatus, errorIndex, varBinds = next(getCmd(SnmpEngine(),
      CommunityData('public', mpModel=0),
      UdpTransportTarget((ip_addr, 161)),
      ContextData(),
      ObjectType(ObjectIdentity('SNMPv2-MIB', 'sysObjectID', 0)))
    )

    # if error in connection, log the error and send "false" to wireless/connected topic
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
def lights():
	global ip_addr

	rospy.init_node("wifi_check", anonymous=True)

    # depending on the wireless setup (i.e. base station or lack of), the IP address to ping changes
	if rospy.has_param("~ip_addr"):
		ip_addr = rospy.get_param("~ip_addr")

    # the lights controller expect a "True" signal on the wireless/connected topic at least 2Hz
    # the ping time is slower than this, so the SNMP pinging is placed on a separate thread
    # and it updates a global variable
	ping_task = threading.Thread(target=ping_loop)
	ping_task.start()

	pub = rospy.Publisher("wireless/connected", Bool, queue_size=1)
	rate = rospy.Rate(2)

    # loop to constantly publish SNMP status on /has_wifi topic
	while not rospy.is_shutdown():
		pub.publish(Bool(status))
		rate.sleep()

if __name__ == '__main__':
	try:
		lights()
	except rospy.ROSInterruptException:
		pass
