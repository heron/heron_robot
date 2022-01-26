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

import os
import rospy
import subprocess
import threading
import time
from std_msgs.msg import Bool

FNULL=open(os.devnull, 'w')

status = False
ip_addr = "192.168.131.50"   # default base-station address

# ping the base station's IP address to make sure we're connected to it
def ping():
    global status

    command = ['ping', '-c', '1', ip_addr]
    status = subprocess.call(command, stdout=FNULL, stderr=subprocess.STDOUT) == 0

    if status:
        rospy.logdebug('Ping to base station {0} succeeded'.format(ip_addr))
    else:
        rospy.logerr('Ping to base station {0} failed'.format(ip_addr))

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
