#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_bringup')

import rospy
from std_msgs.msg import Bool
import subprocess


rospy.init_node('wifi_monitor')

pub = rospy.Publisher('has_wifi', Bool)
dev = rospy.get_param('~dev', 'wlan0')
hz = rospy.get_param('~hz', 1)
previous_error = False
previous_success = False
r = None

while not rospy.is_shutdown():
  try:
    wifi_str = subprocess.check_output(['ifconfig', dev], stderr=subprocess.STDOUT);
    if "inet addr" in wifi_str:
      pub.publish(True)
    else:
      pub.publish(False)

    if not previous_success:
      previous_success = True
      previous_error = False
      rospy.loginfo("Retrieved status of interface %s. Now updating at %f Hz." % (dev, hz))
      r = rospy.Rate(hz)

  except subprocess.CalledProcessError:
    if not previous_error:
      previous_error = True
      previous_success = False
      rospy.logerr("Error checking status of interface %s. Will try again every 10s." % dev)
      r = rospy.Rate(0.1)
    
  r.sleep()
