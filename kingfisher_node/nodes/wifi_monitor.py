#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_bringup')

import rospy
from std_msgs.msg import Bool
from subprocess import check_output

rospy.init_node('wifi_monitor')

r = rospy.Rate(1)
pub = rospy.Publisher('has_wifi', Bool)

while not rospy.is_shutdown():
  r.sleep()
  try:
    print check_output(['wicd-cli', '--wireless', '-d'])
    pub.publish(True)
    continue
  except:
    pub.publish(False)
