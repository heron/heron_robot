#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_bringup')

import rospy
from std_msgs.msg import Bool
from subprocess import check_output
#from subprocess import Popen, PIPE

rospy.init_node('wifi_monitor')

r = rospy.Rate(1)
pub = rospy.Publisher('has_wifi', Bool)

while not rospy.is_shutdown():
  try:
    wifi_str = check_output(['ifconfig', 'wlan0']);
    if "inet addr" in wifi_str:
      pub.publish(True)
    else:
      pub.publish(False)
    r.sleep()
    continue 
  except:
    print "foo"
    # Try again.
    raise #pass
