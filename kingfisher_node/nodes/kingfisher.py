#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_node')
import rospy

from kingfisher_node.twist_subscriber import TwistSubscriber

class Kingfisher(object):
    def __init__(self):
        rospy.init_node('chameleon_twist')
	TwistSubscriber()

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
  Kingfisher().spin()
