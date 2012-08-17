#!/usr/bin/python

import rospy 

from geometry_msgs.msg import Twist
from kingfisher_msgs.msg import Drive
from math import fabs, copysign


class TwistSubscriber:
    def __init__(self):
        self.rotation_scale = rospy.get_param('~rotation_scale', 0.2)
        self.speed_scale = rospy.get_param('~speed_scale', 1.0)

        self.cmd_pub = rospy.Publisher('cmd_drive', Drive)
        rospy.Subscriber("cmd_vel", Twist, self.callback) 

    def callback(self, twist):
        """ Receive twist message, formulate and send Chameleon speed msg. """
        cmd = Drive()
        cmd.left = twist.linear.x - (twist.angular.z * self.rotation_scale)
        cmd.right = twist.linear.x + (twist.angular.z * self.rotation_scale) 

        # Maintain ratio of left/right in saturation
        if fabs(cmd.left) > 1.0:
            cmd.right = cmd.right * 1.0 / fabs(cmd.left)
            cmd.left = copysign(1.0, cmd.left)
        if fabs(cmd.right) > 1.0:
            cmd.left = cmd.left * 1.0 / fabs(cmd.right)
            cmd.right = copysign(1.0, cmd.right)

	self.cmd_pub.publish(cmd)
