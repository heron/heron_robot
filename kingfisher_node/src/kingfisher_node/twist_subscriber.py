#!/usr/bin/python

import rospy 

from geometry_msgs.msg import Twist
from kingfisher_msgs.msg import Drive
from dynamic_reconfigure.server import Server
from kingfisher_node.cfg import TwistConfig
from math import fabs, copysign
from std_msgs.msg import Float32


class TwistSubscriber:
    def __init__(self):
        # self.rotation_scale = rospy.get_param('~rotation_scale', 0.2)
        # self.speed_scale = rospy.get_param('~speed_scale', 1.0)

        self.cmd_pub = rospy.Publisher('cmd_drive', Drive)
        rospy.Subscriber("cmd_vel", Twist, self.callback) 

        srv = Server(TwistConfig, self.reconfigure)

    def reconfigure(self, config, level):
        self.rotation_scale = config['rotation_scale']
        self.fwd_speed_scale = config['fwd_speed_scale']
        self.rev_speed_scale = config['rev_speed_scale']
        self.left_max = config['left_max']
        self.right_max = config['right_max']
        return config

    def callback(self, twist):
        """ Receive twist message, formulate and send Chameleon speed msg. """
        cmd = Drive()
        speed_scale = 1.0
        if twist.linear.x > 0.0: speed_scale = self.fwd_speed_scale
        if twist.linear.x < 0.0: speed_scale = self.rev_speed_scale
        cmd.left = (twist.linear.x * speed_scale) - (twist.angular.z * self.rotation_scale)
        cmd.right = (twist.linear.x * speed_scale) + (twist.angular.z * self.rotation_scale) 

        # Maintain ratio of left/right in saturation
        if fabs(cmd.left) > 1.0:
            cmd.right = cmd.right * 1.0 / fabs(cmd.left)
            cmd.left = copysign(1.0, cmd.left)
        if fabs(cmd.right) > 1.0:
            cmd.left = cmd.left * 1.0 / fabs(cmd.right)
            cmd.right = copysign(1.0, cmd.right)

        # Apply down-scale of left and right thrusts.
        cmd.left *= self.left_max
        cmd.right *= self.right_max

        self.cmd_pub.publish(cmd)
