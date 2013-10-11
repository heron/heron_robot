#!/usr/bin/env python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_helpers import RxHelper
from kingfisher_msgs.msg import Drive, Helm, YawSpd

from math import radians


class HelmPublisher(RxHelper):
  def __init__(self):
    self.pub = rospy.Publisher("cmd_helm", Helm)
    self.listen("DEP", self._cb)

  def _cb(self, header, fields):
    heading_rate, thrust_pct = fields
    self.pub.publish(float(thrust_pct) * 0.01, -radians(float(heading_rate)))


class YawSpdPublisher(RxHelper):
  def __init__(self):
    self.pub = rospy.Publisher("cmd_yawspd", YawSpd)
    self.listen("DEV", self._cb)

  def _cb(self, header, fields):
    heading, speed = fields
    rospy.logwarn(heading)
    rospy.logwarn(speed)
    self.pub.publish(radians(90.0 - float(heading)), float(speed))
    rospy.logwarn("done")


class DrivePublisher(RxHelper):
  def __init__(self):
    self.pub = rospy.Publisher("cmd_drive", Drive)
    self.listen("DIR", self._cb)

  def _cb(self, header, fields):
    left, right = fields
    self.pub.publish(float(left) * 0.01, float(right) * 0.01)



if __name__ == "__main__":
  rospy.init_node('nmea_command_handlers')
  HelmPublisher()
  YawSpdPublisher()
  DrivePublisher()
  rospy.spin()
