#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_helpers import RxHelper
from kingfisher_msgs.msg import Drive  # $PYDIR message
from kingfisher_msgs.msg import Helm   # $PYDEV message
from geometry_msgs.msg import Wrench   # $PYDEP message

from math import radians


class DrivePublisher(RxHelper):
  def __init__(self):
    self.pub = rospy.Publisher("cmd_drive", Drive)
    self.listen("DIR", self._cb)

  def _cb(self, header, fields):
    left, right = fields
    self.pub.publish(left * 0.01, right * 0.01)


class HelmPublisher(RxHelper):
  def __init__(self):
    self.pub = rospy.Publisher("cmd_helm", Helm)
    self.listen("DEV", self._cb)

  def _cb(self, header, fields):
    heading, speed = fields
    self.pub.publish(radians(90.0 - heading), speed)


class WrenchPublisher(RxHelper):
  def __init__(self):
    self.pub = rospy.Publisher("cmd_wrench", Wrench)
    self.msg = Wrench()
    self.listen("DEP", self._cb)

  def _cb(self, header, fields):
    self.msg.force.x, self.msg.torque.z = fields
    self.pub.publish(self.msg)


if __name__ == "__main__":
  rospy.init_node('nmea_command_handlers')
  DrivePublisher()
  HelmPublisher()
  WrenchPublisher()
  rospy.spin()
