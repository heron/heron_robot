#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_helpers import TxHelper
from geometry_msgs.msg import PoseWithCovarianceStamped
from enu.srv import ToFix
from math import degrees, floor, fabs


def nmea_deg(decimal_degrees):
  dd = fabs(decimal_degrees)
  return (floor(dd) * 100) + (dd % 1.0 * 60.0)

def nmea_NS(decimal_degrees):
  return 'N' if decimal_degrees > 0 else 'S'

def nmea_EW(decimal_degrees):
  return 'E' if decimal_degrees > 0 else 'W'


class Nav(TxHelper):
  SENTENCE = "NVG" 

  def __init__(self):
    rospy.wait_for_service('pose_to_fix')
    self.to_fix = rospy.ServiceProxy('pose_to_fix', ToFix)
    self.sub = rospy.Subscriber("odom", PoseWithCovarianceStamped, self._cb)

  def _cb(self, msg):
    fix = self.to_fix(msg.pose).fix

    self.tx(self.gps_time(),
        nmea_deg(fix.latitude), nmea_NS(fix.latitude),
        nmea_deg(fix.longitude), nmea_NS(fix.longitude),
        1, # Pos quality
        0, # Depth
        0, # Heading
        0, # Roll
        0, # Pitch
        self.gps_time(msg.header.stamp))

if __name__ == "__main__":
  rospy.init_node('nmea_nav')
  Nav()
  rospy.spin()
