#!/usr/bin/env python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_helpers import TxHelper
from sensor_msgs.msg import Imu

class Time(TxHelper):
  SENTENCE = "TIME" 

  def __init__(self):
    rospy.Timer(rospy.Duration(0.2), self._cb)

  def _cb(self, ev):
    self.tx(self.gps_time())
    print rospy.Time.now()

if __name__ == "__main__":
  rospy.init_node('nmea_time')
  Time()
  rospy.spin()
