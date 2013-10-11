#!/usr/bin/env python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_helpers import TxHelper
from kingfisher_msgs.msg import Sense


class Battery(TxHelper):
  SENTENCE = "RBS" 

  def __init__(self):
    rospy.Subscriber("sense", Sense, self._cb)
    self.battery_id = 0

  def _cb(self, msg):
    self.tx(self.gps_time(),
        msg.battery,
        msg.battery,
        msg.battery,
        0)


if __name__ == "__main__":
  rospy.init_node('nmea_battery')
  Battery()
  rospy.spin()
