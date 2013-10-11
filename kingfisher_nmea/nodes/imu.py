#!/usr/bin/env python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_helpers import TxHelper
from rpy_helpers import sea_rpy_from_quaternion

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from math import degrees


class RawIMU(TxHelper):
  SENTENCE = "IMU" 

  def __init__(self):
    rospy.Subscriber("imu/data", Imu, self._cb)

  def _cb(self, msg):
    # Must convert these values:
    #  ROS frame: X forward, Y to port, Z up
    #  Sea frame: X forward, Y to starboard, Z down
    self.tx(self.gps_time(msg.header.stamp),
        degrees(msg.angular_velocity.x),
        -degrees(msg.angular_velocity.y),
        -degrees(msg.angular_velocity.z),
        msg.linear_acceleration.x,
        -msg.linear_acceleration.y,
        -msg.linear_acceleration.z)


class RawCompass(TxHelper):
  SENTENCE = "RCM" 

  def __init__(self):
    rospy.Subscriber("imu/data", Imu, self._cb)
    self.compass_id = 0

  def _cb(self, msg):
    roll, pitch, heading = sea_rpy_from_quaternion(msg.orientation)

    self.tx(self.gps_time(),
        self.compass_id,
        degrees(heading),
        degrees(pitch),
        degrees(roll),
        self.gps_time(msg.header.stamp))


if __name__ == "__main__":
  rospy.init_node('nmea_imu')
  RawCompass()
  RawIMU()
  rospy.spin()
