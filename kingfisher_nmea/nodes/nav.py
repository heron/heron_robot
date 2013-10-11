#!/usr/bin/env python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_helpers import TxHelper
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from enu.srv import ToFix
from math import degrees, floor, fabs, atan2, hypot, pi
from rpy_helpers import sea_rpy_from_quaternion
from sensor_msgs.msg import Imu


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
    self.sub_odom = rospy.Subscriber("odom", PoseWithCovarianceStamped, self._cb)
    self.sub_vel = rospy.Subscriber("gps/vel", TwistStamped, self._vel)
    self.vel = None

  def _vel(self, msg):
    self.vel = msg

  def _cb(self, msg):
    fix = self.to_fix(msg.pose).fix
    roll, pitch, heading = sea_rpy_from_quaternion(msg.pose.pose.orientation)
    
    # Override heading with value from GPS, when we have sufficient speed.
    if self.vel and hypot(self.vel.twist.linear.y, self.vel.twist.linear.x) > 0.2:
      heading = (pi/2.0) - atan2(self.vel.twist.linear.y, self.vel.twist.linear.x)
      if heading < 0: heading += 2*pi

    self.tx(self.gps_time(),
        nmea_deg(fix.latitude), nmea_NS(fix.latitude),
        nmea_deg(fix.longitude), nmea_EW(fix.longitude),
        1, # Pos quality. 1 = GPS, for now we don't publish unless there are fixes.
        0, # Altitude from bottom
        0, # Depth from surface
        degrees(heading), # Heading
        degrees(roll), # Roll
        degrees(pitch), # Pitch
        self.gps_time(msg.header.stamp))


class VelRate(TxHelper):
  SENTENCE = "NVR" 

  def __init__(self):
    self.sub_imu = rospy.Subscriber("imu/data", Imu, self._cb)
    self.sub_vel = rospy.Subscriber("gps/vel", TwistStamped, self._vel)
    self.vel = None

  def _vel(self, msg):
    self.vel = msg

  def _cb(self, msg):
    if self.vel:
      self.tx(self.gps_time(),
          self.vel.twist.linear.x,
          self.vel.twist.linear.y,
          0,  # Vertical component of transit velocity
          -degrees(msg.angular_velocity.y),  # Pitch rate
          degrees(msg.angular_velocity.x), # Roll rate
          -degrees(msg.angular_velocity.z)) # Yaw rate



if __name__ == "__main__":
  rospy.init_node('nmea_nav')
  Nav()
  VelRate()
  rospy.spin()
