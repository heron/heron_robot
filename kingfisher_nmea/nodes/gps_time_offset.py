#!/usr/bin/env python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_helpers import RxHelper
from std_msgs.msg import Duration, Float64

import re
from time import mktime
from datetime import datetime, time, date
from collections import deque


# Alternative approach--- get this from the time_reference topic of nmea_gps_driver.
class TimeOffset(RxHelper):
  TALKER = "GP"
  TIME_REGEX = '^([0-9]{2})([0-9]{2})([0-9]{2})(.[0-9]+)?$'
  NUM_MEASUREMENTS = 20

  def __init__(self):
    self.pub = rospy.Publisher("time_offset", Duration)
    self.listen("RMC", self._cb)
    self.re = re.compile(self.TIME_REGEX)
    self.measures = deque()
    self.measure_sum = rospy.Duration(0)

  def _cb(self, header, fields):
    #if msg.fields[1] != "A":
    # No fix, ignore it.
    #  return

    match = self.re.match(str(fields[0]))
    if not match:
      # Unrecognized time format, bail.
      return

    hours_s, minutes_s, seconds_s, frac_seconds_s = match.groups()
    if not frac_seconds_s:
      frac_seconds_s = 0
    gps_time = time(int(hours_s), int(minutes_s), int(seconds_s)) 
    gps_datetime = datetime.combine(date.today(), gps_time)
    gps_rostime = rospy.Time(int(mktime(gps_datetime.timetuple())),
                             int(float(frac_seconds_s) * 1000000000.0))
 
    time_offset = header.stamp - gps_rostime
    self.measure_sum += time_offset
    self.measures.append(time_offset)

    if len(self.measures) > self.NUM_MEASUREMENTS:
      self.measure_sum -= self.measures.popleft()
      smoothed_offset = self.measure_sum / float(len(self.measures))
      self.pub.publish(smoothed_offset)

if __name__ == "__main__":
  rospy.init_node('nmea_gps_time_offset')
  TimeOffset()
  rospy.spin()
