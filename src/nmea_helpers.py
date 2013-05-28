#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_msgs.msg import Sentence
from std_msgs.msg import Duration
from datetime import datetime


class TxHelper(object):
  TALKER = "CP"
  GPS_TIME_FORMAT = "%02d%02d%06.3f"

  def tx(self, *fields):
    if not hasattr(self, 'tx_publisher'):
      self.tx_publisher = rospy.Publisher('tx', Sentence)

    fields = map(str, fields)
    s = Sentence(talker=self.TALKER, 
        sentence=self.SENTENCE,
        fields=fields)
    s.header.stamp = rospy.Time.now()

    self.tx_publisher.publish(s)

  def gps_time(self, rostime=None):
    if not hasattr(self, 'time_offset_subscriber'):
      self.time_offset = rospy.Duration(0)
      def cb(msg):
        self.time_offset = msg.data
      self.time_offset_subscriber = rospy.Subscriber('time_offset', Duration, cb)
    if not rostime:
      rostime = rospy.Time.now()
    print self.time_offset
    time = rostime + self.time_offset
    dt = datetime.fromtimestamp(time.to_sec())
    return self.GPS_TIME_FORMAT % (dt.hour, dt.minute, float(dt.second) + (float(dt.microsecond) / 1000000))
    

class RxHelper(object):
  TALKER = "PY"

  def listen(self, sentence, callback):
    def cb(msg):
      if msg.talker == self.TALKER and msg.sentence == sentence:
        callback(msg)
    rospy.Subscriber("rx", Sentence, cb)

