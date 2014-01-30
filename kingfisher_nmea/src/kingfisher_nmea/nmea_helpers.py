#!/usr/bin/env python

import roslib; roslib.load_manifest('kingfisher_nmea')
import rospy

from nmea_msgs.msg import Sentence
from std_msgs.msg import Duration
from datetime import datetime
from collections import namedtuple

import re, operator


def checksum(sentence):
  return "%02X" % reduce(operator.ixor, [ord(c) for c in sentence])


class TxHelper(object):
  TALKER = "CP"
  GPS_TIME_FORMAT = "%02d%02d%06.3f"
  NMEA_FLOAT_FORMAT = "%.6f"

  def tx(self, *fields):
    if not hasattr(self, 'tx_publisher'):
      self.tx_publisher = rospy.Publisher('tx', Sentence)

    def process_field(val):
        if isinstance(val, str):
            return val
        if isinstance(val, int):
            return str(val)
        if isinstance(val, float):
            return (self.NMEA_FLOAT_FORMAT % val).rstrip('0').rstrip('.')
        return str(val)


    fields = map(process_field, fields)
    sentence_body = "%s%s,%s" % (self.TALKER, self.SENTENCE, ",".join(fields))

    s = Sentence(sentence="$%s*%s" % (sentence_body, checksum(sentence_body)))
    s.header.stamp = rospy.Time.now()

    self.tx_publisher.publish(s)

  def gps_time(self, rostime=None):
    if not hasattr(self, 'time_offset_subscriber'):
      self.time_offset = rospy.Duration(rospy.get_param("time_offset_default", 0))
      def cb(msg):
        self.time_offset = msg.data
      self.time_offset_subscriber = rospy.Subscriber('time_offset', Duration, cb)
    if not rostime:
      rostime = rospy.Time.now()
    time = rostime - self.time_offset
    dt = datetime.fromtimestamp(time.to_sec())
    return self.GPS_TIME_FORMAT % (dt.hour, dt.minute, float(dt.second) + (float(dt.microsecond) / 1000000))
    

class RxHelper(object):
  TALKER = "PY"

  def listen(self, sentence, callback):
    rospy.loginfo("Setting up listener for %s." % sentence);
    def cb(msg):
      #rospy.logwarn("Sentence received (%s)." % sentence);

      age = rospy.Time.now() - msg.header.stamp
      if (age.to_sec() > 0.1):
        rospy.logwarn("Sentence age is %.4f seconds, dropping." % age.to_sec());

      mo = re.match("^\$([A-Za-z0-9,.-]+)\*([0-9A-Za-z]{2})?", msg.sentence)
      if not mo:
        rospy.logwarn("Input is not a sentence.");
        return

      sentence_body, sentence_checksum = mo.groups()
      if sentence_checksum:
        # By default, checksum is optional on received messages.
        if checksum(sentence_body) != sentence_checksum:
          rospy.logwarn("Sentence has bad checksum.");
          return

      raw_fields = sentence_body.split(",")
      if raw_fields[0][:2] == self.TALKER and raw_fields[0][2:5] == sentence:
        def process_field(f):
          if f == '': return None
          if re.match('^[0-9]+$', f): return int(f)
          if re.match('^[0-9.]+$', f): return float(f)
          return f
        fields = map(process_field, raw_fields[1:])
        rospy.loginfo("Calling callback for %s." % sentence);
        callback(msg.header, fields)
    self.listener_sub = rospy.Subscriber("rx", Sentence, cb, queue_size=1)

