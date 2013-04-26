#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_node')
import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import serial, select

ser = None

def _shutdown():
    global ser
    rospy.loginfo("Sonar shutting down.")
    rospy.loginfo("Closing Sonar serial port.")
    ser.close()

def serial_lines(ser, brk="\n"):
    buf = ""
    while True:
        rlist, _, _ = select.select([ ser ], [], [], 1.0)
        if not rlist:
            continue
        new = ser.read(ser.inWaiting())
        buf += new
        if brk in new:
            msg, buf = buf.split(brk)[-2:]
            yield msg

if __name__ == '__main__':
    #global ser
    rospy.init_node('sonarmite')
    range_pub = rospy.Publisher('depth', Range)
    quality_pub = rospy.Publisher('quality', Float32)
    range_msg = Range(radiation_type=Range.ULTRASOUND, 
                      field_of_view=0.26,
                      min_range=0.2, 
                      max_range=100.0)
    range_msg.header.frame_id = "sonar"

    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baud = rospy.get_param('~baud', 9600)
    quality_threshold = rospy.get_param('~quality_threshold', 0.54)
    
    rospy.on_shutdown(_shutdown)

    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=.5)
        lines = serial_lines(ser)

        while not rospy.is_shutdown(): 
            data = lines.next()
            try:
                fields = data.split()

                quality = float(fields[6]) / 128.0
                quality_pub.publish(quality)

                if quality >= quality_threshold:
                    range_msg.range = float(fields[1])
                    range_msg.header.stamp = rospy.Time.now()
                    range_pub.publish(range_msg)
            except ValueError as e:
                rospy.logerr(str(e))
                continue

        rospy.loginfo('Closing Digital Compass Serial port')
        ser.close()

    except rospy.ROSInterruptException:
        pass
