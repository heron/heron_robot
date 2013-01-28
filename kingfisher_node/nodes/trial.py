#!/usr/bin/python

import roslib; roslib.load_manifest('kingfisher_node')
import rospy

from kingfisher_msgs.msg import Drive

steps = [ 0, 0.2, 0.4, 0.6, 0.8, 1.0, 0.0 ]
step_length = 150


class KingfisherTrial(object):
    def __init__(self):
        rospy.init_node('kingfisher_trial')
        self.pub = rospy.Publisher('cmd_drive', Drive)

    def spin(self):
        r = rospy.Rate(10)
        step = 0
        count = step_length
        while not rospy.is_shutdown():
            self.pub.publish(left=steps[step], right=steps[step])
            count-=1
            if count <= 0:
                count = step_length
                step+=1
                if step > len(steps):
                    return
            r.sleep()

if __name__ == "__main__":
  KingfisherTrial().spin()
