#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs.msg 


class TransformFixer:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=10)
        self.rate = rospy.Rate(10) # 10 Hz

    def fix_transform(self):
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform('nav', 'base_footprint', rospy.Time())
                tf_msg = tf2_msgs.msg.TFMessage([trans])
                self.pub.publish(tf_msg)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass # If transform is not available or not accurate enough, do nothing
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('transform_fixer')
    tf_fixer = TransformFixer()
    tf_fixer.fix_transform()
    rospy.spin()
