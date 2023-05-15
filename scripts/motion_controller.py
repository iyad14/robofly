#!/usr/bin/python3

import rospy
from robofly.msg import Collision_info, Height_info
from geometry_msgs.msg import Twist
from robofly.msg import Control
from nav_msgs.msg import Odometry

class MotionController:
    def __init__(self):        
        self.rate = rospy.Rate(10)
        self.on_height = False
        self.control_mode = None
        self.twist = Twist()
        
        rospy.Subscriber("/collision_detection", Collision_info, self.collision_detection_callback)
        rospy.Subscriber("/height_info", Height_info, self.height_info_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.vel_message_callback)  
        rospy.Subscriber("/control_mode", Control, self.control_mode_change_callback)      
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def vel_message_callback(self, msg):
        self.twist = msg
        
    def control_mode_change_callback(self, msg):
        self.control_mode = msg.platform_in_control
        
    def collision_detection_callback(self, collision_info):
        if self.control_mode == "wanderer":
            if collision_info.colliding:
                    if collision_info.direction[0] == 'front':
                        self.twist.linear.x = 0
                        if collision_info.direction[1] == 'right':
                            self.rotate_ccw()
                        else:
                            self.rotate_cw()
                            
                        self.vel_pub.publish(self.twist)
                    elif collision_info.direction[0] == 'right':
                        self.twist.linear.x = 0
                        self.vel_pub.publish(self.twist)
                        self.rotate_ccw()
                    elif collision_info.direction[0] == 'left':
                        self.twist.linear.x = 0
                        self.vel_pub.publish(self.twist)
                        self.rotate_cw()
                    else:
                        pass
            else:
                self.move_forward()
            
    def height_info_callback(self, msg):
        self.on_height = msg.right_altitude
            
    def move_forward(self):
        if self.on_height:
            self.twist.angular.z = 0
            self.twist.linear.x = 0.3
            self.vel_pub.publish(self.twist)

    def move_backward(self):
        if self.on_height:
            self.twist.linear.x = -0.3
            self.vel_pub.publish(self.twist)

    def rotate_cw(self):
        if self.on_height:
            rospy.loginfo("rotating cw")
            self.twist.angular.z = -0.2
            self.vel_pub.publish(self.twist)

    def rotate_ccw(self):
        if self.on_height:
            rospy.loginfo("rotating ccw")
            self.twist.angular.z = 0.2
            self.vel_pub.publish(self.twist)

    def run(self):
        self.twist.linear.x = 0
        self.vel_pub.publish(self.twist)
        while not rospy.is_shutdown():
            self.rate.sleep()
        
if __name__ == '__main__':
    try:
        rospy.init_node("motion_controller", anonymous=True)
        motion_controller = MotionController()
        motion_controller.run()
    except rospy.ROSInterruptException:
        pass