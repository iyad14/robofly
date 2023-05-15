#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class OdometryUpdater:
    def __init__(self):
        rospy.init_node("robot_mover")

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        # robot's current position and orientation (quaternion)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # robot's current velocities (linear and angular)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vroll = 0.0
        self.vpitch = 0.0
        self.vyaw = 0.0

        # initial position (should be set in the launch file or in the map)
        self.initial_x = rospy.get_param("~initial_x", 0.0)
        self.initial_y = rospy.get_param("~initial_y", 0.0)
        self.initial_z = rospy.get_param("~initial_z", 0.0)
        self.initial_yaw = rospy.get_param("~initial_yaw", 0.0)

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.vroll = msg.angular.x
        self.vpitch = msg.angular.y
        self.vyaw = msg.angular.z


    def update_odometry(self):
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        # update position and orientation based on current velocities
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt
        self.roll += self.vroll * dt
        self.pitch += self.vpitch * dt
        self.yaw += self.vyaw * dt

        # create Quaternion from roll, pitch, and yaw angles
        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        # create Odometry message and publish it
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = Pose(Vector3(self.x, self.y, self.z), Quaternion(*q))
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, self.vz), Vector3(self.vroll, self.vpitch, self.vyaw))
        self.odom_pub.publish(odom)

        # update last time
        self.last_time = self.current_time


    def run(self):
        self.rate = rospy.Rate(50)  # 50 Hz update rate
        while not rospy.is_shutdown():
            self.update_odometry()

            # read current position and orientation from odometry message
            try:
                odom = rospy.wait_for_message("odom", Odometry, timeout=rospy.Duration(1))
                rospy.loginfo(odom)
                position = odom.pose.pose.position
                orientation = odom.pose.pose.orientation
                self.x = position.x
                self.y = position.y
                self.z = position.z
                euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
                self.roll = euler[0]
                self.pitch = euler[1]
                self.yaw = euler[2]
            except rospy.exceptions.ROSException as e:
                rospy.logwarn(e)
            
            self.rate.sleep()
            
                


if __name__ == "__main__":
    odometry_updater = OdometryUpdater()
    odometry_updater.run()
