#!/usr/bin/python3

import rospy
from robofly.msg import Collision_info
from  sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector:
    def __init__(self):
      rospy.init_node("collision_detector", anonymous=True)
      
      self.collision_threshold = 0.7
      self.direction = []
      self.pub = rospy.Publisher("/collision_detection", Collision_info, queue_size=10)
      rospy.Subscriber("/scan", LaserScan, self.environment_scanner_callback)

    
    def environment_scanner_callback(self, detection_msg):
        range = detection_msg.ranges
        
        front_ranges = range[480:600]
        right_ranges = range[600:760]
        left_ranges = range[320:480]
        
        front_min_distance = np.min(front_ranges)
        right_min_distance = np.min(right_ranges)
        left_min_distance = np.min(left_ranges)
        
        collision_info = Collision_info()
        
        direction_dict = {'front':front_min_distance, 'right':right_min_distance, 'left':left_min_distance}  
        direction_dict = dict(sorted(direction_dict.items(), key=lambda x: x[1]))
        # check if any of the values in sorted_direction is less than the threshold
        collision_info.colliding = any(val < self.collision_threshold for val in direction_dict.values())

        # Extract the keys into a list
        collision_info.direction = list(direction_dict.keys())

        self.pub.publish(collision_info)
                
         
    def run(self):
        rospy.spin()
       
        
if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        obstacle_detector.run()
    except rospy.ROSInterruptException:
        pass