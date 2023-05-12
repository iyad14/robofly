import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from robofly.msg import Height_info
from robofly.msg import Control


class HeightModerator:

    def __init__(self):
        rospy.init_node('sonar_height_subscriber', anonymous=True)

        self.desired_height = 1
        self.rate = rospy.Rate(10)
        self.height_info = Height_info()
        self.twist = Twist()
        self.control_mode = None
        
        rospy.Subscriber('/sonar_height', Range, self.sonar_height_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.vel_message_callback)
        rospy.Subscriber("/control_mode", Control, self.control_mode_change_callback)      
        
        self.height_info_pub = rospy.Publisher("/height_info", Height_info, queue_size=10)
        self.height_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    def vel_message_callback(self, msg):
        self.twist = msg    
        
    def control_mode_change_callback(self, msg):
        self.control_mode = msg.platform_in_control
         
    def sonar_height_callback(self, data):
        self.height = data.range
        if self.control_mode == "wanderer":
            self.hover()
        
    def hover(self):    
        if self.height < 0.9*self.desired_height:
            self.rise()
            self.height_info.right_altitude = False
            self.height_info_pub.publish(self.height_info)
            
        elif self.height > 1.1*self.desired_height:
            self.down()
            self.height_info.right_altitude = False
            self.height_info_pub.publish(self.height_info)
        
        else:
            self.stabilize_height()
            self.height_info.right_altitude = True
            self.height_info_pub.publish(self.height_info)
            
    def rise(self):
        self.twist.linear.z  = 0.5
        self.height_pub.publish(self.twist)
            
    def down(self):
            self.twist.linear.z  = -0.5
            self.height_pub.publish(self.twist)
            
    def stabilize_height(self):
            self.twist.linear.z  = 0
            self.height_pub.publish(self.twist)
            
    def run(self):
        self.twist.linear.x = 0
        self.height_pub.publish(self.twist)
        
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    sonar_subscriber = HeightModerator()
    sonar_subscriber.run()
