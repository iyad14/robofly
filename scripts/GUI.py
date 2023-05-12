#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
# from tkinter import *
from tkinter import ttk
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tkinter as tk
import math
import rospy
import roslaunch.parent
import rospkg
import os
import roslaunch.rlutil
from robofly.msg import Control
# from std_srvs.srv import SetBool
from PIL import ImageTk
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from hector_uav_msgs.srv import EnableMotors
import cv2




class GUI:
    def __init__(self):
        rospy.init_node('ROBOFLY_GUI', anonymous=False)
        
        self.root = tk.Tk(className="ROBOFLY GUI")
        self.root.title("ROBOFLY GUI")

        # Imagen de icono
        iconImg = """iVBORw0KGgoAAAANSUhEUgAAADAAAAAwCAYAAABXAvmHAAAABmJLR0QA/wD/AP+gvaeTAAAE4klEQVRoge2YW2wUVRjHf+fMXqHGECVB5Vq8EJW
                LAsG0knBpJZCQlmJIKKWSWEuMMSwPvvggMfpiTGTRNECiwbRbYxq6pfGhGisRuQQKNOXyYDS0QA0gGMQIvezuzOfDttDt7nZnZxtRsv992DPf9f+d
                b86cmQN55JFHHnnk8aCjtLKmo3TT68f/i/lcBw4elUxGdZ/vA1HYsS1fUazG0o93Pp3J4IFAaWWNlFbWZJyN+5FvzHYDBEPhDmBx/EpOBKrWv5QTu3HOZ
                +cWWnxvqJY4p2YbWeVL24EDB4+sAbUXmDpaV7ajGIDW9486IZiEDPF+E5HadStfbkulHKMDqcnfB0xVSu1Np3Q5ifjnE8+Czrh8/pV4ab1afjiyeqjyaU
                6JjRN6lVK1ZcuLvk2ltFV207Fjfne/PC5aHlOWnqaRhShKBOaPB0MFZxDaLdRp0VavstTVqF9d2VBU1G/D1zkOHDy8BPSXwByHIX4Ga0v5iqUnnHLI+UZ
                ubj9caGh9wYmvaVmz15cs7c4lf06vEh990fpQ7+9/VDr1v3ztxsa6pqaCXDg4LiDYuH+V12v+gsgHTmMo+DAacf0abNy/ymkMRwUEG8PbEd0GTAF+ROR6
                tjHE4roIh4ApiG7b1dCyzQmXrAsIhsJvIHwCmCi2BaoqlgtsAXqzCNOrDfXa9s0Vy5SoAGCKkuDOhuaabPlktYg/bQjPtxTHAY8oqrdvqmjMNmEqBEMtV
                SD1wCDaWhKofPWsXd+sOiCKzwAfqJ3jRR4gULUuhBAEfFhqt4jYnljbBexsbC4RWArcjFoTdjghOhbc3th7wE1QRcGvwivt+tl+F1Ki3gRQsPud6lV30t
                mVP//1K0rRAkwYpfpLi5Q0n994KpXfWxs23A42hvcgvDuUq90OL1sd+Lj+u4nAGsCyROrS2a1d+M0ErdiTgjzAw5ZSDVtm7vOl8ze0UQcIsHooZ0bY6oB
                X9xVb4FNwLrB5/dVhedWi1lKt2AvMikss8PnHCjXH8vn7qye3Dl/3gNTWnyxvB3h7Y9mVXaHweYG5bnWnCPg+EzdbHRDkRQBRcijBOYH8PcQsk77IILcH
                Brg9MEBfZJCYaaYKPUtIetf/CUAp9YIdbrY6IDAv/q9PjyYw2nYwGiVixhJkpiX0WxE84sLrcifoFBSOvLaUOqVEEGSBHW72FrFSTyGClrE3q5hlEjFjG
                IbBogXzKJwxHUHouXSZU13niMRiGErjMoy0MbTQO3Qc8eT4FSDyKIAg7cFQ+K64c1eiWSQan/lFC+ZRvOxpAG5dg+fmPIMIdHR2ETFjSQUEQ+G7RyjC8D
                CeMxPs7gOP2DGyJJ68cMZ0UDDyYGf2rBkAmJZlM6WyldPuPuAH8N9xT9y6dW3fsLB6cWvC4ZOMoHzrKgmQoeKUSt5kA1UVd4V1TU0F0YjrbyDt43Yk7Hb
                AAJg0qX9wTCMdD3fh4qUkXXdPXGakKGAkbkyePDA0tDW5OX2Rje5AzDTpj0YwDIOF8+dSOHM6AN0XL3P6zFlM08Lv8eDSiWug/mSZYx6OjlXSBjMMPOIi
                EovR0dlFR2dXgt5juJLI54pcT6d7Rgu8Ljd+tweX1qihn0tr/B4PXrc7KYDA/fsmBqlNRcBlGPg9Xgp8Pgp8Pvweb8qZF+jW6NrcOOSRRx7/a/wDNeu3t
                iRlt6UAAAAASUVORK5CYII="""
                
        img = tk.PhotoImage(data=iconImg)
        self.root.tk.call('wm','iconphoto',self.root._w,img)

        self.mainframe = ttk.Frame(self.root, padding ="3 3 12 12")
        self.mainframe.grid(column=0, row=0, sticky=(tk.N,tk.W,tk.E,tk.S))
        self.mainframe.columnconfigure(0,weight=1)

        self.x_p = tk.StringVar()
        self.y_p = tk.StringVar()
        self.z_p = tk.StringVar()
        self.z_o = tk.StringVar()
        
        self.rate = rospy.Rate(10)

        ttk.Label(self.mainframe, textvariable=self.x_p).grid(column=1, row=2, sticky=(tk.W,tk.E))
        ttk.Label(self.mainframe, textvariable=self.y_p).grid(column=2, row=2, sticky=(tk.W,tk.E))
        ttk.Label(self.mainframe, textvariable=self.z_p).grid(column=3, row=2, sticky=(tk.W,tk.E))
        ttk.Label(self.mainframe, textvariable=self.z_o).grid(column=4, row=2, sticky=(tk.W,tk.E))

        ttk.Label(self.mainframe, text="X (m)").grid(column=1, row=3, sticky=tk.W)
        ttk.Label(self.mainframe, text="Y (m)").grid(column=2, row=3, sticky=tk.W)
        ttk.Label(self.mainframe, text="Z (m)").grid(column=3, row=3, sticky=tk.W)
        ttk.Label(self.mainframe, text="Yaw (°)").grid(column=4, row=3, sticky=tk.W)

        ttk.Button(self.mainframe, text="Initialize Drone", command=self.initialize_drone).grid(column=1, row=4, sticky=tk.W)
        ttk.Button(self.mainframe, text="Take Off", command=self.taekoff).grid(column=3, row=4, sticky=tk.W)
        ttk.Button(self.mainframe, text="Land", command=self.land).grid(column=2, row=4, sticky=tk.W)

        ttk.Button(self.mainframe, text="Wander", command=self.wander).grid(column=1, row=5, sticky=tk.W)
        ttk.Button(self.mainframe, text="SLAM map", command=self.slam).grid(column=2, row=5, sticky=tk.W)
        ttk.Button(self.mainframe, text="Full GUI control", command=self.take_control).grid(column=3, row=5, sticky=tk.W)


        ttk.Button(self.mainframe, text="Turn Left", command=self.ccw).grid(column=1, row=6, sticky=tk.W)
        ttk.Button(self.mainframe, text="Fordward", command=self.forward).grid(column=2, row=6, sticky=tk.W)
        ttk.Button(self.mainframe, text="Turn Right", command=self.cw).grid(column=3, row=6, sticky=tk.W)


        ttk.Button(self.mainframe, text="Left", command=self.left).grid(column=1, row=7, sticky=tk.W)
        ttk.Button(self.mainframe, text="Hover", command=self.hover_pub).grid(column=2, row=7, sticky=tk.W)
        ttk.Button(self.mainframe, text="Right", command=self.right).grid(column=3, row=7, sticky=tk.W)

        ttk.Button(self.mainframe, text="Up", command=self.up).grid(column=1, row=8, sticky=tk.W)
        ttk.Button(self.mainframe, text="Backward", command=self.backward).grid(column=2, row=8, sticky=tk.W)
        ttk.Button(self.mainframe, text="Down", command=self.down).grid(column=3, row=8, sticky=tk.W)
        
        ttk.Button(self.mainframe, text="Enable Motors", command=self.enable_motors).grid(column=1, row=9, sticky=tk.W)
        ttk.Button(self.mainframe, text="View Frames", command=self.show_frame).grid(column=2, row=9, sticky=tk.W)
        ttk.Label(self.mainframe, text="1").grid(column=3, row=2, sticky=tk.W)
        
        for child in self.mainframe.winfo_children(): child.grid_configure(padx=5, pady=5)

        self.platform_in_control = "GUI"

        #Subscribers
        self.posicionLider_sub = rospy.Subscriber("/ground_truth/state", Odometry , self.pose_callback)
        self.orientaLider_sub = rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped , self.rot_callback)
        self.image = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        #Publishers
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.mode_pub = rospy.Publisher('/control_mode', Control, queue_size=1)

        self.control_mode = Control()
        self.control_mode.platform_in_control = "GUI"
        self.mode_pub.publish(self.control_mode)


    # Update the camera image in the image_callback method
    def image_callback(self, data):
        # Convert the ROS image to a PIL image
        self.bridge = CvBridge()
        try:
            # Convert the ROS image to a PIL image
            _, self.image = self.bridge_img_cv2.imgmsg_to_cv2(self.image_topic, "bgr8")
        
        except Exception as e:
            print(e)
            
            
    def pose_callback(self, data):
        self.x_p.set("{0:.2f}".format(data.pose.pose.position.x))
        self.y_p.set("{0:.2f}".format(data.pose.pose.position.y))
        self.z_p.set("{0:.2f}".format(data.pose.pose.position.z))

    def rot_callback(self, data):
        self.z_o.set("{0:.2f}".format( math.degrees(self.quaterionToRads(data)) ))
        
    
    def quaterionToRads(self, data):
        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yawZActual = math.atan2(t3, t4)
        if yawZActual < 0:
            yawZActual = 2*math.pi + yawZActual

        return yawZActual
    
    
    def setText(self, text):
        ttk.Label(self.mainframe, text="              ").grid(column=3, row=1, sticky=tk.W)
        ttk.Label(self.mainframe, text=text).grid(column=3, row=1, sticky=tk.W)

    def land(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Land")
            self.land_pub.publish(Empty())

    def enviar_velocidad(self, vx,vy,vz,vaz):
        vel_msg = Twist()
        vel_msg.linear.x = float(vx)
        vel_msg.linear.y = float(vy)
        vel_msg.linear.z = float(vz)
        vel_msg.angular.z = float(vaz)
        vel_msg.angular.x = float(0.0)
        vel_msg.angular.y = float(0.0)
        self.vel_pub.publish(vel_msg)

    def hover_pub(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Hover")
            self.enviar_velocidad(0.0,0.0,0.0,0.0)

    def taekoff(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("TakeOff")
            self.takeoff_pub.publish(Empty())

    def up(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Up")
            vel_msg = Twist()
            vel_msg.linear.z = float(1.0)
            self.vel_pub.publish(vel_msg)

    def down(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Down")
            vel_msg = Twist()
            vel_msg.linear.z = float(-1.0)
            self.vel_pub.publish(vel_msg)

    def forward(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Fordward")
            vel_msg = Twist()
            vel_msg.linear.x = float(1.0)
            self.vel_pub.publish(vel_msg)

    def backward(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Backward")
            vel_msg = Twist()
            vel_msg.linear.x = float(-1.0)
            self.vel_pub.publish(vel_msg)

    def right(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Right")
            vel_msg = Twist()
            vel_msg.linear.y = float(-1.0)
            self.vel_pub.publish(vel_msg)

    def left(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Left")
            vel_msg = Twist()
            vel_msg.linear.y = float(1.0)
            self.vel_pub.publish(vel_msg)

    def cw(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Turn Right")
            vel_msg = Twist()
            vel_msg.angular.z = float(-1.0)
            self.vel_pub.publish(vel_msg)

    def ccw(self):
        if self.control_mode.platform_in_control == "GUI":
            self.setText("Turn Left")
            vel_msg = Twist()
            vel_msg.angular.z = float(1.0)
            self.vel_pub.publish(vel_msg)
        

    def initialize_drone(self):
        # get an instance of the RosPack class
        rospack = rospkg.RosPack()
        # get the path of the package that this script is located in    
        launch_file_path = os.path.join(rospack.get_path('robofly'), 'launch')

        # construct the path to the launch file within the package
        launch_file_path = os.path.join(launch_file_path, 'initialize_quadrobot.launch')

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
        launch.start()    


    def slam(self):
        # get an instance of the RosPack class
        rospack = rospkg.RosPack()
        # get the path of the package that this script is located in    
        launch_file_path = os.path.join(rospack.get_path('robofly'), 'launch')
        # construct the path to the launch file within the package
        launch_file_path = os.path.join(launch_file_path, 'slam.launch')

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
        launch.start()    

    def wander(self):
        self.control_mode.platform_in_control = "wanderer"
        self.mode_pub.publish(self.control_mode)
        # get an instance of the RosPack class
        rospack = rospkg.RosPack()
        # get the path of the package that this script is located in    
        launch_file_path = os.path.join(rospack.get_path('robofly'), 'launch')
        # construct the path to the launch file within the package
        launch_file_path = os.path.join(launch_file_path,'wander.launch')

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
        launch.start() 
        
    def take_control(self):
        self.control_mode.platform_in_control = "GUI"
        self.mode_pub.publish(self.control_mode)
        self.land()
        
    def enable_motors(self):
        # Create a proxy object for the 'enable_motors' service
        enable_motors_proxy = rospy.ServiceProxy('/enable_motors', EnableMotors)
        
        # Call the 'enable_motors' service with 'enable' argument set to True
        try:
            response = enable_motors_proxy(True)
            rospy.loginfo("Motors enabled")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call 'enable_motors' service: {}".format(e))

    def show_frame(self):
        # show camera output in a different window
        pass
              
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        
if __name__ == '__main__':
    try:
        gui = GUI()
        gui.root.mainloop()
        gui.run()
    except rospy.ROSInterruptException:
        pass

