# robofly
The RoboFly package is a control package designed for the Hector Quadrotor in ROS Noetic. It provides collision detection, motion control, height moderation, odometry tracking, and various user control options. This package is built to ensure the safe and efficient movement of the quadrotor in different scenarios.

Package Overview
The RoboFly package consists of the following nodes:

Collision Detection Node: This node checks for potential collisions and publishes the collision distances on different axes (right, left, forward) using the Collision.msg message. It utilizes sensor data to detect obstacles and calculates distances to avoid any collisions.

Motion Controller Node: The Motion Controller node subscribes to the collision information published by the Collision Detection node. It uses this information to move the quadrotor around without colliding with any objects. The node controls the movement of the quadrotor by publishing velocity commands using the twist message and cmd_vel topic.

Height Moderator Node: The Height Moderator node ensures that the quadrotor maintains a certain elevation. It subscribes to the height_info topic, which contains height-related information. This node keeps the quadrotor at the desired height by adjusting the throttle or other appropriate mechanisms.

Odometry Node: The Odometry Node is responsible for tracking the quadrotor's frame and transformations. It provides the necessary odometry information, such as position and orientation, which is crucial for accurate control and mapping.

User Control Options
The RoboFly package offers various user control options:

Control Mode: Users can choose between two control modes: "Wander" and "GUI." In "Wander" mode, the quadrotor autonomously explores its surroundings without user intervention. In "GUI" mode, the user gains full control over the quadrotor's movement using buttons in the GUI.

SLAM Mapping: Users can perform Simultaneous Localization and Mapping (SLAM) using the provided SLAM Map button. This feature allows the quadrotor to generate a map of its environment while moving around autonomously or under manual control.

Path Planner: The package includes a path planner option that enables users to define a final destination for the quadrotor. It utilizes the Dijkstra algorithm to calculate the most optimized path from the current location to the destination.

Dependencies
The RoboFly package depends on the following ROS packages:

Djikstra
hector_localization
hector_quadrotor
hector_slam
navigation

Please ensure that these packages are installed and properly configured in your ROS Noetic environment before using the RoboFly package.

Usage
To use the RoboFly package, follow these steps:

1-Launch the desired world by modifying the world.launch file according to your specific requirements.

2-Launch the RoboFly package by executing the robofly.launch file. This launch file initializes all the necessary nodes and configurations for the package to run.

Note: Make sure you have the following requirements installed on your system:

-ROS Noetic
-Ubuntu 20.04
-Gazebo 11
-RViz

