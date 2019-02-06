# Obstacle-Avoidance-using-LiDAR-and-Kinect
A mapping and obstacle avoidance project using 2D LiDAR scanner and Kinect RGB-D camera 

ROS nodes : LiDAR scanning algorithm node and for the Kinect perception node.

LiDAR Node (in folder my_first) subscribes to the laser_scan message published by rplidar_ros package 
(You will need to install it in your catkin workspace). The Node will apply a median filter to laser scan, followed by 
generation of a point cloud of the indoor environment (a corridor, for example). Using this point cloud, an angle with respect
to current driving direction is calculated and corresponding wheel velocities are generated using a PID controller.
The Kinametic equations may change with respect to the dimensions of the robot.

The Kinect Node, uses Openni package do publish the raw_image message in ROS. This is subscribed by the package built for Kinect
perception. A caffe model for Google's MobileNet CNN is used for detecting objects. This is followed by a Single Shot Detector 
Network which determines the locations of the objects in the image. Corresponding depth images from Kinect camera are used to 
determine the closest objects and driving angle is caluclated using a PID controller. 

This implementation runs in real-time.

Please note that the sensors work individually. It is an experimentation for both sensors. No sensor fusion is used in the
project as yet. 


