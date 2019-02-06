#!/usr/bin/env python

# import the necessary packages

from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image
from kinect_node.msg import objects_detected
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import roslib
import sys
import rospy



############### Kinect Control Class ###############

class kinect_control:
	
	def __init__(self):
		print("Init Successful")
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
		except CvBridgeError as e:
			print(e)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)


####### Kinect_Control Class Ends Here #######



def main(args):
	ic = kinect_control()
	rospy.init_node('kinect_controller', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)

