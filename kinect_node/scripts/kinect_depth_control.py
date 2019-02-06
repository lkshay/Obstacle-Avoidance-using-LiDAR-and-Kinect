#!/usr/bin/env python

# import the necessary packages

from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image
from kinect_node.msg import objects_detected
from cv_bridge import CvBridge, CvBridgeError
from chair_cmd_vel.msg import vel_cmd
import numpy as np
import cv2
import imutils
import roslib
import serial
import sys
import time
import rospy



###################### Kinect Depth Control Class ##########################


class kinect_depth_control:
	
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/depth/image",Image,self.callback)
		self.objects_sub = rospy.Subscriber("/kinect/objects_detected",objects_detected,self.callback_objects)
		self.pub = rospy.Publisher('/chair_vel', vel_cmd, queue_size=10)
		self.pub_vel = vel_cmd()
		self.objects = objects_detected()
		
		# Control Parameters		
		self.fov_x1 = 0
		self.fov_y1 = 0
		self.fov_x2 = 0
		self.fov_y2 = 0
		self.fov_center_x = 0
		self.fov_center_y = 0
		self.vel_r = 15
		self.vel_l = 15
		self.vel_l_init = 30
		self.vel_r_init = 31.5
		self.max_vel = 50
		self.min_vel = -50
		self.Kp = 0.37

	def callback_objects(self,data):
		#rospy.loginfo(data)
		self.objects = data


	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
		except CvBridgeError as e:
			print(e)

		self.prepare_image(cv_image)
		

	def prepare_image(self,cv_image):
	
		#cv_image = imutils.resize(image, width=400)
		h,w = cv_image.shape
		#print(h,w)
		self.fov_x1 = w/6
		self.fov_y1 = h/6
		self.fov_x2 = 5*w/6
		self.fov_y2 = 5*h/6
		self.fov_center_x = w/2
		self.fov_center_y = h/2
		if not self.objects.count==0:
			for i in range(self.objects.count):
				cv2.rectangle(cv_image, (self.objects.box_x1[i], self.objects.box_y1[i]), (self.objects.box_x2[i], self.objects.box_y2[i]),(255,255,255), 2)

		cv2.rectangle(cv_image, (self.fov_x1, self.fov_y1), (self.fov_x2, self.fov_y2),(255,255,255), 2)
		cv2.imshow("Depth Image", cv_image)
		cv2.waitKey(3)
		self.control(cv_image,self.objects)


	def control(self,image,objects):

		closest_object_dist = 999
		index = 0
		pos = ''
		error = 0
		if not objects.count == 0:
			for i in range(objects.count):
				center_x = (objects.box_x2[i]+objects.box_x1[i])/2
				center_y = (objects.box_y2[i]+objects.box_y1[i])/2
				if(image[center_y][center_x]<closest_object_dist):
					closest_object_dist = image[center_y][center_x]
					index = i
						
					if(center_x<=self.fov_center_x):
						pos = 'left'
						error = self.objects.box_x2[index] - self.fov_x1
					else: 
						pos = 'right'
						error = self.fov_x2 - self.objects.box_x1[index]

					if(objects.box_x2[index]>self.fov_x2 and objects.box_x1[index]<self.fov_x1):
						if(image[self.fov_center_y][self.fov_center_x]<2):
							#self.stop_chair()
							if(image[self.fov_center_y][self.fov_center_x-50]<=image[self.fov_center_y][self.fov_center_x+50]):
								self.go_right()
							else:
								self.go_left()

					elif(closest_object_dist<2):
						self.avoid_obstacle(image,index,pos,error)
					else: 
						self.go_straight()
		

		
		
		elif(image[self.fov_center_y][self.fov_center_x]<2):
			#self.stop_chair()
			if(image[self.fov_center_y][self.fov_center_x-50]<=image[self.fov_center_y][self.fov_center_x+50]):
				self.go_right()
			else:
				self.go_left()
		else:
			self.go_straight()
				
			
	def avoid_obstacle(self,image,index,pos,error):
		if pos=='left':
			self.vel_l = self.vel_l_init + self.Kp*(error)
			self.vel_r = self.vel_r_init - self.Kp*(error)
		elif pos=='right':
			self.vel_r = self.vel_r_init + self.Kp*(error)
			self.vel_l = self.vel_l_init - self.Kp*(error)
			
		
		if(self.vel_l>self.max_vel):
			self.vel_l=self.max_vel
		if(self.vel_r>self.max_vel):
			self.vel_r=self.max_vel
		if(self.vel_l<self.min_vel):
			self.vel_l=self.min_vel
		if(self.vel_r<self.min_vel):
			self.vel_r=self.min_vel
		
		self.pub_vel.vel_left = self.vel_l
		self.pub_vel.vel_right = self.vel_r
		

		print("Avoiding obstacle")
		#self.vel_l = int(self.vel_l)
		#self.vel_r = int(self.vel_r)
		self.send_vel()


	def go_straight(self):
		print("going straight")
		self.pub_vel.vel_l = self.vel_l_init
		self.pub_vel.vel_r = self.vel_r_init
		self.send_vel()

	def go_reverse(self):
		self.pub_vel.vel_l = (-1)*self.vel_l_init
		self.pub_vel.vel_r = (-1)*self.vel_r_init
		self.send_vel()
		
	def go_right(self):
		print("going right")
		self.pub_vel.vel_l = 40
		self.pub_vel.vel_r = -40
		self.send_vel()

	def go_left(self):
		print("going left")
		self.pub_vel.vel_l = -40
		self.pub_vel.vel_r = 40
		self.send_vel()

	def stop_chair(self):
		print("Stopping")
		self.pub_vel.vel_l = 0
		self.pub_vel.vel_r = 0
		self.send_vel()


	def send_vel(self):
		print(self.vel_l,self.vel_r)
		# self.s.write(str(str(self.vel_l) + ',' + str(self.vel_r)).encode()+'\n')
		rospy.loginfo(pub_vel)
		self.pub.publish(self.pub_vel)



#################### Kinect Depth Control Class Ends Here #################


def main(args):
	ic = kinect_depth_control()
	rospy.init_node('kinect_depth_controller', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		ic.s.close()

if __name__ == '__main__':
	main(sys.argv)

