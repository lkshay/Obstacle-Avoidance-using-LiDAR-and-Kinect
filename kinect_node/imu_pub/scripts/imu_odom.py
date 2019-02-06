#!/usr/bin/env python

# import the necessary packages

from __future__ import print_function
from imu_pub.msg import imu_msg
import numpy as np
import roslib
import sys
import time
import rospy
import matplotlib.pyplot as plt
#from drawnow import drawnow

#plt.ion()

#fig, ax = plt.subplots()

accel_x = list()
accel_y = list()

vel_x = list()
vel_y= list()

pos_x = list()
pos_y = list()

t = list() 
flag = 0
alpha = 0.9

#scat = ax.scatter(t, accel_x, c=[(255,0,0)], s=100)

###################### imu odometry Class ##########################


class imu_odometry:
	
	def __init__(self):
		self.imu_sub = rospy.Subscriber("/accel_data",imu_msg,self.callback)
		self.curr_accel = imu_msg()
		self.last_accel = imu_msg()
		self.curr_vel_x = 0
		self.curr_vel_y = 0
		self.pos_x = 0
		self.pos_y = 0
		self.theta = 0
		#plot_loop()

	def callback(self,data):
		
		global flag,t,accel_x,accel_y,pos_x,pos_y,aplha
		self.last_accel = self.curr_accel
		self.curr_accel = data

		#accel_x.append(round(data.x,3))
		#t.append(data.timestamp)
		
		#scat.set_array(accel_x)

		print("In Callback")
		#rospy.loginfo(data)

		#print(data.x,data.y)
		if flag:
			self.curr_vel_x = round(alpha*(self.curr_vel_x + self.curr_accel.x*(self.curr_accel.timestamp-self.last_accel.timestamp)) + (1-alpha)*self.curr_vel_x,3)
			#self.curr_vel_y = round(alpha*(self.curr_vel_y + self.curr_accel.y*(self.curr_accel.timestamp-self.last_accel.timestamp)) + (1-alpha)*self.curr_vel_y,3)

			vel_x.append(self.curr_vel_x)
			t.append(self.curr_accel.timestamp)

			print(self.curr_vel_x,self.curr_vel_y)

			self.pos_x = round(alpha*(self.pos_x + self.curr_vel_x*(self.curr_accel.timestamp-self.last_accel.timestamp)) + (1-alpha)*self.pos_x,3)
			self.pos_y = round(alpha*(self.pos_y + self.curr_vel_y*(self.curr_accel.timestamp-self.last_accel.timestamp)) + (1-alpha)*self.pos_y,3)

			#pos_x.append(self.pos_x)
			#pos_y.append(self.pos_y)
			#t.append(self.curr_accel.timestamp)

			#print(self.pos_x,self.pos_y,self.curr_accel.timestamp)

		flag=1


#def plot_loop():
	#global flag,t,accel_x,accel_y,pos_x,pos_y,aplha
#	while 1:
#		print("Drawing")
#		drawnow(make_fig)

def main(args):
	imu_odom = imu_odometry()
	rospy.init_node('imu_odom', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		
	finally:
		
		global accel_x,vel_x,t,pos_x,pos_y
		plt.plot(pos_x,pos_y)
		plt.plot(t,vel_x)
		plt.show()

if __name__ == '__main__':
	main(sys.argv)


