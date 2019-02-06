#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import rospy
import numpy as np
import math as m
import matplotlib.pyplot as plt
import time

coords = np.zeros((180,2), dtype = np.float16)
x_clear = [] #one bunch of continous x's
loc = [] #list of x_clears
head_vel_flag = []

dist = list()
clearance = 1 #1 meter for the chair to pass from a clearance (width of chair)


def callback(data):
	global dist
	#rospy.loginfo(rospy.get_caller_id(), data)
	dist = data.ranges
	ang = data.angle_increment
	#rospy.loginfo('dist:{},ang:{}'.format(dist,ang))
	pass

def my_node_subs1():

	rospy.init_node('my_node_subs1', anonymous=True)
	rospy.Subscriber("scan",LaserScan,callback)


def point_cloud_gen(pub):
	global dist
	global head_vel_flag
	start = time.time()	
	while True:
		flag = 0
		if(len(dist)):
	
			for i in range(180):
				if(dist[i] >= 10):
					a = 6
				else:
					a = dist[i]
				coords[i,0] = a * m.sin(m.radians(i))
				coords[i,1] = a * m.cos(m.radians(i))
				x = coords[i,0]
				y = coords[i,1]
				#print(x,y)
				#plt.plot(x*100,y*100,'ro')
				
				if(y >= 2.0):
			
					x_clear.append([x,i])
					flag = 1

				else:

					if(flag == 1):
						loc.append(x_clear)
					flag = 0
			#print(x_clear)
			index = 0
			for i in range(1,len(loc)):
		
				if(len(loc[i]) >= len(loc[i-1])):
					index = i
					max_clearance = len(loc[i])
				else:
					index = i-1
					max_clearance = len(loc[i-1])

			
		
		#max_clearance is the maximum angular span available for the chair to pass
		#the clearance available at 2 meters with that angular span has yet to be determined
		# this can be determined by first and last x coordinates of the max length lost in loc

			list_interest = loc[index]
			clear_region = abs(list_interest[0][0] - list_interest[-1][0])
			head_new = ((list_interest[-1][1]) + (list_interest[0][1])/2) - 90
		 
			if(clear_region >= clearance):
		
				vel_flag = 1 # yes, there is clearance available for the chair to pass
				
		
			else:
				vel_flag = 0 #no clearance for the chair to pass
	
			head_vel_flag = [head_new, vel_flag]
			t = start - time.time()
			#print(t)			
			#print(head_vel_flag)
			publish(pub)
			
	
# publish this head_vel_flag to the other node which can then decide the velocity values for right and left wheel and can send to the pi 

def publisher_1():

	global head_vel_flag
	#print(head_vel_flag)
	pub = rospy.Publisher('head_vel', Float32MultiArray, queue_size = 10)
	return pub

def publish(pub):
	global head_vel_flag
	pub.publish(head_vel_flag[0],head_vel_flag[1])
	#rospy.loginfo(head_vel_flag)
	print(head_vel_flag)
	
		

if __name__ == '__main__':
	try:
		my_node_subs1()
		pub = publisher_1()
		point_cloud_gen(pub)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
		plt.show()

	except rospy.ROSInterruptException:

        	pass
