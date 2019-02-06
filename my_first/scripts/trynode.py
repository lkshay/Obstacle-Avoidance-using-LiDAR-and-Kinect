#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from my_first.msg import Num
import rospy
import numpy as np
import math as m
import time
import matplotlib as plt

coords = np.zeros((180,2), dtype = np.float16)
head_vel_flag = Num()
head = []

dist = np.zeros((180), dtype = np.float16)
clearance = 1 #1 meter for the chair to pass from a clearance (some gain*width of chair)

#the head is toward the right side as the chair moves forward

def callback(data):
	global dist
	#rospy.loginfo(rospy.get_caller_id(), data)
	dist = data.ranges
	#print(dist)
	#ang = data.angle_increment
	#rospy.loginfo('dist:{},ang:{}'.format(dist,ang))
	pass

def my_node_subs1():

	rospy.init_node('my_node_subs1', anonymous=True)
	rospy.Subscriber("scan",LaserScan,callback)


def point_cloud_gen(pub):
	global dist
	global head_vel_flag
	global head	
	
	
	#start = time.time()
	dist1 = np.zeros((180), dtype = np.float32)
	dist2 = np.zeros((180), dtype = np.float32)
	dist3 = np.zeros((180), dtype = np.float32)
	dist4 = np.zeros((180), dtype = np.float32)
	dist5 = np.zeros((180), dtype = np.float32)
	while True:
		
		flag = 0
		if(len(dist)):
			loc = []
			vel_flag = 0	
			head1 = 0
			clear = []
			j = 0
			
			
			#dist3[0] = np.min([dist2[0],dist1[0],dist[0]])
			#dist3[1] = np.min([dist2[0],dist1[0],dist[0],dist2[1],dist1[1],dist[1]])
			#print(dist3[0],dist3[1])
			for i in range(2,180):
				dist5[i]=np.min([dist2[i-2],dist1[i-2],dist[i-2],dist2[i-1],dist1[i-1],dist[i-1],dist2[i],dist1[i],dist[i],dist3[i-2],dist4[i-2],dist3[i-1],dist4[i-1],dist3[i],dist4[i]])
				if(m.isinf(dist5[i])):
					dist5[i] = dist4[i]
			dist4 = dist3			
			dist3 = dist2               
			dist2 = dist1
			dist1 = dist			
			#print(dist3[150])
			#ignoring dist3[0] and dist[1] due to excessive noise. Also these are out of scope of moving window
			#therefore, there is a dead band of 0 to 2 degree region
			for i in range(2,180): 
			
				if(dist3[i] <= 3.0):
					#print(dist[i])
					coords[i,0] = dist5[i] * m.cos(m.radians(i))
					coords[i,1] = dist5[i] * m.sin(m.radians(i))
					x = coords[i,0]
					y = coords[i,1]
					#print(x,y,i)
					flag = 1
					clear.append([x,y,i])
					#print(clear)

				else:
					#print(x,y)
					if(flag == 1):
						loc.append(clear)
					flag = 0
			head = []
			print(dist5[90],dist5[45])
			if(len(loc) >= 2):								
				#print(loc[0])
				#print(len(loc))	
				#print(loc)
				for j in range(1,len(loc)):
			
					x1 = loc[j][0][0]
					x2 = loc[j-1][-1][0]
					y1 = loc[j][0][1]
					y2 = loc[j-1][-1][1]
					theta1 = loc[j][0][2]
					theta2 = loc[j-1][-1][2]
					#print(theta1)
					#print(x1,y1,theta1)
					dist_clear = m.sqrt((x1 - x2)**2 + (y1 - y2)**2)
					#print(dist_clear)
					#print(theta1,theta2)
					if(dist_clear >= clearance):
						#print(90 - (theta1 + theta2)/2)
						head.append(90 - (theta1 + theta2)/2)
						#print(head)
			#elif(len(loc) == 1):
				
			if(len(head)):	
				head1 = min(head, key=abs)
				
				vel_flag = 1
			else:
				vel_flag = 0		
			#print(head1)
		#print(head)	
		
		head_vel_flag.head = head1  
		head_vel_flag.vel_flag = vel_flag
		publish_my(pub)	
	
	
# publish this head_vel_flag to the other node which can then decide the velocity values for right and left wheel and can send to the pi 

def publisher_1():
	pub = rospy.Publisher('head_vel', Num, queue_size = 10)
	return pub

def publish_my(pub):
	global head_vel_flag
	pub.publish(head_vel_flag)
	#rospy.loginfo(head_vel_flag)
	#print(head_vel_flag)
	
		

if __name__ == '__main__':
	try:
		my_node_subs1()
		pub = publisher_1()
		point_cloud_gen(pub)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
		#plt.show()

	except rospy.ROSInterruptException:

        	pass
