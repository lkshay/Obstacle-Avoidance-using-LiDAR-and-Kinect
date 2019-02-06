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
import matplotlib.pyplot as plt 

coords = np.zeros((360,2), dtype = np.float32)
head_vel_flag = Num()
head = []

dist = np.zeros((360), dtype = np.float32)
dist1 = np.zeros((360), dtype = np.float32)
dist2 = np.zeros((360), dtype = np.float32)
dist3 = np.zeros((360), dtype = np.float32)

clearance = 0.8 #1 meter for the chair to pass from a clearance (some gain*width of chair)

#the head is toward the right side as the chair moves forward

pub = rospy.Publisher('head_vel', Num, queue_size = 10)
k = 0
def callback(data):
	global k
	#rospy.loginfo(rospy.get_caller_id(), data)
	dist = data.ranges
	#print(dist)
	#ang = data.angle_increment
	#rospy.loginfo('dist:{},ang:{}'.format(dist,ang))

	global head_vel_flag,dist1,dist2,dist3,head,coords	
	
	#start = time.time()
	k=k+1
	if(len(dist)):
		loc = []
		vel_flag = 0	
		head1 = 0
		clear = []
		head = []
		
		#dist3[0] = np.min([dist2[0],dist1[0],dist[0]])
		#dist3[1] = np.min([dist2[0],dist1[0],dist[0],dist2[1],dist1[1],dist[1]])
		#print(dist3[0],dist3[1])
		for i in range(2,359):
			dist3[i]=np.median([dist2[i-2],dist1[i-2],dist[i-2],dist2[i-1],dist1[i-1],dist[i-1],dist2[i],dist1[i],dist[i]])               
		
		dist2 = dist1
		dist1 = dist					
		#print(dist3[269],dist3[239],dist3[180])
		#ignoring dist3[0] and dist[1] due to excessive noise. Also these are out of scope of moving window
		#therefore, there is a dead band of 0 to 2 degree region
		a = 0		
		for i in range(180,359): 
			if(dist3[i] <= 3):
				
				#print(dist[i])
				coords[a,0] = dist3[i] * m.cos(m.radians(i))
				coords[a,1] = dist3[i] * m.sin(m.radians(i))
				x = coords[a,0]
				y = coords[a,1]
				#print(x,y,i)
				
				if(a):					
					if(m.sqrt((x-coords[a-1,0])**2 + (y - coords[a-1,1])**2) > clearance):
						loc.append(clear)
						
						t1 = clear[-1][2]
						t2 = i
						head.append(270-(t1+t2)/2)
						#print(loc,head) 
						clear = []
						
				
				clear.append([x,y,i])
				a = a+1
				#print(clear)
				#print(len(head))
		if(len(head) >= 1):
			vel_flag = 1
			head1 = min(head,key = abs)
		else:
			vel_flag = 0
		
			
		#if(k==100):
		#	plt.plot(coords[0:a,0],coords[0:a,1],'ro')
		#	plt.axis([-5,5,-5,5])
		#	plt.show()			
		
		
		head_vel_flag.head = head1  
		head_vel_flag.vel_flag = vel_flag
		publish_my()	
	
	


def my_node_subs1():

	rospy.init_node('my_node_subs1', anonymous=True)
	rospy.Subscriber("scan",LaserScan,callback)


#def point_cloud_gen(pub):

	
	
# publish this head_vel_flag to the other node which can then decide the velocity values for right and left wheel and can send to the pi 

#def publisher_1():
#	pub = rospy.Publisher('head_vel', Num, queue_size = 10)
#	return pub

def publish_my():
	global head_vel_flag,pub
	pub.publish(head_vel_flag)
	rospy.loginfo(head_vel_flag)
	#print(head_vel_flag)
	
		

if __name__ == '__main__':
	try:
		my_node_subs1()
		#pub = publisher_1()
		#point_cloud_gen(pub)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
		#plt.show()
		

	except rospy.ROSInterruptException:

        	pass
