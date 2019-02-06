#!/usr/bin/env python

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from my_first.msg import Num
import numpy as np
import rospy
import serial


W = 0.5 #distance between wheel bases
R = 0.125 #radius of the wheels
head = 0.1 #head recieved from the other node
vel_flag = 0
vl_ref = 45
vr_ref = 46
velocity = []


s = serial.Serial('/dev/ttyUSB1',9600)
if s.isOpen():
	s.close()
s.open()



def callback(data):

	global vel_flag
	global head
	head = data.head
	vel_flag = data.vel_flag
	#print(data)
	vel()
	
#s = serial.Serial('dev/ttyUSB0',9600)

def head_subscriber():
	
	rospy.init_node('head_subscribe',anonymous = True)
	rospy.Subscriber('/head_vel',Num,callback)


def vel():
	
	global head
	global vel_flag
	global velocity	
	global vl_ref
	global vr_ref
	K = 0.27 # proportionality constant between the PWM and speed of the chair
	const = 1 #constant that relates the head and the amount of time in which the outer wheel should cover the extra distance

	theta_extra = W*abs(head)/R

	if(head):
		direction = abs(head)/head # if +1, then right, otherwise left
		print(direction)
		delta_t = const/head

		vl = vel_flag*(vl_ref + (direction)*K*theta_extra) 

		vr = vel_flag*(vr_ref + (-direction)*K*theta_extra)

	else:
		vl = vel_flag*vl_ref
		vr = vel_flag*vr_ref
	
	send_vel(vl,vr)	 
	#velocity = [vl, vr]
	#print(vl,vr)


def send_vel(vel_l,vel_r):
	global s
	print(vel_l,vel_r)
	s.write(str(str(vel_l) + ',' + str(vel_r)).encode()+'\n')
	

if __name__ == '__main__':
	head_subscriber()
	rospy.spin()
	
 
	
