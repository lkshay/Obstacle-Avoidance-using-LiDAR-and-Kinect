#!/usr/bin/env python

# import the necessary packages

from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image
from kinect_node.msg import objects_detected
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
import cv2
import serial
import roslib
import sys
import rospy



# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))





# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe("MobileNetSSD_deploy.prototxt.txt", "MobileNetSSD_deploy.caffemodel")






############### Kinect Control Class ###############

class kinect_control:
	
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
		self.pub = rospy.Publisher("/kinect/objects_detected", objects_detected, queue_size=10)


	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
		except CvBridgeError as e:
			print(e)
		
		self.detect(cv_image)
		#(rows,cols,channels) = cv_image.shape
		#if cols > 60 and rows > 60 :
		#cv2.circle(cv_image, (50,50), 10, 255)

		# cv2.imshow("Image window", cv_image)
		# cv2.waitKey(3)


	def detect(self,image):

		# grab the frame from the threaded video stream and resize it
		# to have a maximum width of 400 pixels
		#frame = imutils.resize(image, width=400)
		#frame = imutils.resize(frame, width=640)
		frame = image

		# grab the frame dimensions and convert it to a blob
		h, w, c = frame.shape
		# print(h,w,c)
		 
		blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
			0.007843, (300, 300), 127.5)

		# pass the blob through the network and obtain the detections and
		# predictions
		net.setInput(blob)
		detections = net.forward()
		
		count = 0
		labels_list = list()
		x1_list = list()
		y1_list = list()
		x2_list = list()
		y2_list = list()
		
		# loop over the detections
		for i in np.arange(0, detections.shape[2]):
			# extract the confidence (i.e., probability) associated with
			# the prediction
			confidence = detections[0, 0, i, 2]
			# filter out weak detections by ensuring the `confidence` is
			# greater than the minimum confidence
			if confidence > 0.2:
				# extract the index of the class label from the
				# `detections`, then compute the (x, y)-coordinates of
				# the bounding box for the object
				idx = int(detections[0, 0, i, 1])
				box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")
				#label = "{}: {:.2f}%".format(CLASSES[idx],confidence * 100)
				label = CLASSES[idx]
				
				labels_list.append(label)
				x1_list.append(startX)
				y1_list.append(startY)
				x2_list.append(endX)
				y2_list.append(endY)
				count = count + 1


				# draw the prediction on the frame
				cv2.rectangle(frame, (startX, startY), (endX, endY),COLORS[idx], 2)

				
				#y = startY - 15 if startY - 15 > 15 else startY + 15
				#cv2.putText(frame, label, (startX, y),
					#cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
			

		# show the output frame
		#cv2.rectangle(frame, (self.fov_x1,self.fov_y1), (self.fov_x2,self.fov_y2),(0,0,255), 2)
		cv2.imshow("Frame", frame)
		cv2.waitKey(3)
		
		objects = objects_detected()
		objects.count = count
		objects.label  = labels_list
		objects.box_x1 = x1_list
		objects.box_y1 = y1_list
		objects.box_x2 = x2_list
		objects.box_y2 = y2_list
		
		#rospy.loginfo(objects)
		self.publish_objects(objects)




	def publish_objects(self,objects):
		if not rospy.is_shutdown():
			self.pub.publish(objects)




####### Kinect_Control Class Ends Here #######





def main(args):
	ic = kinect_control()
	rospy.init_node('kinect_controller', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		ic.s.close()

if __name__ == '__main__':
	main(sys.argv)

