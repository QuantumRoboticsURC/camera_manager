import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,Int8,Float64,Header,Int32
from geometry_msgs.msg import Twist,Point
from sensor_msgs.msg import Image
from custom_interfaces.msg import CA
from cv_bridge import CvBridge
import numpy as np
import time
import cv2
import math

class Camera(Node):
	
	def __init__(self):
		super().__init__("Camera_Node")
		self.arm_cam_publisher = self.create_publisher(Image,"/arm_cam",10)
		self.ant_cam_publisher = self.create_publisher(Image,"/ant_cam",10)
		self.arm_camera = cv2.VideoCapture(0)
		self.antenna_camera = cv2.VideoCapture(3)
		self.quality = 18
		self.timer = self.create_timer(0.0001,self.cameras)
		self.bridge = CvBridge()
		
		
	def cv2_to_imgmsg(self, image):
		msg = self.bridge.cv2_to_imgmsg(image, encoding = "bgr8")
		return msg

	def cv2_to_imgmsg_resized(self, image, scale_percent):
		widht = int(image.shape[1] * scale_percent / 100)
		height = int(image.shape[0] * scale_percent / 100)
		dim = (widht, height)
		resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
		msg = self.bridge.cv2_to_imgmsg(resized_image, encoding = "bgr8")
		return msg
	
	def cameras(self):
		print('tttttttt')
		ret_arm,frame_arm = self.arm_camera.read()
		ret_ant,frame_ant = self.antenna_camera.read()
		print(ret_arm,ret_ant)
		if ret_arm:
			self.arm_cam_publisher.publish(self.cv2_to_imgmsg_resized(frame_arm,self.quality))
   
		if ret_ant:
			self.ant_cam_publisher.publish(self.cv2_to_imgmsg_resized(frame_ant,self.quality))


def main(args=None):
	rclpy.init(args=args)
	cameras = Camera()
	rclpy.spin(cameras)
	cameras.zed.close()
	cameras.destroy_node()
	rclpy.shutdown()
	
if __name__=="__main__":
	main()
	
			
			