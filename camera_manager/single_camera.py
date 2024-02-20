import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int8
import time
import math
import cv2

class Camera(Node):
	def __init__(self):
		super().__init__("Single_camera")
		self.ciencias = self.create_publisher(Image, "/science_camera", 10)
		self.create_subscription(Int8, "image_quality", self.quality_callback, 1)
		self.quality = 18
		self.bridge = CvBridge()
		self.timer = self.create_timer(0.0001, self.cameras)

		# Inicializar cámaras
		self.arm_camera = cv2.VideoCapture(0)

  
	def quality_callback(self, msg):
		self.quality = msg.data

	def cv2_to_imgmsg_resized(self, image, scale_percent):
		width = int(image.shape[1] * scale_percent / 100)
		height = int(image.shape[0] * scale_percent / 100)
		dim = (width, height)
		resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
		msg = self.bridge.cv2_to_imgmsg(resized_image, encoding = "bgr8")
		return msg

	def cameras(self):
		if self.arm_camera:
			ret_arm, frame_arm = self.arm_camera.read()
			if ret_arm:
				self.ciencias.publish(self.cv2_to_imgmsg_resized(frame_arm, self.quality))


def main(args=None):
	rclpy.init(args=args)
	cameras = Camera()
	rclpy.spin(cameras)
	cameras.zed.close()
	cameras.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
			
			