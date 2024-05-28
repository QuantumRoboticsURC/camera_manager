import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int8
import cv2

class Camera(Node):
    def __init__(self):
        super().__init__("Camera_Node")
        listener_group = ReentrantCallbackGroup()
        self.sci_cam1_publisher = self.create_publisher(Image, "/sci_cam1", 10)
        self.sci_cam2_publisher = self.create_publisher(Image, "/sci_cam2", 10)
        self.sci_cam3_publisher = self.create_publisher(Image, "/sci_cam3", 10)
        self.sci_cam4_publisher = self.create_publisher(Image, "/sci_cam4", 10)
        self.create_subscription(Int8, "image_quality", self.quality_callback, 1)
        self.create_subscription(Int8, "/selected_camera", self.update_camera, 1, callback_group=listener_group)
        self.quality = 18
        self.camera = -1 
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.0001, self.cameras)

        # Inicializar cámaras
        self.sci_camera1 = self.initialize_camera()
        self.sci_camera2 = self.initialize_camera()
        self.sci_camera3 = self.initialize_camera()
        self.sci_camera4 = self.initialize_camera()

    def quality_callback(self, msg):
        self.quality = msg.data
  
    def initialize_camera(self):
        num_cameras = 5  # Rango de posibles índices generados
        for index in range(num_cameras):
            camera = cv2.VideoCapture(index)
            if camera.isOpened():
                self.get_logger().info(f"Cámara encontrada en el índice {index}")
                return camera
            camera.release()

        self.get_logger().error("No se encontraron cámaras disponibles.")
        return None

    def cv2_to_imgmsg(self, image):
        return self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

    def cv2_to_imgmsg_resized(self, image, scale_percent):
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
        return self.bridge.cv2_to_imgmsg(resized_image, encoding="bgr8")

    def update_camera(self, msg):
        self.camera = msg.data
        print(self.camera)

    def cameras(self):
        if self.sci_camera1 and self.camera == 0:
            ret_sci1, frame_sci1 = self.sci_camera1.read()
            if ret_sci1:
                self.sci_cam1_publisher.publish(self.cv2_to_imgmsg_resized(frame_sci1, self.quality))

        elif self.sci_camera2 and self.camera == 1:
            ret_sci2, frame_sci2 = self.sci_camera2.read()
            if ret_sci2:
                self.sci_cam2_publisher.publish(self.cv2_to_imgmsg_resized(frame_sci2, self.quality))
        elif self.sci_camera3 and self.camera == 2:
            ret_sci3, frame_sci3 = self.sci_camera3.read()
            if ret_sci3:
                self.sci_cam3_publisher.publish(self.cv2_to_imgmsg_resized(frame_sci3, self.quality))
        elif self.sci_camera4 and self.camera == 3:
            ret_sci4, frame_sci4 = self.sci_camera4.read()
            if ret_sci4:
                self.sci_cam4_publisher.publish(self.cv2_to_imgmsg_resized(frame_sci4, self.quality))

def main(args=None):
    rclpy.init(args=args)
    cameras = Camera()
    rclpy.spin(cameras)
    if cameras.sci_camera1:
        cameras.sci_camera1.release()
    if cameras.sci_camera2:
        cameras.sci_camera2.release()
    if cameras.sci_camera3:
        cameras.sci_camera3.release()
    if cameras.sci_camera4:
        cameras.sci_camera4.release()
    cameras.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
