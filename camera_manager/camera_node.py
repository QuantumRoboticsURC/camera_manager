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
        self.arm_cam_publisher = self.create_publisher(Image, "/arm_cam", 10)
        self.ant_cam_publisher = self.create_publisher(Image, "/ant_cam", 10)
        self.create_subscription(Int8, "image_quality", self.quality_callback, 1)
        self.create_subscription(Int8, "/selected_camera", self.update_camera, 1, callback_group=listener_group)
        self.quality = 18
        self.camera = -1  
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.0001, self.cameras)

        # Inicializar cámaras
        self.arm_camera = self.initialize_camera()
        self.antenna_camera = self.initialize_camera()
  
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
        if self.arm_camera and self.camera == 1:
            ret_arm, frame_arm = self.arm_camera.read()
            if ret_arm:
                print("arm_camera publishing")
                self.arm_cam_publisher.publish(self.cv2_to_imgmsg_resized(frame_arm, self.quality))

        elif self.antenna_camera and self.camera == 2:
            ret_ant, frame_ant = self.antenna_camera.read()
            print(self.antenna_camera)
            print(ret_ant)
            if ret_ant:
                print("Ant cam publishing")
                self.ant_cam_publisher.publish(self.cv2_to_imgmsg_resized(frame_ant, self.quality))

def main(args=None):
    rclpy.init(args=args)
    cameras = Camera()
    rclpy.spin(cameras)
    if cameras.arm_camera:
        cameras.arm_camera.release()
    if cameras.antenna_camera:
        cameras.antenna_camera.release()
    cameras.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
