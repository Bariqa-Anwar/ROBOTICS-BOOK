import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # You'll need to install OpenCV for Python and ros-humble-cv-bridge
import cv2

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        self.subscription = self.create_subscription(
            Image,
            '/isaac_camera/image_raw', # Topic from Isaac Sim
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/processed_image', 10)


    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")

        # Perform some basic image processing (e.g., grayscale)
        # For simplicity, let's just convert it to grayscale
        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        self.publisher_.publish(self.br.cv2_to_imgmsg(gray_frame, "mono8"))
        

def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessorNode()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
