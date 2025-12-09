import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(Int32, '/dominant_color_code', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Moyenne par canal
        b_mean = float(np.mean(cv_image[:, :, 0]))
        g_mean = float(np.mean(cv_image[:, :, 1]))
        r_mean = float(np.mean(cv_image[:, :, 2]))

        channel_means = [r_mean, g_mean, b_mean]  # 0 R 1 G 2 B
        dominant = int(np.argmax(channel_means))

        out = Int32()
        out.data = dominant
        self.publisher_.publish(out)
        # self.get_logger().info(f'Dominant color: {dominant}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
