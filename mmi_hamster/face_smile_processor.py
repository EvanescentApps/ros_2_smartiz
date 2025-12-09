import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2

# On utilise le chemin fourni par OpenCV pour les cascades
FACE_CASCADE = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
SMILE_CASCADE = "/usr/share/opencv4/haarcascades/haarcascade_smile.xml"


class FaceSmileProcessor(Node):
    def __init__(self):
        super().__init__("face_smile_processor")
        self.bridge = CvBridge()

        self.face_detector = cv2.CascadeClassifier(FACE_CASCADE)
        self.smile_detector = cv2.CascadeClassifier(SMILE_CASCADE)

        if self.face_detector.empty():
            self.get_logger().error(f"Cannot load face cascade at {FACE_CASCADE}")
        if self.smile_detector.empty():
            self.get_logger().error(f"Cannot load smile cascade at {SMILE_CASCADE}")

        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )
        # On réutilise le même topic que pour hamster pour ne rien changer côté Arduino
        self.publisher_ = self.create_publisher(Int32, "/dominant_color_code", 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.face_detector.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(80, 80)
        )

        code = 0  # par défaut: pas de visage → rouge

        if len(faces) > 0:
            code = 1  # au moins un visage → vert

            # On regarde le premier visage détecté pour le sourire
            (x, y, w, h) = faces[0]
            roi_gray = gray[y:y + h, x:x + w]

            smiles = self.smile_detector.detectMultiScale(
                roi_gray,
                scaleFactor=1.7,
                minNeighbors=20,
                minSize=(25, 25)
            )

            if len(smiles) > 0:
                code = 2  # visage + sourire → bleu

        msg_out = Int32()
        msg_out.data = code
        self.publisher_.publish(msg_out)
        # self.get_logger().info(f"Face/smile code: {code}")

def main(args=None):
    rclpy.init(args=args)
    node = FaceSmileProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
