from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
class Save_one_img(Node):
    isSaved = False
    def __init__(self):
        super().__init__("Saver_img")
        self.bridge = CvBridge()

        self.get_logger().info('service not available, waiting again...')
        self.create_subscription(Image,"/security_camera/rgb/image_raw",self.save_only_one_msg_callback,5)

    def save_only_one_msg_callback(self, msg:Image):
        if not self.isSaved:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv.imwrite("my_img.jpg",img)
            self.isSaved = True
            1/0

def main(args=None):
    rclpy.init(args=args)
    save_one_img = Save_one_img()

    rclpy.spin(save_one_img)