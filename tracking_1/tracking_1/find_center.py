from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
class Find_center(Node):
    isSaved = False
    def __init__(self, angels, img_topic):
        super().__init__("Saver_img")

        self.angels = angels
        self.bridge = CvBridge()

        self.get_logger().info('service not available, waiting again...')
        self.create_subscription(Image,img_topic,self.camera_callback,5)
        self.my_pub = self.create_publisher(Image,"my_img",5)

    def camera_callback(self, msg:Image):
        img_bgr = self.bridge.imgmsg_to_cv2(msg)
        img_bgr = self.find_center(img_bgr)

        # cv.imwrite('123.jpg',img_bgr)
        # 1/0
        msg_out = self.bridge.cv2_to_imgmsg(img_bgr, encoding='rgb8')
        self.my_pub.publish(msg_out)

    
    def find_center(self, img_bgr):
        angels = self.angels

        h_AB = np.sqrt(((angels["A"][0] - angels["B"][0])**2) + ((angels["A"][1] - angels["B"][1])**2))
        w_BC = np.sqrt(((angels["C"][0] - angels["B"][0])**2) + ((angels["C"][1] - angels["B"][1])**2))
        h_CD = np.sqrt(((angels["C"][0] - angels["D"][0])**2) + ((angels["C"][1] - angels["D"][1])**2))
        w_AD = np.sqrt(((angels["A"][0] - angels["D"][0])**2) + ((angels["A"][1] - angels["D"][1])**2))

        h_AB, w_BC, h_CD, w_AD

        max_w = max(w_BC,w_AD)
        max_h = max(h_AB,h_CD)


        input_pts = np.float32(list(angels.values()))
        output_pts = np.float32([
                                [0,0],
                                [0, max_h-1],
                                [max_w -1, max_h -1],
                                [max_w -1, 0]
                                ])

        input_pts, output_pts

        M = cv.getPerspectiveTransform(input_pts, output_pts)
        out = cv.warpPerspective(img_bgr,M, (int(max_w),int(max_h)), flags = cv.INTER_LINEAR)
        out = cv.rotate(out,cv.ROTATE_180)
        out = cv.flip(out, 1)

        out_gray = cv.cvtColor(out, cv.COLOR_RGB2GRAY)
        thresh = cv.threshold(out_gray, 50, 255, cv.THRESH_BINARY_INV)[1]


        cnts = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]

        filtred_conture = []
        for n,c in enumerate(cnts):
            rect = cv.minAreaRect(c)
            area = int(rect[1][0]*rect[1][1])
            if area>100:
                filtred_conture.append(c)




        cx_list = []
        cy_list = []
        for i in filtred_conture:
            M = cv.moments(i)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cx_list.append(cx)
            cy_list.append(cy)

        figure_cx = int(sum(cx_list)/len(cx_list))
        figure_cy = int(sum(cy_list)/len(cy_list))

        
        cv.circle(out,(figure_cx,figure_cy), 2, (0,0,255),-1)
        cv.putText(out, "center", (figure_cx - 20, figure_cy - 20),
                    cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        return out.astype('uint8')


def main(args=None):
    angels = {"A":(100,400), "B":(200,200), "C":(430,195), "D":(500,420)}
    topic_name = "/security_camera/rgb/image_raw"
    rclpy.init(args=args)
    find_center = Find_center(angels, topic_name)

    rclpy.spin(find_center)