#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthFilter(Node):
    def __init__(self):
        super().__init__('depth_filter')
        self.bridge = CvBridge()
        sub_depth_img = "/zed/zed_node/depth/depth_registered"
        pub_depth_img = "/zed/zed_node/depth/depth_filtered"
        self.sub = self.create_subscription(Image, sub_depth_img, self.callback, 10)
        self.pub = self.create_publisher(Image, pub_depth_img, 10)
        self.get_logger().info('Depth Filter Node Started')
        self.get_logger().info(f'Listening to : {sub_depth_img}')
        self.get_logger().info(f'Subscribed to : {pub_depth_img}')


    def callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        # your filtering here

        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth_mm = (depth * 1000).astype(np.float32)

        grad_x = cv2.Sobel(depth_mm, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_mm, cv2.CV_32F, 0, 1, ksize=3)
        gradient = cv2.magnitude(grad_x, grad_y)

        # 4. Mask out high discontinuity regions (flying pixels / edge bleeding)
        discontinuity_mask = (gradient > 500).astype(np.uint8)  # tune threshold

        # 5. Dilate mask to cover surrounding bleeding pixels
        kernel = np.ones((3, 3), np.uint8)
        discontinuity_mask = cv2.dilate(discontinuity_mask, kernel, iterations=2)

        # 6. Zero out discontinuity regions
        depth[discontinuity_mask > 0] = 0.0

        # 7. Fill small holes left behind using inpaint
        hole_mask = (depth == 0).astype(np.uint8)
        depth_mm = (depth * 1000).astype(np.float32)
        depth_mm = cv2.inpaint(depth_mm, hole_mask, inpaintRadius=3, flags=cv2.INPAINT_TELEA)
        depth = depth_mm / 1000.0

        # 8. Final bilateral smooth
        depth_mm = (depth * 1000).astype(np.float32)
        depth_mm = cv2.bilateralFilter(depth_mm, d=5, sigmaColor=300, sigmaSpace=5)
        depth = depth_mm / 1000.0

        out = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        out.header = msg.header
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = DepthFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # node.get_logger().info('Keyboard interrupt — shutting down filter')
        node.destroy_node()
        rclpy.try_shutdown()
        print('[depth_filter] Node stopped cleanly')

if __name__ == '__main__':
    main()