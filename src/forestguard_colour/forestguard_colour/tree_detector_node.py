#!/usr/bin/env python3
"""
Tree Detector (Step 3)
- Subscribes to camera image
- Converts to HSV
- Segments red/green with tunable ranges
- Publishes a debug overlay to /tree_masks_debug
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def _parse_csv_triplet(s: str, fallback):
    # Accepts "H,S,V" as string and returns np.array([H,S,V], dtype=np.uint8)
    try:
        vals = [int(x.strip()) for x in str(s).split(',')]
        if len(vals) == 3:
            return np.array(vals, dtype=np.uint8)
    except Exception:
        pass
    return np.array(fallback, dtype=np.uint8)

class TreeDetector(Node):
    def __init__(self):
        super().__init__('tree_detector')
        self.bridge = CvBridge()

        # ---- Parameters ----
        self.declare_parameter('image_topic', '/camera/image')

        # green range (defaults: dark green)
        self.declare_parameter('green_low',  '50,100,20')
        self.declare_parameter('green_high', '90,255,120')

        # red wrap ranges (two bands around 0/180)
        self.declare_parameter('red1_low',  '0,150,50')
        self.declare_parameter('red1_high', '10,255,255')
        self.declare_parameter('red2_low',  '170,150,50')
        self.declare_parameter('red2_high', '180,255,255')

        # morphology
        self.declare_parameter('kernel', 5)             # kernel size
        self.declare_parameter('open_iters', 1)
        self.declare_parameter('close_iters', 2)

        # read params
        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.lower_green = _parse_csv_triplet(self.get_parameter('green_low').value,  (50,100,20))
        self.upper_green = _parse_csv_triplet(self.get_parameter('green_high').value, (90,255,120))
        self.lower_red1  = _parse_csv_triplet(self.get_parameter('red1_low').value,   (0,150,50))
        self.upper_red1  = _parse_csv_triplet(self.get_parameter('red1_high').value,  (10,255,255))
        self.lower_red2  = _parse_csv_triplet(self.get_parameter('red2_low').value,   (170,150,50))
        self.upper_red2  = _parse_csv_triplet(self.get_parameter('red2_high').value,  (180,255,255))
        ksize            = int(self.get_parameter('kernel').value)
        self.open_iters  = int(self.get_parameter('open_iters').value)
        self.close_iters = int(self.get_parameter('close_iters').value)

        self.kernel = np.ones((max(1,ksize), max(1,ksize)), np.uint8)

        # ---- Subs/Pubs ----
        self.sub = self.create_subscription(Image, img_topic, self.image_callback, 10)
        self.pub_debug = self.create_publisher(Image, '/tree_masks_debug', 10)

        self.get_logger().info(
            f"TreeDetector subscribed to: {img_topic}\n"
            f" green_low={self.lower_green.tolist()} green_high={self.upper_green.tolist()}\n"
            f" red1={self.lower_red1.tolist()}..{self.upper_red1.tolist()}  "
            f"red2={self.lower_red2.tolist()}..{self.upper_red2.tolist()}\n"
            f" morph: kernel={ksize} open={self.open_iters} close={self.close_iters}"
        )

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build masks
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_red1  = cv2.inRange(hsv, self.lower_red1,  self.upper_red1)
        mask_red2  = cv2.inRange(hsv, self.lower_red2,  self.upper_red2)
        mask_red   = cv2.bitwise_or(mask_red1, mask_red2)

        # Morphology clean-up
        if self.open_iters > 0:
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, self.kernel, iterations=self.open_iters)
            mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_OPEN, self.kernel, iterations=self.open_iters)
        if self.close_iters > 0:
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, self.kernel, iterations=self.close_iters)
            mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_CLOSE, self.kernel, iterations=self.close_iters)

        # Overlay debug (green/red paint on original)
        debug = frame.copy()
        debug[mask_green > 0] = [0, 255, 0]
        debug[mask_red > 0]   = [0,   0, 255]

        # Publish debug image
        out = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        out.header = msg.header
        self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TreeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
