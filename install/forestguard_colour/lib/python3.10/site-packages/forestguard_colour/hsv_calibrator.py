#!/usr/bin/env python3
"""
HSV Calibrator for ForestGuard
Subscribes to /camera/image and shows sliders to tune HSV thresholds live.
Press ESC to quit; prints final ranges to console.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        self.bridge = CvBridge()
        self.frame = None

        # Subscribe to Husky camera (change if your topic differs)
        self.create_subscription(Image, '/camera/image', self.image_cb, 10)
        self.get_logger().info("Subscribed to /camera/image")

        # OpenCV window + trackbars
        cv2.namedWindow("HSV Calibrator", cv2.WINDOW_NORMAL)
        self._make_trackbars()

    def _make_trackbars(self):
        cv2.createTrackbar('H_low', "HSV Calibrator", 0, 179, lambda x: None)
        cv2.createTrackbar('S_low', "HSV Calibrator", 0, 255, lambda x: None)
        cv2.createTrackbar('V_low', "HSV Calibrator", 0, 255, lambda x: None)
        cv2.createTrackbar('H_high', "HSV Calibrator", 179, 179, lambda x: None)
        cv2.createTrackbar('S_high', "HSV Calibrator", 255, 255, lambda x: None)
        cv2.createTrackbar('V_high', "HSV Calibrator", 255, 255, lambda x: None)

    def image_cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def spin_once(self):
        if self.frame is None:
            return

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # Read trackbar positions
        hL = cv2.getTrackbarPos('H_low', "HSV Calibrator")
        sL = cv2.getTrackbarPos('S_low', "HSV Calibrator")
        vL = cv2.getTrackbarPos('V_low', "HSV Calibrator")
        hH = cv2.getTrackbarPos('H_high', "HSV Calibrator")
        sH = cv2.getTrackbarPos('S_high', "HSV Calibrator")
        vH = cv2.getTrackbarPos('V_high', "HSV Calibrator")

        lower = np.array([hL, sL, vL])
        upper = np.array([hH, sH, vH])

        mask = cv2.inRange(hsv, lower, upper)
        overlay = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        stacked = np.hstack([
            self.frame,
            cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
            overlay
        ])

        cv2.imshow("HSV Calibrator", stacked)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print(f"Final lower: {lower.tolist()} upper: {upper.tolist()}")
            cv2.destroyAllWindows()
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibrator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.spin_once()
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
