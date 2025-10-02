#!/usr/bin/env python3
"""
HSV Calibrator for ForestGuard
Subscribes to /camera/image and shows trackbars to tune HSV thresholds.
Press ESC to exit.
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

        # Subscribe to camera
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.get_logger().info("HSV Calibrator subscribed to /camera/image")

        # Create OpenCV window + trackbars
        cv2.namedWindow("Calibrate HSV", cv2.WINDOW_NORMAL)
        self._create_trackbars()

        self.frame = None

    def _create_trackbars(self):
        # Trackbars for lower/upper HSV
        cv2.createTrackbar('H_low', "Calibrate HSV", 0, 179, lambda x: None)
        cv2.createTrackbar('S_low', "Calibrate HSV", 0, 255, lambda x: None)
        cv2.createTrackbar('V_low', "Calibrate HSV", 0, 255, lambda x: None)
        cv2.createTrackbar('H_high', "Calibrate HSV", 179, 179, lambda x: None)
        cv2.createTrackbar('S_high', "Calibrate HSV", 255, 255, lambda x: None)
        cv2.createTrackbar('V_high', "Calibrate HSV", 255, 255, lambda x: None)

    def image_callback(self, msg):
        # Save the most recent frame
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def spin_once(self):
        if self.frame is None:
            return

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # Get trackbar positions
        hL = cv2.getTrackbarPos('H_low', "Calibrate HSV")
        sL = cv2.getTrackbarPos('S_low', "Calibrate HSV")
        vL = cv2.getTrackbarPos('V_low', "Calibrate HSV")
        hH = cv2.getTrackbarPos('H_high', "Calibrate HSV")
        sH = cv2.getTrackbarPos('S_high', "Calibrate HSV")
        vH = cv2.getTrackbarPos('V_high', "Calibrate HSV")

        lower = np.array([hL, sL, vL])
        upper = np.array([hH, sH, vH])

        # Mask + overlay
        mask = cv2.inRange(hsv, lower, upper)
        overlay = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        stacked = np.hstack([self.frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), overlay])
        cv2.imshow("Calibrate HSV", stacked)
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
