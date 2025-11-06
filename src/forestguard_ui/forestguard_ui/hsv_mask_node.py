#!/usr/bin/env python3
"""
HSV mask publisher.

Subscribes to an RGB image stream, applies the forest guard colour thresholds,
and publishes a colour-coded mask image for visualisation. Keeps the UI light
by performing the conversions in a separate process.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import numpy as np
    import cv2
except Exception:  # pragma: no cover - fallback if deps missing
    np = None
    cv2 = None


class HSVMaskNode(Node):
    def __init__(self):
        super().__init__("hsv_mask_node")

        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("mask_topic", "/camera/image_hsv_mask")
        self.declare_parameter("use_sim_time", True)

        self.declare_parameter("green_low",  [50, 130, 25])
        self.declare_parameter("green_high", [70, 255, 255])
        self.declare_parameter("red1_low",   [0, 155, 75])
        self.declare_parameter("red1_high",  [34, 255, 255])
        self.declare_parameter("red2_low",   [170, 160, 77])
        self.declare_parameter("red2_high",  [179, 255, 255])

        in_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("mask_topic").get_parameter_value().string_value

        if np is None or cv2 is None:
            self.get_logger().error("OpenCV / NumPy not available; HSV mask node will not run.")
            raise RuntimeError("OpenCV/NumPy required")

        self.bridge = CvBridge()

        self.green_low = np.array(self.get_parameter("green_low").value, dtype=np.uint8)
        self.green_high = np.array(self.get_parameter("green_high").value, dtype=np.uint8)
        self.red1_low = np.array(self.get_parameter("red1_low").value, dtype=np.uint8)
        self.red1_high = np.array(self.get_parameter("red1_high").value, dtype=np.uint8)
        self.red2_low = np.array(self.get_parameter("red2_low").value, dtype=np.uint8)
        self.red2_high = np.array(self.get_parameter("red2_high").value, dtype=np.uint8)

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(Image, in_topic, self._on_image, qos_sensor)
        self.pub = self.create_publisher(Image, out_topic, qos_sensor)

        self.get_logger().info(
            f"HSV mask node ready: {in_topic} -> {out_topic}"
        )

    def _on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().throttle(2000, f"cv_bridge conversion failed: {exc}")
            return

        try:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            mask_g = cv2.inRange(hsv, self.green_low, self.green_high)
            mask_r = cv2.inRange(hsv, self.red1_low, self.red1_high) | cv2.inRange(hsv, self.red2_low, self.red2_high)

            overlay = np.zeros_like(bgr)
            overlay[mask_g > 0] = (0, 255, 0)
            overlay[mask_r > 0] = (0, 0, 255)

            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        except Exception as exc:
            self.get_logger().throttle(2000, f"HSV processing failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HSVMaskNode()
    except Exception:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
