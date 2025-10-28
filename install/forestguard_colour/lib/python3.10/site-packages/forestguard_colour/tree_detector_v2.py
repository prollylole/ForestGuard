#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

def _csv3(s, fb):
    try:
        v = [int(x.strip()) for x in str(s).split(',')]
        return np.array(v[:3], dtype=np.uint8) if len(v) >= 3 else np.array(fb, dtype=np.uint8)
    except Exception:
        return np.array(fb, dtype=np.uint8)

class TreeDetector(Node):
    def __init__(self):
        super().__init__('tree_detector')
        self.bridge = CvBridge()

        # ---- params
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('green_low',  '50,100,20')
        self.declare_parameter('green_high', '90,255,255')
        self.declare_parameter('red1_low',   '0,140,60')
        self.declare_parameter('red1_high',  '12,255,255')
        self.declare_parameter('red2_low',   '170,140,60')
        self.declare_parameter('red2_high',  '179,255,255')
        self.declare_parameter('kernel', 5)
        self.declare_parameter('open_iters', 1)
        self.declare_parameter('close_iters', 2)
        self.declare_parameter('roi_ymin', 0.25)
        self.declare_parameter('min_area_px', 1200)
        self.declare_parameter('aspect_min', 1.2)

        img_topic      = self.get_parameter('image_topic').value
        self.lower_g   = _csv3(self.get_parameter('green_low').value,  (50,100,20))
        self.upper_g   = _csv3(self.get_parameter('green_high').value, (90,255,255))
        self.lower_r1  = _csv3(self.get_parameter('red1_low').value,   (0,140,60))
        self.upper_r1  = _csv3(self.get_parameter('red1_high').value,  (12,255,255))
        self.lower_r2  = _csv3(self.get_parameter('red2_low').value,   (170,140,60))
        self.upper_r2  = _csv3(self.get_parameter('red2_high').value,  (179,255,255))
        ksize          = int(self.get_parameter('kernel').value)
        self.open_it   = int(self.get_parameter('open_iters').value)
        self.close_it  = int(self.get_parameter('close_iters').value)
        self.roi_ymin  = float(self.get_parameter('roi_ymin').value)
        self.min_area  = int(self.get_parameter('min_area_px').value)
        self.aspect_min= float(self.get_parameter('aspect_min').value)

        self.kernel = np.ones((max(1,ksize), max(1,ksize)), np.uint8)

        # publishers
        self.sub = self.create_subscription(Image, img_topic, self.image_cb, 10)
        self.pub_debug   = self.create_publisher(Image, '/tree_masks_debug', 10)
        self.pub_counts  = self.create_publisher(Int32MultiArray, '/tree_counts', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/tree_detections', 10)
        self.marker_seq = 0

        # simple tracking memory
        self.track_memory = []  # list of (cx, cy, timestamp)
        self.track_timeout = 1.5   # seconds before forgetting a track
        self.dist_thresh = 60.0    # pixels distance threshold to consider same tree

        self.last_time = time.time()

        self.get_logger().info(
            f"Subscribed: {img_topic}\n"
            f"green={self.lower_g.tolist()}..{self.upper_g.tolist()} "
            f"red1={self.lower_r1.tolist()}..{self.upper_r1.tolist()} "
            f"red2={self.lower_r2.tolist()}..{self.upper_r2.tolist()}\n"
            f"ROI={self.roi_ymin} min_area={self.min_area} aspect_min={self.aspect_min}"
        )

    def _morph(self, mask):
        if self.open_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.open_it)
        if self.close_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.close_it)
        return mask

    def _update_tracks(self, cx, cy):
        """Keep short-term memory of previously seen blobs to avoid double counting."""
        now = time.time()
        self.track_memory = [(x, y, t) for (x, y, t) in self.track_memory if now - t < self.track_timeout]
        for x, y, t in self.track_memory:
            if np.hypot(cx - x, cy - y) < self.dist_thresh:
                return False  # same object seen again
        self.track_memory.append((cx, cy, now))
        return True

    def _count_and_mark(self, frame, mask, color_id, header):
        n = 0
        markers = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect = h / float(w + 1e-3)
            if aspect < self.aspect_min:
                continue

            cx, cy = x + w / 2, y + h / 2
            if not self._update_tracks(cx, cy):
                continue  # skip duplicate detection

            n += 1
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          (0, 255, 0) if color_id == 0 else (0, 0, 255), 2)

            # Add marker (offset in front of camera)
            m = Marker()
            m.header.frame_id = header.frame_id if header.frame_id else 'camera_link'
            m.header.stamp = header.stamp
            m.ns = 'trees'
            m.id = self.marker_seq
            self.marker_seq += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = 1.0  # 1 m forward from camera (for visibility)
            m.pose.position.y = (cx - frame.shape[1] / 2) / 400.0  # horizontal spread
            m.pose.position.z = (frame.shape[0] / 2 - cy) / 400.0  # vertical spread
            m.scale.x = m.scale.y = m.scale.z = 0.25
            if color_id == 0:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.1, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 0.9
            markers.append(m)
        return n, markers

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        H, W, _ = frame.shape
        y0 = int(self.roi_ymin * H)
        roi = frame[y0:, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_g  = cv2.inRange(hsv, self.lower_g, self.upper_g)
        mask_r1 = cv2.inRange(hsv, self.lower_r1, self.upper_r1)
        mask_r2 = cv2.inRange(hsv, self.lower_r2, self.upper_r2)
        mask_r  = cv2.bitwise_or(mask_r1, mask_r2)

        mask_g = self._morph(mask_g)
        mask_r = self._morph(mask_r)

        num_g, m_g = self._count_and_mark(roi, mask_g, 0, msg.header)
        num_r, m_r = self._count_and_mark(roi, mask_r, 1, msg.header)

        counts = Int32MultiArray()
        counts.data = [num_g, num_r]
        self.pub_counts.publish(counts)

        ma = MarkerArray()
        ma.markers = m_g + m_r
        self.pub_markers.publish(ma)

        # paint overlay
        overlay = np.zeros_like(roi)
        overlay[mask_g > 0] = [0, 255, 0]
        overlay[mask_r > 0] = [0, 0, 255]
        debug = frame.copy()
        debug[y0:, :] = cv2.addWeighted(roi, 0.6, overlay, 0.4, 0)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

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
