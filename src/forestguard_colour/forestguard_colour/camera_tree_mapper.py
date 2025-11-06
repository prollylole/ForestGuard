#!/usr/bin/env python3
import math, time, threading
from typing import Dict, Optional, List
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations as tft 

class TrackedTree:
    __slots__ = ("id", "cls", "x", "y", "first_seen", "last_seen", "hits")
    def __init__(self, tid: int, cls: str, x: float, y: float):
        self.id = tid
        self.cls = cls
        self.x = x
        self.y = y
        now = time.time()
        self.first_seen = now
        self.last_seen = now
        self.hits = 1

class TreeMapperNode(Node):
    def __init__(self):
        super().__init__('tree_mapper')

        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_hfov_deg', 69.0)

        self.declare_parameter('green_low',  [50,100,20])
        self.declare_parameter('green_high', [90,255,120])
        self.declare_parameter('red1_low',   [0,115,83])
        self.declare_parameter('red1_high',  [33,255,255])
        self.declare_parameter('red2_low',   [170,150,50])
        self.declare_parameter('red2_high',  [180,255,255])

        self.declare_parameter('kernel', 5)
        self.declare_parameter('open_iters', 1)
        self.declare_parameter('close_iters', 2)

        self.declare_parameter('min_range_m', 0.3)
        self.declare_parameter('max_range_m', 30.0)
        self.declare_parameter('merge_radius_m', 0.6)
        self.declare_parameter('prune_age_s', 60.0)
        self.declare_parameter('publish_height_m', 1.5)
        self.declare_parameter('marker_radius_m', 0.18)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_img  = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_scan = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5, reliability=QoSReliabilityPolicy.RELIABLE)

        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self.sub_img  = self.create_subscription(Image, img_topic, self.on_image, qos_img)
        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_scan)

        self.pub_markers = self.create_publisher(MarkerArray, '/trees/markers', 10)
        self.pub_poses   = self.create_publisher(PoseArray, '/trees/poses', 10)
        self.pub_count   = self.create_publisher(Int32, '/trees/count', 1)

        self.last_scan: Optional[LaserScan] = None
        self.tree_db: Dict[int, TrackedTree] = {}
        self.next_id = 1
        self.db_lock = threading.Lock()

        self.create_timer(0.5, self.maintenance_tick)
        self.get_logger().info(f"tree_mapper ready: image={img_topic} scan={scan_topic}")

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_image(self, msg: Image):
        if self.last_scan is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        H, W = frame.shape[:2]
        hfov_rad = math.radians(float(self.get_parameter('camera_hfov_deg').value))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        gL = np.array(self.get_parameter('green_low').value, dtype=np.uint8)
        gH = np.array(self.get_parameter('green_high').value, dtype=np.uint8)
        r1L = np.array(self.get_parameter('red1_low').value, dtype=np.uint8)
        r1H = np.array(self.get_parameter('red1_high').value, dtype=np.uint8)
        r2L = np.array(self.get_parameter('red2_low').value, dtype=np.uint8)
        r2H = np.array(self.get_parameter('red2_high').value, dtype=np.uint8)

        mask_g = cv2.inRange(hsv, gL, gH)
        mask_r = cv2.inRange(hsv, r1L, r1H) | cv2.inRange(hsv, r2L, r2H)

        k = np.ones((int(self.get_parameter('kernel').value),)*2, np.uint8)
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, k, iterations=int(self.get_parameter('open_iters').value))
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_CLOSE, k, iterations=int(self.get_parameter('close_iters').value))
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, k, iterations=int(self.get_parameter('open_iters').value))
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_CLOSE, k, iterations=int(self.get_parameter('close_iters').value))

        dets = []
        dets += self._extract_blobs(mask_g, 'green', W, H, hfov_rad)
        dets += self._extract_blobs(mask_r, 'red',   W, H, hfov_rad)
        fused = self._fuse_with_scan(dets)
        self._update_tracks(fused)

    def _extract_blobs(self, mask, cls, W, H, hfov_rad):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            if cv2.contourArea(c) < 150:
                continue
            x, y, w, h = cv2.boundingRect(c)
            cx = x + 0.5*w
            x_norm = (cx - (W/2)) / (W/2)          # -1..+1
            bearing = x_norm * (hfov_rad/2.0)      # rad
            out.append({'cls': cls, 'bearing': bearing})
        return out

    def _fuse_with_scan(self, dets: List[dict]) -> List[dict]:
        scan = self.last_scan
        if scan is None or not dets:
            return []
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = np.asarray(scan.ranges, dtype=np.float32)

        min_r = float(self.get_parameter('min_range_m').value)
        max_r = float(self.get_parameter('max_range_m').value)
        fused = []
        for d in dets:
            b = d['bearing']
            idx = int(round((b - angle_min)/angle_inc))
            if not (0 <= idx < len(ranges)):
                continue
            r = float(ranges[idx])
            if not math.isfinite(r) or r < min_r or r > max_r:
                continue

            bx = r*math.cos(b)
            by = r*math.sin(b)

            try:
                tf = self.tf_buffer.lookup_transform(
                    self.get_parameter('map_frame').value,
                    self.get_parameter('base_frame').value,
                    Time())
                tx, ty, tz = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
                qx, qy, qz, qw = tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w
                R = tft.quaternion_matrix([qx,qy,qz,qw])
                p_bl = np.array([bx, by, 0.0, 1.0])
                p_map = R @ p_bl
                p_map[0] += tx; p_map[1] += ty; p_map[2] += tz
                mx, my = float(p_map[0]), float(p_map[1])
            except TransformException as e:
                self.get_logger().throttle(2.0, f"TF lookup failed: {e}")
                continue

            d['map_xy'] = (mx, my)
            fused.append(d)
        return fused

    def _update_tracks(self, fused: List[dict]):
        if not fused:
            return
        merge_r2 = float(self.get_parameter('merge_radius_m').value)**2
        now = time.time()
        with self.db_lock:
            for d in fused:
                mx, my = d['map_xy']; cls = d['cls']
                best = None; best_d2 = merge_r2
                for tid, tt in self.tree_db.items():
                    d2 = (mx-tt.x)**2 + (my-tt.y)**2
                    if d2 <= best_d2:
                        best_d2 = d2; best = tid
                if best is None:
                    tid = self.next_id; self.next_id += 1
                    self.tree_db[tid] = TrackedTree(tid, 'unknown', mx, my)
                else:
                    tt = self.tree_db[best]
                    tt.x = 0.7*tt.x + 0.3*mx
                    tt.y = 0.7*tt.y + 0.3*my
                    tt.hits += 1
                    tt.last_seen = now
                    if tt.hits < 3: tt.cls = 'unknown'

    def maintenance_tick(self):
        now = time.time()
        prune_age = float(self.get_parameter('prune_age_s').value)
        h = float(self.get_parameter('publish_height_m').value)
        r = float(self.get_parameter('marker_radius_m').value)
        map_frame = self.get_parameter('map_frame').value

        markers = MarkerArray()
        poses = PoseArray()
        poses.header.frame_id = map_frame
        poses.header.stamp = self.get_clock().now().to_msg()

        to_delete = []
        with self.db_lock:
            for tid, tt in self.tree_db.items():
                if now - tt.last_seen > prune_age:
                    to_delete.append(tid); continue

                p = Pose()
                p.position.x = tt.x; p.position.y = tt.y; p.position.z = 0.0
                p.orientation = Quaternion(w=1.0)
                poses.poses.append(p)

                m = Marker()
                m.header.frame_id = map_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'trees'; m.id = tid
                m.type = Marker.CYLINDER; m.action = Marker.ADD
                m.pose.position.x = tt.x; m.pose.position.y = tt.y; m.pose.position.z = h*0.5
                m.pose.orientation.w = 1.0
                m.scale.x = 2*r; m.scale.y = 2*r; m.scale.z = h
                m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.8, 0.1, 0.9)
                m.lifetime = Duration(seconds=2.0).to_msg()
                markers.markers.append(m)

            for tid in to_delete:
                del self.tree_db[tid]

        self.pub_poses.publish(poses)
        self.pub_markers.publish(markers)
        self.pub_count.publish(Int32(data=len(self.tree_db)))

def main():
    rclpy.init()
    node = TreeMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
