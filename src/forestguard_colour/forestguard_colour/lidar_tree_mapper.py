#!/usr/bin/env python3
import math, time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros
from tf_transformations import quaternion_from_euler

def _polar_to_xy(r: float, ang: float) -> Tuple[float,float]:
    return (r*math.cos(ang), r*math.sin(ang))

class LidarTreeMapper(Node):
    """
    Minimal LiDAR-only tree tagger.
    - Clusters contiguous LaserScan returns into blobs
    - Estimates each blob's center in base_link
    - Transforms to map with TF and keeps a 'deadzone' so the same tree isn't recounted
    - Publishes:
        /tree_markers (MarkerArray, frame_id=map)
        /tree_positions (PoseArray, frame_id=map)
        /tree_count (Int32)  — total unique trees seen this run
    """
    def __init__(self):
        super().__init__("lidar_tree_mapper")

        # --- params
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("min_cluster_pts", 5)
        self.declare_parameter("cluster_break_m", 0.20)     # split cluster if adjacent points jump > this
        self.declare_parameter("max_range_m", 12.0)         # ignore points beyond this
        self.declare_parameter("deadzone_m", 1.10)          # min sep between unique trees
        self.declare_parameter("min_arc_deg", 20.0)         # reject tiny specks
        self.declare_parameter("min_r_m", 0.06)             # rough trunk radius guard (weak check)
        self.declare_parameter("max_r_m", 0.50)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame  = self.get_parameter("map_frame").value
        self.min_pts    = int(self.get_parameter("min_cluster_pts").value)
        self.break_m    = float(self.get_parameter("cluster_break_m").value)
        self.max_range  = float(self.get_parameter("max_range_m").value)
        self.deadzone   = float(self.get_parameter("deadzone_m").value)
        self.min_arc    = math.radians(float(self.get_parameter("min_arc_deg").value))
        self.min_r      = float(self.get_parameter("min_r_m").value)
        self.max_r      = float(self.get_parameter("max_r_m").value)

        # --- pubs
        self.pub_markers   = self.create_publisher(MarkerArray, "/tree_markers", 10)
        self.pub_positions = self.create_publisher(PoseArray,   "/tree_positions", 10)
        self.pub_count     = self.create_publisher(Int32,       "/tree_count", 10)

        # --- TF
        self.tfbuf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf, self)

        # --- state
        self.trees_map: List[Tuple[float,float]] = []   # [(x,y) in map]
        self.tree_count = 0
        self._next_marker_id = 0

        # --- sub
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.get_logger().info(f"LidarTreeMapper listening on {self.scan_topic}")

    # ---------- main callback ----------
    def _on_scan(self, scan: LaserScan):
        # 1) Collect valid points (x,y) in base_link
        pts: List[Tuple[float,float,float]] = []  # (x,y,angle)
        ang = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and 0.05 < r < self.max_range:
                x,y = _polar_to_xy(r, ang)
                pts.append((x,y,ang))
            ang += scan.angle_increment
        if not pts:
            return

        # 2) Split into contiguous clusters by range jumps & angle
        clusters: List[List[Tuple[float,float,float]]] = []
        current: List[Tuple[float,float,float]] = [pts[0]]
        for i in range(1, len(pts)):
            x0,y0,_ = pts[i-1]
            x1,y1,_ = pts[i]
            if math.hypot(x1-x0, y1-y0) > self.break_m:
                clusters.append(current); current = [pts[i]]
            else:
                current.append(pts[i])
        if current: clusters.append(current)

        # 3) For each cluster, do quick sanity checks & get centroid in base_link
        detections_base: List[Tuple[float,float,float]] = []  # (x,y, est_radius)
        for C in clusters:
            if len(C) < self.min_pts:
                continue
            th_min, th_max = C[0][2], C[-1][2]
            if (th_max - th_min) < self.min_arc:
                continue
            xs = [p[0] for p in C]; ys = [p[1] for p in C]
            cx = sum(xs)/len(xs); cy = sum(ys)/len(ys)

            # rough radius estimate = mean distance to centroid (works okay for arc-ish blobs)
            r_est = sum(math.hypot(x-cx, y-cy) for x,y,_ in C)/len(C)
            if not (self.min_r <= r_est <= self.max_r):
                continue

            detections_base.append((cx, cy, r_est))

        if not detections_base:
            return

        # 4) Transform each detection to map frame and apply deadzone de-duplication
        new_pts_map: List[Tuple[float,float]] = []
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            self.get_logger().throttle(2000, "TF map<-base_link unavailable yet")
            return

        tx = tf.transform.translation
        q  = tf.transform.rotation
        # yaw from quaternion (2D)
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        cos_y = math.cos(yaw); sin_y = math.sin(yaw)
        for (bx, by, _r) in detections_base:
            mx = tx.x + (bx*cos_y - by*sin_y)
            my = tx.y + (bx*sin_y + by*cos_y)

            # Deadzone against existing trees
            if any((mx-x0)**2 + (my-y0)**2 < self.deadzone**2 for (x0,y0) in self.trees_map):
                continue

            self.trees_map.append((mx, my))
            self.tree_count += 1
            new_pts_map.append((mx, my))

        if not new_pts_map:
            return

        # 5) Publish markers, poses, and count
        ma = MarkerArray()
        for (mx,my) in new_pts_map:
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns = "trees"
            m.id = self._next_marker_id; self._next_marker_id += 1
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = float(mx); m.pose.position.y = float(my); m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.1, 0.95
            ma.markers.append(m)
        self.pub_markers.publish(ma)

        pa = PoseArray()
        pa.header.frame_id = self.map_frame
        for (mx,my) in self.trees_map:
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(mx), float(my), 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_positions.publish(pa)

        self.pub_count.publish(Int32(data=int(self.tree_count)))
        self.get_logger().info(f"Trees total: {self.tree_count}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarTreeMapper()
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
