#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32, UInt32
from std_srvs.srv import Empty

import tf2_ros

def _polar_to_xy(r: float, ang: float) -> Tuple[float, float]:
    return (r*math.cos(ang), r*math.sin(ang))

class LidarTreeMapper(Node):
    """
    LiDAR-only tree tagger tuned for a deterministic, procedurally generated world.

    Clusters contiguous LaserScan returns -> centroid in base_link -> TF to map.
    Uses a deadzone proportional to the known Poisson min-sep to avoid recounting.

    Publishes:
      /tree_markers    (MarkerArray, frame_id=map)
      /tree_positions  (PoseArray,   frame_id=map)  [all unique so far]
      /trees/total     (UInt32)                     [for UI]
      /trees/bad       (UInt32)                     [always 0 here; colour node can own this]
      /tree_count      (Int32)                      [legacy/compat]

    Services:
      /lidar_tree_mapper/reset (std_srvs/Empty): clears memory & markers
    """
    def __init__(self):
        super().__init__("lidar_tree_mapper")

        # ---- Parameters (world-aware defaults) ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame",  "map")

        # From your world generator (deterministic): min separation between tree centers
        self.declare_parameter("min_sep_m", 3.0)          # match --min-sep
        self.declare_parameter("deadzone_frac", 0.5)      # deadzone = frac * min_sep_m

        # Cluster / filter shaping
        self.declare_parameter("min_cluster_pts", 5)
        self.declare_parameter("cluster_break_m", 0.20)   # split cluster if adjacent points jump > this
        self.declare_parameter("max_range_m", 12.0)       # ignore points beyond this
        self.declare_parameter("min_arc_deg", 20.0)       # reject tiny specks
        self.declare_parameter("trunk_r_min_m", 0.06)     # rough trunk radius bounds
        self.declare_parameter("trunk_r_max_m", 0.50)

        # Marker cosmetics
        self.declare_parameter("marker_scale_m", 0.35)
        self.declare_parameter("marker_lifetime_s", 0.0)  # 0 => forever

        # Read params
        self.scan_topic = self.get_parameter("scan_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame  = self.get_parameter("map_frame").value

        self.min_sep    = float(self.get_parameter("min_sep_m").value)
        self.deadzone   = float(self.get_parameter("deadzone_frac").value) * self.min_sep

        self.min_pts    = int(self.get_parameter("min_cluster_pts").value)
        self.break_m    = float(self.get_parameter("cluster_break_m").value)
        self.max_range  = float(self.get_parameter("max_range_m").value)
        self.min_arc    = math.radians(float(self.get_parameter("min_arc_deg").value))
        self.min_r      = float(self.get_parameter("trunk_r_min_m").value)
        self.max_r      = float(self.get_parameter("trunk_r_max_m").value)

        self.marker_scale = float(self.get_parameter("marker_scale_m").value)
        self.marker_lifetime_s = float(self.get_parameter("marker_lifetime_s").value)

        # ---- Publishers ----
        self.pub_markers    = self.create_publisher(MarkerArray, "/tree_markers", 10)
        self.pub_positions  = self.create_publisher(PoseArray,   "/tree_positions", 10)
        self.pub_total_u32  = self.create_publisher(UInt32,      "/trees/total", 10)   # UI wants this
        self.pub_bad_u32    = self.create_publisher(UInt32,      "/trees/bad",   10)   # UI wants this
        self.pub_count_i32  = self.create_publisher(Int32,       "/tree_count",  10)   # legacy

        # ---- TF ----
        self.tfbuf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf, self)

        # ---- State ----
        self.trees_map: List[Tuple[float, float]] = []  # list of unique detections (x,y) in map
        self._next_marker_id = 0
        self._bad_count = 0  # LiDAR-only can't tell; leave 0 so UI shows "bad 0"
        # If you later integrate colour/semantic, expose a subscriber to bump this.

        # ---- Subscriptions ----
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)

        # ---- Services ----
        self.create_service(Empty, "/lidar_tree_mapper/reset", self._on_reset)

        self.get_logger().info(
            f"LiDAR mapper on {self.scan_topic} | min_sep={self.min_sep:.2f} m, "
            f"deadzone={self.deadzone:.2f} m, trunk_r=[{self.min_r:.2f},{self.max_r:.2f}] m"
        )

    # -------------------- Reset --------------------
    def _on_reset(self, _req, _res):
        self.trees_map.clear()
        self._next_marker_id = 0
        self._bad_count = 0
        # Also clear markers in RViz by publishing DELETEALL once
        kill = Marker()
        kill.header.frame_id = self.map_frame
        kill.action = Marker.DELETEALL
        ma = MarkerArray(); ma.markers.append(kill)
        self.pub_markers.publish(ma)

        # Publish zeros so UI updates immediately
        self.pub_total_u32.publish(UInt32(data=0))
        self.pub_bad_u32.publish(UInt32(data=0))
        self.pub_count_i32.publish(Int32(data=0))
        self.get_logger().info("State reset.")
        return _res

    # -------------------- Scan callback --------------------
    def _on_scan(self, scan: LaserScan):
        # 1) Collect points in base_link
        pts: List[Tuple[float, float, float]] = []
        ang = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and 0.05 < r < self.max_range:
                x, y = _polar_to_xy(r, ang)
                pts.append((x, y, ang))
            ang += scan.angle_increment
        if not pts:
            return

        # 2) Cluster by adjacency jump
        clusters: List[List[Tuple[float, float, float]]] = []
        current: List[Tuple[float, float, float]] = [pts[0]]
        for i in range(1, len(pts)):
            x0, y0, _ = pts[i - 1]
            x1, y1, _ = pts[i]
            if math.hypot(x1 - x0, y1 - y0) > self.break_m:
                clusters.append(current)
                current = [pts[i]]
            else:
                current.append(pts[i])
        if current:
            clusters.append(current)

        # 3) Filter clusters, compute centroid + crude radius (arc spread)
        detections_base: List[Tuple[float, float, float]] = []  # (x, y, r_est)
        for C in clusters:
            if len(C) < self.min_pts:
                continue
            th_min, th_max = C[0][2], C[-1][2]
            if (th_max - th_min) < self.min_arc:
                continue
            xs = [p[0] for p in C]; ys = [p[1] for p in C]
            cx = sum(xs) / len(xs); cy = sum(ys) / len(ys)
            r_est = sum(math.hypot(x - cx, y - cy) for x, y, _ in C) / len(C)
            if not (self.min_r <= r_est <= self.max_r):
                continue
            detections_base.append((cx, cy, r_est))
        if not detections_base:
            return

        # 4) Transform to map; deduplicate using deterministic deadzone
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            # Throttle log ~2s
            self.get_logger().throttle(2000, "TF map<-base_link unavailable yet")
            return

        tx = tf.transform.translation
        q  = tf.transform.rotation
        # yaw from quaternion
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        new_pts_map: List[Tuple[float, float]] = []
        for (bx, by, _r) in detections_base:
            mx = tx.x + (bx*cy - by*sy)
            my = tx.y + (bx*sy + by*cy)
            # deadzone check
            if any((mx - x0)**2 + (my - y0)**2 < self.deadzone**2 for (x0, y0) in self.trees_map):
                continue
            self.trees_map.append((mx, my))
            new_pts_map.append((mx, my))

        if not new_pts_map:
            return

        # 5) Publish
        now = self.get_clock().now().to_msg()

        # Markers for *new* trees (spheres)
        ma = MarkerArray()
        for (mx, my) in new_pts_map:
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = "trees"
            m.id = self._next_marker_id; self._next_marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(mx); m.pose.position.y = float(my); m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.marker_scale
            m.color.r, m.color.g, m.color.b, m.color.a = 0.10, 0.80, 0.10, 0.95
            m.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime_s).to_msg()
            ma.markers.append(m)
        self.pub_markers.publish(ma)

        # PoseArray for *all* unique trees so far (nice for logging / saving)
        pa = PoseArray()
        pa.header.frame_id = self.map_frame
        pa.header.stamp = now
        for (mx, my) in self.trees_map:
            p = Pose()
            p.position.x = float(mx); p.position.y = float(my); p.position.z = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_positions.publish(pa)

        total = len(self.trees_map)
        self.pub_total_u32.publish(UInt32(data=total))   # UI reads this
        self.pub_bad_u32.publish(UInt32(data=self._bad_count))
        self.pub_count_i32.publish(Int32(data=total))    # legacy

        self.get_logger().info(f"Trees total: {total}")

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
