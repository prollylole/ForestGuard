#!/usr/bin/env python3
import math, time
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
    return (r * math.cos(ang), r * math.sin(ang))


class LidarTreeMapper(Node):
    """
    LiDAR-only tree tagger for a deterministic world.
    Clusters LaserScan -> centroid (base_link) -> TF to map -> de-dup by deadzone.

    Publishes:
      /trees/markers (MarkerArray, frame_id=map)         [new hits]
      /trees/poses   (PoseArray,   frame_id=map)         [all unique so far]
      /trees/total   (UInt32)
      /trees/bad     (UInt32)                            [always 0 here]
      /tree_count    (Int32)                             [legacy/compat]
    """
    def __init__(self):
        super().__init__("lidar_tree_mapper")

        # ---- Parameters ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame",  "map")

        self.declare_parameter("min_sep_m", 3.0)           # expected spawn spacing
        self.declare_parameter("deadzone_frac", 0.5)       # deadzone = frac * min_sep

        self.declare_parameter("min_cluster_pts", 5)
        self.declare_parameter("cluster_break_m", 0.20)    # base break distance
        self.declare_parameter("max_range_m", 12.0)
        self.declare_parameter("min_arc_deg", 20.0)
        self.declare_parameter("trunk_r_min_m", 0.06)
        self.declare_parameter("trunk_r_max_m", 0.50)

        # hill/wall rejection
        self.declare_parameter("adaptive_break_k", 2.0)    # scales with r * dtheta
        self.declare_parameter("max_cluster_span_m", 1.2)  # reject wide blobs/walls
        self.declare_parameter("circularity_min", 0.35)    # λ_min / λ_max threshold

        self.declare_parameter("marker_scale_m", 0.35)
        self.declare_parameter("marker_lifetime_s", 0.0)   # 0 => forever

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

        self.adapt_k    = float(self.get_parameter("adaptive_break_k").value)
        self.max_span   = float(self.get_parameter("max_cluster_span_m").value)
        self.circ_min   = float(self.get_parameter("circularity_min").value)

        self.marker_scale = float(self.get_parameter("marker_scale_m").value)
        self.marker_lifetime_s = float(self.get_parameter("marker_lifetime_s").value)

        # ---- Publishers ----
        self.pub_markers    = self.create_publisher(MarkerArray, "/trees/markers", 10)
        self.pub_positions  = self.create_publisher(PoseArray,   "/trees/poses",   10)
        self.pub_total_u32  = self.create_publisher(UInt32,      "/trees/total",   10)
        self.pub_bad_u32    = self.create_publisher(UInt32,      "/trees/bad",     10)
        self.pub_count_i32  = self.create_publisher(Int32,       "/tree_count",    10)  # legacy

        # ---- TF ----
        self.tfbuf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf, self)

        # ---- State ----
        self.trees_map: List[Tuple[float, float]] = []
        self._next_marker_id = 0
        self._bad_count = 0
        self._last_tf_warn = 0.0

        # ---- Sub/Service ----
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
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

        kill = Marker()
        kill.header.frame_id = self.map_frame
        kill.action = Marker.DELETEALL
        ma = MarkerArray(); ma.markers.append(kill)
        self.pub_markers.publish(ma)

        self.pub_total_u32.publish(UInt32(data=0))
        self.pub_bad_u32.publish(UInt32(data=0))
        self.pub_count_i32.publish(Int32(data=0))
        self.get_logger().info("State reset.")
        return _res

    # -------------------- Scan callback --------------------
    def _on_scan(self, scan: LaserScan):
        # 1) Gather points (base_link)
        pts: List[Tuple[float, float, float]] = []
        ang = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and 0.05 < r < self.max_range:
                x, y = _polar_to_xy(r, ang)
                pts.append((x, y, ang))
            ang += scan.angle_increment
        if not pts:
            return

        # 2) Cluster with adaptive break (range- and dtheta-aware)
        clusters: List[List[Tuple[float, float, float]]] = []
        current = [pts[0]]
        for i in range(1, len(pts)):
            x0, y0, a0 = pts[i - 1]
            x1, y1, a1 = pts[i]
            gap = math.hypot(x1 - x0, y1 - y0)
            r_near = math.hypot(x0, y0)
            dtheta = abs(a1 - a0)
            adaptive = max(self.break_m, self.adapt_k * max(0.001, r_near) * max(0.0001, dtheta))
            if gap > adaptive:
                clusters.append(current); current = [pts[i]]
            else:
                current.append(pts[i])
        clusters.append(current)

        # 3) Filter clusters (size, circularity, span, trunk radius)
        def _eigvals_2x2(a: float, b: float, c: float) -> Tuple[float, float]:
            tr = a + c
            disc = (a - c)*(a - c) + 4.0*b*b
            s = math.sqrt(max(0.0, disc))
            l1 = 0.5 * (tr + s)
            l2 = 0.5 * (tr - s)
            return (l1, l2) if l1 >= l2 else (l2, l1)

        detections_base: List[Tuple[float, float, float]] = []
        for C in clusters:
            if len(C) < self.min_pts:
                continue
            th_min, th_max = C[0][2], C[-1][2]
            if (th_max - th_min) < self.min_arc:
                continue

            xs = [p[0] for p in C]; ys = [p[1] for p in C]
            n = float(len(C))
            cx = sum(xs) / n; cy = sum(ys) / n

            # covariance about centroid
            sxx = sum((x - cx)*(x - cx) for x in xs) / n
            syy = sum((y - cy)*(y - cy) for y in ys) / n
            sxy = sum((x - cx)*(y - cy) for x, y in zip(xs, ys)) / n
            lam_max, lam_min = _eigvals_2x2(sxx, sxy, syy)
            circ = (lam_min / lam_max) if lam_max > 1e-9 else 1.0  # ~1 round, ~0 line

            # approximate physical span via bbox diagonal
            minx, maxx = min(xs), max(xs)
            miny, maxy = min(ys), max(ys)
            span = math.hypot(maxx - minx, maxy - miny)

            # mean radial residual -> trunk radius estimate
            r_est = sum(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)) / n

            if circ < self.circ_min:
                continue            # line-like -> hillside/wall
            if span > self.max_span:
                continue            # too wide to be a trunk
            if not (self.min_r <= r_est <= self.max_r):
                continue            # not a plausible trunk

            detections_base.append((cx, cy, r_est))
        if not detections_base:
            return

        # 4) TF to map; de-dup
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            now_t = time.time()
            if now_t - self._last_tf_warn > 2.0:
                self.get_logger().warn("TF map<-base_link unavailable yet")
                self._last_tf_warn = now_t
            return

        tx = tf.transform.translation
        q  = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        new_pts_map: List[Tuple[float, float]] = []
        for (bx, by, _r) in detections_base:
            mx = tx.x + (bx*cy - by*sy)
            my = tx.y + (bx*sy + by*cy)
            if any((mx - x0)**2 + (my - y0)**2 < self.deadzone**2 for (x0, y0) in self.trees_map):
                continue
            self.trees_map.append((mx, my))
            new_pts_map.append((mx, my))

        if not new_pts_map:
            return

        # 5) Publish
        now_msg = self.get_clock().now().to_msg()

        ma = MarkerArray()
        for (mx, my) in new_pts_map:
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now_msg
            m.ns = "trees"
            m.id = self._next_marker_id; self._next_marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(mx); m.pose.position.y = float(my); m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.marker_scale
            m.color.r, m.color.g, m.color.b, m.color.a = 0.10, 0.80, 0.10, 0.95
            m.lifetime = Duration(seconds=self.marker_lifetime_s).to_msg()
            ma.markers.append(m)
        self.pub_markers.publish(ma)

        pa = PoseArray()
        pa.header.frame_id = self.map_frame
        pa.header.stamp = now_msg
        for (mx, my) in self.trees_map:
            p = Pose()
            p.position.x = float(mx); p.position.y = float(my); p.position.z = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_positions.publish(pa)

        total = len(self.trees_map)
        self.pub_total_u32.publish(UInt32(data=total))
        self.pub_bad_u32.publish(UInt32(data=self._bad_count))
        self.pub_count_i32.publish(Int32(data=total))  # legacy
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
