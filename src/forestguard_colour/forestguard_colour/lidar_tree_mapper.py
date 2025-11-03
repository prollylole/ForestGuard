#!/usr/bin/env python3
import math
from typing import List, Tuple, Optional

import numpy as np

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

# ---- diameter estimators ------------------------------------------------------

def _diameter_from_span(cluster: List[Tuple[float,float,float]]) -> Optional[float]:
    th0 = cluster[0][2]; th1 = cluster[-1][2]
    dtheta = abs(th1 - th0)
    if dtheta < 1e-3: return None
    rs = [math.hypot(x, y) for (x,y,_) in cluster]
    rbar = float(sum(rs) / len(rs))
    return 2.0 * rbar * math.sin(0.5 * dtheta)

def _diameter_from_edges(cluster: List[Tuple[float,float,float]]) -> Optional[float]:
    (xL, yL, _), (xR, yR, _) = cluster[0], cluster[-1]
    return math.hypot(xR - xL, yR - yL)

def _circle_fit_taubin(cluster: List[Tuple[float,float,float]]) -> Optional[Tuple[float,float,float]]:
    P = np.array([[x, y] for (x,y,_) in cluster], dtype=float)
    if P.shape[0] < 6: return None
    x = P[:,0]; y = P[:,1]
    x_m = x.mean(); y_m = y.mean()
    u = x - x_m; v = y - y_m
    Suu = np.dot(u,u); Svv = np.dot(v,v); Suv = np.dot(u,v)
    Suuu = np.dot(u,u*u); Svvv = np.dot(v,v*v)
    Suvv = np.dot(u,v*v); Svuu = np.dot(v,u*u)
    den = 2.0*(Suu*Svv - Suv*Suv)
    if abs(den) < 1e-9: return None
    uc = (Svv*(Suuu+Suvv) - Suv*(Svvv+Svuu)) / den
    vc = (Suu*(Svvv+Svuu) - Suv*(Suuu+Suvv)) / den
    xc = x_m + uc; yc = y_m + vc
    R = math.sqrt(uc*uc + vc*vc + (Suu + Svv)/P.shape[0])
    resid = np.abs(np.hypot(x - xc, y - yc) - R).mean()
    if not np.isfinite(resid) or resid > 0.20:
        return None
    return xc, yc, R

def _fused_diameter(cluster: List[Tuple[float,float,float]]) -> Optional[float]:
    # centroid fallback
    cxs = [p[0] for p in cluster]; cys = [p[1] for p in cluster]
    cx = sum(cxs)/len(cxs); cy = sum(cys)/len(cys)

    cands: List[float] = []

    d = _diameter_from_span(cluster)
    if d and np.isfinite(d): cands.append(d)

    d = _diameter_from_edges(cluster)
    if d and np.isfinite(d): cands.append(d)

    fit = _circle_fit_taubin(cluster)
    if fit:
        _,_,R = fit
        d = 2.0*R
        if np.isfinite(d): cands.append(d)

    # fallback
    d_cent = 2.0 * (sum(math.hypot(x-cx, y-cy) for x,y,_ in cluster) / len(cluster))
    if np.isfinite(d_cent): cands.append(d_cent)

    cands = [v for v in cands if 0.05 < v < 5.0]
    if not cands: return None
    cands.sort()
    return cands[len(cands)//2]  # median
# -----------------------------------------------------------------------------

class LidarTreeMapper(Node):
    """
    LiDAR-only tree tagger (deterministic-world-friendly).

    Publishes:
      /tree_markers   (MarkerArray, frame_id=map)
      /tree_positions (PoseArray,   frame_id=map)
      /trees/total    (UInt32)
      /trees/bad      (UInt32)        # stays 0 here; camera node can bump it
      /tree_count     (Int32)         # legacy

    Service:
      /lidar_tree_mapper/reset (std_srvs/Empty)
    """
    def __init__(self):
        super().__init__("lidar_tree_mapper")

        # ---- Parameters ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame",  "map")

        self.declare_parameter("min_sep_m", 3.0)
        self.declare_parameter("deadzone_frac", 0.5)

        self.declare_parameter("min_cluster_pts", 5)
        self.declare_parameter("cluster_break_m", 0.20)
        self.declare_parameter("max_range_m", 12.0)
        self.declare_parameter("min_arc_deg", 20.0)

        # NEW: work in DIAMETER
        self.declare_parameter("trunk_diam_min_m", 0.30)
        self.declare_parameter("trunk_diam_max_m", 2.00)

        self.declare_parameter("marker_scale_m", 0.35)
        self.declare_parameter("marker_lifetime_s", 0.0)
        self.declare_parameter("publish_text_diameter", True)

        # ---- Read params ----
        self.scan_topic = self.get_parameter("scan_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame  = self.get_parameter("map_frame").value

        self.min_sep    = float(self.get_parameter("min_sep_m").value)
        self.deadzone   = float(self.get_parameter("deadzone_frac").value) * self.min_sep

        self.min_pts    = int(self.get_parameter("min_cluster_pts").value)
        self.break_m    = float(self.get_parameter("cluster_break_m").value)
        self.max_range  = float(self.get_parameter("max_range_m").value)
        self.min_arc    = math.radians(float(self.get_parameter("min_arc_deg").value))

        self.trunk_dmin = float(self.get_parameter("trunk_diam_min_m").value)
        self.trunk_dmax = float(self.get_parameter("trunk_diam_max_m").value)

        self.marker_scale = float(self.get_parameter("marker_scale_m").value)
        self.marker_lifetime_s = float(self.get_parameter("marker_lifetime_s").value)
        self._pub_text_diam = bool(self.get_parameter("publish_text_diameter").value)

        # ---- Publishers ----
        self.pub_markers    = self.create_publisher(MarkerArray, "/tree_markers", 10)
        self.pub_positions  = self.create_publisher(PoseArray,   "/tree_positions", 10)
        self.pub_total_u32  = self.create_publisher(UInt32,      "/trees/total", 10)
        self.pub_bad_u32    = self.create_publisher(UInt32,      "/trees/bad",   10)
        self.pub_count_i32  = self.create_publisher(Int32,       "/tree_count",  10)

        # ---- TF ----
        self.tfbuf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf, self)

        # ---- State ----
        self.trees_map: List[Tuple[float, float]] = []
        self._next_marker_id = 0
        self._bad_count = 0

        # ---- Subs/Service ----
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.create_service(Empty, "/lidar_tree_mapper/reset", self._on_reset)

        self.get_logger().info(
            f"Lidar mapper on {self.scan_topic} | min_sep={self.min_sep:.2f} m, "
            f"deadzone={self.deadzone:.2f} m, trunk_diam=[{self.trunk_dmin:.2f},{self.trunk_dmax:.2f}] m"
        )

    # -------------------- Reset --------------------
    def _on_reset(self, _req, _res):
        self.trees_map.clear()
        self._next_marker_id = 0
        self._bad_count = 0
        kill = Marker(); kill.header.frame_id = self.map_frame; kill.action = Marker.DELETEALL
        ma = MarkerArray(); ma.markers.append(kill); self.pub_markers.publish(ma)
        self.pub_total_u32.publish(UInt32(data=0))
        self.pub_bad_u32.publish(UInt32(data=0))
        self.pub_count_i32.publish(Int32(data=0))
        self.get_logger().info("State reset.")
        return _res

    # -------------------- Scan callback --------------------
    def _on_scan(self, scan: LaserScan):
        # 1) points (base_link)
        pts: List[Tuple[float, float, float]] = []
        ang = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and 0.05 < r < self.max_range:
                x, y = _polar_to_xy(r, ang)
                pts.append((x, y, ang))
            ang += scan.angle_increment
        if not pts: return

        # 2) clusters
        clusters: List[List[Tuple[float, float, float]]] = []
        cur: List[Tuple[float, float, float]] = [pts[0]]
        for i in range(1, len(pts)):
            x0, y0, _ = pts[i-1]; x1, y1, _ = pts[i]
            if math.hypot(x1 - x0, y1 - y0) > self.break_m:
                clusters.append(cur); cur = [pts[i]]
            else:
                cur.append(pts[i])
        if cur: clusters.append(cur)

        # 3) filter clusters, compute fused diameter
        det_base: List[Tuple[float, float, float]] = []  # (cx, cy, diam)
        for C in clusters:
            if len(C) < self.min_pts: continue
            th_min, th_max = C[0][2], C[-1][2]
            if (th_max - th_min) < self.min_arc: continue

            xs = [p[0] for p in C]; ys = [p[1] for p in C]
            cx, cy = sum(xs)/len(xs), sum(ys)/len(ys)

            d_est = _fused_diameter(C)
            if d_est is None or not (self.trunk_dmin <= d_est <= self.trunk_dmax):
                continue
            det_base.append((cx, cy, d_est))
        if not det_base: return

        # 4) transform to map & dedupe
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            self.get_logger().throttle(2000, "TF map<-base_link unavailable yet")
            return

        tx = tf.transform.translation; q = tf.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        new_pts_map: List[Tuple[float, float, float]] = []
        for (bx, by, d_est) in det_base:
            mx = tx.x + (bx*cy - by*sy)
            my = tx.y + (bx*sy + by*cy)
            if any((mx - x0)**2 + (my - y0)**2 < self.deadzone**2 for (x0, y0) in self.trees_map):
                continue
            self.trees_map.append((mx, my))
            new_pts_map.append((mx, my, d_est))
        if not new_pts_map: return

        # 5) publish
        now = self.get_clock().now().to_msg()

        ma = MarkerArray()
        for (mx, my, d_est) in new_pts_map:
            m = Marker()
            m.header.frame_id = self.map_frame; m.header.stamp = now
            m.ns = "trees"; m.id = self._next_marker_id; self._next_marker_id += 1
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = float(mx); m.pose.position.y = float(my); m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.marker_scale
            m.color.r, m.color.g, m.color.b, m.color.a = 0.10, 0.80, 0.10, 0.95
            m.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime_s).to_msg()
            ma.markers.append(m)

            if self._pub_text_diam:
                t = Marker()
                t.header.frame_id = self.map_frame; t.header.stamp = now
                t.ns = "tree_text"; t.id = self._next_marker_id; self._next_marker_id += 1
                t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
                t.pose.position.x = float(mx); t.pose.position.y = float(my); t.pose.position.z = 1.2
                t.pose.orientation.w = 1.0
                t.scale.z = 0.30
                t.color.r = t.color.g = t.color.b = 1.0; t.color.a = 0.9
                t.text = f"{d_est:.2f} m"
                t.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime_s).to_msg()
                ma.markers.append(t)

        self.pub_markers.publish(ma)

        pa = PoseArray(); pa.header.frame_id = self.map_frame; pa.header.stamp = now
        for (mx, my) in self.trees_map:
            p = Pose(); p.position.x = float(mx); p.position.y = float(my); p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_positions.publish(pa)

        total = len(self.trees_map)
        self.pub_total_u32.publish(UInt32(data=total))
        self.pub_bad_u32.publish(UInt32(data=self._bad_count))
        self.pub_count_i32.publish(Int32(data=total))
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
