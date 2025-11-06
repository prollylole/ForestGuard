#!/usr/bin/env python3
import math, time
from typing import List, Tuple, Dict, Optional
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import MarkerArray
from nav2_msgs.action import FollowWaypoints
from sensor_msgs.msg import Joy

MAP_FRAME = "map"

def yaw_to_quat(yaw: float):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)

class Autonomy(Node):
    def __init__(self):
        super().__init__("autonomy_behaviour")

        # ---- parameters
        self.declare_parameter("settle_time_s", 5.0)       # robot falling/settling after spawn
        self.declare_parameter("scan_time_s",   3.0)        # initial scan burst
        self.declare_parameter("confirm_range", 8.0)        # “certain if seen within this many meters”
        self.declare_parameter("batch_size",    4)          # waypoints per FollowWaypoints goal
        self.declare_parameter("lidar_max_m",   12.0)
        self.declare_parameter("room_radius_m", 50.0)
        self.declare_parameter("room_polygon_xy", "[]")  # YAML string
        ...
        self.room_radius_m  = float(self.get_parameter("room_radius_m").value)
        poly_str = str(self.get_parameter("room_polygon_xy").value or "[]")
        try:
            parsed = yaml.safe_load(poly_str)
            if isinstance(parsed, list):
                # coerce to floats and ensure uniform type
                self.room_poly_flat = [float(v) for v in parsed]
            else:
                self.room_poly_flat = []
        except Exception:
            self.room_poly_flat = []

        self.declare_parameter("min_separation_m", 2.0)     # merge nearby trees
        self.declare_parameter("furthest_bias",  True)      # prefer furthest “uncertain” clusters first

        self.settle_s       = float(self.get_parameter("settle_time_s").value)
        self.scan_s         = float(self.get_parameter("scan_time_s").value)
        self.confirm_range  = float(self.get_parameter("confirm_range").value)
        self.batch_size     = int(self.get_parameter("batch_size").value)
        self.lidar_max_m    = float(self.get_parameter("lidar_max_m").value)
        self.room_radius_m  = float(self.get_parameter("room_radius_m").value)
        self.room_poly_flat = list(self.get_parameter("room_polygon_xy").value)
        self.min_sep        = float(self.get_parameter("min_separation_m").value)
        self.furthest_bias  = bool(self.get_parameter("furthest_bias").value)

        # QoS
        self.reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # subs/pubs
        self.create_subscription(MarkerArray, "/trees/markers", self._on_markers, self.reliable)
        self.create_subscription(MarkerArray, "/trees_coloured", self._on_markers, self.reliable)

        # teleop interrupt (RB)
        self._teleop_btn = int(self.declare_parameter("joy_rb_index", 5).value)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)

        self.state_pub   = self.create_publisher(String, "/ui/autonomy_state", 10)
        self.done_pub    = self.create_publisher(Bool, "/ui/mission_complete", 10)
        self.way_pub     = self.create_publisher(PoseArray, "/goal_pose", 10)  # for UI Waypoint tab mirror

        # action client
        self.way_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        # internal state
        self._t0 = self.get_clock().now()
        self._phase = "init"
        self._trees: Dict[Tuple[float,float], Dict] = {}
        self._robot_xy = (0.0, 0.0)  # optional: fill from tf/odom later if desired
        self._teleop_held = False
        self._last_sent: Optional[List[Tuple[float,float]]] = None
        self._room_poly = self._make_poly(self.room_poly_flat)

        self._tick_timer = self.create_timer(0.25, self._tick)
        self._announce("idle")  # immediately show something in UI

    # ------------- inputs -------------
    def _on_markers(self, msg: MarkerArray):
        now = self.get_clock().now()
        for m in msg.markers:
            if m.action in (m.DELETE, m.DELETEALL):
                continue
            x, y = float(m.pose.position.x), float(m.pose.position.y)
            key = (round(x,2), round(y,2))
            # room gating
            if not self._inside_room(x, y):
                continue
            # status by colour
            st = "unknown"
            try:
                r,g,b,a = m.color.r, m.color.g, m.color.b, m.color.a
                if r > g: st = "bad"
                elif g > r: st = "good"
            except Exception:
                pass
            e = self._trees.get(key, {"x":x, "y":y, "hits":0, "status":"unknown"})
            e["x"], e["y"] = x, y
            e["status"] = st
            # simple certainty bump if “seen”
            e["hits"] = min(10, e.get("hits",0) + 1)
            e["stamp"] = now
            self._trees[key] = e
        # optional merge nearby to keep it ≤ ~75
        self._merge_close()

    def _on_joy(self, msg: Joy):
        try:
            self._teleop_held = bool(msg.buttons[self._teleop_btn] > 0)
        except Exception:
            self._teleop_held = False

    # ------------- geometry helpers -------------
    def _make_poly(self, flat: List[float]):
        if not flat or len(flat) < 6:  # need at least 3 points
            return None
        pts = []
        it = iter(flat)
        for x in it:
            try:
                y = next(it)
            except StopIteration:
                break
            pts.append((float(x), float(y)))
        return pts if len(pts) >= 3 else None

    def _inside_room(self, x: float, y: float) -> bool:
        if self.room_radius_m > 0.0:
            if math.hypot(x, y) > self.room_radius_m + 1e-6:
                return False
        if self._room_poly:
            # point-in-polygon (ray casting)
            cnt = 0
            for i in range(len(self._room_poly)):
                x1,y1 = self._room_poly[i]
                x2,y2 = self._room_poly[(i+1) % len(self._room_poly)]
                if ((y1 > y) != (y2 > y)):
                    xin = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-9) + x1
                    if x < xin: cnt += 1
            return (cnt % 2) == 1
        return True

    def _merge_close(self):
        if not self._trees:
            return
        keys = list(self._trees.keys())
        taken = set()
        merged = {}
        for i, k in enumerate(keys):
            if k in taken: continue
            xi, yi = k
            base = self._trees[k].copy()
            for j in range(i+1, len(keys)):
                kj = keys[j]
                if kj in taken: continue
                xj, yj = kj
                if math.hypot(xi - xj, yi - yj) <= self.min_sep:
                    # merge into base (keep stronger status/hits)
                    base["x"] = (base["x"] + xj)/2.0
                    base["y"] = (base["y"] + yj)/2.0
                    base["hits"] = max(base["hits"], self._trees[kj].get("hits",0))
                    if self._trees[kj]["status"] == "bad":
                        base["status"] = "bad"
                    taken.add(kj)
            merged[(round(base["x"],2), round(base["y"],2))] = base
        self._trees = merged

    # ------------- autonomy cycle -------------
    def _tick(self):
        # teleop interrupt: pause autonomy immediately
        if self._teleop_held:
            if self._phase != "teleop":
                self._announce("teleop")
                self.get_logger().info("Takeover confirmed (teleop held).")
            return

        # phases
        if self._phase == "init":
            if (self.get_clock().now() - self._t0) >= Duration(seconds=self.settle_s):
                self._phase = "scanning"
                self._scan_t0 = self.get_clock().now()
                self._announce("scanning")
                self.get_logger().info(f"Scanning for {self.scan_s:.1f}s…")
        elif self._phase == "scanning":
            if (self.get_clock().now() - self._scan_t0) >= Duration(seconds=self.scan_s):
                self._phase = "planning"
                self._announce("planning")
        elif self._phase == "planning":
            targets = self._choose_targets()
            if not targets:
                # nothing left to improve – mission complete
                self._phase = "complete"
                self._announce("complete")
                self.done_pub.publish(Bool(data=True))
                self.get_logger().info("Mission complete.")
                return
            self._send_waypoints(targets)
            self._phase = "executing"
            self._announce("executing")
        elif self._phase == "executing":
            # light monitor: if the server cancels or we flip back to planning after delay
            # here we just sleep a little then plan again (simple iterative improve)
            self._phase = "planning"
            self._announce("planning")
        elif self._phase == "teleop":
            # wait until teleop released
            if not self._teleop_held:
                self._phase = "planning"
                self._announce("planning")
        elif self._phase == "complete":
            pass
        else:
            self._phase = "init"
            self._announce("idle")

    def _uncertainty(self, x: float, y: float) -> float:
        # simple: farther than confirm_range -> high uncertainty, closer -> low
        d = math.hypot(x - self._robot_xy[0], y - self._robot_xy[1])
        if not math.isfinite(d): d = self.confirm_range
        return max(0.0, min(1.0, d / max(0.1, self.confirm_range)))

    def _choose_targets(self) -> List[Tuple[float,float]]:
        if not self._trees:
            return []
        # only consider trees that are not “good/bad” confirmed near us
        candidates = []
        for k, e in self._trees.items():
            x, y = e["x"], e["y"]
            if not self._inside_room(x, y):
                continue
            u = self._uncertainty(x, y)
            # if clearly “good” or “bad” and seen multiple times nearby, skip
            if e["status"] in ("good","bad") and e.get("hits",0) >= 2 and u < 0.3:
                continue
            d = math.hypot(x - self._robot_xy[0], y - self._robot_xy[1])
            # don’t try targets way past lidar reach (room edge)
            if d > self.lidar_max_m + 2.0:
                continue
            score = (d if self.furthest_bias else -d) + 10.0*u
            candidates.append((score, (x,y)))
        if not candidates:
            return []
        # sort and take a batch
        candidates.sort(reverse=True)
        pts = [xy for _, xy in candidates[:max(1, self.batch_size)]]
        return pts

    def _send_waypoints(self, pts_xy):
        if not pts_xy:
            return

        # PoseArray mirror for UI
        pa = PoseArray()
        pa.header.frame_id = MAP_FRAME
        pa.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in pts_xy:
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(x), float(y), 0.0
            qx, qy, qz, qw = yaw_to_quat(0.0)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            pa.poses.append(p)
        self.way_pub.publish(pa)

        # Action: must be PoseStamped[]
        if not self.way_client.server_is_ready():
            self.get_logger().warn("Waypoint server not ready.")
            return

        goal = FollowWaypoints.Goal()
        now = self.get_clock().now().to_msg()
        stamped = []
        for p in pa.poses:
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.header.stamp = now
            ps.pose = p
            stamped.append(ps)
        goal.poses = stamped
        self._last_sent = pts_xy

        fut = self.way_client.send_goal_async(goal)
        def _accepted(_f):
            gh = _f.result()
            if not gh.accepted:
                self.get_logger().warn("Waypoints rejected.")
                return
            self.get_logger().info(f"Waypoints accepted ({len(stamped)})")
            gh.get_result_async().add_done_callback(lambda _r: self.get_logger().info("Waypoints finished."))
        fut.add_done_callback(_accepted)

    def _announce(self, name: str):
        self.state_pub.publish(String(data=name))

def main():
    rclpy.init()
    node = Autonomy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
