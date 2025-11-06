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
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from nav2_msgs.action import FollowWaypoints
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

MAP_FRAME = "map"

def yaw_to_quat(yaw: float):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)

class Autonomy(Node):
    def __init__(self):
        super().__init__("autonomy_behaviour")

        # ---- parameters
        self.declare_parameter("settle_time_s", 5.0)
        self.declare_parameter("scan_time_s",   3.0)
        self.declare_parameter("confirm_range", 8.0)
        self.declare_parameter("batch_size",    4)
        self.declare_parameter("lidar_max_m",   12.0)
        self.declare_parameter("room_radius_m", 50.0)
        self.declare_parameter("room_polygon_xy", "[]")  # YAML string of [x1,y1,x2,y2,...]
        self.declare_parameter("min_separation_m", 2.0)
        self.declare_parameter("furthest_bias",  True)
        self.declare_parameter("joy_rb_index",   5)
        self.declare_parameter("min_markers_to_start", 1)  # wait for at least N trees seen
        self.declare_parameter("perception_trigger_service", "/perception/snapshot")  # std_srvs/Trigger
        self.declare_parameter("start_topic", "/ui/start_mission")  # std_msgs/Bool true
        self.declare_parameter("amcl_topic", "/amcl_pose")
        self.declare_parameter("max_scan_retries", 2)

        self.settle_s       = float(self.get_parameter("settle_time_s").value)
        self.scan_s         = float(self.get_parameter("scan_time_s").value)
        self.confirm_range  = float(self.get_parameter("confirm_range").value)
        self.batch_size     = int(self.get_parameter("batch_size").value)
        self.lidar_max_m    = float(self.get_parameter("lidar_max_m").value)
        self.room_radius_m  = float(self.get_parameter("room_radius_m").value)
        self.min_sep        = float(self.get_parameter("min_separation_m").value)
        self.furthest_bias  = bool(self.get_parameter("furthest_bias").value)
        self._teleop_btn    = int(self.get_parameter("joy_rb_index").value)
        self._min_markers   = int(self.get_parameter("min_markers_to_start").value)
        self._start_topic   = str(self.get_parameter("start_topic").value)
        self._amcl_topic    = str(self.get_parameter("amcl_topic").value)
        self._max_scan_retries = max(0, int(self.get_parameter("max_scan_retries").value))

        # parse polygon once
        poly_str = str(self.get_parameter("room_polygon_xy").value or "[]")
        try:
            parsed = yaml.safe_load(poly_str)
            flat = [float(v) for v in parsed] if isinstance(parsed, list) else []
        except Exception:
            flat = []
        self._room_poly = self._make_poly(flat)

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
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(Bool, self._start_topic, self._on_start, 10)
        self.create_subscription(PoseWithCovarianceStamped, self._amcl_topic, self._on_amcl, 10)

        self.state_pub   = self.create_publisher(String, "/ui/autonomy_state", 10)
        self.done_pub    = self.create_publisher(Bool, "/ui/mission_complete", 10)
        self.way_pub     = self.create_publisher(PoseArray, "/goal_pose", 10)  # mirror to UI

        # action client
        self.way_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        # services
        self._perception = self.create_client(Trigger, self.get_parameter("perception_trigger_service").value)
        self._perception_warned = False

        # internal state
        self._t0 = self.get_clock().now()
        self._phase = "idle"          # idle until Start pressed
        self._armed = False           # becomes True on Start button
        self._trees: Dict[Tuple[float,float], Dict] = {}
        self._robot_xy = (0.0, 0.0)
        self._teleop_held = False
        self._teleop_prev = False
        self._last_sent: Optional[List[Tuple[float,float]]] = None
        self._last_markers_stamp = None
        self._waypoint_goal_handle = None
        self._scan_attempts = 0

        self._tick_timer = self.create_timer(0.25, self._tick)
        self._announce("idle")

    # ------------- inputs -------------
    def _on_start(self, msg: Bool):
        try:
            requested = bool(getattr(msg, "data", False))
        except Exception:
            requested = False
        if requested:
            if not self._armed:
                self._armed = True
                self._phase = "init"
                self._t0 = self.get_clock().now()
                self._scan_attempts = 0
                self._trees.clear()
                self._perception_warned = False
                self.get_logger().info("Commencing mission!")
                self._announce("init")
        else:
            if self._armed or self._phase not in ("idle", "complete"):
                self.get_logger().info("Autonomy stop requested.")
            self._armed = False
            self._phase = "idle"
            self._scan_attempts = 0
            self._trees.clear()
            self._perception_warned = False
            self._cancel_active_goal()
            try:
                self.done_pub.publish(Bool(data=False))
            except Exception:
                pass
            self._announce("idle")

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        self._robot_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def _on_markers(self, msg: MarkerArray):
        now = self.get_clock().now()
        updated = False
        for m in msg.markers:
            if m.action in (m.DELETE, m.DELETEALL):
                continue
            x, y = float(m.pose.position.x), float(m.pose.position.y)
            if not self._inside_room(x, y):
                continue
            key = (round(x,2), round(y,2))
            st = "unknown"
            r,g,b,a = m.color.r, m.color.g, m.color.b, m.color.a
            if r > g: st = "bad"
            elif g > r: st = "good"
            e = self._trees.get(key, {"x":x, "y":y, "hits":0, "status":"unknown"})
            e["x"], e["y"] = x, y
            e["status"] = st
            e["hits"] = min(10, e.get("hits",0) + 1)
            e["stamp"] = now
            self._trees[key] = e
            updated = True
        if updated:
            self._last_markers_stamp = now
            self._merge_close()

    def _on_joy(self, msg: Joy):
        held = False
        try:
            held = bool(msg.buttons[self._teleop_btn] > 0)
        except Exception:
            held = False
        # edge-triggered logging
        if held and not self._teleop_held:
            self.get_logger().info("Takeover confirmed (teleop held).")
        if (not held) and self._teleop_held:
            self.get_logger().info("Teleop released.")
        self._teleop_held = held

    # ------------- geometry helpers -------------
    def _make_poly(self, flat):
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
        if self.room_radius_m > 0.0 and math.hypot(x, y) > self.room_radius_m + 1e-6:
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
        # pause autonomy on teleop
        if self._teleop_held:
            self._announce("teleop")
            return

        if not self._armed:
            # sit idle until Start pressed
            return

        # phases
        if self._phase == "init":
            # optional: kick perception once at start
            self._call_perception_once()
            if (self.get_clock().now() - self._t0) >= Duration(seconds=self.settle_s):
                self._phase = "scanning"
                self._scan_t0 = self.get_clock().now()
                self._scan_attempts = 0
                self._announce("scanning")
                self.get_logger().info(f"Scanning for {self.scan_s:.1f}s…")

        elif self._phase == "scanning":
            # wait for scan window AND at least some markers
            enough = (len(self._trees) >= self._min_markers)
            elapsed = self.get_clock().now() - self._scan_t0
            if enough:
                self._phase = "planning"
                self._announce("planning")
                self._scan_attempts = 0
            elif elapsed >= Duration(seconds=self.scan_s):
                if self._scan_attempts >= self._max_scan_retries:
                    if self._max_scan_retries > 0:
                        self.get_logger().warn("Scan retries exhausted without markers; proceeding to planning.")
                    if not self._trees:
                        # Nothing seen yet; stay in scanning and try again later.
                        self._scan_attempts = 0
                        self._scan_t0 = self.get_clock().now()
                        self._announce("scanning")
                        self._call_perception_once()
                        return
                    self._phase = "planning"
                    self._announce("planning")
                    self._scan_attempts = 0
                else:
                    self._scan_attempts += 1
                    self.get_logger().info("No trees seen yet; triggering perception burst and rescanning.")
                    self._call_perception_once()
                    self._scan_t0 = self.get_clock().now()
                    return

        elif self._phase == "planning":
            targets = self._choose_targets()
            if not targets:
                # If we literally never saw markers recently, keep scanning instead of "complete"
                if self._last_markers_stamp is None or (self.get_clock().now() - self._last_markers_stamp) > Duration(seconds=5.0):
                    self._phase = "scanning"
                    self._scan_t0 = self.get_clock().now()
                    self._scan_attempts = 0
                    self._announce("scanning")
                    return
                # else declare complete
                self._transition_to_complete()
                return
            self._send_waypoints(targets)
            self._phase = "executing"
            self._announce("executing")

        elif self._phase == "executing":
            # wait for action result callback to flip back to planning
            pass

        elif self._phase == "complete":
            pass

        else:
            self._phase = "init"
            self._announce("idle")

    def _uncertainty(self, x: float, y: float) -> float:
        d = math.hypot(x - self._robot_xy[0], y - self._robot_xy[1])
        if not math.isfinite(d): d = self.confirm_range
        return max(0.0, min(1.0, d / max(0.1, self.confirm_range)))

    def _choose_targets(self) -> List[Tuple[float,float]]:
        if not self._trees:
            return []
        candidates = []
        for e in self._trees.values():
            x, y = e["x"], e["y"]
            if not self._inside_room(x, y):
                continue
            u = self._uncertainty(x, y)
            if e["status"] in ("good","bad") and e.get("hits",0) >= 2 and u < 0.3:
                continue
            d = math.hypot(x - self._robot_xy[0], y - self._robot_xy[1])
            if d > self.lidar_max_m + 2.0:
                continue
            score = (d if self.furthest_bias else -d) + 10.0*u
            candidates.append((score, (x,y)))
        if not candidates:
            return []
        candidates.sort(reverse=True)
        return [xy for _, xy in candidates[:max(1, self.batch_size)]]

    def _send_waypoints(self, pts_xy):
        if not pts_xy:
            return

        # PoseArray for UI
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
                self._phase = "planning"
                self._announce("planning")
                return
            self._waypoint_goal_handle = gh
            self.get_logger().info(f"Waypoints accepted ({len(stamped)})")
            gh.get_result_async().add_done_callback(self._on_waypoints_done)
        fut.add_done_callback(_accepted)

    def _on_waypoints_done(self, _res):
        # call perception once at the stop, then plan again
        self._call_perception_once()
        self._phase = "planning"
        self._announce("planning")

    def _call_perception_once(self):
        try:
            if self._perception.service_is_ready():
                req = Trigger.Request()
                self._perception.call_async(req)  # fire-and-forget, we only need the burst
                self._perception_warned = False
                return
        except Exception:
            pass
        if not self._perception_warned:
            self.get_logger().info("Perception trigger service not available; continuing without snapshot.")
            self._perception_warned = True

    def _transition_to_complete(self):
        if self._phase == "complete":
            return
        self._phase = "complete"
        self._announce("complete")
        try:
            self.done_pub.publish(Bool(data=True))
        except Exception:
            pass
        self._armed = False
        self.get_logger().info("Mission complete.")

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
