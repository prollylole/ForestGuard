#!/usr/bin/env python3
import math
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import yaml


class LidarGate(Node):
    def __init__(self):
        super().__init__("lidar_gate")
        self.declare_parameter("room_radius_m", 0.0)
        self.declare_parameter("room_polygon_xy", "[]")
        self.room_r = float(self.get_parameter("room_radius_m").value)
        poly_str = str(self.get_parameter("room_polygon_xy").value or "[]")
        try:
            parsed = yaml.safe_load(poly_str)
            self.poly = [float(v) for v in parsed] if isinstance(parsed, list) else []
        except Exception:
            self.poly = []

        self.sub = self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.pub = self.create_publisher(LaserScan, "/scan_gated", 10)

    def _inside_poly(self, x: float, y: float) -> bool:
        if not self.poly or len(self.poly) < 6:
            return True
        pts = [(self.poly[i], self.poly[i+1]) for i in range(0, len(self.poly)-1, 2)]
        cnt = 0
        for i in range(len(pts)):
            x1,y1 = pts[i]; x2,y2 = pts[(i+1)%len(pts)]
            if ((y1 > y) != (y2 > y)):
                xin = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-9) + x1
                if x < xin: cnt += 1
        return (cnt % 2) == 1

    def _on_scan(self, msg: LaserScan):
        out = LaserScan()
        out = msg  # shallow copy OK
        rng = list(msg.ranges)
        ang = msg.angle_min
        for i, r in enumerate(rng):
            ok = True
            if math.isfinite(r):
                x = r * math.cos(ang); y = r * math.sin(ang)
                if self.room_r > 0.0 and math.hypot(x,y) > self.room_r: ok = False
                if ok and not self._inside_poly(x,y): ok = False
            else:
                ok = False
            if not ok:
                rng[i] = float("inf")
            ang += msg.angle_increment
        out.ranges = rng
        self.pub.publish(out)

def main():
    rclpy.init()
    n = LidarGate()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
