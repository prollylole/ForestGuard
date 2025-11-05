#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, qos_profile_sensor_data,
    ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class TwistScaler(Node):
    """
    Scales /cmd_vel_raw by a factor from /ui/speed_cmd (Float32) and republishes on /cmd_vel.
    Publishes the authoritative, clamped, latched speed on /ui/speed.

    Flow:
      GUI  ----(Float32)---->  /ui/speed_cmd
      Scaler <---subscribes----/
      Scaler ----(latched)----> /ui/speed        # single source of truth for UIs
      GUI    <---subscribes----/

      /cmd_vel_raw --scaled by 'speed'--> /cmd_vel
    """

    def __init__(self):
        super().__init__('twist_scaler')

        # ---- parameters
        self.declare_parameter('in_topic',           '/cmd_vel_raw')
        self.declare_parameter('out_topic',          '/cmd_vel')
        self.declare_parameter('speed_cmd_topic',    '/ui/speed_cmd')   # GUI publishes here
        self.declare_parameter('speed_state_topic',  '/ui/speed')       # scaler publishes here (latched)
        self.declare_parameter('default_scale',      0.60)
        self.declare_parameter('min_scale',          0.10)
        self.declare_parameter('max_scale',          1.50)
        self.declare_parameter('eps',                0.01)
        # Optional legacy input if you *must* listen to an old publisher
        self.declare_parameter('enable_legacy_input', False)
        self.declare_parameter('legacy_speed_in_topic', '/ui/speed')

        in_topic   = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic  = self.get_parameter('out_topic').get_parameter_value().string_value
        speed_cmd  = self.get_parameter('speed_cmd_topic').get_parameter_value().string_value
        speed_state= self.get_parameter('speed_state_topic').get_parameter_value().string_value
        self.min_s = float(self.get_parameter('min_scale').value)
        self.max_s = float(self.get_parameter('max_scale').value)
        self.eps   = float(self.get_parameter('eps').value)
        self.enable_legacy_input = bool(self.get_parameter('enable_legacy_input').value)
        legacy_in  = self.get_parameter('legacy_speed_in_topic').get_parameter_value().string_value

        # ---- current scale
        self.scale = float(self.get_parameter('default_scale').value)
        self.scale = max(self.min_s, min(self.max_s, self.scale))

        # ---- QoS
        qos_cmd_out = QoSProfile(depth=10)
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,   # latched
            history=HistoryPolicy.KEEP_LAST,
        )

        # ---- pubs/subs
        self.sub_cmd  = self.create_subscription(Twist, in_topic, self._on_cmd, qos_profile_sensor_data)
        self.pub_cmd  = self.create_publisher(Twist, out_topic, qos_cmd_out)

        # authoritative, latched state for UIs
        self.pub_speed = self.create_publisher(Float32, speed_state, qos_latched)

        # accept GUI commands here
        self.sub_speed_cmd = self.create_subscription(Float32, speed_cmd, self._on_speed_cmd, 10)

        # optional legacy input (NOT recommended; can reintroduce loops if another node also republishes)
        if self.enable_legacy_input and legacy_in != speed_state:
            self.sub_speed_legacy = self.create_subscription(Float32, legacy_in, self._on_speed_cmd, 10)
        else:
            self.sub_speed_legacy = None

        self.get_logger().info(
            f"TwistScaler: {in_topic} -> {out_topic}; init scale={self.scale:.2f} "
            f"(min={self.min_s:.2f}, max={self.max_s:.2f}); "
            f"cmd_in={speed_cmd} state_out={speed_state}"
            + (f" legacy_in={legacy_in}" if self.sub_speed_legacy else "")
        )

        # publish initial state so new UIs sync immediately
        self._publish_speed(self.scale, reason="initial")

        # small anti-bounce memory to avoid reacting to our own just-published value
        self._last_pub_value = self.scale
        self._last_pub_time  = time.monotonic()

        # throttle debug
        self._last_dbg_ns = 0

    # ---------- callbacks ----------

    def _on_speed_cmd(self, msg: Float32):
        """Handle incoming GUI speed commands; clamp and publish authoritative state if changed."""
        req = float(msg.data)
        clamped = max(self.min_s, min(self.max_s, req))

        # ignore if indistinguishable
        if not math.isfinite(clamped) or abs(clamped - self.scale) <= self.eps:
            return

        # light guard against our own echo if someone (wrongly) bridges state->cmd
        now = time.monotonic()
        if abs(clamped - (self._last_pub_value or clamped+999)) <= self.eps and (now - self._last_pub_time) < 0.05:
            return

        self.scale = clamped
        self._publish_speed(self.scale, reason="updated")

    def _on_cmd(self, msg: Twist):
        """Scale incoming Twist and republish."""
        s = self.scale
        if abs(s - 1.0) < 1e-6:
            self.pub_cmd.publish(msg)
            return

        out = Twist()
        out.linear.x  = msg.linear.x  * s
        out.linear.y  = msg.linear.y  * s
        out.linear.z  = msg.linear.z  * s
        out.angular.x = msg.angular.x * s
        out.angular.y  = msg.angular.y * s
        out.angular.z  = msg.angular.z * s
        if out.linear.x < -1e-4:
            out.angular.z = -out.angular.z
        self.pub_cmd.publish(out)

        # throttled debug (2 Hz)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_dbg_ns > 500_000_000:
            self._last_dbg_ns = now_ns
            self.get_logger().debug(f"Scaled cmd with s={s:.2f}")

    # ---------- helpers ----------

    def _publish_speed(self, val: float, reason: str):
        msg = Float32(data=float(val))
        self.pub_speed.publish(msg)
        self._last_pub_value = float(val)
        self._last_pub_time  = time.monotonic()
        self.get_logger().info(f"Updated scale -> {val:.2f} ({reason})")


def main(args=None):
    rclpy.init(args=args)
    node = TwistScaler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
