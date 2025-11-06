#!/usr/bin/env python3
import math, time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

@dataclass
class Rates:
    idle_per_sec: float
    k_lin: float
    k_ang: float
    v_max: float
    w_max: float

class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_sim')

        # -------- params (all can be overridden from launch) --------
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('use_sim_time', True)

        # 3S LiPo-ish curve (demo values): full=12.6V, empty=9.6V
        self.declare_parameter('voltage_full', 12.6)
        self.declare_parameter('voltage_empty', 9.6)

        # Idle drains 100% → 0% in ~600s (10min)
        idle_per_sec = 1.0 / 600.0
        self.declare_parameter('idle_drain_per_sec', idle_per_sec)

        # Motion scaling (dimensionless). With v≈v_max and w≈w_max,
        # drain ≈ idle * (1 + k_lin + k_ang). Tweak for drama.
        self.declare_parameter('k_lin', 3.0)
        self.declare_parameter('k_ang', 1.5)
        self.declare_parameter('v_max', 0.6)   # m/s (demo)
        self.declare_parameter('w_max', 1.5)   # rad/s (demo)

        # Initial SOC (0..1)
        self.declare_parameter('initial_soc', 1.0)

        # -------- resolve params --------
        batt_topic = self.get_parameter('battery_topic').get_parameter_value().string_value
        cmd_topic  = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.volt_full = float(self.get_parameter('voltage_full').value)
        self.volt_empty = float(self.get_parameter('voltage_empty').value)

        self.rates = Rates(
            idle_per_sec=float(self.get_parameter('idle_drain_per_sec').value),
            k_lin=float(self.get_parameter('k_lin').value),
            k_ang=float(self.get_parameter('k_ang').value),
            v_max=max(1e-6, float(self.get_parameter('v_max').value)),
            w_max=max(1e-6, float(self.get_parameter('w_max').value)),
        )
        self.soc = max(0.0, min(1.0, float(self.get_parameter('initial_soc').value)))

        # -------- pubs/subs --------
        qos_cmd = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self._on_cmd, qos_cmd)

        self.pub_batt = self.create_publisher(BatteryState, batt_topic, 10)

        # state
        self._last_twist = Twist()
        self._last_time = self.get_clock().now()

        # 10 Hz update
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"BatterySim on {batt_topic} (cmd from {cmd_topic}); "
            f"idle drain ~10min; v_full={self.volt_full:.1f}V v_empty={self.volt_empty:.1f}V"
        )

    def _on_cmd(self, msg: Twist):
        self._last_twist = msg

    def _current_drain_per_sec(self) -> float:
        v = abs(self._last_twist.linear.x) + abs(self._last_twist.linear.y) + abs(self._last_twist.linear.z)
        w = abs(self._last_twist.angular.z) + abs(self._last_twist.angular.x) + abs(self._last_twist.angular.y)
        lin_term = min(1.0, v / self.rates.v_max)
        ang_term = min(1.0, w / self.rates.w_max)
        return self.rates.idle_per_sec * (1.0 + self.rates.k_lin * lin_term + self.rates.k_ang * ang_term)

    def _soc_to_voltage(self, soc: float) -> float:
        # simple LiPo-ish shape: slight plateaus around mid SOC
        s = max(0.0, min(1.0, soc))
        # quadratic ease for a tiny knee: feel more "drone-like"
        eased = 0.15 + 0.85 * (1.0 - (1.0 - s) * (1.0 - s))
        return self.volt_empty + (self.volt_full - self.volt_empty) * eased

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self._last_time = now

        drain = self._current_drain_per_sec() * dt
        self.soc = max(0.0, self.soc - drain)

        msg = BatteryState()
        msg.header.stamp = now.to_msg()

        msg.voltage = float(self._soc_to_voltage(self.soc))
        msg.current = float('nan')           # not modeled
        msg.charge = float('nan')            # not modeled
        msg.capacity = float('nan')          # not modeled
        msg.design_capacity = float('nan')   # not modeled
        msg.percentage = self.soc            # 0..1
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if self.soc > 0.0 else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True

        self.pub_batt.publish(msg)

def main():
    rclpy.init()
    node = BatterySim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
