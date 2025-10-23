#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TwistScaler(Node):
    """
    Scales /cmd_vel_raw by /ui/speed and republishes on /cmd_vel.
    """
    def __init__(self):
        super().__init__('twist_scaler')
        self.declare_parameter('in_topic',  '/cmd_vel_raw')
        self.declare_parameter('out_topic', '/cmd_vel')

        self.scale = 0.5  # same default as ControllerBridge
        in_topic  = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value

        self.sub_cmd = self.create_subscription(
            Twist, in_topic, self._on_cmd, qos_profile_sensor_data
        )
        self.pub_cmd = self.create_publisher(Twist, out_topic, QoSProfile(depth=10))
        self.sub_gain = self.create_subscription(Float32, '/ui/speed', self._on_speed, 10)

        self.get_logger().info(f'TwistScaler: {in_topic} -> {out_topic}; init scale={self.scale:.2f}')

    def _on_speed(self, msg: Float32):
        self.scale = max(0.1, min(1.5, float(msg.data)))
        self.get_logger().info(f'Updated scale -> {self.scale:.2f}')

    def _on_cmd(self, msg: Twist):
        if self.scale == 1.0:
            self.pub_cmd.publish(msg); return
        out = Twist()
        out.linear.x  = msg.linear.x  * self.scale
        out.linear.y  = msg.linear.y  * self.scale
        out.linear.z  = msg.linear.z  * self.scale
        out.angular.x = msg.angular.x * self.scale
        out.angular.y = msg.angular.y * self.scale
        out.angular.z = msg.angular.z * self.scale
        self.pub_cmd.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = TwistScaler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
