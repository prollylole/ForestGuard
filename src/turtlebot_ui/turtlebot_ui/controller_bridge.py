import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

class ControllerBridge(Node):
    def __init__(self):
        super().__init__('controller_bridge')
        self.lin_scale = 0.5
        self.ang_scale = 1.0
        self.cameras = [
            '/camera/front/image_raw',
            '/camera/left/image_raw',
            '/camera/back/image_raw',
            '/camera/right/image_raw'
        ]
        self.current_cam_idx = 0

        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_pub = self.create_publisher(Float32, '/ui/speed', 10)
        self.cam_pub = self.create_publisher(String, '/ui/camera_topic', 10)
        self.event_pub = self.create_publisher(String, '/ui/event', 10)

    def _on_joy(self, msg: Joy):
        # axes: [LS X, LS Y, RS X, RS Y, LT, RT, Dpad X, Dpad Y]
        # buttons: [A,B,X,Y,LB,RB,Back,Start,Power,LS,RS]
        lin = msg.axes[1] * self.lin_scale
        ang = msg.axes[3] * self.ang_scale

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.cmd_pub.publish(twist)

        # D-pad up/down adjust speed
        dpad_y = msg.axes[7]
        if dpad_y == 1.0:
            self._adjust_speed(0.1)
        elif dpad_y == -1.0:
            self._adjust_speed(-0.1)

        # LB = E-STOP
        if msg.buttons[4]:
            self._estop()

        # Y = camera cycle
        if msg.buttons[3]:
            self._cycle_camera()

    def _adjust_speed(self, delta):
        self.lin_scale = max(0.1, min(1.5, self.lin_scale + delta))
        self.get_logger().info(f'Speed scale {self.lin_scale:.2f}')
        self.speed_pub.publish(Float32(data=self.lin_scale))
        self.event_pub.publish(String(data=f'SPEED:{self.lin_scale:.2f}'))

    def _estop(self):
        self.get_logger().warn('E-STOP triggered!')
        self.cmd_pub.publish(Twist())  # zero velocity
        self.event_pub.publish(String(data='E-STOP'))

    def _cycle_camera(self):
        self.current_cam_idx = (self.current_cam_idx + 1) % len(self.cameras)
        topic = self.cameras[self.current_cam_idx]
        self.cam_pub.publish(String(data=topic))
        self.event_pub.publish(String(data=f'CAM:{topic}'))
        self.get_logger().info(f'Camera -> {topic}')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
