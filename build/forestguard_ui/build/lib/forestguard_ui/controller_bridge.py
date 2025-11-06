import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

IMG_MSG_TYPE = 'sensor_msgs/msg/Image'


class ControllerBridge(Node):
    def __init__(self):
        super().__init__('controller_bridge')

        # Speed scales (teleop_twist_joy should use this as a "hint" via /ui/speed)
        self.lin_scale = 0.5
        self.ang_scale = 1.0  # kept for parity if you later add angular adjustments

        # Camera topic discovery + fallbacks
        self.fallback_cameras = [
            '/camera/image',
            '/camera/depth/image',
            '/camera/front/image_raw',
            '/camera/left/image_raw',
            '/camera/back/image_raw',
            '/camera/right/image_raw',
        ]
        self._live_cameras = []     # discovered Image topics with >=1 publisher
        self.current_cam_idx = -1   # start at -1 so first cycle picks index 0

        # Debounce state
        self._prev_buttons = []
        self._prev_axes = []

        # Optional per-button cooldown to guard against jitter (Y & LB mainly)
        self._cooldown = Duration(seconds=0.25)
        self._last_fire = {}  # key: ('btn', idx) -> rclpy.time.Time

        # Subscriptions
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        # UI / helper pubs
        self.speed_pub = self.create_publisher(Float32, '/ui/speed', 10)
        self.cam_pub   = self.create_publisher(String,  '/ui/camera_topic', 10)
        self.event_pub = self.create_publisher(String,  '/ui/event', 10)

        # Lazy /cmd_vel zero publisher for E-STOP
        self._zero_pub = None

        # Periodic discovery of live camera topics
        self.create_timer(1.0, self._refresh_live_cameras)

    # ------------------- helpers -------------------

    def _refresh_live_cameras(self):
        topics_types = dict(self.get_topic_names_and_types())
        live = []
        for t, types in topics_types.items():
            if IMG_MSG_TYPE in types and self.count_publishers(t) > 0:
                live.append(t)
        live.sort()
        if live != self._live_cameras:
            self._live_cameras = live
            self.get_logger().info(f'Live camera topics: {self._live_cameras}')

    def _image_topic_pool(self):
        # Prefer discovered live topics; only use fallback if none found
        return self._live_cameras if self._live_cameras else self.fallback_cameras

    def _button_rising(self, buttons, idx, use_cooldown=True):
        """True exactly once on 0->1 transition; optional cooldown while held."""
        now = buttons[idx] if idx < len(buttons) else 0
        was = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
        if was == 0 and now == 1:
            if use_cooldown:
                key = ('btn', idx)
                tnow = self.get_clock().now()
                last = self._last_fire.get(key)
                if last is None or (tnow - last) > self._cooldown:
                    self._last_fire[key] = tnow
                    return True
                return False
            return True
        return False

    def _axis_edge(self, axes, idx, target):
        """Triggers once when axis moves into target (+1 or -1)."""
        now = axes[idx] if idx < len(axes) else 0.0
        was = self._prev_axes[idx] if idx < len(self._prev_axes) else 0.0
        return (was != target and now == target)

    # ------------------- callbacks -------------------

    def _on_joy(self, msg: Joy):
        # NOTE: We don't publish /cmd_vel here; teleop_twist_joy is already running.

        # D-pad up/down -> adjust speed (edge only)
        if self._axis_edge(msg.axes, 7, +1.0):
            self._adjust_speed(+0.1)
        if self._axis_edge(msg.axes, 7, -1.0):
            self._adjust_speed(-0.1)

        # LB (index 4) -> E-STOP (rising edge + cooldown)
        if self._button_rising(msg.buttons, 4, use_cooldown=True):
            self._estop()

        # Y (index 3) -> cycle camera (rising edge + cooldown)
        if self._button_rising(msg.buttons, 3, use_cooldown=True):
            self._cycle_camera()

        # Save states for next edge detection
        self._prev_buttons = list(msg.buttons)
        self._prev_axes = list(msg.axes)

    # ------------------- actions -------------------

    def _adjust_speed(self, delta):
        self.lin_scale = max(0.1, min(1.5, self.lin_scale + delta))
        self.get_logger().info(f'Speed scale {self.lin_scale:.2f}')
        self.speed_pub.publish(Float32(data=self.lin_scale))
        self.event_pub.publish(String(data=f'SPEED:{self.lin_scale:.2f}'))

    def _estop(self):
        self.get_logger().warn('E-STOP triggered!')
        if self._zero_pub is None:
            self._zero_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._zero_pub.publish(Twist())  # zero velocity once
        self.event_pub.publish(String(data='E-STOP'))

    def _cycle_camera(self):
        pool = self._image_topic_pool()
        if not pool:
            self.get_logger().warn('No camera topics available to cycle.')
            self.event_pub.publish(String(data='CAM:NONE'))
            return
        self.current_cam_idx = (self.current_cam_idx + 1) % len(pool)
        topic = pool[self.current_cam_idx]
        self.cam_pub.publish(String(data=topic))
        self.event_pub.publish(String(data=f'CAM:{topic}'))
        self.get_logger().info(f'Camera -> {topic}')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
