import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
import sys
import termios
import tty


MSG = """
Moving around:
        w
    a   s    d
        x

t/l: takeoff/land (upper/lower case)
q/e : increase/decrease linear and angular velocity (upper/lower case)
A/D: rotate left/right
r/f : rise/fall (upper/lower case)

---------------------------
CTRL-C to quit
---------------------------

"""


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_node')

        # service client for mission 
        self.mission_client = self.create_client(SetBool, '/X4/mission')
        # wait a short time for service to be available (non-blocking best, but we can wait a few seconds)
        if not self.mission_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/X4/mission service not available (will retry when requesting).')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/X4/cmd_vel', 10)
        # self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        # self.land_publisher = self.create_publisher(Empty, 'land', 10)

        # Velocity parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_increment = 0.05
        self.angular_increment = 0.05
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0

        # Start a timer to listen to keyboard inputs
        self.create_timer((1/30), self.read_keyboard_input)

    def call_mission_service(self, value: bool) -> None:
        """Call /X4/mission SetBool service (non-blocking)."""
        if not self.mission_client.service_is_ready():

            # try to wait a moment (you can also log and return instead)
            if not self.mission_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Mission service not available, cannot call.')
                return
            
        req = SetBool.Request()
        req.data = value
        future = self.mission_client.call_async(req)

        # callback to print response when available
        def _cb(fut):
            try:
                res = fut.result()
                self.get_logger().info(f'/X4/mission response: success=%s message="%s"' % (res.success, res.message))
            except Exception as e:
                self.get_logger().error(f'Failed calling /X4/mission: {e}')
        future.add_done_callback(_cb)

    def get_velocity_msg(self) -> str:
        return "Linear Velocity: " + str(self.linear_velocity) + "\nAngular Velocity: " \
            + str(self.angular_velocity) + "\n"

    def read_keyboard_input(self) -> None:
        """
        Read keyboard inputs and publish corresponding commands
        """
        try:
            # while rclpy.ok():
                # Print the instructions
                print(MSG+self.get_velocity_msg())
                # Implement a non-blocking keyboard read
                key = self.get_key()
                
                # Handle velocity changes
                if key.lower() == 'q':
                    self.linear_velocity = min(self.linear_velocity + self.linear_increment,
                                            self.max_linear_velocity)
                    self.angular_velocity = min(self.angular_velocity + self.angular_increment,
                                                self.max_angular_velocity)
                elif key.lower() == 'e':
                    self.linear_velocity = max(self.linear_velocity - self.linear_increment,
                                            -self.max_linear_velocity)
                    self.angular_velocity = max(self.angular_velocity - self.angular_increment,
                                                -self.max_angular_velocity)
                elif key.lower() == 'w':
                    # Move forward
                    linear_vec = Vector3()
                    linear_vec.x = self.linear_velocity
                    self.publish_cmd_vel(linear_vec)
                elif key.lower() == 's':
                    # Hover
                    self.publish_cmd_vel()
                elif key.lower() == 'x':
                    # Move backward
                    linear_vec = Vector3()
                    linear_vec.x = -self.linear_velocity
                    self.publish_cmd_vel(linear_vec)
                elif key == 'a':
                    # Move Left
                    linear_vec = Vector3()
                    linear_vec.y = self.linear_velocity
                    self.publish_cmd_vel(linear_vec)
                elif key == 'd':
                    # Move right
                    linear_vec = Vector3()
                    linear_vec.y = -self.linear_velocity
                    self.publish_cmd_vel(linear_vec)
                elif key == 'A':
                    # Rotate Left
                    angular_vec = Vector3()
                    angular_vec.z = self.angular_velocity
                    self.publish_cmd_vel(angular_vec=angular_vec)
                elif key == 'D':
                    # Rotate right
                    angular_vec = Vector3()
                    angular_vec.z = -self.angular_velocity
                    self.publish_cmd_vel(angular_vec=angular_vec)
                elif key.lower() == 'r':
                    # Rise
                    linear_vec = Vector3()
                    linear_vec.z = self.linear_velocity
                    self.publish_cmd_vel(linear_vec)
                elif key.lower() == 'f':
                    # Fall
                    linear_vec = Vector3()
                    linear_vec.z = -self.angular_velocity
                    self.publish_cmd_vel(linear_vec)
                # Handle other keys for different movements
                elif key.lower() == 't':
                    # Takeoff
                    self.call_mission_service(True)
                elif key.lower() == 'l':
                    # Land
                    self.publish_cmd_vel()
                    self.call_mission_service(False)

        except Exception as e:
            self.get_logger().error(f'Error in keyboard handler: {e}')

        finally:
            # Stop the drone and shutdown ROS2 cleanly
            self.publish_cmd_vel()
            rclpy.shutdown()

    def get_key(self) -> str:
        """
        Function to capture keyboard input
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_cmd_vel(self, linear_vec: Vector3 = Vector3(),
                        angular_vec: Vector3 = Vector3()) -> None:
        """
        Publish a Twist message to cmd_vel topic
        """
        twist = Twist(linear=linear_vec, angular=angular_vec)
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()