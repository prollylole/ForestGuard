#!/usr/bin/env python3
# forestguard_colour/tree_colour_confirmer.py
import math, numpy as np, cv2
from typing import Dict, Tuple
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray, UInt32
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

class TreeColourConfirmer(Node):
    """
    Confirms red/green per LiDAR tree:
    - Subscribes: /tree_positions (PoseArray in map), /camera/image
    - For each tree pose, projects bearing into image column using camera HFOV
    - Samples a narrow vertical stripe near the bottom half; HSV vote => red/green/unknown
    - Publishes: /tree_colour_counts [greens, reds], and /trees_coloured (MarkerArray)
    """
    def __init__(self):
        super().__init__("tree_colour_confirmer")
        self.bridge = CvBridge()
        self.frame = None

        # --- params
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('tree_pose_topic', '/trees/poses')
        self.declare_parameter('map_frame',   'map')
        self.declare_parameter('base_frame',  'base_link')
        self.declare_parameter('camera_hfov_deg', 120.0)   # matches URDF (2.0944 rad)
        self.declare_parameter('roi_ymin_frac', 0.35)      # only check bottom 65% of image
        self.declare_parameter('stripe_half_px', 16)       # half-width of the vertical stripe
        self.declare_parameter('min_votes', 40)            # min coloured pixels to accept a label
        self.declare_parameter('confirm_range_m', 8.0)     # require proximity before colouring

        # NOTE: launch may already inject use_sim_time; don't redeclare if present
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # HSV ranges
        self.declare_parameter('green_low',  [50,130,25])
        self.declare_parameter('green_high', [70,255,255])
        self.declare_parameter('red1_low',   [0,155,75])
        self.declare_parameter('red1_high',  [34,255,255])
        self.declare_parameter('red2_low',   [170,160,77])
        self.declare_parameter('red2_high',  [179,255,255])

        img_topic = self.get_parameter('image_topic').value
        poses_topic = self.get_parameter('tree_pose_topic').value
        self.create_subscription(Image, img_topic, self.on_image, 10)
        self.create_subscription(PoseArray, poses_topic, self.on_positions, 10)

        self.pub_counts  = self.create_publisher(Int32MultiArray, '/tree_colour_counts', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/trees_coloured', 10)
        self.pub_bad     = self.create_publisher(UInt32, '/trees/bad', 10)

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self._label_cache: Dict[Tuple[int, int], str] = {}

        # Startup log of resolved params (handy sanity check)
        self.get_logger().info(
            "TreeColourConfirmer ready\n"
            f"  image_topic={img_topic}\n"
            f"  tree_pose_topic={poses_topic}\n"
            f"  camera_hfov_deg={self.get_parameter('camera_hfov_deg').value}\n"
            f"  confirm_range_m={self.get_parameter('confirm_range_m').value}\n"
            f"  green_low={self.get_parameter('green_low').value}, green_high={self.get_parameter('green_high').value}\n"
            f"  red1_low={self.get_parameter('red1_low').value}, red1_high={self.get_parameter('red1_high').value}\n"
            f"  red2_low={self.get_parameter('red2_low').value}, red2_high={self.get_parameter('red2_high').value}"
        )

    def on_image(self, msg: Image):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def on_positions(self, poses: PoseArray):
        if self.frame is None or not poses.poses:
            return

        H, W = self.frame.shape[:2]
        hfov = math.radians(float(self.get_parameter('camera_hfov_deg').value))
        roi_ymin = int(float(self.get_parameter('roi_ymin_frac').value) * H)
        half = int(self.get_parameter('stripe_half_px').value)
        min_votes = int(self.get_parameter('min_votes').value)

        # HSV thresholds
        gL = np.array(self.get_parameter('green_low').value, dtype=np.uint8)
        gH = np.array(self.get_parameter('green_high').value, dtype=np.uint8)
        r1L = np.array(self.get_parameter('red1_low').value, dtype=np.uint8)
        r1H = np.array(self.get_parameter('red1_high').value, dtype=np.uint8)
        r2L = np.array(self.get_parameter('red2_low').value, dtype=np.uint8)
        r2H = np.array(self.get_parameter('red2_high').value, dtype=np.uint8)

        # Get T_base_map (maps points from map → base_link)
        try:
            T = self.tf.lookup_transform(
                self.get_parameter('base_frame').value,
                poses.header.frame_id or self.get_parameter('map_frame').value,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().throttle(2000, f"TF base_link<-map not ready: {e}")
            return

        tx = T.transform.translation.x
        ty = T.transform.translation.y
        q  = T.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        confirm_range = max(0.01, float(self.get_parameter('confirm_range_m').value))

        updated_labels: Dict[Tuple[int, int], str] = {}
        markers = MarkerArray()

        for i, p in enumerate(poses.poses):
            mx, my = p.position.x, p.position.y

            # map -> base: [bx, by] = R*yaw * [mx, my] + t
            bx = cy*mx - sy*my + tx
            by = sy*mx + cy*my + ty
            key = (int(round(mx * 100.0)), int(round(my * 100.0)))
            prev = self._label_cache.get(key, 'unknown')
            dist = math.hypot(bx, by)
            bearing = math.atan2(by, bx)
            within_fov = (bx > 0.3) and (abs(bearing) <= hfov / 2)

            measured = 'unknown'
            if within_fov:
                u = int((bearing / (hfov / 2)) * (W / 2) + (W / 2))
                u0 = max(0, u - half)
                u1 = min(W, u + half)
                stripe = hsv[roi_ymin:H, u0:u1]
                if stripe.size > 0 and dist <= confirm_range:
                    mg = cv2.inRange(stripe, gL, gH)
                    mr = cv2.inRange(stripe, r1L, r1H) | cv2.inRange(stripe, r2L, r2H)
                    vg = int(np.count_nonzero(mg))
                    vr = int(np.count_nonzero(mr))
                    if vg >= min_votes or vr >= min_votes:
                        measured = 'green' if vg > vr else 'red'

            if prev == 'red' and measured != 'red':
                label = 'red'
            elif measured == 'red':
                label = 'red'
            elif measured == 'green':
                label = 'green'
            elif measured == 'unknown' and prev in ('green', 'red'):
                label = prev
            else:
                label = measured

            if prev != label and label == 'red':
                self.get_logger().info(f"Tree at ({mx:.2f}, {my:.2f}) marked BAD")

            # coloured marker at tree position (map frame)
            m = Marker()
            m.header.frame_id = poses.header.frame_id or self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'trees_coloured'; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = mx, my, 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            if label == 'green':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.1, 0.9
            elif label == 'red':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.6, 0.6, 0.6

            confidence = 0.0
            if dist <= confirm_range:
                confidence = max(0.0, min(1.0, 1.0 - (dist / confirm_range)))
            display_alpha = 0.4 + 0.5 * confidence
            m.color.a = float(min(1.0, max(0.05, display_alpha)))

            markers.markers.append(m)
            updated_labels[key] = label
            self._label_cache[key] = label

        # prune entries that vanished from poses
        stale = set(self._label_cache.keys()) - set(updated_labels.keys())
        for k in stale:
            del self._label_cache[k]

        greens = sum(1 for v in self._label_cache.values() if v == 'green')
        reds = sum(1 for v in self._label_cache.values() if v == 'red')

        # publish counts and coloured markers
        self.pub_markers.publish(markers)
        counts = Int32MultiArray(); counts.data = [greens, reds]
        self.pub_counts.publish(counts)
        self.pub_bad.publish(UInt32(data=reds))

def main():
    rclpy.init()
    n = TreeColourConfirmer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
