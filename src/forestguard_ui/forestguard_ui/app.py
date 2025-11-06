#!/usr/bin/env python3
# Robot UI (dark theme) — camera, D-pad (square), logs/tree-table stack, tree counter, speed slider
# + Run Sim + Rebuild Code + Map+Trees overlay + Depth Cloud counter + Teleop LED + Battery
from __future__ import annotations

import sys, os, math, signal
from typing import Optional, Dict, Tuple, List
from threading import Lock

from PySide6.QtCore import Qt, QObject, QThread, QTimer, Signal, Slot, QPointF, QRect, QProcess
from PySide6.QtGui import QColor, QPainter, QBrush, QImage, QPixmap, QIcon, QPalette
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QSlider, QFrame,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSizePolicy, QGraphicsDropShadowEffect,
    QStyleFactory, QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QTabWidget, QDoubleSpinBox, QSpinBox
)

# ---------- topics / env ----------
CAMERA_TOPIC_DEFAULT = "/camera/image"
ROSOUT_TOPIC         = "/rosout"
CAMERA_TOPIC_CTRL    = "/ui/camera_topic"
CMD_VEL_TOPIC        = "/cmd_vel"

BATTERY_TOPIC = os.environ.get("UI_BATTERY_TOPIC", "/battery_state")

TREE_TOTAL_TOPIC = os.environ.get("UI_TREE_TOTAL_TOPIC", "/trees/count")
TREE_BAD_TOPIC   = os.environ.get("UI_TREE_BAD_TOPIC",   "/trees/bad")

# Map/markers topics (can override via env)
MAP_TOPIC            = os.environ.get("UI_MAP_TOPIC", "/map")
ROBOT_POSE_TOPIC     = os.environ.get("UI_ROBOT_POSE_TOPIC", "/amcl_pose")
TREE_MARKERS_TOPIC_A = os.environ.get("UI_TREE_MARKERS", "/trees/markers")
TREE_MARKERS_TOPIC_B = os.environ.get("UI_TREE_MARKER",  "/trees_coloured")  # alt/fused colours

WAYPOINT_TOPIC = os.environ.get("UI_WAYPOINT_TOPIC", "/goal_pose")
SPEED_CMD_TOPIC = os.environ.get("UI_SPEED_CMD_TOPIC", "/ui/speed_cmd")
SPEED_STATE_TOPIC = os.environ.get("UI_SPEED_STATE_TOPIC", "/ui/speed")
GLOBAL_PATH_TOPIC = os.environ.get("UI_GLOBAL_PATH_TOPIC", "/plan")
LOCAL_PATH_TOPIC = os.environ.get("UI_LOCAL_PATH_TOPIC", "/local_plan")
GLOBAL_COSTMAP_TOPIC = os.environ.get("UI_GLOBAL_COSTMAP_TOPIC", "/global_costmap/costmap")
LOCAL_COSTMAP_TOPIC = os.environ.get("UI_LOCAL_COSTMAP_TOPIC", "/local_costmap/costmap")

# Point cloud source shown next to the Teleop LED
PC_TOPIC = os.environ.get("UI_PC_TOPIC", "/camera/depth/points")

# Joy: teleop enable button (Xbox RB=5)
JOY_TELEOP_BTN = int(os.environ.get("UI_JOY_TELEOP_BTN", "5"))
# Require teleop button to send /cmd_vel
REQUIRE_TELEOP_BTN = os.environ.get("UI_REQUIRE_TELEOP_BTN", "1") == "1"

# Sim heartbeat (uses /clock)
SIM_HEARTBEAT_TOPIC = os.environ.get("UI_SIM_HEARTBEAT_TOPIC", "/clock")
SIM_TIMEOUT_S = float(os.environ.get("UI_SIM_TIMEOUT_S", "1.0"))
MAP_EMIT_PERIOD_S = float(os.environ.get("UI_MAP_EMIT_PERIOD", "0.4"))
CPU_SAMPLE_PERIOD_S = float(os.environ.get("UI_CPU_SAMPLE_PERIOD", "1.0"))

CMD_PUB_RATE_HZ = 20
MAX_LINEAR_MPS  = 0.40
MAX_ANGULAR_RPS = 1.50

# Launch control (Run Sim button)
DEFAULT_LAUNCH_CMD = os.environ.get(
    "UI_LAUNCH_CMD",
    "ros2 launch forestguard_sim johnAUTO2.launch.py ui:=true teleop:=true amcl:=true slam:=false map:=true"
)

# Build control (Rebuild Code button)
_DEFAULT_WS = os.path.expanduser("~/git/RS1/john_branch")
DEFAULT_BUILD_CWD = os.environ.get("UI_BUILD_CWD", _DEFAULT_WS if os.path.isdir(_DEFAULT_WS) else os.getcwd())
DEFAULT_BUILD_PRE = os.environ.get("UI_BUILD_PRE", "source /opt/ros/humble/setup.bash")
DEFAULT_BUILD_CMD = os.environ.get(
    "UI_BUILD_CMD",
    "colcon build --symlink-install "
    "--packages-select forestguard_perception forestguard_controller "
    "forestguard_localisation forestguard_sim forestguard_ui"
)

# ---------- optional deps ----------
try:
    import numpy as np
except Exception:
    np = None

try:
    import cv2
except Exception:
    cv2 = None

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE_OK = True
except Exception:
    CvBridge = None
    _CV_BRIDGE_OK = False

try:
    import psutil
except Exception:
    psutil = None

# ---- ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.logging import LoggingSeverity
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, PoseArray
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CompressedImage as RosCompressedImage
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rcl_interfaces.msg import Log as RosLog
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32, UInt32, Float32, String, Bool
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from rosgraph_msgs.msg import Clock
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from tf2_ros import Buffer, TransformListener

# ========================= tiny widgets ===============================
class LedIndicator(QLabel):
    def __init__(self, color=QColor("#666666"), diameter=14, parent=None):
        super().__init__(parent)
        self._color = QColor(color)
        self._diameter = diameter
        self.setFixedSize(diameter, diameter)

    def set_color(self, color: QColor | str):
        self._color = QColor(color); self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QBrush(self._color))
        p.setPen(Qt.black)
        p.drawEllipse(0, 0, self._diameter, self._diameter)

class SquareContainer(QWidget):
    def __init__(self, child: QWidget, parent=None):
        super().__init__(parent)
        self._child = child
        self._child.setParent(self)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)

    def resizeEvent(self, _):
        w, h = self.width(), self.height()
        side = min(w, h)
        x = (w - side) // 2
        y = (h - side) // 2
        self._child.setGeometry(QRect(x, y, side, side))

class WaypointPanel(QWidget):
    def __init__(self, send_callback, pose_provider=None, parent=None):
        super().__init__(parent)
        self._send_callback = send_callback
        self._pose_provider = pose_provider
        self._rows: List[Dict[str, float]] = []
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        control_row = QHBoxLayout()
        control_row.setSpacing(8)
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-1000.0, 1000.0)
        self.spin_x.setDecimals(2)
        self.spin_x.setSingleStep(0.25)
        self.spin_x.setSuffix(" m")
        self.spin_x.setPrefix("X: ")

        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-1000.0, 1000.0)
        self.spin_y.setDecimals(2)
        self.spin_y.setSingleStep(0.25)
        self.spin_y.setSuffix(" m")
        self.spin_y.setPrefix("Y: ")

        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-360.0, 360.0)
        self.spin_yaw.setDecimals(1)
        self.spin_yaw.setSingleStep(5.0)
        self.spin_yaw.setSuffix("°")
        self.spin_yaw.setPrefix("Yaw: ")

        control_row.addWidget(self.spin_x)
        control_row.addWidget(self.spin_y)
        control_row.addWidget(self.spin_yaw)

        add_btn = QPushButton("Add waypoint")
        add_btn.clicked.connect(self._add_waypoint)
        control_row.addWidget(add_btn)

        pose_btn = QPushButton("Add current pose")
        pose_btn.clicked.connect(self._add_current_pose)
        control_row.addWidget(pose_btn)

        control_row.addStretch(1)
        layout.addLayout(control_row)

        self.table = QTableWidget(0, 4, self)
        self.table.setHorizontalHeaderLabels(["#", "X (m)", "Y (m)", "Yaw (deg)"])
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        layout.addWidget(self.table, 1)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)

        remove_btn = QPushButton("Remove selected")
        remove_btn.clicked.connect(self._remove_selected)
        btn_row.addWidget(remove_btn)

        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self._clear_all)
        btn_row.addWidget(clear_btn)

        send_btn = QPushButton("Send to robot")
        send_btn.clicked.connect(self._send)
        btn_row.addWidget(send_btn)

        btn_row.addStretch(1)
        layout.addLayout(btn_row)

    def set_pose_provider(self, provider):
        self._pose_provider = provider

    def _add_waypoint(self):
        entry = {
            "x": float(self.spin_x.value()),
            "y": float(self.spin_y.value()),
            "z": 0.0,
            "yaw_deg": float(self.spin_yaw.value()),
        }
        self._rows.append(entry)
        self._refresh_table()

    def _add_current_pose(self):
        if self._pose_provider is None:
            return
        pose = self._pose_provider()
        if pose is None:
            return
        x, y, yaw = pose
        entry = {"x": x, "y": y, "z": 0.0, "yaw_deg": math.degrees(yaw)}
        self._rows.append(entry)
        self._refresh_table()

    def _remove_selected(self):
        indexes = self.table.selectionModel().selectedRows()
        if not indexes:
            return
        row = indexes[0].row()
        if 0 <= row < len(self._rows):
            self._rows.pop(row)
            self._refresh_table()

    def _clear_all(self):
        if not self._rows:
            return
        self._rows.clear()
        self._refresh_table()

    def _send(self):
        if self._send_callback is None:
            return
        self._send_callback(list(self._rows))

    def _refresh_table(self):
        self.table.setRowCount(len(self._rows))
        for idx, entry in enumerate(self._rows):
            cells = [
                str(idx + 1),
                f"{entry['x']:.2f}",
                f"{entry['y']:.2f}",
                f"{entry['yaw_deg']:.1f}",
            ]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                self.table.setItem(idx, col, item)

# ========================= helpers ================================
def _level_from_str(name: str) -> LoggingSeverity:
    name = (name or "").strip().lower()
    return {
        "debug": LoggingSeverity.DEBUG,
        "info": LoggingSeverity.INFO,
        "warn":  LoggingSeverity.WARN,
        "warning": LoggingSeverity.WARN,
        "error": LoggingSeverity.ERROR,
        "fatal": LoggingSeverity.FATAL,
    }.get(name, LoggingSeverity.INFO)

def _rosout_level_num(name: str) -> int:
    name = (name or "").strip().lower()
    return {"debug":10, "info":20, "warn":30, "warning":30, "error":40, "fatal":50}.get(name, 20)

def _slider_to_scale(v: int) -> float:
    return 0.10 + (float(v) / 20.0) * (1.50 - 0.10)

def _scale_to_slider(s: float) -> int:
    s = max(0.10, min(1.50, float(s)))
    return int(round((s - 0.10) / (1.50 - 0.10) * 20.0))

def _yaw_from_q(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def _quat_from_yaw(yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)

# ========================= ROS <-> Qt bridge ========================
class RosSignals(QObject):
    image = Signal(object)
    map_image = Signal(object)
    robot_pose = Signal(object)
    speed_scale = Signal(float)
    estop_toggle = Signal()
    log = Signal(str)
    teleop_led = Signal(bool, bool)
    ok = Signal(bool, str)
    bump_speed = Signal(int)
    tree_counts = Signal(int, int)
    pc_points = Signal(int)
    tree_table = Signal(object)
    battery = Signal(float, float, bool)  # percent [0-100], voltage [V] (nan if unknown), charging
    camera_mode = Signal(str)
    autonomy = Signal(str) # "idle"/"scanning"/"planning"/"executing"/"teleop"/"complete"
    mission_done = Signal(bool)

# ========================= ROS Node ================================
class GuiRosNode(Node):
    def __init__(self, signals: RosSignals):
        super().__init__("forestguard_ui")
        self.signals = signals

        node_level = _level_from_str(os.environ.get("UI_LOG_LEVEL", "info"))
        try: self.get_logger().set_level(node_level)
        except Exception: pass

        self._rosout_min_level = _rosout_level_num(os.environ.get("UI_ROSOUT_LEVEL", "info"))
        self._prefer_compressed = os.environ.get("UI_CAMERA_COMPRESSED", "0") == "1"
        try:
            self._emit_hz = float(os.environ.get("UI_CAMERA_EMIT_HZ", "30"))
            self._emit_hz = max(1.0, min(60.0, self._emit_hz))
        except Exception:
            self._emit_hz = 30.0

        # QoS
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        # pubs
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.waypoint_pub = self.create_publisher(PoseArray, WAYPOINT_TOPIC, 10)
        self.speed_cmd_pub = self.create_publisher(Float32, SPEED_CMD_TOPIC, 10)

        # /rosout
        if os.environ.get("UI_DISABLE_ROSOUT", "0") != "1":
            self.create_subscription(RosLog, ROSOUT_TOPIC, self._on_rosout, self.reliable_qos)

        # camera topic control
        self.create_subscription(RosString, CAMERA_TOPIC_CTRL, self._on_camera_topic_switch, 10)

        # cv bridge
        self.bridge: Optional[object] = CvBridge() if _CV_BRIDGE_OK else None

        # camera state
        self._camera_sub = None
        self._camera_topic_raw = CAMERA_TOPIC_DEFAULT.rstrip('/')
        self._camera_mode = 'rgb'
        self._latest_rgb = None
        self._img_lock = Lock()
        self._emit_timer = self.create_timer(1.0 / self._emit_hz, self._emit_image_tick)
        self._subscribe_camera(self._camera_topic_raw)
        self._prev_cycle_btn = False
        self._btn_cycle = int(os.environ.get("UI_JOY_BTN_CAMERA", "3"))
        self._hsv_ready = (np is not None and cv2 is not None)
        if self._hsv_ready:
            self._green_low = np.array([50, 130, 25], dtype=np.uint8)
            self._green_high = np.array([70, 255, 255], dtype=np.uint8)
            self._red1_low = np.array([0, 155, 75], dtype=np.uint8)
            self._red1_high = np.array([34, 255, 255], dtype=np.uint8)
            self._red2_low = np.array([170, 160, 77], dtype=np.uint8)
            self._red2_high = np.array([179, 255, 255], dtype=np.uint8)
        else:
            self.get_logger().warn("HSV view unavailable (missing OpenCV/NumPy)")

        # point cloud quick stats (shown next to Teleop LED)
        try:
            self.create_subscription(PointCloud2, PC_TOPIC, self._on_pc, self.sensor_qos)
            self._pc_latest_count = 0
            self._pc_timer = self.create_timer(0.5, self._emit_pc_count)
            self.get_logger().info(f"Point cloud stats on {PC_TOPIC}")
        except Exception:
            pass

        # tree counters
        self._tree_total = 0
        self._tree_bad   = 0
        self._sub_total = self._subscribe_int_counter(TREE_TOTAL_TOPIC, is_bad=False)
        self._sub_bad   = self._subscribe_int_counter(TREE_BAD_TOPIC,   is_bad=True)
        self._speed_scale = float(os.environ.get("UI_SPEED_DEFAULT", "0.60"))
        self.create_subscription(Float32, SPEED_STATE_TOPIC, self._on_speed_state, 10)
        self._trail_points: List[Tuple[float, float]] = []
        self._trail_max = int(os.environ.get("UI_TRAIL_MAX", "1200"))

        # joystick bump + teleop enable (RB)
        joy_topic = os.environ.get("UI_JOY_TOPIC", "/joy")
        self._hat_axis_v = int(os.environ.get("UI_JOY_HAT_AXIS_V", "7"))
        self._btn_up = int(os.environ.get("UI_JOY_BTN_UP", "-1"))
        self._btn_down = int(os.environ.get("UI_JOY_BTN_DOWN", "-1"))
        self._teleop_btn = JOY_TELEOP_BTN
        self._btn_estop = int(os.environ.get("UI_JOY_BTN_ESTOP", "1"))
        self._prev_up = False
        self._prev_down = False
        self._prev_estop_btn = False
        self._teleop_enabled = False
        try:
            from sensor_msgs.msg import Joy
            self.create_subscription(Joy, joy_topic, self._on_joy, 10)
            self.get_logger().info(f"D-pad on {joy_topic} (axis_v={self._hat_axis_v}, btn_up={self._btn_up}, btn_down={self._btn_down}); teleop_btn={self._teleop_btn}")
        except Exception as e:
            self.get_logger().warn(f"Joy subscribe failed: {e}")

        # ---- Sim heartbeat (/clock) ----
        self._sim_alive = False
        self._last_clock_ns = 0
        try:
            self.create_subscription(Clock, SIM_HEARTBEAT_TOPIC, self._on_clock, 10)
            self._sim_timer = self.create_timer(0.2, self._check_sim_alive)
        except Exception as e:
            self.get_logger().warn(f"Clock subscribe failed: {e}")

        # ---- Map + trees overlay state ----
        self._map_msg: Optional[OccupancyGrid] = None
        self._map_lock = Lock()
        self._map_res = None
        self._map_origin = None  # (ox, oy)
        self._robot_pose = None  # (x, y, yaw)
        self._tree_lock = Lock()
        self._tree_states: Dict[Tuple[float, float], Dict[str, object]] = {}
        self._confirm_range_hint = float(os.environ.get("UI_CONFIRM_RANGE", "8.0"))
        self._path_lock = Lock()
        self._global_path_pts: List[Tuple[float, float]] = []
        self._local_path_pts: List[Tuple[float, float]] = []
        self._costmap_lock = Lock()
        self._global_costmap: Optional[OccupancyGrid] = None
        self._local_costmap: Optional[OccupancyGrid] = None
        self._map_frame_name = os.environ.get("UI_TREE_MAP_FRAME", "map")
        self._base_frame_name = os.environ.get("UI_TREE_BASE_FRAME", "base_link")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, 1)
        self.create_subscription(PoseWithCovarianceStamped, ROBOT_POSE_TOPIC, self._on_pose, 10)
        self.create_subscription(MarkerArray, TREE_MARKERS_TOPIC_A, self._on_markers, 10)
        if TREE_MARKERS_TOPIC_B != TREE_MARKERS_TOPIC_A:
            self.create_subscription(MarkerArray, TREE_MARKERS_TOPIC_B, self._on_markers, 10)
        self.create_subscription(Path, GLOBAL_PATH_TOPIC, self._on_global_path, 10)
        self.create_subscription(Path, LOCAL_PATH_TOPIC, self._on_local_path, 10)
        self.create_subscription(OccupancyGrid, GLOBAL_COSTMAP_TOPIC, self._on_global_costmap, 1)
        self.create_subscription(OccupancyGrid, LOCAL_COSTMAP_TOPIC, self._on_local_costmap, 1)
        self._map_timer = self.create_timer(MAP_EMIT_PERIOD_S, self._emit_map_image)
        self.create_subscription(String, "/ui/autonomy_state", lambda m: self.signals.autonomy.emit(m.data or ""), 10)
        self.create_subscription(Bool, "/ui/mission_complete", lambda m: self.signals.mission_done.emit(bool(m.data)), 10)

        self.signals.ok.emit(True, "ROS node initialised")
        # store downsampled XY for map overlays if you want them later
        self._pc_xy_samples = []
        self._pc_xy_lock = Lock()
        self._follow_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        # battery
        self._battery_pct = float('nan')
        self._battery_v   = float('nan')
        self._battery_chg = False
        try:
            types_map = dict(self.get_topic_names_and_types())
            bt_t = types_map.get(BATTERY_TOPIC, [])
            try:
                from sensor_msgs.msg import BatteryState
                _BatteryState = BatteryState
            except Exception:
                _BatteryState = None

            if _BatteryState and any("sensor_msgs/msg/BatteryState" in t for t in bt_t):
                self.create_subscription(_BatteryState, BATTERY_TOPIC, self._on_battery_state, 10)
                self.get_logger().info(f"Battery on {BATTERY_TOPIC} (BatteryState)")
            else:
                def _on_batt_f32(msg):
                    try:
                        v = float(getattr(msg, "data", float('nan')))
                        if v <= 1.01: pct = max(0.0, min(100.0, v * 100.0))
                        else:         pct = max(0.0, min(100.0, v))
                        self._battery_pct = pct
                        self.signals.battery.emit(self._battery_pct, self._battery_v, self._battery_chg)
                    except Exception:
                        pass
                self.create_subscription(Float32, BATTERY_TOPIC, _on_batt_f32, 10)
                self.get_logger().warn(f"Battery on {BATTERY_TOPIC} as Float32 (assuming percent)")
        except Exception as e:
            self.get_logger().warn(f"Battery subscribe failed: {e}")

    def _on_battery_state(self, msg):
        try:
            pct = float(getattr(msg, "percentage", float('nan')))
            if not math.isnan(pct):
                pct = max(0.0, min(1.0, pct)) * 100.0
            volt = float(getattr(msg, "voltage", float('nan')))
            chg_flag = False
            try:
                chg_flag = int(getattr(msg, "power_supply_status", 0)) == 1
            except Exception:
                chg_flag = False
            self._battery_pct = pct
            self._battery_v = volt
            self._battery_chg = chg_flag
            self.signals.battery.emit(self._battery_pct, self._battery_v, self._battery_chg)
        except Exception:
            pass

    # ---- subscribe helper (Int32/UInt32 autodetect)
    def _subscribe_int_counter(self, topic_name: str, is_bad: bool):
        types_map = dict(self.get_topic_names_and_types())
        preferred = None
        if topic_name in types_map and types_map[topic_name]:
            tname = types_map[topic_name][0]
            if "std_msgs/msg/Int32" in tname:
                preferred = Int32
            elif "std_msgs/msg/UInt32" in tname:
                preferred = UInt32
        order = ([preferred] if preferred is not None else []) + [Int32, UInt32]
        seen = set()
        for candidate in order:
            if candidate is None or candidate in seen:
                continue
            seen.add(candidate)
            try:
                def _cb(msg, bad=is_bad):
                    v = int(getattr(msg, "data", 0))
                    if bad:
                        self._tree_bad = v
                    else:
                        self._tree_total = v
                    self.signals.tree_counts.emit(self._tree_total, self._tree_bad)
                sub = self.create_subscription(candidate, topic_name, _cb, 10)
                self.get_logger().info(f"Subscribed {topic_name} as {candidate.__name__}")
                return sub
            except Exception as e:
                self.get_logger().warn(f"{topic_name}: {candidate.__name__} failed ({e})")
        self.get_logger().error(f"{topic_name}: unable to subscribe as Int32/UInt32")
        return None

    # ------------------- publishers -------------------
    @Slot(float, float)
    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear); msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    @Slot(float)
    def set_speed_scale(self, scale: float):
        value = max(0.05, min(2.0, float(scale)))
        self._speed_scale = value
        self.speed_cmd_pub.publish(Float32(data=float(value)))
        self.signals.speed_scale.emit(value)

    @Slot(object)
    def publish_waypoints(self, waypoints: List[Dict[str, float]]):
        if self.waypoint_pub is None:
            self.signals.ok.emit(False, "Waypoint publisher not ready")
            return
        if not waypoints:
            self.signals.ok.emit(False, "No waypoints to send")
            return
        pa = PoseArray()
        pa.header.frame_id = self._map_frame_name
        pa.header.stamp = self.get_clock().now().to_msg()
        for entry in waypoints:
            pose = Pose()
            pose.position.x = float(entry.get("x", 0.0))
            pose.position.y = float(entry.get("y", 0.0))
            pose.position.z = float(entry.get("z", 0.0))
            yaw_deg = float(entry.get("yaw_deg", 0.0))
            qx, qy, qz, qw = _quat_from_yaw(math.radians(yaw_deg))
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            pa.poses.append(pose)
        self.waypoint_pub.publish(pa)
        self.signals.ok.emit(True, f"Sent {len(pa.poses)} waypoint(s)")

    @Slot()
    def cancel_waypoints(self):
        if self._follow_client is None:
            self.signals.ok.emit(False, "Waypoint client not available")
            return
        try:
            if not self._follow_client.server_is_ready():
                self._follow_client.wait_for_server(timeout_sec=0.2)
            if not self._follow_client.server_is_ready():
                self.signals.ok.emit(False, "Waypoint server not ready")
                return
            future = self._follow_client.cancel_all_goals()
            future.add_done_callback(lambda _f: self.signals.ok.emit(True, "Waypoint goals cancelled"))
        except Exception as e:
            self.signals.ok.emit(False, f"Cancel failed: {e}")

    @Slot(object)
    def set_hsv_params(self, params):
        if not self._hsv_ready:
            self.signals.ok.emit(False, "HSV view unavailable on this system")
            return
        try:
            mapping = [
                ("green_low", "_green_low"),
                ("green_high", "_green_high"),
                ("red1_low", "_red1_low"),
                ("red1_high", "_red1_high"),
                ("red2_low", "_red2_low"),
                ("red2_high", "_red2_high"),
            ]
            for key, attr in mapping:
                vals = params.get(key)
                if vals is None:
                    continue
                arr = np.clip(np.array(list(vals), dtype=np.int16), 0, 255).astype(np.uint8)
                setattr(self, attr, arr)
            self.signals.ok.emit(True, "HSV thresholds updated")
        except Exception as e:
            self.signals.ok.emit(False, f"HSV update failed: {e}")

    # ------------------- subs/callbacks ----------------
    def _on_rosout(self, msg: RosLog):
        if int(getattr(msg, "level", 20)) < self._rosout_min_level:
            return
        level_map = {10:"DEBUG", 20:"INFO", 30:"WARN", 40:"ERROR", 50:"FATAL"}
        self.signals.log.emit(f"[{level_map.get(msg.level, msg.level)}] {msg.name}: {msg.msg}")

    def _on_camera_topic_switch(self, msg: RosString):
        topic = (msg.data or "").strip()
        if not topic:
            return
        if topic.upper().startswith('MODE:'):
            mode = topic.split(':', 1)[1]
            self.set_camera_mode(mode)
            return
        topic = topic.rstrip('/')
        if topic == self._camera_topic_raw:
            return
        self._subscribe_camera(topic)

    def _subscribe_camera(self, topic_base: str):
        topic_base = (topic_base or "").strip()
        if not topic_base:
            return
        if self._camera_sub is not None:
            try: self.destroy_subscription(self._camera_sub)
            except Exception: pass
            self._camera_sub = None
        if not self._prefer_compressed and topic_base.rstrip('/').endswith("/compressed"):
            topic_base = topic_base.rstrip('/')[:-len("/compressed")]
        self._camera_topic_raw = topic_base.rstrip('/')
        if self._prefer_compressed:
            comp_topic = self._camera_topic_raw + "/compressed" if not self._camera_topic_raw.endswith("/compressed") else self._camera_topic_raw
            try:
                self._camera_sub = self.create_subscription(RosCompressedImage, comp_topic, self._on_image_compressed, self.sensor_qos)
                self.get_logger().info(f"Subscribed camera (compressed): {comp_topic}")
                return
            except Exception as e:
                self.get_logger().warn(f"Compressed subscribe failed: {e}); falling back to raw.")
        self._camera_sub = self.create_subscription(RosImage, self._camera_topic_raw, self._on_image_raw, self.sensor_qos)
        self.get_logger().info(f"Subscribed camera (raw): {self._camera_topic_raw}")

    def _on_image_raw(self, msg: RosImage):
        rgb = self._image_to_rgb_numpy(msg)
        if rgb is None: return
        with self._img_lock: self._latest_rgb = rgb

    def _on_image_compressed(self, msg: RosCompressedImage):
        try:
            if np is None or cv2 is None: return
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None: return
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            with self._img_lock: self._latest_rgb = rgb
        except Exception as e:
            self.get_logger().warn(f"Compressed decode failed: {e}")

    def _emit_image_tick(self):
        rgb = None
        with self._img_lock:
            if self._latest_rgb is not None:
                rgb = self._latest_rgb
                self._latest_rgb = None
        if rgb is not None:
            if self._camera_mode == 'hsv' and self._hsv_ready:
                try:
                    view = self._make_hsv_view(rgb)
                except Exception as exc:
                    self.get_logger().throttle(2000, f"HSV view failed: {exc}")
                    view = rgb
            else:
                view = rgb
            self.signals.image.emit(view)

    def _on_pc(self, msg: PointCloud2):
        try:
            cnt = 0
            xy = []
            step = 10  # sample every 10th point
            for i, p in enumerate(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)):
                if i % step == 0:
                    xy.append((float(p[0]), float(p[1])))
                    cnt += 1
                    if cnt >= 5000:  # cap for perf
                        break
            with self._pc_xy_lock:
                self._pc_xy_samples = xy
            self._pc_latest_count = cnt * step
        except Exception:
            pass

    def _emit_pc_count(self):
        if getattr(self, "_pc_latest_count", 0):
            self.signals.pc_points.emit(int(self._pc_latest_count))
            self._pc_latest_count = 0

    def _make_hsv_view(self, rgb_np):
        if not self._hsv_ready:
            return rgb_np
        bgr = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask_g = cv2.inRange(hsv, self._green_low, self._green_high)
        mask_r = cv2.inRange(hsv, self._red1_low, self._red1_high) | cv2.inRange(hsv, self._red2_low, self._red2_high)
        overlay = np.zeros_like(bgr)
        overlay[mask_g > 0] = (0, 255, 0)
        overlay[mask_r > 0] = (0, 0, 255)
        return cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)

    def set_camera_mode(self, mode: str, notify: bool = True):
        target_mode = 'hsv' if str(mode).lower() == 'hsv' else 'rgb'
        if target_mode == 'hsv' and not self._hsv_ready:
            if notify:
                self.signals.ok.emit(False, "HSV view unavailable (missing OpenCV/Numpy)")
            return
        if target_mode != self._camera_mode:
            self._camera_mode = target_mode
            if notify:
                self.signals.ok.emit(True, "Camera view: HSV mask" if target_mode == 'hsv' else "Camera view: Camera")
        self.signals.camera_mode.emit(self._camera_mode)

    def set_camera_topic(self, topic: str):
        topic = (topic or "").strip()
        if not topic:
            return
        topic = topic.rstrip('/')
        if topic == self._camera_topic_raw:
            return
        self._subscribe_camera(topic)

    def _cycle_camera_mode(self):
        self.set_camera_mode('hsv' if self._camera_mode == 'rgb' else 'rgb')

    def _on_joy(self, msg):
        up = down = False
        if self._hat_axis_v is not None and self._hat_axis_v >= 0:
            try:
                v = msg.axes[self._hat_axis_v]
                up = (v > 0.5); down = (v < -0.5)
            except Exception:
                pass
        if self._btn_up >= 0:
            try: up = up or (msg.buttons[self._btn_up] > 0)
            except Exception: pass
        if self._btn_down >= 0:
            try: down = down or (msg.buttons[self._btn_down] > 0)
            except Exception: pass

        cycle_pressed = False
        if self._btn_cycle >= 0:
            try:
                cycle_pressed = bool(msg.buttons[self._btn_cycle] > 0)
            except Exception:
                cycle_pressed = False

        try:
            self._teleop_enabled = bool(msg.buttons[self._teleop_btn] > 0)
        except Exception:
            self._teleop_enabled = False

        self.signals.teleop_led.emit(self._teleop_enabled, self._sim_alive)
        if up and not self._prev_up:   self.signals.bump_speed.emit(+1)
        if down and not self._prev_down: self.signals.bump_speed.emit(-1)
        self._prev_up, self._prev_down = up, down
        if cycle_pressed and not self._prev_cycle_btn:
            self._cycle_camera_mode()
        self._prev_cycle_btn = cycle_pressed

        estop_pressed = False
        if self._btn_estop >= 0:
            try:
                estop_pressed = bool(msg.buttons[self._btn_estop] > 0)
            except Exception:
                estop_pressed = False
        if estop_pressed and not self._prev_estop_btn:
            self.signals.estop_toggle.emit()
        self._prev_estop_btn = estop_pressed

    # ---- Sim heartbeat ----
    def _on_clock(self, _msg: Clock):
        self._last_clock_ns = self.get_clock().now().nanoseconds

    def _check_sim_alive(self):
        now_ns = self.get_clock().now().nanoseconds
        alive = (now_ns - self._last_clock_ns) < int(SIM_TIMEOUT_S * 1e9)
        if alive != self._sim_alive:
            self._sim_alive = alive
            self.signals.teleop_led.emit(self._teleop_enabled, self._sim_alive)

    # ---- Map + trees callbacks ----
    def _on_map(self, msg: OccupancyGrid):
        with self._map_lock:
            self._map_msg = msg
            self._map_res = msg.info.resolution
            self._map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        yaw = _yaw_from_q(msg.pose.pose.orientation)
        self._robot_pose = (pos.x, pos.y, yaw)
        self.signals.robot_pose.emit(self._robot_pose)
        if math.isfinite(pos.x) and math.isfinite(pos.y):
            self._trail_points.append((pos.x, pos.y))
            if len(self._trail_points) > self._trail_max:
                del self._trail_points[0:len(self._trail_points) - self._trail_max]
        self._emit_tree_table()

    def _on_speed_state(self, msg: Float32):
        try:
            scale = float(getattr(msg, "data", float('nan')))
        except Exception:
            return
        if not math.isfinite(scale):
            return
        scale = max(0.05, min(2.0, scale))
        if abs(scale - self._speed_scale) <= 1e-3:
            return
        self._speed_scale = scale
        self.signals.speed_scale.emit(scale)

    def _on_global_path(self, msg: Path):
        try:
            pts = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        except Exception:
            pts = []
        with self._path_lock:
            self._global_path_pts = pts

    def _on_local_path(self, msg: Path):
        try:
            pts = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        except Exception:
            pts = []
        with self._path_lock:
            self._local_path_pts = pts

    def _on_global_costmap(self, msg: OccupancyGrid):
        with self._costmap_lock:
            self._global_costmap = msg

    def _on_local_costmap(self, msg: OccupancyGrid):
        with self._costmap_lock:
            self._local_costmap = msg

    def _on_markers(self, msg: MarkerArray):
        changed = False
        now_ns = self.get_clock().now().nanoseconds
        with self._tree_lock:
            for m in msg.markers:
                if m.action == Marker.DELETEALL:
                    self._tree_states.clear()
                    changed = True
                    continue
                if m.action == Marker.DELETE:
                    continue
                x, y, z = float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z)
                key = (round(x, 2), round(y, 2))
                entry = self._tree_states.get(key, {
                    "x": x, "y": y, "z": z,
                    "status": "unknown",
                    "source": "",
                    "alpha": 1.0,
                    "updated_ns": now_ns,
                })
                entry["x"] = x; entry["y"] = y; entry["z"] = z
                entry["alpha"] = float(getattr(m.color, "a", 1.0))
                entry["updated_ns"] = now_ns
                if getattr(m, "color", None) is not None:
                    entry["bgr"] = (
                        int(m.color.b * 255),
                        int(m.color.g * 255),
                        int(m.color.r * 255),
                    )
                    if m.ns == 'trees_coloured':
                        entry["source"] = "coloured"
                        if entry["bgr"][2] > entry["bgr"][1]:
                            entry["status"] = "bad"
                        elif entry["bgr"][1] > entry["bgr"][2]:
                            entry["status"] = "good"
                        else:
                            entry["status"] = "unknown"
                    elif entry.get("source") != "coloured":
                        entry["source"] = m.ns or "raw"
                        entry["status"] = "unknown"
                else:
                    entry["bgr"] = (255, 255, 0)
                entry["key"] = key
                self._tree_states[key] = entry
                changed = True
        if changed:
            self._emit_tree_table()

    def _grid_to_bgr(self, grid: OccupancyGrid):
        if np is None: return None
        w, h = grid.info.width, grid.info.height
        data = np.frombuffer(bytes(grid.data), dtype=np.int8).astype(np.int16).reshape(h, w)
        img = np.full((h, w), 128, dtype=np.uint8)
        img[data == 0]  = 255
        img[data >= 50] = 0
        if cv2 is None:
            return np.stack([img, img, img], axis=-1)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def _world_to_px(self, x: float, y: float, img_h: int):
        ox, oy = self._map_origin
        col = int((x - ox) / self._map_res)
        row = int((y - oy) / self._map_res)
        row = img_h - 1 - row
        return col, row

    def _draw_path(self, img, points: List[Tuple[float, float]], color: Tuple[int, int, int]):
        if cv2 is None or not points:
            return
        pix = []
        h = img.shape[0]
        for wx, wy in points:
            try:
                px, py = self._world_to_px(wx, wy, h)
            except Exception:
                continue
            if px < 0 or py < 0 or px >= img.shape[1] or py >= h:
                continue
            pix.append([px, py])
        if len(pix) >= 2:
            poly = np.array(pix, dtype=np.int32)
            cv2.polylines(img, [poly], False, color, 2, cv2.LINE_AA)
        elif len(pix) == 1:
            cv2.circle(img, tuple(pix[0]), 3, color, -1)

    def _overlay_costmap(self, img, grid: Optional[OccupancyGrid], color: Tuple[int, int, int]):
        if grid is None or np is None or self._map_res is None or self._map_origin is None:
            return
        try:
            data = np.frombuffer(bytes(grid.data), dtype=np.int8).reshape((grid.info.height, grid.info.width))
        except Exception:
            return
        h_img = img.shape[0]
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        nz = np.argwhere(data >= 50)
        if nz.size == 0:
            return
        # subsample to keep drawing budget reasonable
        step = max(1, int(len(nz) / 4000))
        for cell in nz[::step]:
            cy, cx = cell
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
            try:
                px, py = self._world_to_px(wx, wy, h_img)
            except Exception:
                continue
            if 0 <= px < img.shape[1] and 0 <= py < h_img:
                cv2.circle(img, (px, py), 2, color, -1)

    def _draw_trail(self, img, pts: List[Tuple[float, float]]):
        if cv2 is None or not pts:
            return
        h = img.shape[0]
        pix = []
        for wx, wy in pts:
            try:
                px, py = self._world_to_px(wx, wy, h)
            except Exception:
                continue
            if 0 <= px < img.shape[1] and 0 <= py < h:
                pix.append([px, py])
        if len(pix) >= 2:
            poly = np.array(pix, dtype=np.int32)
            cv2.polylines(img, [poly], False, (0, 200, 255), 2, cv2.LINE_AA)

    def _draw_scale_bar(self, img, meters: float = 5.0):
        if cv2 is None or self._map_res is None:
            return
        px_per_m = 1.0 / self._map_res
        length_px = int(meters * px_per_m)
        if length_px < 20:
            length_px = int(3 * px_per_m)
            meters = 3.0
        h, w = img.shape[:2]
        pad = 16
        y = h - pad
        x_start = w - pad - length_px
        x_end = w - pad
        if x_start < pad:
            x_start = pad
        cv2.line(img, (x_start, y), (x_end, y), (255, 255, 255), 3, cv2.LINE_AA)
        cv2.line(img, (x_start, y), (x_start, y-8), (255, 255, 255), 2, cv2.LINE_AA)
        cv2.line(img, (x_end, y), (x_end, y-8), (255, 255, 255), 2, cv2.LINE_AA)
        label = f"{meters:.0f} m"
        cv2.putText(img, label, (x_start, y-12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    def _emit_map_image(self):
        if np is None:
            return
        with self._map_lock:
            grid = self._map_msg
        if grid is None or self._map_res is None or self._map_origin is None:
            return

        base = self._grid_to_bgr(grid)
        if base is None:
            return

        if cv2 is None:
            rgb = base[:, :, ::-1]
            self.signals.map_image.emit({"trees": rgb, "nav": rgb})
            return

        bgr_trees = base.copy()
        bgr_nav = base.copy()
        h, _ = bgr_trees.shape[:2]

        # Trees (markers + labels)
        with self._tree_lock:
            tree_items = list(self._tree_states.values())
        for t in tree_items:
            try:
                cx, cy = self._world_to_px(t["x"], t["y"], h)
                color = t.get("bgr", (255, 255, 0))
                cv2.circle(bgr_trees, (cx, cy), 5, color, -1)
                label = t.get("status", "unknown")
                cv2.putText(bgr_trees, label[:3].upper(), (cx+6, cy-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
            except Exception:
                pass

        # Nav overlays = trees + costmaps + paths + trail + robot pose
        bgr_nav[:] = bgr_trees
        with self._costmap_lock:
            global_cost = self._global_costmap
            local_cost = self._local_costmap
        self._overlay_costmap(bgr_nav, global_cost, (120, 120, 255))
        self._overlay_costmap(bgr_nav, local_cost, (255, 180, 80))
        with self._path_lock:
            g_path = list(self._global_path_pts)
            l_path = list(self._local_path_pts)
        self._draw_path(bgr_nav, g_path, (255, 0, 255))
        self._draw_path(bgr_nav, l_path, (0, 255, 255))
        if self._trail_points:
            self._draw_trail(bgr_nav, list(self._trail_points))

        if self._robot_pose:
            try:
                rx, ry, yaw = self._robot_pose
                cx, cy = self._world_to_px(rx, ry, h)
                tip = (int(cx + 18*math.cos(yaw)), int(cy - 18*math.sin(yaw)))
                for img in (bgr_trees, bgr_nav):
                    cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)
                    cv2.arrowedLine(img, (cx, cy), tip, (0, 0, 255), 2, tipLength=0.35)
            except Exception:
                pass

        self._draw_scale_bar(bgr_trees)
        self._draw_scale_bar(bgr_nav)

        payload = {
            "trees": cv2.cvtColor(bgr_trees, cv2.COLOR_BGR2RGB),
            "nav":   cv2.cvtColor(bgr_nav,   cv2.COLOR_BGR2RGB),
        }
        self.signals.map_image.emit(payload)


    def _emit_tree_table(self):
        with self._tree_lock:
            entries = list(self._tree_states.items())
        rp = self._robot_pose
        if rp is None and self._tf_buffer is not None:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._map_frame_name,
                    self._base_frame_name,
                    Time())
                tx = tf.transform.translation
                q = tf.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                rp = (tx.x, tx.y, yaw)
                self._robot_pose = rp
            except Exception:
                rp = None
        rows = []
        for key, entry in entries:
            dist = float('nan')
            if rp is not None:
                dist = math.hypot(entry["x"] - rp[0], entry["y"] - rp[1])
            uncertainty = 1.0
            if math.isfinite(dist) and self._confirm_range_hint > 0.0:
                uncertainty = max(0.0, min(1.0, dist / self._confirm_range_hint))
            rows.append({
                "key": key,
                "x": entry["x"],
                "y": entry["y"],
                "z": entry.get("z", 0.0),
                "status": entry.get("status", "unknown"),
                "uncertainty": uncertainty,
                "distance": dist,
            })
        rows.sort(key=lambda r: (0 if r["status"] == "bad" else 1,
                                 r["distance"] if math.isfinite(r["distance"]) else float('inf')))
        total = len(rows)
        bad_count = sum(1 for r in rows if r["status"] == "bad")
        self._tree_total = total
        self._tree_bad = bad_count
        self.signals.tree_counts.emit(total, bad_count)
        self.signals.tree_table.emit(rows)

    # ------------------- helpers ----------------------
    def _image_to_rgb_numpy(self, msg: RosImage):
        try:
            if self.bridge is not None and _CV_BRIDGE_OK:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # type: ignore
                if cv2 is not None: return cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                else: return cv_image[:, :, ::-1].copy()
            else:
                if np is None: return None
                enc = (msg.encoding or "").lower()
                if enc in ("rgb8", "bgr8"):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    return arr[:, :, ::-1].copy() if enc == "bgr8" else arr
                if enc in ("mono8", "8uc1"):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                    return np.stack([arr]*3, axis=-1)
                return None
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return None

# ============================ GUI ===================================
class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Panel")
        self._apply_icon()

        self.setMinimumSize(900, 580)
        self._quitting = False
        self._estopped = False
        self._lin = 0.0
        self._ang = 0.0
        self._dpad_forward = False
        self._dpad_back = False
        self._dpad_left = False
        self._dpad_right = False
        self._dpad_engaged = False
        self._teleop_enabled = False
        self._sim_alive = False
        self._robot_pose: Optional[Tuple[float, float, float]] = None
        self._suppress_speed_emit = False
        self._last_speed_sent: Optional[float] = None
        self._tree_table_keys: List[Tuple[float, float]] = []
        self._tree_table_cache: Dict[Tuple[float, float], Dict[str, object]] = {}
        self._cpu_process = psutil.Process(os.getpid()) if psutil else None
        if self._cpu_process:
            try:
                self._cpu_process.cpu_percent(None)
                psutil.cpu_percent(None)
            except Exception:
                pass

        # QProcess for launch/build
        self._proc_launch = QProcess(self)
        self._proc_launch.setProcessChannelMode(QProcess.MergedChannels)
        self._proc_launch.readyReadStandardOutput.connect(self._on_launch_output)
        self._proc_launch.finished.connect(self._on_launch_finished)

        self._proc_build = QProcess(self)
        self._proc_build.setProcessChannelMode(QProcess.MergedChannels)
        self._proc_build.readyReadStandardOutput.connect(self._on_build_output)
        self._proc_build.finished.connect(self._on_build_finished)

        self.send_waypoints = lambda _rows: None
        self._install_sigint_handler()
        self._hsv_params = {
            "green_low": [50, 140, 13],
            "green_high": [57, 255, 255],
            "red1_low": [0, 155, 75],
            "red1_high": [34, 255, 255],
            "red2_low": [170, 160, 77],
            "red2_high": [179, 255, 255],
        }
        self._hsv_inputs = {}
        self._hsv_pending = False
        self._build_ui()
        self._wire_behaviour()
        self._start_ros()
        self.speed.setValue(_scale_to_slider(0.60))
        self._reflect_camera_mode('rgb')
        self._update_cpu()

    def _apply_icon(self):
        icon_path = os.environ.get("UI_APP_ICON", "")
        if not icon_path:
            guesses = [
                "~/git/RS1/john_branch/src/forestguard_ui/assets/app_icon.png",
                "~/git/RS1/john_branch/john/assets/app_icon.png",
            ]
            for g in guesses:
                p = os.path.expanduser(g)
                if os.path.exists(p):
                    icon_path = p; break
        if icon_path and os.path.exists(icon_path):
            try: self.setWindowIcon(QIcon(icon_path))
            except Exception: pass

    def rounded_pane(self, w: QWidget, pad=10):
        f = QFrame(); f.setObjectName("pane")
        ly = QVBoxLayout(f); ly.setContentsMargins(pad, pad, pad, pad); ly.addWidget(w)
        return f

    def _build_ui(self):
        # Camera
        self.camera_lbl = QLabel("Camera")
        self.camera_lbl.setAlignment(Qt.AlignCenter)
        self.camera_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.camera_lbl.setMinimumHeight(200)
        self.cam_mode_btn = QPushButton("View: Camera")
        self.cam_mode_btn.setCheckable(True)
        self.cam_mode_btn.toggled.connect(self._toggle_cam_mode_btn)
        cam_box = QWidget(); cam_col = QVBoxLayout(cam_box)
        cam_col.setContentsMargins(0, 0, 0, 0)
        cam_col.setSpacing(6)
        cam_col.addWidget(self.cam_mode_btn, 0, Qt.AlignLeft)
        cam_col.addWidget(self.camera_lbl, 1)
        cam = self.rounded_pane(cam_box, pad=10)

        # Map panels (tabs)
        self.map_tree_lbl = QLabel("Map – Trees")
        self.map_tree_lbl.setAlignment(Qt.AlignCenter)
        self.map_tree_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tree_holder = self.rounded_pane(self.map_tree_lbl, pad=10)

        self.map_nav_lbl = QLabel("Map – Nav Layers")
        self.map_nav_lbl.setAlignment(Qt.AlignCenter)
        self.map_nav_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        nav_holder = self.rounded_pane(self.map_nav_lbl, pad=10)

        self.hsv_widget = self._build_hsv_tab()
        hsv_holder = self.rounded_pane(self.hsv_widget, pad=10)

        self.map_tabs = QTabWidget()
        self.map_tabs.addTab(tree_holder, "Trees")
        self.map_tabs.addTab(nav_holder, "Nav Layers")
        self.map_tabs.addTab(hsv_holder, "HSV Tuning")
        self.map_tabs.currentChanged.connect(self._on_map_tab_changed)
        self._map_tab_index = 0

        # Tree table tab
        self.tree_table = QTableWidget(0, 6, self)
        self.tree_table.setHorizontalHeaderLabels(["X", "Y", "Z", "Status", "Uncertainty", "Distance"])
        header = self.tree_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.Stretch)
        self.tree_table.verticalHeader().setVisible(False)
        self.tree_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.tree_table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.tree_table.setFocusPolicy(Qt.NoFocus)
        self.tree_table.setAlternatingRowColors(True)

        tree_tab = QWidget()
        tree_tab_layout = QVBoxLayout(tree_tab)
        tree_tab_layout.setContentsMargins(6, 6, 6, 6)
        tree_tab_layout.addWidget(self.tree_table)

        # Log tab
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("Log")
        self.log.setMinimumSize(300, 160)
        log_tab = QWidget()
        log_tab_layout = QVBoxLayout(log_tab)
        log_tab_layout.setContentsMargins(6, 6, 6, 6)
        log_tab_layout.addWidget(self.log)

        # Waypoint manager tab
        self.waypoint_panel = WaypointPanel(self._send_waypoint_list, self._get_current_robot_pose)
        waypoint_tab = QWidget()
        waypoint_layout = QVBoxLayout(waypoint_tab)
        waypoint_layout.setContentsMargins(6, 6, 6, 6)
        waypoint_layout.addWidget(self.waypoint_panel)

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(log_tab, "Log")
        self.tabs.addTab(tree_tab, "Tree Table")
        self.tabs.addTab(waypoint_tab, "Waypoints")
        self.waypoint_panel.set_pose_provider(self._get_current_robot_pose)

        tabs_holder = self.rounded_pane(self.tabs, pad=6)

        left_col = QVBoxLayout()
        left_col.addWidget(cam, 3)
        left_col.addWidget(tabs_holder, 2)

        # Right column: top = Map/Panel, bottom split 50/50 (tree counter + speed)
        self.tree_total = 0
        self.tree_bad = 0
        self.tree_lbl = QLabel("Trees: 0 (bad 0)")
        self.tree_lbl.setAlignment(Qt.AlignCenter)
        self.tree_lbl.setWordWrap(True)
        self.tree_lbl.setStyleSheet("font-size: 20px; font-weight: 600; padding:4px 6px;")
        tree_frame = QFrame()
        tree_frame.setObjectName("pane")
        tree_layout = QVBoxLayout(tree_frame)
        tree_layout.setContentsMargins(6, 6, 6, 6)
        tree_layout.addWidget(self.tree_lbl)

        self.odom_lbl = QLabel("x=0.00\ny=0.00\nyaw=0.0°")
        self.odom_lbl.setAlignment(Qt.AlignLeft)
        odom_frame = QFrame()
        odom_frame.setObjectName("pane")
        odom_layout = QVBoxLayout(odom_frame)
        odom_layout.setContentsMargins(6, 6, 6, 6)
        odom_layout.setSpacing(4)
        
        odom_layout.addWidget(QLabel("Odometry", alignment=Qt.AlignLeft))
        odom_layout.addWidget(self.odom_lbl)

        self.speed = QSlider(Qt.Vertical); self.speed.setRange(0, 20); self.speed.setTickInterval(5)
        self.speed.setTickPosition(QSlider.TicksRight)
        self.speed.setFixedWidth(34)
        tick_col = QVBoxLayout()
        for i, v in enumerate([20, 15, 10, 5, 0]):
            lab = QLabel(str(v)); lab.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            tick_col.addWidget(lab)
            if i < 4: tick_col.addStretch(1)

        speed_row = QHBoxLayout()
        speed_row.addStretch(1)
        speed_row.addWidget(self.speed, 0, Qt.AlignHCenter)
        speed_row.addLayout(tick_col)
        speed_row.addStretch(1)

        speed_pane = QFrame(); speed_pane.setObjectName("pane")
        sp_l = QVBoxLayout(speed_pane); sp_l.setContentsMargins(10,10,10,10)
        sp_l.addLayout(speed_row, 1)
        sp_l.addWidget(QLabel("Speed", alignment=Qt.AlignHCenter))

        right_col = QVBoxLayout()
        right_col.addWidget(self.map_tabs, 3)
        
        # D-pad pane (optional on-screen nudging)
        dpad_core = QWidget()
        g = QGridLayout(dpad_core)
        g.setSpacing(4)
        g.setContentsMargins(0, 0, 0, 0)

        def _mk_btn(name: str, text: str) -> QPushButton:
            b = QPushButton(text)
            b.setFixedSize(60, 52)
            b.setObjectName(name)
            return b

        self.btn_up = _mk_btn("dpad_up", "▲")
        self.btn_left = _mk_btn("dpad_left", "◀")
        self.btn_right = _mk_btn("dpad_right", "▶")
        self.btn_down = _mk_btn("dpad_down", "▼")
        g.addWidget(self.btn_up, 0, 1)
        g.addWidget(self.btn_left, 1, 0)
        g.addWidget(self.btn_right, 1, 2)
        g.addWidget(self.btn_down, 2, 1)
        dpad_square = SquareContainer(dpad_core)
        dpad_square.setMinimumSize(160, 160)
        dpad_square.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        dpad = self.rounded_pane(dpad_square, pad=6)
        dpad.setMinimumWidth(180)
        dpad.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        left_controls = QWidget()
        left_stack = QVBoxLayout(left_controls)
        left_stack.setContentsMargins(0, 0, 0, 0)
        left_stack.setSpacing(8)
        left_stack.addWidget(tree_frame, 0)
        left_stack.addWidget(odom_frame, 0, Qt.AlignLeft)
        left_stack.addStretch(1)

        right_bottom = QHBoxLayout()
        right_bottom.setSpacing(12)
        right_bottom.addWidget(left_controls, 1)
        right_bottom.addWidget(dpad, 2)
        right_bottom.addWidget(speed_pane, 0)
        right_col.addLayout(right_bottom, 2)

        # Bottom bar: Teleop LED + Depth Cloud + Battery + Run/Build/E-stop
        run = QHBoxLayout()
        run.addWidget(QLabel("Teleop"))
        self.led = LedIndicator("#666666")
        run.addWidget(self.led)

        self.pc_name = QLabel("Depth Cloud")
        self.pc_name.setStyleSheet("color:#bbb; margin-left:14px;")
        self.pc_val  = QLabel("~0")
        self.pc_val.setStyleSheet("font-weight:600;")
        run.addWidget(self.pc_name)
        run.addWidget(self.pc_val)
        self.auto_name = QLabel("Autonomy")
        self.auto_led = LedIndicator("#666666")
        run.addWidget(self.auto_name); run.addWidget(self.auto_led)

        # Battery
        run.addWidget(QLabel("Battery"))
        self.batt_lbl = QLabel("—%")
        self.batt_lbl.setStyleSheet("font-weight:700; margin-left:6px;")
        run.addWidget(self.batt_lbl)
        self.cpu_lbl = QLabel("CPU: --")
        self.cpu_lbl.setStyleSheet("color:#bbb; margin-left:14px;")
        self.cpu_lbl.setToolTip("Use `top` (watch controller_bridge, run_ui) and `ros2 topic hz /camera/image` for rate.")
        run.addWidget(self.cpu_lbl)

        run.addStretch(1)
        run_w = QWidget(); run_w.setLayout(run)

        self.launch_btn = QPushButton("Run Sim")
        self.launch_btn.setCheckable(True)
        self.launch_btn.clicked.connect(self._toggle_launch)

        self.build_btn = QPushButton("Rebuild Code")
        self.build_btn.clicked.connect(self._start_build)

        self.estop = QPushButton("E-STOP"); self.estop.setObjectName("estop"); self.estop.setMinimumSize(140, 44)
        self.estop.setCheckable(True)
        shadow = QGraphicsDropShadowEffect(blurRadius=16, offset=QPointF(0, 2))
        shadow.setColor(QColor(0, 0, 0, 160)); self.estop.setGraphicsEffect(shadow)

        bottom = QHBoxLayout()
        bottom.addWidget(run_w, 1)
        bottom.addWidget(self.launch_btn, 0)
        bottom.addWidget(self.build_btn, 0)
        bottom.addWidget(self.estop, 0, Qt.AlignRight)

        root = QVBoxLayout(self)
        top = QHBoxLayout(); top.addLayout(left_col, 3); top.addLayout(right_col, 2)
        root.addLayout(top, 1); root.addLayout(bottom)
        self._apply_dark_theme()

    def _apply_dark_theme(self):
        try: QApplication.instance().setStyle(QStyleFactory.create("Fusion"))
        except Exception: pass

        pal = QPalette()
        pal.setColor(QPalette.Window, QColor(0x12,0x12,0x12))
        pal.setColor(QPalette.WindowText, Qt.white)
        pal.setColor(QPalette.Base, QColor(0x18,0x18,0x18))
        pal.setColor(QPalette.AlternateBase, QColor(0x20,0x20,0x20))
        pal.setColor(QPalette.ToolTipBase, Qt.white)
        pal.setColor(QPalette.ToolTipText, Qt.white)
        pal.setColor(QPalette.Text, Qt.white)
        pal.setColor(QPalette.Button, QColor(0x22,0x22,0x22))
        pal.setColor(QPalette.ButtonText, Qt.white)
        pal.setColor(QPalette.BrightText, Qt.red)
        pal.setColor(QPalette.Highlight, QColor(66, 133, 244))
        pal.setColor(QPalette.HighlightedText, Qt.white)
        QApplication.instance().setPalette(pal)

        self.setStyleSheet("""
            QWidget { font-size: 18px; color: #eaeaea; }
            QFrame#pane {
                border: 1px solid #2e2e2e;
                border-radius: 12px;
                background: #1a1a1a;
            }
            QTextEdit { background: #111; border: 1px solid #333; }
            QLabel { color: #eaeaea; }
            QSlider::groove:vertical { background: #333; width: 6px; border-radius: 3px; }
            QSlider::handle:vertical { background: #ccc; height: 18px; margin: -4px -8px; border-radius: 6px; }
            QPushButton {
                background: #2c2c2c; border: 1px solid #3a3a3a; border-radius: 8px; padding: 6px 10px;
            }
            QPushButton:hover { background: #3a3a3a; }
            QPushButton#estop {
                background-color: #e53935; color: white; border: 2px solid #b71c1c; border-radius: 18px;
                padding: 8px 18px; font-weight: 700;
            }
            QPushButton#estop:hover   { background-color: #f44336; }
            QPushButton#estop:pressed { background-color: #c62828; }
        """)

    def _wire_behaviour(self):
        self.speed.valueChanged.connect(self._on_slider_changed)
        self.estop.clicked.connect(self._on_estop)

        self.btn_up.pressed.connect(lambda: self._set_motion(forward=True))
        self.btn_up.released.connect(lambda: self._set_motion(forward=False))
        self.btn_down.pressed.connect(lambda: self._set_motion(back=True))
        self.btn_down.released.connect(lambda: self._set_motion(back=False))
        self.btn_left.pressed.connect(lambda: self._set_motion(left=True))
        self.btn_left.released.connect(lambda: self._set_motion(left=False))
        self.btn_right.pressed.connect(lambda: self._set_motion(right=True))
        self.btn_right.released.connect(lambda: self._set_motion(right=False))

        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._publish_cmd)
        self.cmd_timer.start(int(1000 / CMD_PUB_RATE_HZ))
        self._cpu_timer = QTimer(self)
        self._cpu_timer.timeout.connect(self._update_cpu)
        self._cpu_timer.start(int(max(0.5, CPU_SAMPLE_PERIOD_S) * 1000))

    def _start_ros(self):
        self.ros = RosWorker()
        self.ros.signals.image.connect(self._update_camera)
        self.ros.signals.map_image.connect(self._update_map)
        self.ros.signals.log.connect(self._append_log)
        self.ros.signals.ok.connect(lambda ok, msg: self._append_log(("OK " if ok else "ERR ") + msg))
        self.ros.signals.bump_speed.connect(self._bump_speed)
        self.ros.signals.tree_counts.connect(self._update_tree_counts)
        self.ros.signals.tree_table.connect(self._update_tree_table)
        self.ros.signals.camera_mode.connect(self._reflect_camera_mode)
        self.ros.signals.pc_points.connect(self._update_pc_points)
        self.ros.signals.teleop_led.connect(self._set_teleop_led)
        self.ros.signals.battery.connect(self._update_battery)
        self.ros.signals.robot_pose.connect(self._update_robot_pose)
        self.ros.signals.speed_scale.connect(self._sync_speed_slider)
        self.ros.signals.estop_toggle.connect(self._toggle_estop_from_pad)
        self.send_cmd = self.ros.send_cmd
        self.send_waypoints = self.ros.send_waypoints
        self.send_speed_scale = self.ros.set_speed_scale
        self.cancel_waypoints = self.ros.cancel_waypoints
        self.send_hsv_params  = self.ros.set_hsv_params
        self.ros.signals.autonomy.connect(self._on_autonomy_state)
        self.ros.signals.mission_done.connect(self._on_mission_done)
        self.ros.start()
        QTimer.singleShot(250, self._notify_speed_change)

    # ---- Motion logic
    def _current_scale(self) -> float:
        return _slider_to_scale(self.speed.value())

    def _recompute_dpad_motion(self):
        s = self._current_scale()
        lin = 0.0
        ang = 0.0
        if self._dpad_forward and not self._dpad_back:
            lin = MAX_LINEAR_MPS * s
        elif self._dpad_back and not self._dpad_forward:
            lin = -MAX_LINEAR_MPS * s
        if self._dpad_left and not self._dpad_right:
            ang = MAX_ANGULAR_RPS * s
        elif self._dpad_right and not self._dpad_left:
            ang = -MAX_ANGULAR_RPS * s
        self._lin = lin
        self._ang = ang
        self._dpad_engaged = self._dpad_forward or self._dpad_back or self._dpad_left or self._dpad_right

    def _set_motion(self, forward=None, back=None, left=None, right=None):
        if forward is not None:
            self._dpad_forward = bool(forward)
        if back is not None:
            self._dpad_back = bool(back)
        if left is not None:
            self._dpad_left = bool(left)
        if right is not None:
            self._dpad_right = bool(right)
        self._recompute_dpad_motion()

    def _bump_speed(self, delta_steps: int):
        v = int(self.speed.value())
        v = max(self.speed.minimum(), min(self.speed.maximum(), v + int(delta_steps)))
        self.speed.setValue(v)
        s = self._current_scale()
        if self._dpad_engaged:
            self._recompute_dpad_motion()
        self._append_log(f"[INFO] GUI speed scale -> {s:.2f}")

    def _on_slider_changed(self, _v: int):
        if not self._suppress_speed_emit:
            self._notify_speed_change()
        if self._dpad_engaged:
            self._recompute_dpad_motion()
        self._update_teleop_led_color()

    def _publish_cmd(self):
        if not hasattr(self, "ros") or not self.ros.ready():
            return
        if self._estopped:
            self.send_cmd(0.0, 0.0)
            return
        if REQUIRE_TELEOP_BTN and not self._teleop_enabled and not self._dpad_engaged:
            self.send_cmd(0.0, 0.0)
            return
        self.send_cmd(self._lin, self._ang)

    def _on_estop(self):
        self._set_estop(not getattr(self, "_estopped", False), source="gui")

    # Teleop LED handling
    @Slot(bool, bool)
    def _set_teleop_led(self, teleop_enabled: bool, sim_alive: bool):
        self._teleop_enabled = teleop_enabled
        self._sim_alive = sim_alive
        self._update_teleop_led_color()

    def _update_teleop_led_color(self):
        if getattr(self, "_estopped", False):
            self.led.set_color("#e53935")
            return
        if self._teleop_enabled:
            self.led.set_color("#46d160")
        elif self._sim_alive:
            self.led.set_color("#f6c343")
        else:
            self.led.set_color("#666666")

    def _notify_speed_change(self):
        scale = self._current_scale()
        if self._last_speed_sent is not None and abs(self._last_speed_sent - scale) <= 1e-3:
            return
        self._last_speed_sent = scale
        sender = getattr(self, "send_speed_scale", None)
        if callable(sender):
            try:
                sender(scale)
            except Exception:
                pass

    @Slot(float)
    def _sync_speed_slider(self, scale: float):
        scale = max(0.05, min(2.0, float(scale)))
        target = _scale_to_slider(scale)
        if self.speed.value() == target:
            self._last_speed_sent = scale
            return
        self._suppress_speed_emit = True
        self.speed.setValue(target)
        self._suppress_speed_emit = False
        self._last_speed_sent = scale

    @Slot()
    def _toggle_estop_from_pad(self):
        self._set_estop(not getattr(self, "_estopped", False), source="pad")

    def _set_estop(self, latched: bool, source: str = "gui"):
        if getattr(self, "_estopped", False) == latched:
            self.estop.setChecked(latched)
            return
        self._estopped = latched
        self.estop.blockSignals(True)
        self.estop.setChecked(latched)
        self.estop.setText("Release E-STOP" if latched else "E-STOP")
        self.estop.blockSignals(False)

        if latched:
            self._dpad_forward = self._dpad_back = self._dpad_left = self._dpad_right = False
            self._dpad_engaged = False
            self._lin = 0.0
            self._ang = 0.0
            self._append_log(">>> E-STOP engaged <<<" + (" (controller)" if source == "pad" else ""))
            try:
                if hasattr(self, "send_cmd"):
                    self.send_cmd(0.0, 0.0)
            except Exception:
                pass

            # cancel any active waypoint navigation when E-STOP is latched
            if hasattr(self, "cancel_waypoints"):
                try:
                    self.cancel_waypoints()
                except Exception:
                    pass

        else:
            self._append_log(">>> E-STOP released <<<" + (" (controller)" if source == "pad" else ""))

        self._update_teleop_led_color()

    def _toggle_cam_mode_btn(self, checked: bool):
        mode = 'hsv' if checked else 'rgb'
        if hasattr(self, "ros"):
            self.ros.set_camera_mode(mode)
        self._reflect_camera_mode(mode)

    @Slot(str)
    def _reflect_camera_mode(self, mode: str):
        checked = (str(mode).lower() == 'hsv')
        if hasattr(self, "cam_mode_btn"):
            self.cam_mode_btn.blockSignals(True)
            self.cam_mode_btn.setChecked(checked)
            self.cam_mode_btn.setText("View: HSV mask" if checked else "View: Camera")
            self.cam_mode_btn.blockSignals(False)

    def _format_tree_row(self, row: Dict[str, object]):
        status = str(row.get("status", "unknown")).lower()
        color = None
        if status == "bad":
            color = QColor("#ff6666")
        elif status == "good":
            color = QColor("#66ff66")
        dist = float(row.get("distance", float('nan')))
        return [
            f"{float(row.get('x', 0.0)):.2f}",
            f"{float(row.get('y', 0.0)):.2f}",
            f"{float(row.get('z', 0.0)):.2f}",
            (status.capitalize(), color),
            f"{float(row.get('uncertainty', 1.0)):.2f}",
            f"{dist:.2f}" if math.isfinite(dist) else "--",
        ]

    def _populate_tree_row(self, index: int, row: Dict[str, object]):
        display = self._format_tree_row(row)
        for col, val in enumerate(display):
            if col == 3:
                text, color = val
                self._set_tree_cell(index, col, text, color)
            else:
                self._set_tree_cell(index, col, val)

    def _update_tree_row(self, index: int, row: Dict[str, object]):
        key = row.get("key")
        prev_entry = self._tree_table_cache.get(key, {})
        prev_display = prev_entry.get("display")
        new_display = self._format_tree_row(row)
        if not prev_display:
            self._populate_tree_row(index, row)
        else:
            for col, val in enumerate(new_display):
                if col == 3:
                    text, color = val
                    prev_text, prev_color = prev_display[col]
                    if text != prev_text or color != prev_color:
                        self._set_tree_cell(index, col, text, color)
                else:
                    if prev_display[col] != val:
                        self._set_tree_cell(index, col, val)
        self._tree_table_cache[key] = {"display": new_display}

    # ---- Launch control (QProcess) ----
    def _toggle_launch(self, checked: bool):
        if checked:
            self._start_launch(DEFAULT_LAUNCH_CMD)
        else:
            self._stop_launch()

    def _start_launch(self, cmd: str):
        if self._proc_launch.state() != QProcess.NotRunning:
            self._append_log("[WARN] launch already running")
            self.launch_btn.setChecked(True)
            return
        self._append_log(f"[INFO] Launching: {cmd}")
        self._proc_launch.start("bash", ["-lc", cmd])
        if not self._proc_launch.waitForStarted(3000):
            self._append_log("[ERROR] failed to start launch process")
            self.launch_btn.setChecked(False)

    def _stop_launch(self):
        if self._proc_launch.state() == QProcess.NotRunning:
            return
        self._append_log("[INFO] Stopping launch…")
        self._proc_launch.terminate()
        QTimer.singleShot(3000, lambda: self._proc_launch.kill()
                          if self._proc_launch.state() != QProcess.NotRunning else None)

    @Slot()
    def _on_launch_output(self):
        try:
            text = self._proc_launch.readAllStandardOutput().data().decode(errors="ignore")
            if text:
                for line in text.splitlines():
                    self._append_log(line)
        except Exception:
            pass

    @Slot(int, QProcess.ExitStatus)
    def _on_launch_finished(self, code: int, _status: QProcess.ExitStatus):
        self._append_log(f"[INFO] launch exited (rc={code})")
        self.launch_btn.setChecked(False)

    # ---- Build control (QProcess) ----
    def _start_build(self):
        if self._proc_build.state() != QProcess.NotRunning:
            self._append_log("[WARN] build already running")
            return
        self.build_btn.setEnabled(False)
        pre = DEFAULT_BUILD_PRE.strip()
        cmd = DEFAULT_BUILD_CMD
        full_cmd = f"{pre}; {cmd}"
        cwd = DEFAULT_BUILD_CWD or os.getcwd()
        self._append_log(f"[INFO] Rebuilding in {cwd} -> `{cmd}`")
        self._proc_build.setWorkingDirectory(cwd)
        self._proc_build.start("bash", ["-lc", full_cmd])
        if not self._proc_build.waitForStarted(3000):
            self._append_log("[ERROR] failed to start build")
            self.build_btn.setEnabled(True)

    @Slot()
    def _on_build_output(self):
        try:
            text = self._proc_build.readAllStandardOutput().data().decode(errors="ignore")
            if text:
                for line in text.splitlines():
                    self._append_log(line)
        except Exception:
            pass

    @Slot(int, QProcess.ExitStatus)
    def _on_build_finished(self, code: int, _status: QProcess.ExitStatus):
        self._append_log(f"[INFO] build finished (rc={code})")
        self.build_btn.setEnabled(True)

    # ---- UI updates
    def _update_tree_counts(self, total: int, bad: int):
        self.tree_total, self.tree_bad = total, bad
        self.tree_lbl.setText(f"Trees: {total}  (bad {bad})")

    def _update_tree_table(self, rows: List[Dict[str, object]]):
        if not hasattr(self, "tree_table"):
            return
        keys = [tuple(row.get("key", (round(row.get("x", 0.0), 2), round(row.get("y", 0.0), 2)))) for row in rows]
        if keys != self._tree_table_keys:
            self.tree_table.setRowCount(len(rows))
            for idx, row in enumerate(rows):
                self._populate_tree_row(idx, row)
            self._tree_table_keys = keys
        else:
            for idx, row in enumerate(rows):
                self._update_tree_row(idx, row)
        self._tree_table_cache = {
            key: {"display": self._format_tree_row(row)}
            for key, row in zip(keys, rows)
        }
        self.tree_table.resizeRowsToContents()

    @Slot(object) 
    def _update_robot_pose(self, pose_obj): 
        try: 
            if pose_obj is None: 
                return 
            x, y, yaw = pose_obj 
            self._robot_pose = (float(x), float(y), float(yaw)) 
            if hasattr(self, "odom_lbl"): 
                self.odom_lbl.setText(f"Pose: x={float(x):.2f} y={float(y):.2f} yaw={math.degrees(float(yaw)):.1f}°") 
        except Exception: 
            pass
    
    def _get_current_robot_pose(self):
        return self._robot_pose

    def _send_waypoint_list(self, rows: List[Dict[str, float]]):
        if not rows:
            self._append_log("[WARN] No waypoints to send.")
            return
        if not hasattr(self, "ros") or not self.ros.ready():
            self._append_log("[WARN] ROS node not ready; cannot send waypoints.")
            return
        try:
            self.send_waypoints(rows)
            self._append_log(f"[INFO] Sent {len(rows)} waypoint(s).")
        except Exception as e:
            self._append_log(f"[ERROR] Failed to send waypoints: {e}")

    def _set_tree_cell(self, row: int, col: int, text: str, color: Optional[QColor] = None):
        item = QTableWidgetItem(text)
        item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        if color is not None:
            item.setForeground(color)
        self.tree_table.setItem(row, col, item)

    def _append_log(self, text: str):
        if not self._quitting:
            self.log.append(text)

    @Slot(int)
    def _update_pc_points(self, n: int):
        self.pc_val.setText(self._fmt_points(n))

    def _fmt_points(self, n: int) -> str:
        if n >= 1_000_000: return f"~{n/1_000_000:.1f}M"
        if n >= 1000:      return f"~{n/1000:.0f}k"
        return f"~{n}"

    @Slot()
    def _update_cpu(self):
        if not hasattr(self, "cpu_lbl"):
            return
        if self._cpu_process and psutil:
            try:
                proc_pct = self._cpu_process.cpu_percent(interval=None)
                sys_pct = psutil.cpu_percent(interval=None)
                self.cpu_lbl.setText(f"CPU: app {proc_pct:.1f}% | sys {sys_pct:.0f}%")
                return
            except Exception:
                pass
        self.cpu_lbl.setText("CPU: use top • ros2 topic hz /camera/image")

    @Slot(float, float, bool)
    def _update_battery(self, pct, volt, charging):
        try:
            if pct is None or math.isnan(pct):
                txt = "—%"
                col = "#bbb"
            else:
                txt = f"{pct:.0f}%"
                col = "#46d160" if pct >= 60 else "#f6c343" if pct >= 25 else "#e53935"
                if charging: col = "#46d160"
            if volt is not None and not math.isnan(volt):
                txt += f" ({volt:.1f}V)"
            if charging:
                txt += " ⚡"
            self.batt_lbl.setText(txt)
            self.batt_lbl.setStyleSheet(f"font-weight:700; margin-left:6px; color:{col};")
        except Exception:
            pass

    @Slot(object)
    def _update_camera(self, rgb_np):
        if self._quitting or rgb_np is None: return
        h, w, _ = rgb_np.shape
        qimg = QImage(rgb_np.data, w, h, 3 * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        self.camera_lbl.setPixmap(pix.scaled(
            self.camera_lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    def _set_map_pixmap(self, label: QLabel, rgb_np):
        if label is None or rgb_np is None:
            return
        h, w, _ = rgb_np.shape
        qimg = QImage(rgb_np.data, w, h, 3*w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        label.setPixmap(pix.scaled(
            label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    @Slot(object)
    def _update_map(self, payload):
        if self._quitting or payload is None:
            return
        if isinstance(payload, dict):
            self._set_map_pixmap(self.map_tree_lbl, payload.get("trees"))
            self._set_map_pixmap(self.map_nav_lbl, payload.get("nav"))
        else:
            # backward compatibility
            self._set_map_pixmap(self.map_tree_lbl, payload)
            self._set_map_pixmap(self.map_nav_lbl, payload)

    # ---- Clean shutdown
    def closeEvent(self, event):
        try:
            self._quitting = True
            if hasattr(self, "cmd_timer"):
                self.cmd_timer.stop()
                try: self.cmd_timer.timeout.disconnect(self._publish_cmd)
                except Exception: pass
            if hasattr(self, "_cpu_timer"):
                try: self._cpu_timer.stop()
                except Exception: pass

            try: self._stop_launch()
            except Exception: pass

            if self._proc_build.state() != QProcess.NotRunning:
                try: self._proc_build.terminate()
                except Exception: pass

            try: self.ros.signals.image.disconnect(self._update_camera)
            except Exception: pass
            try: self.ros.signals.map_image.disconnect(self._update_map)
            except Exception: pass
            try: self.ros.signals.log.disconnect(self._append_log)
            except Exception: pass
            try: self.ros.signals.ok.disconnect()
            except Exception: pass
            try: self.ros.signals.bump_speed.disconnect(self._bump_speed)
            except Exception: pass
            try: self.ros.signals.tree_counts.disconnect(self._update_tree_counts)
            except Exception: pass
            try: self.ros.signals.tree_table.disconnect(self._update_tree_table)
            except Exception: pass
            try: self.ros.signals.camera_mode.disconnect(self._reflect_camera_mode)
            except Exception: pass
            try: self.ros.signals.pc_points.disconnect(self._update_pc_points)
            except Exception: pass
            try: self.ros.signals.teleop_led.disconnect(self._set_teleop_led)
            except Exception: pass
            try: self.ros.signals.battery.disconnect(self._update_battery)
            except Exception: pass
            try: self.ros.signals.robot_pose.disconnect(self._update_robot_pose)
            except Exception: pass
            try: self.ros.signals.speed_scale.disconnect(self._sync_speed_slider)
            except Exception: pass
            try: self.ros.signals.estop_toggle.disconnect(self._toggle_estop_from_pad)
            except Exception: pass

            self.send_cmd = lambda *_a, **_k: None
            self.ros.stop(); self.ros.wait(2000)
        except Exception:
            pass
        super().closeEvent(event)

    def _install_sigint_handler(self):
        def on_sigint(_signum, _frame):
            if not self._quitting:
                self._quitting = True
                QTimer.singleShot(0, self.close)
            else:
                signal.signal(signal.SIGINT, signal.SIG_DFL)
        try: signal.signal(signal.SIGINT, on_sigint)
        except Exception: pass

    def _build_hsv_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        note = QLabel("Adjust HSV thresholds for the UI mask. Values update live when in HSV view.")
        note.setWordWrap(True)
        layout.addWidget(note)

        entries = [
            ("Green Low", "green_low"),
            ("Green High", "green_high"),
            ("Red1 Low", "red1_low"),
            ("Red1 High", "red1_high"),
            ("Red2 Low", "red2_low"),
            ("Red2 High", "red2_high"),
        ]
        channels = ["H", "S", "V"]
        for title, key in entries:
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(6)
            lbl = QLabel(title)
            lbl.setMinimumWidth(90)
            row_layout.addWidget(lbl)
            for idx, chan in enumerate(channels):
                spin = QSpinBox()
                spin.setButtonSymbols(QSpinBox.NoButtons)
                spin.setFixedWidth(56)
                max_val = 179 if chan == "H" else 255
                spin.setRange(0, max_val)
                spin.setValue(int(self._hsv_params.get(key, [0, 0, 0])[idx]))
                spin.valueChanged.connect(lambda val, k=key, i=idx: self._on_hsv_spin_changed(k, i, val))
                row_layout.addWidget(QLabel(chan))
                row_layout.addWidget(spin)
                self._hsv_inputs[(key, idx)] = spin
            row_layout.addStretch(1)
            layout.addWidget(row)

        layout.addStretch(1)
        self.hsv_status_lbl = QLabel("HSV sliders ready.")
        layout.addWidget(self.hsv_status_lbl, alignment=Qt.AlignLeft)
        return widget

    def _on_hsv_spin_changed(self, key: str, idx: int, value: int):
        arr = self._hsv_params.setdefault(key, [0, 0, 0])
        arr[idx] = int(value)
        self._queue_hsv_publish()

    def _queue_hsv_publish(self):
        if self._hsv_pending:
            return
        self._hsv_pending = True
        QTimer.singleShot(150, self._publish_hsv_params)

    def _publish_hsv_params(self):
        self._hsv_pending = False
        payload = {k: list(v) for k, v in self._hsv_params.items()}
        sender = getattr(self, "send_hsv_params", None)
        if callable(sender):
            try:
                sender(payload)
                if hasattr(self, "hsv_status_lbl"):
                    self.hsv_status_lbl.setText(
                        f"HSV updated (G {payload['green_low']}-{payload['green_high']}, "
                        f"R1 {payload['red1_low']}-{payload['red1_high']}, "
                        f"R2 {payload['red2_low']}-{payload['red2_high']})"
                    )
            except Exception as e:
                if hasattr(self, "hsv_status_lbl"):
                    self.hsv_status_lbl.setText(f"HSV update failed: {e}")

    def _on_map_tab_changed(self, idx: int):
        self._map_tab_index = idx
        if idx == 2 and hasattr(self, "hsv_status_lbl"):
            self.hsv_status_lbl.setText("HSV sliders ready.")

# ============================ ROS worker ============================
class RosWorker(QThread):
    def __init__(self):
        super().__init__()
        self.signals = RosSignals()
        self._executor: Optional[MultiThreadedExecutor] = None
        self._node: Optional[GuiRosNode] = None
        self._stop = False
        self._ready = False

    def run(self):
        try:
            rclpy.init(args=None)
            self._node = GuiRosNode(self.signals)
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._ready = True
            while not self._stop:
                self._executor.spin_once(timeout_sec=0.01)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self.signals.ok.emit(False, f"ROS error: {e}")
        finally:
            self._ready = False
            try:
                if self._executor and self._node:
                    self._executor.remove_node(self._node)
                if self._node:
                    self._node.destroy_node()
            except Exception:
                pass
            try: rclpy.shutdown()
            except Exception: pass

    def stop(self): self._stop = True

    @Slot(float, float)
    def send_cmd(self, linear: float, angular: float):
        if self._node is not None and self._ready and not self._stop:
            try: self._node.publish_cmd(linear, angular)
            except Exception: pass

    @Slot(float)
    def set_speed_scale(self, scale: float):
        if self._node is not None and self._ready and not self._stop:
            try:
                self._node.set_speed_scale(scale)
            except Exception:
                pass

    @Slot(object)
    def send_waypoints(self, waypoints):
        if self._node is not None and self._ready and not self._stop:
            try: self._node.publish_waypoints(waypoints)
            except Exception:
                pass

    def ready(self) -> bool:
        try:
            return self._ready and not self._stop and rclpy.ok()
        except Exception:
            return False

    @Slot(str)
    def set_camera_mode(self, mode: str):
        if self._node is not None and self._ready and not self._stop:
            try: self._node.set_camera_mode(mode)
            except Exception:
                pass

    @Slot()
    def cancel_waypoints(self):
        if self._node is not None and self._ready and not self._stop:
            try:
                self._node.cancel_waypoints()
            except Exception:
                pass

    @Slot(object)
    def set_hsv_params(self, params):
        if self._node is not None and self._ready and not self._stop:
            try:
                self._node.set_hsv_params(params)
            except Exception:
                pass

    @Slot(str)
    def _on_autonomy_state(self, s: str):
        s = (s or "").lower()
        # same palette as the node uses
        color_map = {
            "idle":      "#666666",
            "scanning":  "#4aa3ff",
            "planning":  "#ffcc00",
            "executing": "#46d160",
            "teleop":    "#f6c343",
            "complete":  "#9c27b0",
        }
        self.auto_led.set_color(color_map.get(s, "#666666"))
        self._append_log(f"[AUTO] {s}")

    @Slot(bool)
    def _on_mission_done(self, ok: bool):
        if ok:
            self._append_log(">>> MISSION COMPLETE ✅")
            self.auto_led.set_color("#9c27b0")  # purple flash on completion

            
# ============================== main ================================
def _apply_qt_logging_rules_from_env():
    if os.environ.get("UI_SUPPRESS_QT_DEBUG", "1") == "1" and "QT_LOGGING_RULES" not in os.environ:
        os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

def main():
    _apply_qt_logging_rules_from_env()
    app = QApplication(sys.argv)
    app.setApplicationName("ForestGuardUI")
    app.setOrganizationName("ForestGuard")
    app.setDesktopFileName("forestguard-ui")

    icon_path = os.environ.get("UI_APP_ICON", os.path.expanduser(
        "~/git/RS1/john_branch/src/forestguard_ui/assets/app_icon.png"
    ))
    if os.path.exists(icon_path):
        app.setWindowIcon(QIcon(icon_path))

    w = ControlGUI()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
