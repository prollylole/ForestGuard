#!/usr/bin/env python3
# Robot UI (dark theme) — camera, D-pad (square), logs, tree counter, speed slider + Run Sim + Rebuild Code
from __future__ import annotations

import sys, os, math, signal, shlex
from typing import Optional
from threading import Lock

from PySide6.QtCore import Qt, QObject, QThread, QTimer, Signal, Slot, QPointF, QRect, QProcess
from PySide6.QtGui import QColor, QPainter, QBrush, QImage, QPixmap, QIcon, QPalette
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QSlider, QFrame,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSizePolicy, QGraphicsDropShadowEffect,
    QStyleFactory
)

# ---------- topics / env ----------
CAMERA_TOPIC_DEFAULT = "/camera/image"
ROSOUT_TOPIC         = "/rosout"
CAMERA_TOPIC_CTRL    = "/ui/camera_topic"
CMD_VEL_TOPIC        = "/cmd_vel"

TREE_TOTAL_TOPIC = os.environ.get("UI_TREE_TOTAL_TOPIC", "/trees/total")
TREE_BAD_TOPIC   = os.environ.get("UI_TREE_BAD_TOPIC",   "/trees/bad")

CMD_PUB_RATE_HZ = 20
MAX_LINEAR_MPS  = 0.40
MAX_ANGULAR_RPS = 1.50

# Launch control (Run Sim button)
DEFAULT_LAUNCH_CMD = os.environ.get(
    "UI_LAUNCH_CMD",
    "ros2 launch john JOHNAUTO.launch.py rviz:=false ui:=false teleop:=true amcl:=true slam:=false map:=true"
)

# Build control (Rebuild Code button)  — Option B defaults
_DEFAULT_WS = os.path.expanduser("~/git/RS1/john_branch")
DEFAULT_BUILD_CWD = os.environ.get(
    "UI_BUILD_CWD",
    _DEFAULT_WS if os.path.isdir(_DEFAULT_WS) else os.getcwd()
)
DEFAULT_BUILD_PRE = os.environ.get(
    "UI_BUILD_PRE",
    "source /opt/ros/humble/setup.bash"
)
DEFAULT_BUILD_CMD = os.environ.get(
    "UI_BUILD_CMD",
    "colcon build --symlink-install "
    "--packages-select forestguard_colour forestguard_controller "
    "forestguard_localisation forestguard_sim forestguard_ui john"
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

# ---- ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CompressedImage as RosCompressedImage
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rcl_interfaces.msg import Log as RosLog
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32, UInt32

# ========================= tiny widgets ===============================
class LedIndicator(QLabel):
    def __init__(self, color=QColor("#46d160"), diameter=14, parent=None):
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
    """Keeps its single child perfectly square (centered)."""
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

def _slider_to_scale(v: int) -> float:       # 0..20 -> 0.10..1.50
    return 0.10 + (float(v) / 20.0) * (1.50 - 0.10)

def _scale_to_slider(s: float) -> int:
    s = max(0.10, min(1.50, float(s)))
    return int(round((s - 0.10) / (1.50 - 0.10) * 20.0))

# ========================= ROS <-> Qt bridge ========================
class RosSignals(QObject):
    image = Signal(object)
    log = Signal(str)
    running = Signal(bool)
    ok = Signal(bool, str)
    bump_speed = Signal(int)
    tree_counts = Signal(int, int)   # (total, bad)

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

        # /rosout
        if os.environ.get("UI_DISABLE_ROSOUT", "0") != "1":
            self.create_subscription(RosLog, ROSOUT_TOPIC, self._on_rosout, self.reliable_qos)

        # camera topic control
        self.create_subscription(RosString, CAMERA_TOPIC_CTRL, self._on_camera_topic_switch, 10)

        # cv bridge
        self.bridge: Optional[object] = CvBridge() if _CV_BRIDGE_OK else None

        # camera state
        self._camera_sub = None
        self._camera_topic_raw = CAMERA_TOPIC_DEFAULT
        self._latest_rgb = None
        self._img_lock = Lock()
        self._emit_timer = self.create_timer(1.0 / self._emit_hz, self._emit_image_tick)
        self._subscribe_camera(self._camera_topic_raw)

        # point cloud quick stats (optional)
        try:
            self.create_subscription(PointCloud2, "/camera/depth/points", self._on_pc, self.sensor_qos)
            self._pc_latest_count = 0
            self._pc_log_timer = self.create_timer(0.5, self._emit_pc_count_log)
        except Exception:
            pass

        # tree counters — subscribe ONCE with the correct type
        self._tree_total = 0
        self._tree_bad   = 0
        self._sub_total = self._subscribe_int_counter(TREE_TOTAL_TOPIC, is_bad=False)
        self._sub_bad   = self._subscribe_int_counter(TREE_BAD_TOPIC,   is_bad=True)

        # ---- Joy D-pad -> bump slider ----
        joy_topic = os.environ.get("UI_JOY_TOPIC", "/joy")
        self._hat_axis_v = int(os.environ.get("UI_JOY_HAT_AXIS_V", "7"))
        self._btn_up = int(os.environ.get("UI_JOY_BTN_UP", "-1"))
        self._btn_down = int(os.environ.get("UI_JOY_BTN_DOWN", "-1"))
        self._prev_up = False
        self._prev_down = False
        try:
            from sensor_msgs.msg import Joy
            self.create_subscription(Joy, joy_topic, self._on_joy, 10)
            self.get_logger().info(f"D-pad on {joy_topic} (axis_v={self._hat_axis_v}, btn_up={self._btn_up}, btn_down={self._btn_down})")
        except Exception as e:
            self.get_logger().warn(f"Joy subscribe failed: {e}")

        self.signals.ok.emit(True, "ROS node initialised")

    # ---- subscribe helper (Int32/UInt32 autodetect, no duplicates)
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
        if topic == self._camera_topic_raw or topic.rstrip('/') == self._camera_topic_raw.rstrip('/'):
            return
        try:
            self._subscribe_camera(topic)
            self.signals.ok.emit(True, f"Switched camera to: {topic}")
        except Exception as e:
            self.signals.ok.emit(False, f"Failed to switch camera: {e}")

    def _subscribe_camera(self, topic_base: str):
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
                self.get_logger().warn(f"Compressed subscribe failed ({e}); falling back to raw.")

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
            self.signals.image.emit(rgb)

    def _on_pc(self, msg: PointCloud2):
        try:
            cnt = 0
            for i, _ in enumerate(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)):
                if i % 10 == 0: cnt += 1
                if cnt >= 20000: break
            self._pc_latest_count = cnt * 10
        except Exception:
            pass

    def _emit_pc_count_log(self):
        if getattr(self, "_pc_latest_count", 0):
            self.signals.log.emit(f"[INFO] pointcloud: ~{self._pc_latest_count} points (approx, decimated)")
            self._pc_latest_count = 0

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
        if up and not self._prev_up:   self.signals.bump_speed.emit(+1)
        if down and not self._prev_down: self.signals.bump_speed.emit(-1)
        self._prev_up, self._prev_down = up, down

    # ------------------- helpers ----------------------
    def _image_to_rgb_numpy(self, msg: RosImage):
        try:
            if self.bridge is not None and _CV_BRIDGE_OK:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # type: ignore
                if cv2 is not None:
                    return cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                else:
                    return cv_image[:, :, ::-1].copy()
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
        self._drive_enabled = os.environ.get("UI_ENABLE_DRIVE", "0") == "1"

        # QProcess for launch/build (keeps IO on Qt thread; no crashes)
        self._proc_launch = QProcess(self)
        self._proc_launch.setProcessChannelMode(QProcess.MergedChannels)
        self._proc_launch.readyReadStandardOutput.connect(self._on_launch_output)
        self._proc_launch.finished.connect(self._on_launch_finished)

        self._proc_build = QProcess(self)
        self._proc_build.setProcessChannelMode(QProcess.MergedChannels)
        self._proc_build.readyReadStandardOutput.connect(self._on_build_output)
        self._proc_build.finished.connect(self._on_build_finished)

        self._install_sigint_handler()
        self._build_ui()
        self._wire_behaviour()
        self._start_ros()
        self.speed.setValue(_scale_to_slider(0.60))

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

    # ---- UI
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
        cam = self.rounded_pane(self.camera_lbl, pad=10)

        # Right-top placeholder (map / future)
        self.right_top = QLabel("Map / Panel")
        self.right_top.setAlignment(Qt.AlignCenter)
        self.right_top.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_top_pane = self.rounded_pane(self.right_top, pad=10)

        # D-Pad — square
        dpad_core = QWidget(); g = QGridLayout(dpad_core)
        g.setSpacing(6); g.setContentsMargins(0, 0, 0, 0)
        def btn(t): b = QPushButton(t); b.setFixedSize(48, 36); return b
        self.btn_up, self.btn_left, self.btn_right, self.btn_down = btn("▲"), btn("◀"), btn("▶"), btn("▼")
        g.addWidget(self.btn_up, 0, 1); g.addWidget(self.btn_left, 1, 0)
        g.addWidget(self.btn_right, 1, 2); g.addWidget(self.btn_down, 2, 1)
        dpad_square = SquareContainer(dpad_core)
        dpad = self.rounded_pane(dpad_square, pad=10)
        dpad.setMinimumWidth(240)

        # Log Panel
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setPlaceholderText("Log")
        logpane = self.rounded_pane(self.log, pad=8); logpane.setMinimumSize(300, 160)

        # Left bottom row: D-pad (square) + Log
        left_bottom = QHBoxLayout()
        left_bottom.addWidget(dpad, 0)
        left_bottom.addWidget(logpane, 1)

        left_col = QVBoxLayout()
        left_col.addWidget(cam, 3)
        left_col.addLayout(left_bottom, 2)

        # Right column: top = Map/Panel, bottom split 50/50 (tree counter + speed)
        # --- Tree counter
        self.tree_total = 0
        self.tree_bad = 0
        self.tree_lbl = QLabel("Trees: 0 (bad 0)")
        self.tree_lbl.setAlignment(Qt.AlignCenter)
        self.tree_lbl.setWordWrap(True)
        self.tree_lbl.setStyleSheet("font-size: 18px; font-weight: 600;")
        tree_pane = self.rounded_pane(self.tree_lbl, pad=10)

        # --- Speed slider + tick labels (vertical)
        self.speed = QSlider(Qt.Vertical); self.speed.setRange(0, 20); self.speed.setTickInterval(5)
        self.speed.setTickPosition(QSlider.TicksRight)
        self.speed.setFixedWidth(46)
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
        right_col.addWidget(right_top_pane, 3)
        right_bottom = QVBoxLayout()
        right_bottom.addWidget(tree_pane, 1)
        right_bottom.addWidget(speed_pane, 1)
        right_col.addLayout(right_bottom, 2)

        # Bottom bar: running LED + Run Sim + Rebuild Code + E-stop
        run = QHBoxLayout()
        run.addWidget(QLabel("Running")); self.led = LedIndicator(); run.addWidget(self.led); run.addStretch(1)
        run_w = QWidget(); run_w.setLayout(run)

        self.launch_btn = QPushButton("Run Sim")
        self.launch_btn.setCheckable(True)
        self.launch_btn.clicked.connect(self._toggle_launch)

        self.build_btn = QPushButton("Rebuild Code")
        self.build_btn.setCheckable(False)
        self.build_btn.clicked.connect(self._start_build)

        self.estop = QPushButton("E Stop"); self.estop.setObjectName("estop"); self.estop.setMinimumSize(140, 44)
        shadow = QGraphicsDropShadowEffect(blurRadius=16, offset=QPointF(0, 2))
        shadow.setColor(QColor(0, 0, 0, 160)); self.estop.setGraphicsEffect(shadow)

        bottom = QHBoxLayout()
        bottom.addWidget(run_w, 1)
        bottom.addWidget(self.launch_btn, 0)
        bottom.addWidget(self.build_btn, 0)
        bottom.addWidget(self.estop, 0, Qt.AlignRight)

        # Root
        root = QVBoxLayout(self)
        top = QHBoxLayout(); top.addLayout(left_col, 3); top.addLayout(right_col, 2)
        root.addLayout(top, 1); root.addLayout(bottom)

        self._apply_dark_theme()

    # ---- Dark theme
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
            QWidget { font-size: 14px; color: #eaeaea; }
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

    # ---- Behaviour
    def _wire_behaviour(self):
        self.btn_up.pressed.connect(lambda: self._set_motion(forward=True))
        self.btn_up.released.connect(lambda: self._set_motion(forward=False))
        self.btn_down.pressed.connect(lambda: self._set_motion(back=True))
        self.btn_down.released.connect(lambda: self._set_motion(back=False))
        self.btn_left.pressed.connect(lambda: self._set_motion(left=True))
        self.btn_left.released.connect(lambda: self._set_motion(left=False))
        self.btn_right.pressed.connect(lambda: self._set_motion(right=True))
        self.btn_right.released.connect(lambda: self._set_motion(right=False))

        self.speed.valueChanged.connect(self._on_slider_changed)
        self.estop.clicked.connect(self._on_estop)

        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._publish_cmd)
        self.cmd_timer.start(int(1000 / CMD_PUB_RATE_HZ))

    def _start_ros(self):
        self.ros = RosWorker()
        self.ros.signals.image.connect(self._update_camera)
        self.ros.signals.log.connect(self._append_log)
        self.ros.signals.running.connect(self._set_running_led)
        self.ros.signals.ok.connect(lambda ok, msg: self._append_log(("OK " if ok else "ERR ") + msg))
        self.ros.signals.bump_speed.connect(self._bump_speed)
        self.ros.signals.tree_counts.connect(self._update_tree_counts)
        self.send_cmd = self.ros.send_cmd
        self.ros.start()

    # ---- Motion logic
    def _current_scale(self) -> float:
        return _slider_to_scale(self.speed.value())

    def _set_motion(self, forward=None, back=None, left=None, right=None):
        s = self._current_scale()
        if forward is not None:
            self._lin = (MAX_LINEAR_MPS * s) if forward else 0.0 if not back else self._lin
        if back is not None:
            self._lin = (-MAX_LINEAR_MPS * s) if back else 0.0 if not forward else self._lin
        if left is not None:
            self._ang = (MAX_ANGULAR_RPS * s) if left else 0.0 if not right else self._ang
        if right is not None:
            self._ang = (-MAX_ANGULAR_RPS * s) if right else 0.0 if not left else self._ang
        self._update_running_led()

    def _bump_speed(self, delta_steps: int):
        v = int(self.speed.value())
        v = max(self.speed.minimum(), min(self.speed.maximum(), v + int(delta_steps)))
        self.speed.setValue(v)
        s = self._current_scale()
        sign_lin = (1.0 if self._lin > 0 else -1.0 if self._lin < 0 else 0.0)
        sign_ang = (1.0 if self._ang > 0 else -1.0 if self._ang < 0 else 0.0)
        if sign_lin != 0.0: self._lin = sign_lin * MAX_LINEAR_MPS * s
        if sign_ang != 0.0: self._ang = sign_ang * MAX_ANGULAR_RPS * s
        self._append_log(f"[INFO] GUI speed scale -> {s:.2f}")

    def _on_slider_changed(self, _v: int):
        self._update_running_led()

    def _publish_cmd(self):
        if not hasattr(self, "ros") or not self.ros.ready(): return
        if self._estopped: self.send_cmd(0.0, 0.0); return
        if not self._drive_enabled: return
        self.send_cmd(self._lin, self._ang)

    def _on_estop(self):
        self._lin = 0.0; self._ang = 0.0; self._estopped = True
        self._update_running_led()
        self._append_log(">>> EMERGENCY STOP PRESSED! <<<")
        QTimer.singleShot(0, lambda: setattr(self, "_estopped", False))  # remove to latch

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
        # Use bash -lc so sourced env/aliases work (UI should be started from a sourced shell anyway)
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
        full_cmd = f"{pre}; {cmd}" if pre else cmd
        cwd = DEFAULT_BUILD_CWD or os.getcwd()

        self._append_log(f"[INFO] Rebuilding in {cwd} -> `{cmd}`")
        self._proc_build.setWorkingDirectory(cwd)
        # ensure same env as our process (already sourced when launching the UI)
        env = os.environ.copy()
        # Use bash -lc so `source` works inside the shell
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
    def _update_running_led(self):
        running = (abs(self._lin) > 1e-3 or abs(self._ang) > 1e-3) and not self._estopped
        self.led.set_color("#46d160" if running else "#666666")

    def _set_running_led(self, running: bool):
        self.led.set_color("#46d160" if running else "#666666")

    def _update_tree_counts(self, total: int, bad: int):
        self.tree_total, self.tree_bad = total, bad
        self.tree_lbl.setText(f"Trees: {total}  (bad {bad})")

    def _append_log(self, text: str):
        if not self._quitting:
            self.log.append(text)

    @Slot(object)
    def _update_camera(self, rgb_np):
        if self._quitting or rgb_np is None: return
        h, w, _ = rgb_np.shape
        qimg = QImage(rgb_np.data, w, h, 3 * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        self.camera_lbl.setPixmap(pix.scaled(
            self.camera_lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    # ---- Clean shutdown
    def closeEvent(self, event):
        try:
            self._quitting = True
            if hasattr(self, "cmd_timer"):
                self.cmd_timer.stop()
                try: self.cmd_timer.timeout.disconnect(self._publish_cmd)
                except Exception: pass

            try: self._stop_launch()
            except Exception: pass

            if self._proc_build.state() != QProcess.NotRunning:
                try: self._proc_build.terminate()
                except Exception: pass

            try: self.ros.signals.image.disconnect(self._update_camera)
            except Exception: pass
            try: self.ros.signals.log.disconnect(self._append_log)
            except Exception: pass
            try: self.ros.signals.running.disconnect(self._set_running_led)
            except Exception: pass
            try: self.ros.signals.ok.disconnect()
            except Exception: pass
            try: self.ros.signals.bump_speed.disconnect(self._bump_speed)
            except Exception: pass
            try: self.ros.signals.tree_counts.disconnect(self._update_tree_counts)
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

    def ready(self) -> bool:
        try:
            return self._ready and not self._stop and rclpy.ok()
        except Exception:
            return False

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
