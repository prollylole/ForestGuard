#!/usr/bin/env python3
# Robot Control Panel — fixed layout
# - Camera keeps aspect (UI_CAMERA_ASPECT env, default 16:9)
# - D-pad is always square
# - Bottom row: Log (left) and Speed box (right) share the same height
# - Top-right is a panel (Tree Count by default; you can swap in your map)

from __future__ import annotations
import sys, os, math, signal
from typing import Optional
from threading import Lock

from PySide6.QtCore import Qt, QObject, QThread, QTimer, Signal, Slot, QPointF, QSize
from PySide6.QtGui import QColor, QPainter, QBrush, QImage, QPixmap, QFont
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QSlider, QFrame,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSizePolicy, QGraphicsDropShadowEffect
)

# --------------------------- CONFIG ---------------------------------
CAMERA_TOPIC_DEFAULT = "/camera/image"
ROSOUT_TOPIC         = "/rosout"
CAMERA_TOPIC_CTRL    = "/ui/camera_topic"
CMD_VEL_TOPIC        = "/cmd_vel"

# Tree count topics
TREE_COUNT_TOPIC = os.environ.get("UI_TREE_COUNT_TOPIC", "/trees/count")
TREE_GOOD_TOPIC  = os.environ.get("UI_TREE_GOOD_TOPIC", "/trees/good_count")
TREE_BAD_TOPIC   = os.environ.get("UI_TREE_BAD_TOPIC",  "/trees/bad_count")

CMD_PUB_RATE_HZ = 20
MAX_LINEAR_MPS  = 0.40
MAX_ANGULAR_RPS = 1.50

# Camera aspect (e.g., "16:9", "4:3", "1:1", "640x480")
_UI_AR = os.environ.get("UI_CAMERA_ASPECT", "16:9")
def _parse_aspect(s: str) -> float:
    s = s.strip().lower()
    if "x" in s:
        w, h = s.split("x", 1)
        return float(w)/float(h)
    if ":" in s:
        w, h = s.split(":", 1)
        return float(w)/float(h)
    try:
        r = float(s)
        return r if r > 0 else 16/9
    except Exception:
        return 16/9
CAMERA_ASPECT = _parse_aspect(_UI_AR)
# --------------------------------------------------------------------

# Optional deps
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
from sensor_msgs.msg import Image as RosImage, CompressedImage as RosCompressedImage, PointCloud2, Joy
import sensor_msgs_py.point_cloud2 as pc2
from rcl_interfaces.msg import Log as RosLog
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32 as RosInt32

# ========================= Qt helpers ===============================
class LedIndicator(QLabel):
    def __init__(self, color=QColor("limegreen"), diameter=14, parent=None):
        super().__init__(parent)
        self._color = QColor(color); self._diameter = diameter
        self.setFixedSize(diameter, diameter)
    def set_color(self, color): self._color = QColor(color); self.update()
    def paintEvent(self, _):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QBrush(self._color)); p.setPen(Qt.black)
        p.drawEllipse(0, 0, self._diameter, self._diameter)

class AspectRatioLabel(QLabel):
    """Keeps a target width:height ratio, scaling pixmap with Qt.KeepAspectRatio."""
    def __init__(self, ratio: float = 16/9, *a, **k):
        super().__init__(*a, **k)
        self._ratio = max(0.1, float(ratio))
        self.setAlignment(Qt.AlignCenter)
        sp = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sp.setHeightForWidth(True)
        self.setSizePolicy(sp)
    def set_ratio(self, r: float):
        self._ratio = max(0.1, float(r)); self.updateGeometry(); self.update()
    def hasHeightForWidth(self): return True
    def heightForWidth(self, w: int) -> int: return int(round(w / self._ratio))
    def sizeHint(self) -> QSize: return QSize(960, int(960/self._ratio))
    def minimumSizeHint(self) -> QSize: return QSize(320, int(320/self._ratio))
    def setPixmapKeep(self, pm: QPixmap):
        if pm.isNull(): self.setText("Camera Feed"); return
        box = self.size()
        pm2 = pm.scaled(box, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        super().setPixmap(pm2)
    def resizeEvent(self, e):
        # rescale existing pixmap during window resize
        pm = self.pixmap()
        if pm: self.setPixmapKeep(pm)
        super().resizeEvent(e)

class SquareHolder(QWidget):
    """A widget that enforces width == height (square) while allowing it to expand."""
    def __init__(self, inner: QWidget, pad=12, parent=None):
        super().__init__(parent)
        self._inner = inner
        self._pad = pad
        ly = QVBoxLayout(self); ly.setContentsMargins(pad, pad, pad, pad); ly.addWidget(inner)
        sp = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        sp.setHeightForWidth(True)
        self.setSizePolicy(sp)
    def hasHeightForWidth(self): return True
    def heightForWidth(self, w: int) -> int: return w
    def sizeHint(self) -> QSize: return QSize(160, 160)

def rounded_pane(w: QWidget, pad=10) -> QFrame:
    f = QFrame(); f.setObjectName("pane")
    ly = QVBoxLayout(f); ly.setContentsMargins(pad, pad, pad, pad); ly.addWidget(w)
    return f

# ========================= helpers ================================
def _level_from_str(name: str) -> LoggingSeverity:
    name = (name or "").strip().lower()
    return {"debug":LoggingSeverity.DEBUG,"info":LoggingSeverity.INFO,
            "warn":LoggingSeverity.WARN,"warning":LoggingSeverity.WARN,
            "error":LoggingSeverity.ERROR,"fatal":LoggingSeverity.FATAL}.get(name, LoggingSeverity.INFO)
def _rosout_level_num(name: str) -> int:
    name = (name or "").strip().lower()
    return {"debug":10,"info":20,"warn":30,"warning":30,"error":40,"fatal":50}.get(name, 20)
def _slider_to_scale(v: int) -> float: return 0.10 + (float(v)/20.0)*(1.50-0.10)
def _scale_to_slider(s: float) -> int:
    s = max(0.10, min(1.50, float(s))); return int(round((s-0.10)/(1.50-0.10)*20.0))

# ========================= ROS <-> Qt bridge ========================
class RosSignals(QObject):
    image = Signal(object)
    log = Signal(str)
    running = Signal(bool)
    ok = Signal(bool, str)
    bump_speed = Signal(int)
    tree_counts = Signal(int, int, int)   # total, good, bad

class GuiRosNode(Node):
    def __init__(self, signals: RosSignals):
        super().__init__("forestguard_ui")
        self.signals = signals

        try: self.get_logger().set_level(_level_from_str(os.environ.get("UI_LOG_LEVEL", "info")))
        except Exception: pass
        self._rosout_min_level = _rosout_level_num(os.environ.get("UI_ROSOUT_LEVEL", "info"))
        self._prefer_compressed = os.environ.get("UI_CAMERA_COMPRESSED", "0") == "1"
        try:
            self._emit_hz = float(os.environ.get("UI_CAMERA_EMIT_HZ", "30"))
            self._emit_hz = max(1.0, min(60.0, self._emit_hz))
        except Exception:
            self._emit_hz = 30.0

        # QoS
        self.sensor_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE,
                                     history=HistoryPolicy.KEEP_LAST, depth=1)
        self.reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.VOLATILE,
                                       history=HistoryPolicy.KEEP_LAST, depth=50)

        # pubs
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # subs
        if os.environ.get("UI_DISABLE_ROSOUT", "0") != "1":
            self.create_subscription(RosLog, ROSOUT_TOPIC, self._on_rosout, self.reliable_qos)
        self.create_subscription(RosString, CAMERA_TOPIC_CTRL, self._on_camera_topic_switch, 10)

        self.bridge: Optional[object] = CvBridge() if _CV_BRIDGE_OK else None
        self._camera_sub = None
        self._camera_topic_raw = CAMERA_TOPIC_DEFAULT
        self._latest_rgb = None
        self._img_lock = Lock()
        self._emit_timer = self.create_timer(1.0/self._emit_hz, self._emit_image_tick)
        self._subscribe_camera(self._camera_topic_raw)

        # point cloud (light log)
        try:
            self.create_subscription(PointCloud2, "/camera/depth/points", self._on_pc, self.sensor_qos)
            self._pc_latest_count = 0
            self._pc_log_timer = self.create_timer(0.5, self._emit_pc_count_log)
        except Exception:
            pass

        # Joy D-pad bumps
        joy_topic = os.environ.get("UI_JOY_TOPIC", "/joy")
        self._hat_axis_v = int(os.environ.get("UI_JOY_HAT_AXIS_V", "7"))
        self._btn_up = int(os.environ.get("UI_JOY_BTN_UP", "-1"))
        self._btn_down = int(os.environ.get("UI_JOY_BTN_DOWN", "-1"))
        self._prev_up = False; self._prev_down = False
        try:
            self.create_subscription(Joy, joy_topic, self._on_joy, 10)
            self.get_logger().info(f"Listening for D-pad on {joy_topic} "
                                   f"(axis_v={self._hat_axis_v}, btn_up={self._btn_up}, btn_down={self._btn_down})")
        except Exception as e:
            self.get_logger().warn(f"Joy sub failed: {e}")

        # Tree counts
        self._tree_total = 0; self._tree_good = -1; self._tree_bad = -1
        try: self.create_subscription(RosInt32, TREE_COUNT_TOPIC, self._on_tree_total, 10)
        except Exception as e: self.get_logger().warn(f"Tree total sub failed: {e}")
        if TREE_GOOD_TOPIC:
            try: self.create_subscription(RosInt32, TREE_GOOD_TOPIC, self._on_tree_good, 10)
            except Exception as e: self.get_logger().warn(f"Tree good sub failed: {e}")
        if TREE_BAD_TOPIC:
            try: self.create_subscription(RosInt32, TREE_BAD_TOPIC, self._on_tree_bad, 10)
            except Exception as e: self.get_logger().warn(f"Tree bad sub failed: {e}")

        self.signals.ok.emit(True, "ROS node initialised")

    # ---------- pubs
    @Slot(float, float)
    def publish_cmd(self, linear: float, angular: float):
        msg = Twist(); msg.linear.x = float(linear); msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # ---------- subs
    def _on_tree_total(self, msg: RosInt32):
        self._tree_total = int(getattr(msg, "data", 0))
        self.signals.tree_counts.emit(self._tree_total, self._tree_good, self._tree_bad)
    def _on_tree_good(self, msg: RosInt32):
        self._tree_good = int(getattr(msg, "data", -1))
        if self._tree_bad >= 0: self._tree_total = self._tree_good + self._tree_bad
        self.signals.tree_counts.emit(self._tree_total, self._tree_good, self._tree_bad)
    def _on_tree_bad(self, msg: RosInt32):
        self._tree_bad = int(getattr(msg, "data", -1))
        if self._tree_good >= 0: self._tree_total = self._tree_good + self._tree_bad
        self.signals.tree_counts.emit(self._tree_total, self._tree_good, self._tree_bad)

    def _on_rosout(self, msg: RosLog):
        if int(getattr(msg, "level", 20)) < self._rosout_min_level: return
        level_map = {10:"DEBUG",20:"INFO",30:"WARN",40:"ERROR",50:"FATAL"}
        self.signals.log.emit(f"[{level_map.get(msg.level, msg.level)}] {msg.name}: {msg.msg}")

    def _on_camera_topic_switch(self, msg: RosString):
        topic = (msg.data or "").strip()
        if not topic: return
        if topic.rstrip('/') == self._camera_topic_raw.rstrip('/'): return
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
                self.get_logger().info(f"Subscribed to camera (compressed): {comp_topic}")
                return
            except Exception as e:
                self.get_logger().warn(f"Compressed subscribe failed ({e}); falling back to raw.")
        self._camera_sub = self.create_subscription(RosImage, self._camera_topic_raw, self._on_image_raw, self.sensor_qos)
        self.get_logger().info(f"Subscribed to camera (raw): {self._camera_topic_raw}")

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
                rgb = self._latest_rgb; self._latest_rgb = None
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

    def _on_joy(self, msg: Joy):
        up = down = False
        if self._hat_axis_v >= 0:
            try:
                v = msg.axes[self._hat_axis_v]
                up = (v > 0.5); down = (v < -0.5)
            except Exception: pass
        if self._btn_up >= 0:
            try: up = up or (msg.buttons[self._btn_up] > 0)
            except Exception: pass
        if self._btn_down >= 0:
            try: down = down or (msg.buttons[self._btn_down] > 0)
            except Exception: pass
        if up and not getattr(self, "_prev_up", False): self.signals.bump_speed.emit(+1)
        if down and not getattr(self, "_prev_down", False): self.signals.bump_speed.emit(-1)
        self._prev_up, self._prev_down = up, down

    def _image_to_rgb_numpy(self, msg: RosImage):
        try:
            if self.bridge is not None and _CV_BRIDGE_OK:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if cv2 is not None: return cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                else: return cv_image[:, :, ::-1].copy()
            else:
                if np is None: return None
                enc = (msg.encoding or "").lower()
                if enc in ("rgb8","bgr8"):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    return arr[:, :, ::-1].copy() if enc == "bgr8" else arr
                if enc in ("mono8","8uc1"):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                    return np.stack([arr]*3, axis=-1)
                return None
        except Exception:
            return None

# ============================ GUI ===================================
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
                if self._executor and self._node: self._executor.remove_node(self._node)
                if self._node: self._node.destroy_node()
            except Exception: pass
            try: rclpy.shutdown()
            except Exception: pass
    def stop(self): self._stop = True
    @Slot(float, float)
    def send_cmd(self, linear: float, angular: float):
        if self._node is not None and self._ready and not self._stop:
            try: self._node.publish_cmd(linear, angular)
            except Exception: pass
    def ready(self) -> bool:
        try: return self._ready and not self._stop and rclpy.ok()
        except Exception: return False

class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Panel")
        self.setMinimumSize(900, 600)

        self._quitting = False
        self._estopped = False
        self._lin = 0.0; self._ang = 0.0
        self._drive_enabled = os.environ.get("UI_ENABLE_DRIVE", "0") == "1"

        self._tree_total = 0; self._tree_good = -1; self._tree_bad = -1

        self._install_sigint_handler()
        self._build_ui()
        self._wire_behaviour()
        self._start_ros()
        self.speed.setValue(_scale_to_slider(0.60))

    # ---- build
    def _build_ui(self):
        # Top-left: camera with fixed aspect
        self.camera_lbl = AspectRatioLabel(CAMERA_ASPECT)
        cam = rounded_pane(self.camera_lbl, pad=14)

        # Top-right: tree counts (placeholder for map if you want later)
        self.tree_count_big = QLabel("Trees: 0"); self.tree_count_big.setAlignment(Qt.AlignCenter)
        fnt = QFont(); fnt.setPointSize(22); fnt.setBold(True); self.tree_count_big.setFont(fnt)
        self.tree_count_sub = QLabel(""); self.tree_count_sub.setAlignment(Qt.AlignCenter)
        subf = QFont(); subf.setPointSize(12); self.tree_count_sub.setFont(subf)
        tree_box_inner = QVBoxLayout(); tree_box_inner.addWidget(self.tree_count_big); tree_box_inner.addWidget(self.tree_count_sub)
        tree_w = QWidget(); tree_w.setLayout(tree_box_inner)
        right_top = rounded_pane(tree_w, pad=18)

        # Bottom-left: D-pad (square) + Log panel, same overall height as bottom-right
        # D-pad
        def btn(t): b = QPushButton(t); b.setFixedSize(48, 36); return b
        dpad_w = QWidget(); d = QGridLayout(dpad_w); d.setSpacing(6); d.setContentsMargins(0,0,0,0)
        self.btn_up, self.btn_left, self.btn_right, self.btn_down = btn("▲"), btn("◀"), btn("▶"), btn("▼")
        d.addWidget(self.btn_up, 0, 1); d.addWidget(self.btn_left, 1, 0)
        d.addWidget(self.btn_right, 1, 2); d.addWidget(self.btn_down, 2, 1)
        # square holder around dpad
        dpad_square = SquareHolder(dpad_w, pad=12)
        dpad_box = rounded_pane(dpad_square, pad=0)

        # Log panel (fills remaining width)
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setPlaceholderText("Log Panel")
        logpane = rounded_pane(self.log, pad=12)

        bottom_left_row = QHBoxLayout()
        bottom_left_row.setSpacing(10)
        bottom_left_row.addWidget(dpad_box, 0)  # stays square
        bottom_left_row.addWidget(logpane, 1)   # stretches
        bottom_left_w = QWidget(); bottom_left_w.setLayout(bottom_left_row)
        left_bottom = bottom_left_w  # wrapped by grid cell

        # Bottom-right: Speed slider box (same row height as left -> same height)
        self.speed = QSlider(Qt.Vertical)
        self.speed.setRange(0, 20); self.speed.setValue(8)
        self.speed.setTickInterval(5); self.speed.setTickPosition(QSlider.TicksRight)
        self.speed.setFixedWidth(42)
        labels_col = QVBoxLayout()
        for i, v in enumerate([20, 15, 10, 5, 0]):
            lab = QLabel(str(v)); lab.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            labels_col.addWidget(lab)
            if i < 4: labels_col.addStretch(1)
        speed_row = QHBoxLayout()
        speed_row.addStretch(1)
        speed_row.addWidget(self.speed, 0, Qt.AlignHCenter)
        speed_row.addLayout(labels_col)
        speed_row.addStretch(1)
        speed_panel = QVBoxLayout(); speed_panel.addLayout(speed_row, 1)
        speed_panel.addWidget(QLabel("Speed", alignment=Qt.AlignHCenter))
        speed_box_w = QWidget(); speed_box_w.setLayout(speed_panel)
        right_bottom = rounded_pane(speed_box_w, pad=12)

        # Bottom status row
        bottom_status = QHBoxLayout()
        run = QHBoxLayout(); run.addWidget(QLabel("Running")); self.led = LedIndicator(); run.addWidget(self.led); run.addStretch(1)
        run_w = QWidget(); run_w.setLayout(run)
        self.estop = QPushButton("E Stop"); self.estop.setObjectName("estop"); self.estop.setMinimumSize(140, 44)
        self.estop.setStyleSheet("""
            QPushButton#estop { background:#e53935; color:#fff; border:2px solid #b71c1c; border-radius:18px; padding:8px 18px; font-weight:700; }
            QPushButton#estop:hover   { background:#f44336; }
            QPushButton#estop:pressed { background:#c62828; }
        """)
        shadow = QGraphicsDropShadowEffect(blurRadius=16, offset=QPointF(0, 2)); shadow.setColor(QColor(0,0,0,90))
        self.estop.setGraphicsEffect(shadow)
        bottom_status.addWidget(run_w, 1)
        bottom_status.addWidget(self.estop, 0, Qt.AlignRight)

        # --------- GRID (2x2) so bottom-left and bottom-right share height ----------
        grid = QGridLayout()
        grid.setHorizontalSpacing(10); grid.setVerticalSpacing(10)
        grid.addWidget(cam,        0, 0)
        grid.addWidget(right_top,  0, 1)
        grid.addWidget(left_bottom,1, 0)
        grid.addWidget(right_bottom,1, 1)
        grid.setColumnStretch(0, 3)  # left column wider (camera + log)
        grid.setColumnStretch(1, 1)  # right column narrower (map/trees + slider)
        grid.setRowStretch(0, 1)     # top row
        grid.setRowStretch(1, 1)     # bottom row -> same height as top, but adaptive

        root = QVBoxLayout(self)
        root.addLayout(grid, 1)
        root.addLayout(bottom_status)

        self.setStyleSheet("""
            QWidget { font-size: 14px; }
            QFrame#pane { border: 1.6px solid #222; border-radius: 12px; background: #fff; }
        """)

    # ---- behaviour
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

        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._publish_cmd)
        self.cmd_timer.start(int(1000/CMD_PUB_RATE_HZ))

        self.estop.clicked.connect(self._on_estop)

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

    # ---- motion
    def _current_scale(self) -> float: return _slider_to_scale(self.speed.value())
    def _set_motion(self, forward=None, back=None, left=None, right=None):
        s = self._current_scale()
        if forward is not None:
            self._lin = (MAX_LINEAR_MPS*s) if forward else 0.0 if not back else self._lin
        if back is not None:
            self._lin = (-MAX_LINEAR_MPS*s) if back else 0.0 if not forward else self._lin
        if left is not None:
            self._ang = (MAX_ANGULAR_RPS*s) if left else 0.0 if not right else self._ang
        if right is not None:
            self._ang = (-MAX_ANGULAR_RPS*s) if right else 0.0 if not left else self._ang
        self._update_running_led()
    def _bump_speed(self, delta_steps: int):
        v = max(self.speed.minimum(), min(self.speed.maximum(), int(self.speed.value()) + int(delta_steps)))
        self.speed.setValue(v)
        s = self._current_scale()
        sign_lin = (1.0 if self._lin>0 else -1.0 if self._lin<0 else 0.0)
        sign_ang = (1.0 if self._ang>0 else -1.0 if self._ang<0 else 0.0)
        if sign_lin: self._lin = sign_lin*MAX_LINEAR_MPS*s
        if sign_ang: self._ang = sign_ang*MAX_ANGULAR_RPS*s
        self._append_log(f"[INFO] GUI speed scale -> {s:.2f}")
    def _on_slider_changed(self, _): self._update_running_led()
    def _publish_cmd(self):
        if not hasattr(self, "ros") or not self.ros.ready(): return
        if self._estopped: self.send_cmd(0.0, 0.0); return
        if not self._drive_enabled: return
        self.send_cmd(self._lin, self._ang)
    def _on_estop(self):
        self._lin = 0.0; self._ang = 0.0; self._estopped = True
        self._update_running_led(); self._append_log(">>> EMERGENCY STOP PRESSED! <<<")
        QTimer.singleShot(0, lambda: setattr(self, "_estopped", False))

    # ---- UI updates
    def _update_tree_counts(self, total: int, good: int, bad: int):
        self._tree_total, self._tree_good, self._tree_bad = total, good, bad
        self.tree_count_big.setText(f"Trees: {max(0, int(total))}")
        if good >= 0 and bad >= 0:
            self.tree_count_sub.setText(f"(Good: {int(good)}  /  Bad: {int(bad)})")
        else:
            self.tree_count_sub.setText("")
    def _update_running_led(self):
        running = (abs(self._lin)>1e-3 or abs(self._ang)>1e-3) and not self._estopped
        self.led.set_color("limegreen" if running else "#bbbbbb")
    def _set_running_led(self, running: bool):
        self.led.set_color("limegreen" if running else "#bbbbbb")
    def _append_log(self, text: str):
        if not self._quitting: self.log.append(text)
    @Slot(object)
    def _update_camera(self, rgb_np):
        if self._quitting or rgb_np is None: return
        h, w, _ = rgb_np.shape
        qimg = QImage(rgb_np.data, w, h, 3*w, QImage.Format_RGB888)
        pm = QPixmap.fromImage(qimg)
        self.camera_lbl.setPixmapKeep(pm)

    # ---- shutdown
    def closeEvent(self, event):
        try:
            self._quitting = True
            if hasattr(self, "cmd_timer"): self.cmd_timer.stop()
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

# ============================== main ================================
def _apply_qt_logging_rules_from_env():
    if os.environ.get("UI_SUPPRESS_QT_DEBUG", "1") == "1" and "QT_LOGGING_RULES" not in os.environ:
        os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

def main():
    _apply_qt_logging_rules_from_env()
    app = QApplication(sys.argv)
    def _excepthook(exc_type, exc, tb):
        if exc_type is KeyboardInterrupt:
            try: QApplication.instance().quit()
            except Exception: pass
            return
        sys.__excepthook__(exc_type, exc, tb)
    sys.excepthook = _excepthook
    w = ControlGUI(); w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
