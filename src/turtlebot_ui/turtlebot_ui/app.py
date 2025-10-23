#!/usr/bin/env python3
"""
PySide6 + ROS 2 GUI for TurtleBot / Husky control (compressed camera + frame-drop).

- Publishes /cmd_vel (Twist) at 20 Hz while D-pad buttons are held.
- Subscribes to /camera/image[/compressed] -> shows in camera panel.
- Listens on /ui/camera_topic (String) to switch camera source at runtime.
- Subscribes to /rosout (Log) -> streams to Log panel (filterable).
- (Optional) Subscribes to /camera/depth/points and logs a decimated count.
- Runs ROS in a QThread (MultiThreadedExecutor) so Qt stays responsive.
- Includes red rounded E-STOP. Running LED reflects motion state.
- Image pipeline stores only the LATEST frame and emits at a fixed rate to avoid backlog.

Env knobs:
  UI_SUPPRESS_QT_DEBUG=1 (default): silence noisy Qt debug logs.
  UI_LOG_LEVEL=warn|info|error|debug|fatal (default: info): this node log level.
  UI_ROSOUT_LEVEL=warn|info|error|debug|fatal (default: info): min /rosout level shown.
  UI_DISABLE_ROSOUT=1: don't subscribe to /rosout.
  UI_ENABLE_DRIVE=1: actually publish /cmd_vel (else UI shows but doesn't drive).
  UI_CAMERA_COMPRESSED=1 (default): prefer /<topic>/compressed.
  UI_CAMERA_EMIT_HZ=15 (default): UI redraw rate for camera frames.
"""

from __future__ import annotations

import sys, os, math, signal
from typing import Optional
from threading import Lock

from PySide6.QtCore import Qt, QObject, QThread, QTimer, Signal, Slot, QPointF
from PySide6.QtGui import QColor, QPainter, QBrush, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QSlider, QFrame,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSizePolicy, QGraphicsDropShadowEffect
)

# --------------------------- CONFIG ---------------------------------
CAMERA_TOPIC_DEFAULT = "/camera/image"    # ros_gz_bridge default 'Image'
ROSOUT_TOPIC         = "/rosout"
CAMERA_TOPIC_CTRL    = "/ui/camera_topic" # std_msgs/String
CMD_VEL_TOPIC        = "/cmd_vel"

CMD_PUB_RATE_HZ = 20

MAX_LINEAR_MPS  = 0.40
MAX_ANGULAR_RPS = 1.50
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
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CompressedImage as RosCompressedImage
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rcl_interfaces.msg import Log as RosLog
from std_msgs.msg import String as RosString


# ========================= Qt widgets ===============================
class LedIndicator(QLabel):
    def __init__(self, color=QColor("limegreen"), diameter=14, parent=None):
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


# ========================= ROS <-> Qt bridge ========================
class RosSignals(QObject):
    image = Signal(object)           # numpy RGB array (H,W,3) or None
    log = Signal(str)                # rosout text
    running = Signal(bool)           # running state (for LED)
    ok = Signal(bool, str)           # status, message


class GuiRosNode(Node):
    """ROS2 node performing pubs/subs and emitting Qt signals."""
    def __init__(self, signals: RosSignals):
        super().__init__("turtlebot_gui")
        self.signals = signals

        # Log levels via env
        node_level = _level_from_str(os.environ.get("UI_LOG_LEVEL", "info"))
        try: self.get_logger().set_level(node_level)
        except Exception: pass

        self._rosout_min_level = _rosout_level_num(os.environ.get("UI_ROSOUT_LEVEL", "info"))
        self._prefer_compressed = os.environ.get("UI_CAMERA_COMPRESSED", "1") == "1"
        try:
            self._emit_hz = float(os.environ.get("UI_CAMERA_EMIT_HZ", "15"))
            self._emit_hz = max(1.0, min(60.0, self._emit_hz))
        except Exception:
            self._emit_hz = 15.0

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

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # /rosout (optional)
        if os.environ.get("UI_DISABLE_ROSOUT", "0") != "1":
            self.create_subscription(RosLog, ROSOUT_TOPIC, self._on_rosout, self.reliable_qos)

        # camera topic control
        self.create_subscription(RosString, CAMERA_TOPIC_CTRL, self._on_camera_topic_switch, 10)

        # cv_bridge handle (optional)
        self.bridge: Optional[object] = CvBridge() if _CV_BRIDGE_OK else None

        # Camera state
        self._camera_sub = None
        self._camera_topic_raw = CAMERA_TOPIC_DEFAULT  # base without /compressed
        self._latest_rgb = None
        self._img_lock = Lock()
        self._emit_timer = self.create_timer(1.0 / self._emit_hz, self._emit_image_tick)

        # Subscribe initial camera
        self._subscribe_camera(self._camera_topic_raw)

        # Optional PointCloud2 subscription (lightweight logging)
        try:
            self.create_subscription(PointCloud2, "/camera/depth/points", self._on_pc, self.sensor_qos)
            self._pc_latest_count = 0
            self._pc_log_timer = self.create_timer(0.5, self._emit_pc_count_log)  # 2 Hz log
        except Exception:
            pass

        self.signals.ok.emit(True, "ROS node initialised")

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
        level = level_map.get(msg.level, str(msg.level))
        self.signals.log.emit(f"[{level}] {msg.name}: {msg.msg}")

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
            self.signals.ok.emit(False, f"Failed to switch camera to '{topic}': {e}")

    def _subscribe_camera(self, topic_base: str):
        """Subscribe to compressed if preferred, else raw. topic_base should be the *raw* image base (no /compressed)."""
        # tear down old
        if self._camera_sub is not None:
            try: self.destroy_subscription(self._camera_sub)
            except Exception: pass
            self._camera_sub = None

        self._camera_topic_raw = topic_base.rstrip('/')

        # Try compressed
        if self._prefer_compressed:
            comp_topic = self._camera_topic_raw + "/compressed" if not self._camera_topic_raw.endswith("/compressed") else self._camera_topic_raw
            try:
                self._camera_sub = self.create_subscription(
                    RosCompressedImage, comp_topic, self._on_image_compressed, self.sensor_qos
                )
                self.get_logger().info(f"Subscribed to camera (compressed): {comp_topic}")
                return
            except Exception as e:
                self.get_logger().warn(f"Compressed subscribe failed ({e}); falling back to raw.")

        # Fallback raw
        self._camera_sub = self.create_subscription(RosImage, self._camera_topic_raw, self._on_image_raw, self.sensor_qos)
        self.get_logger().info(f"Subscribed to camera (raw): {self._camera_topic_raw}")

    # Store-latest image (no direct emit here)
    def _on_image_raw(self, msg: RosImage):
        rgb = self._image_to_rgb_numpy(msg)
        if rgb is None:
            return
        with self._img_lock:
            self._latest_rgb = rgb

    def _on_image_compressed(self, msg: RosCompressedImage):
        try:
            if np is None or cv2 is None:
                # No numpy/cv2 — ignore compressed images
                return
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            with self._img_lock:
                self._latest_rgb = rgb
        except Exception as e:
            self.get_logger().warn(f"Compressed decode failed: {e}")

    # Emit most-recent at fixed rate and drop backlog
    def _emit_image_tick(self):
        rgb = None
        with self._img_lock:
            if self._latest_rgb is not None:
                rgb = self._latest_rgb
                self._latest_rgb = None
        if rgb is not None:
            self.signals.image.emit(rgb)

    # Point cloud: light-weight count so we know it's alive
    def _on_pc(self, msg: PointCloud2):
        try:
            cnt = 0
            for i, _ in enumerate(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)):
                if i % 10 == 0:  # decimate by 10 cheaply
                    cnt += 1
                if cnt >= 20000:  # cap work
                    break
            self._pc_latest_count = cnt * 10
        except Exception:
            pass

    def _emit_pc_count_log(self):
        if getattr(self, "_pc_latest_count", 0):
            self.signals.log.emit(f"[INFO] pointcloud: ~{self._pc_latest_count} points (approx, decimated)")
            self._pc_latest_count = 0

    # ------------------- helpers ----------------------
    def _image_to_rgb_numpy(self, msg: RosImage):
        try:
            if self.bridge is not None and _CV_BRIDGE_OK:
                # type: ignore[attr-defined]
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # type: ignore
                if cv2 is not None:
                    return cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                else:
                    return cv_image[:, :, ::-1].copy()
            else:
                if np is None:
                    return None
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


class RosWorker(QThread):
    """Owns the ROS node and spins it in a background thread."""
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


# ============================ GUI ===================================
class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Panel")
        self.setMinimumSize(780, 540)

        self._quitting = False
        self._estopped = False
        self._lin = 0.0
        self._ang = 0.0

        self._install_sigint_handler()
        self._build_ui()
        self._wire_behaviour()
        self._start_ros()
        self._drive_enabled = os.environ.get("UI_ENABLE_DRIVE", "0") == "1"

    # ---- UI builders
    def rounded_pane(self, w: QWidget, pad=10):
        f = QFrame(); f.setObjectName("pane")
        ly = QVBoxLayout(f); ly.setContentsMargins(pad, pad, pad, pad); ly.addWidget(w)
        return f

    def _build_ui(self):
        # Camera
        self.camera_lbl = QLabel("Camera Feed")
        self.camera_lbl.setAlignment(Qt.AlignCenter)
        self.camera_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.camera_lbl.setMinimumHeight(150)
        cam = self.rounded_pane(self.camera_lbl, pad=14)

        # D-Pad
        dpad_w = QWidget(); g = QGridLayout(dpad_w)
        g.setSpacing(6); g.setContentsMargins(0, 0, 0, 0)
        def btn(t): b = QPushButton(t); b.setFixedSize(48, 36); return b
        self.btn_up, self.btn_left, self.btn_right, self.btn_down = btn("▲"), btn("◀"), btn("▶"), btn("▼")
        g.addWidget(self.btn_up, 0, 1); g.addWidget(self.btn_left, 1, 0)
        g.addWidget(self.btn_right, 1, 2); g.addWidget(self.btn_down, 2, 1)
        dpad = self.rounded_pane(dpad_w, pad=12)

        # Log Panel
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setPlaceholderText("Log Panel")
        logpane = self.rounded_pane(self.log, pad=12); logpane.setMinimumSize(260, 140)

        middle = QHBoxLayout(); middle.addWidget(dpad); middle.addWidget(logpane, 1)

        left = QVBoxLayout(); left.addWidget(cam, 1); left.addLayout(middle, 1)

        # Speed slider 0..20 (0 bottom).  Labels on right.
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

        speed_panel = QVBoxLayout()
        speed_panel.addLayout(speed_row, 1)
        speed_panel.addWidget(QLabel("Speed", alignment=Qt.AlignHCenter))

        # Bottom row
        bottom = QHBoxLayout()
        run = QHBoxLayout()
        run.addWidget(QLabel("Running")); self.led = LedIndicator(); run.addWidget(self.led); run.addStretch(1)
        run_w = QWidget(); run_w.setLayout(run)

        self.estop = QPushButton("E Stop"); self.estop.setObjectName("estop"); self.estop.setMinimumSize(140, 44)
        self.estop.setStyleSheet("""
            QPushButton#estop {
                background-color: #e53935; color: white;
                border: 2px solid #b71c1c; border-radius: 18px;
                padding: 8px 18px; font-weight: 700;
            }
            QPushButton#estop:hover   { background-color: #f44336; }
            QPushButton#estop:pressed { background-color: #c62828; }
        """)
        shadow = QGraphicsDropShadowEffect(blurRadius=16, offset=QPointF(0, 2))
        shadow.setColor(QColor(0, 0, 0, 90)); self.estop.setGraphicsEffect(shadow)

        bottom.addWidget(run_w, 1)
        bottom.addWidget(self.estop, 0, Qt.AlignRight)

        # Root
        root = QVBoxLayout(self)
        top = QHBoxLayout(); top.addLayout(left, 3); top.addLayout(speed_panel, 1)
        root.addLayout(top, 1); root.addLayout(bottom)

        # Style for panes
        self.setStyleSheet("""
            QWidget { font-size: 14px; }
            QFrame#pane { border: 1.6px solid #222; border-radius: 12px; background: #fff; }
        """)

    # ---- Behaviour wiring
    def _wire_behaviour(self):
        # D-pad press/release set desired command (held while pressed)
        self.btn_up.pressed.connect(lambda: self._set_motion(forward=True))
        self.btn_up.released.connect(lambda: self._set_motion(forward=False))
        self.btn_down.pressed.connect(lambda: self._set_motion(back=True))
        self.btn_down.released.connect(lambda: self._set_motion(back=False))
        self.btn_left.pressed.connect(lambda: self._set_motion(left=True))
        self.btn_left.released.connect(lambda: self._set_motion(left=False))
        self.btn_right.pressed.connect(lambda: self._set_motion(right=True))
        self.btn_right.released.connect(lambda: self._set_motion(right=False))

        self.speed.valueChanged.connect(self._update_command_from_speed)
        self.estop.clicked.connect(self._on_estop)

        # Timer to send cmd_vel continuously
        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._publish_cmd)
        self.cmd_timer.start(int(1000 / CMD_PUB_RATE_HZ))

    # ---- ROS worker thread
    def _start_ros(self):
        self.ros = RosWorker()
        self.ros.signals.image.connect(self._update_camera)
        self.ros.signals.log.connect(self._append_log)
        self.ros.signals.running.connect(self._set_running_led)
        self.ros.signals.ok.connect(lambda ok, msg: self._append_log(("OK " if ok else "ERR ") + msg))
        self.send_cmd = self.ros.send_cmd
        self.ros.start()

    # ---- Motion logic
    def _current_speed_fraction(self) -> float:
        return float(self.speed.value()) / float(self.speed.maximum() or 1)

    def _set_motion(self, forward=None, back=None, left=None, right=None):
        f = self._current_speed_fraction()
        if forward is not None:
            self._lin = (MAX_LINEAR_MPS * f) if forward else 0.0 if not back else self._lin
        if back is not None:
            self._lin = (-MAX_LINEAR_MPS * f) if back else 0.0 if not forward else self._lin
        if left is not None:
            self._ang = (MAX_ANGULAR_RPS * f) if left else 0.0 if not right else self._ang
        if right is not None:
            self._ang = (-MAX_ANGULAR_RPS * f) if right else 0.0 if not left else self._ang
        self._update_running_led()

    def _update_command_from_speed(self, _v):
        f = self._current_speed_fraction()
        self._lin = math.copysign(min(abs(self._lin), MAX_LINEAR_MPS * f), self._lin)
        self._ang = math.copysign(min(abs(self._ang), MAX_ANGULAR_RPS * f), self._ang)
        self._update_running_led()

    def _publish_cmd(self):
        if not hasattr(self, "ros") or not self.ros.ready():
            return
        if self._estopped:
            self.send_cmd(0.0, 0.0); return
        if os.environ.get("UI_ENABLE_DRIVE", "0") != "1":
            return
        self.send_cmd(self._lin, self._ang)

    def _on_estop(self):
        self._lin = 0.0; self._ang = 0.0; self._estopped = True
        self._update_running_led()
        self._append_log(">>> EMERGENCY STOP PRESSED! <<<")
        QTimer.singleShot(0, lambda: setattr(self, "_estopped", False))  # remove for latched E-STOP

    # ---- UI updates
    def _update_running_led(self):
        running = (abs(self._lin) > 1e-3 or abs(self._ang) > 1e-3) and not self._estopped
        self.led.set_color("limegreen" if running else "#bbbbbb")

    def _set_running_led(self, running: bool):
        self.led.set_color("limegreen" if running else "#bbbbbb")

    def _append_log(self, text: str):
        if not self._quitting:
            self.log.append(text)

    @Slot(object)
    def _update_camera(self, rgb_np):
        if self._quitting or rgb_np is None:
            return
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

            try: self.ros.signals.image.disconnect(self._update_camera)
            except Exception: pass
            try: self.ros.signals.log.disconnect(self._append_log)
            except Exception: pass
            try: self.ros.signals.running.disconnect(self._set_running_led)
            except Exception: pass
            try: self.ros.signals.ok.disconnect()
            except Exception: pass

            self.send_cmd = lambda *_a, **_k: None
            self.ros.stop()
            self.ros.wait(2000)
        except Exception:
            pass
        super().closeEvent(event)

    # --- SIGINT niceness
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

    # quiet Ctrl-C tracebacks
    def _excepthook(exc_type, exc, tb):
        if exc_type is KeyboardInterrupt:
            try: QApplication.instance().quit()
            except Exception: pass
            return
        sys.__excepthook__(exc_type, exc, tb)
    sys.excepthook = _excepthook

    w = ControlGUI()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
