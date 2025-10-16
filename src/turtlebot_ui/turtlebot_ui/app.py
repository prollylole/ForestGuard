
"""
PySide6 + ROS 2 GUI for TurtleBot control.

- Publishes /cmd_vel (Twist) at 20 Hz while D-pad buttons are held.
- Subscribes to /camera/image_raw (Image) -> shows in camera panel.
- Subscribes to /rosout (Log) -> streams to Log panel.
- Runs ROS in a QThread (MultiThreadedExecutor) so Qt stays responsive.
- Includes red rounded E-STOP. Running LED reflects motion state.

Notes:
- Fixes Pylance "Variable not allowed in type expression" by NOT using 'CvBridge'
  in type annotations; we store the bridge as Optional[object] and gate usage with
  a runtime flag.
"""

import sys
import math
from typing import Optional

from PySide6.QtCore import Qt, QObject, QThread, QTimer, Signal, Slot, QPointF
from PySide6.QtGui import QColor, QPainter, QBrush, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QSlider, QFrame,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSizePolicy, QGraphicsDropShadowEffect
)

# --------------------------- CONFIG ---------------------------------
CAMERA_TOPIC = "/camera/image_raw"   # sensor_msgs/msg/Image
ROSOUT_TOPIC = "/rosout"             # rcl_interfaces/msg/Log
CMD_VEL_TOPIC = "/cmd_vel"

CMD_PUB_RATE_HZ = 20                 # publish rate for cmd_vel while moving

MAX_LINEAR_MPS = 0.40                # tune for your bot
MAX_ANGULAR_RPS = 1.50               # rad/s
# --------------------------------------------------------------------

# ---- Optional deps for image conversion (works with or without cv_bridge)
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
    CvBridge = None        # runtime fallback symbol so code keeps working
    _CV_BRIDGE_OK = False

# ---- ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as RosImage
from rcl_interfaces.msg import Log as RosLog


# ========================= Qt widgets ===============================

class LedIndicator(QLabel):
    def __init__(self, color=QColor("limegreen"), diameter=14, parent=None):
        super().__init__(parent)
        self._color = QColor(color)
        self._diameter = diameter
        self.setFixedSize(diameter, diameter)

    def set_color(self, color: QColor | str):
        self._color = QColor(color)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QBrush(self._color))
        p.setPen(Qt.black)
        p.drawEllipse(0, 0, self._diameter, self._diameter)


# ========================= ROS <-> Qt bridge ========================

class RosSignals(QObject):
    image = Signal(object)           # numpy RGB array (H,W,3) or None
    log = Signal(str)                # rosout text
    running = Signal(bool)           # running state (for LED)
    ok = Signal(bool, str)           # status, message (init errors etc.)


class GuiRosNode(Node):
    """ROS2 node performing pubs/subs and emitting Qt signals."""
    def __init__(self, signals: RosSignals):
        super().__init__("turtlebot_gui")
        self.signals = signals

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # Subscribers
        self.create_subscription(RosLog, ROSOUT_TOPIC, self._on_rosout, 50)

        # cv_bridge handle (typed as object to avoid Pylance issue)
        self.bridge: Optional[object] = CvBridge() if _CV_BRIDGE_OK else None

        try:
            self.create_subscription(RosImage, CAMERA_TOPIC, self._on_image, 10)
        except Exception as e:
            self.get_logger().warn(f"Camera subscription failed: {e}")

        self.signals.ok.emit(True, "ROS node initialised")

    # --- Call from Qt thread via queued connection
    @Slot(float, float)
    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # --- Sub callbacks
    def _on_rosout(self, msg: RosLog):
        level_map = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        level = level_map.get(msg.level, str(msg.level))
        self.signals.log.emit(f"[{level}] {msg.name}: {msg.msg}")

    def _on_image(self, msg: RosImage):
        rgb = self._image_to_rgb_numpy(msg)
        if rgb is not None:
            self.signals.image.emit(rgb)

    # Helper: convert sensor_msgs/Image to RGB numpy (H, W, 3)
    def _image_to_rgb_numpy(self, msg: RosImage):
        try:
            if self.bridge is not None and _CV_BRIDGE_OK:
                # type: ignore[attr-defined] - runtime-checked
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # type: ignore
                if cv2 is not None:
                    rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                else:
                    rgb = cv_image[:, :, ::-1].copy()
                return rgb
            else:
                if np is None:
                    return None
                if msg.encoding.lower() in ("rgb8", "bgr8"):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        (msg.height, msg.width, 3)
                    )
                    if msg.encoding.lower() == "bgr8":
                        return arr[:, :, ::-1].copy()
                    return arr
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

    def run(self):
        try:
            rclpy.init(args=None)
            self._node = GuiRosNode(self.signals)
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            while not self._stop:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.signals.ok.emit(False, f"ROS error: {e}")
        finally:
            try:
                if self._executor and self._node:
                    self._executor.remove_node(self._node)
                if self._node:
                    self._node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def stop(self):
        self._stop = True

    # public slot used by the GUI thread
    @Slot(float, float)
    def send_cmd(self, linear: float, angular: float):
        if self._node is not None:
            self._node.publish_cmd(linear, angular)


# ============================ GUI ===================================

class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Panel")
        self.setMinimumSize(780, 540)

        self._estopped = False
        self._lin = 0.0
        self._ang = 0.0

        self._build_ui()
        self._wire_behaviour()
        self._start_ros()

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

        # Timer to send cmd_vel continuously (common for mobile bases)
        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._publish_cmd)
        self.cmd_timer.start(int(1000 / CMD_PUB_RATE_HZ))

    # ---- ROS worker thread
    def _start_ros(self):
        self.ros = RosWorker()
        # Incoming signals
        self.ros.signals.image.connect(self._update_camera)
        self.ros.signals.log.connect(self._append_log)
        self.ros.signals.running.connect(self._set_running_led)
        self.ros.signals.ok.connect(lambda ok, msg: self._append_log(("OK " if ok else "ERR ") + msg))
        # Outgoing commands to ROS thread
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
        if self._estopped:
            self.send_cmd(0.0, 0.0)
            return
        self.send_cmd(self._lin, self._ang)

    def _on_estop(self):
        self._lin = 0.0; self._ang = 0.0; self._estopped = True
        self._update_running_led()
        self._append_log(">>> EMERGENCY STOP PRESSED! <<<")
        # If you want latched E-Stop, remove the next line
        QTimer.singleShot(0, lambda: setattr(self, "_estopped", False))

    # ---- UI updates
    def _update_running_led(self):
        running = (abs(self._lin) > 1e-3 or abs(self._ang) > 1e-3) and not self._estopped
        self.led.set_color("limegreen" if running else "#bbbbbb")

    def _set_running_led(self, running: bool):
        self.led.set_color("limegreen" if running else "#bbbbbb")

    def _append_log(self, text: str):
        self.log.append(text)

    @Slot(object)
    def _update_camera(self, rgb_np):
        if rgb_np is None:
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
            self.ros.stop()
            self.ros.wait(2000)
        except Exception:
            pass
        super().closeEvent(event)


# ============================== main ================================

def main():
    import sys
    from PySide6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    w = ControlGUI()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

