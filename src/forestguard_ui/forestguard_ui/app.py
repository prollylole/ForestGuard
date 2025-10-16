# src/forestguard_ui/forestguard_ui/app.py
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QSplitter, QPlainTextEdit, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt, Signal, QObject, QTimer
from PySide6.QtGui import QImage, QPixmap
import sys, numpy as np

# Toggle this to disable ROS during debugging
DEBUG_NO_ROS = False  # change to False after the UI stays open

try:
    from .ros_side import RosSide
except Exception as e:
    RosSide = None
    _ros_err = e

class FireRiskMeter(QWidget):
    changed = Signal(str)
    def __init__(self):
        super().__init__()
        self.setObjectName("FireRiskMeter")
        self.wrap = QFrame(objectName="riskWrap")
        row = QHBoxLayout(self.wrap)
        row.setContentsMargins(10,10,10,10)
        row.setSpacing(0)
        self.levels = [("Moderate","green"),("High","yellow"),
                       ("Extreme","orange"),("Catastrophic","red")]
        self.btns = []
        for name, key in self.levels:
            b = QPushButton(name)
            b.setCheckable(True)
            b.setObjectName(f"risk_{key}")
            b.clicked.connect(lambda _, n=name: self.changed.emit(n))
            self.btns.append(b)
            row.addWidget(b)
        self.btns[0].setChecked(True)
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0,0,0,0)
        outer.addWidget(self.wrap)
        self.setMinimumWidth(540)
        self.setStyleSheet("""
        #riskWrap { border:2px solid #2b2b2b; border-radius:16px; background:#fff; }
        #riskWrap QPushButton { border:0; padding:10px 24px; font-size:14px; min-width:120px; }
        #riskWrap QPushButton + QPushButton { border-left:1px solid #cfcfcf; }
        #riskWrap QPushButton:checked#risk_green  { background:#e6f5e6; }
        #riskWrap QPushButton:checked#risk_yellow { background:#fff7d1; }
        #riskWrap QPushButton:checked#risk_orange { background:#ffe5cc; }
        #riskWrap QPushButton:checked#risk_red    { background:#ffe0e0; }
        """)

class UiSignals(QObject):
    set_mode = Signal(str)
    fire_risk = Signal(str)
    battery = Signal(int)
    trees = Signal(int)

class MainWindow(QMainWindow):
    def __init__(self, camera_topic="/camera/image"):
        super().__init__()
        self.setWindowTitle("ForestGuard")
        self.sig = UiSignals()
        self._build_ui()
        self._wire()

        # --- ROS side ---
        self.ros = None
        if not DEBUG_NO_ROS:
            try:
                if RosSide is None:
                    self._log(f"[UI] ROS not available: {_ros_err}")
                else:
                    self.ros = RosSide(camera_topic=camera_topic)
                    self.ros.sig_frame.connect(self._display_frame)
                    self._spin = QTimer(self)
                    def _safe_spin():
                        try:
                            if self.ros is not None:
                                self.ros.spin_once(1)
                        except Exception as e:
                            self._log(f"[UI] spin_once error: {e}")
                    self._spin.timeout.connect(_safe_spin)
                    self._spin.start(10)
                    self._log("[UI] RosSide started.")
            except Exception as e:
                self._log(f"[UI] RosSide init failed: {e}")
                self.ros = None
        else:
            self._log("[UI] DEBUG_NO_ROS=True -> skipping ROS wiring")

        # initial state
        self.sig.set_mode.emit("Running")
        self.sig.fire_risk.emit("Moderate")
        self.sig.battery.emit(63)
        self.sig.trees.emit(7)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        topRow = QHBoxLayout()
        topRow.setContentsMargins(12,10,12,0)
        topRow.setSpacing(12)

        contentCard = QFrame(objectName="contentCard")
        contentCard.setStyleSheet("#contentCard{border:2px solid #2b2b2b; border-radius:18px; background:#fff;}")
        contentCard.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        cv = QVBoxLayout(contentCard)
        cv.setContentsMargins(12,12,12,12)
        cv.setSpacing(10)

        self.camera = QLabel("Camera")
        self.camera.setAlignment(Qt.AlignCenter)
        self.map = QLabel("Map")
        self.map.setAlignment(Qt.AlignCenter)
        for w in (self.camera, self.map):
            w.setStyleSheet("border:2px solid #2b2b2b; background:#fbf7fb; font-size:18px;")
            w.setMinimumHeight(220)

        split = QSplitter(Qt.Vertical)
        split.setHandleWidth(6)
        split.setChildrenCollapsible(False)
        split.addWidget(self.camera)
        split.addWidget(self.map)
        split.setSizes([460,310])
        cv.addWidget(split, 1)

        self.log = QPlainTextEdit()
        self.log.setPlaceholderText("Log Panel…")
        self.log.setFixedHeight(92)
        self.log.setStyleSheet("border:2px solid #2b2b2b; background:#ffffff;")
        cv.addWidget(self.log, 0)
        topRow.addWidget(contentCard, 7)

        self.panel = QFrame(self.centralWidget())
        self.panel.setObjectName("controlPanel")
        self.panel.setMaximumWidth(340)
        self.panel.setStyleSheet("""
        #controlPanel{ border:2px solid #2b2b2b; border-radius:22px; background:#ffffff; }
        QPushButton { border:1px solid #b7b7b7; border-radius:14px; padding:10px 16px; font-size:15px; }
        QPushButton[class="home"]  { background:#ffe4a3; }
        QPushButton[class="wp"]    { background:#f7e6a1; }
        QPushButton[class="hover"] { background:#cfe4ff; }
        QPushButton[class="estop"] { background:#f7c8c8; }
        QPushButton[class="multi"] { background:#e6d1f2; }
        QPushButton[class="power"] { background:#cfead1; }
        """)
        pv = QVBoxLayout(self.panel)
        pv.setContentsMargins(22,24,22,24)
        pv.setSpacing(24)
        def mk(text, cls):
            b = QPushButton(text, self.panel)
            b.setProperty("class", cls)
            b.setMinimumHeight(54)
            b.setMaximumHeight(60)
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            return b
        self.b_home=mk("Return / Home","home")
        self.b_wp=mk("Set way point","wp")
        self.b_hover=mk("Stay / Hover","hover")
        self.b_stop=mk("E stop / Safety\nMode","estop")
        self.b_multi=mk("Multi Button","multi")
        self.b_power=mk("Off / On","power")
        pv.addStretch(1)
        for b in (self.b_home,self.b_wp,self.b_hover,self.b_stop,self.b_multi,self.b_power):
            pv.addWidget(b)
        pv.addStretch(1)
        topRow.addWidget(self.panel, 3)

        rootLayout = QVBoxLayout(root)
        rootLayout.setContentsMargins(0,0,0,0)
        rootLayout.setSpacing(10)
        rootLayout.addLayout(topRow, 1)
        self.resize(1240,760)

    def _wire(self):
        self.b_home.clicked.connect(lambda: self._log("Return/Home clicked"))
        self.b_wp.clicked.connect(lambda: self._log("Set waypoint clicked"))
        self.b_hover.clicked.connect(lambda: self._log("Hover clicked"))
        self.b_stop.clicked.connect(lambda: self._log("E-stop clicked"))
        self.b_multi.clicked.connect(lambda: self._log("Multi clicked"))
        self.b_power.clicked.connect(lambda: self._log("Power toggled"))
        self.sig.set_mode.connect(self._apply_mode)
        self.sig.fire_risk.connect(self._apply_risk)
        self.sig.battery.connect(lambda v: None)
        self.sig.trees.connect(lambda n: self._log(f"Trees Scanned : {n}"))

    def _display_frame(self, bgr: np.ndarray):
        if bgr is None or bgr.size == 0: return
        rgb = bgr[..., ::-1]
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch*w, QImage.Format.Format_RGB888)
        self.camera.setPixmap(QPixmap.fromImage(qimg))

    def _log(self, msg):
        print(msg)
        self.log.appendPlainText(msg)

    def _apply_mode(self, mode): self._log(f"Mode -> {mode}")
    def _apply_risk(self, risk): self._log(f"Fire risk -> {risk}")

def main():
    import threading, time
    # ✅ properly declare app first
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    # Step 2 tracing
    app.aboutToQuit.connect(lambda: print("Qt aboutToQuit() fired"))
    app.lastWindowClosed.connect(lambda: print("Qt lastWindowClosed() fired"))

    w = MainWindow()
    w.show()
    print("ForestGuard UI launched successfully!")

    # heartbeat
    def heartbeat():
        while True:
            print("Qt event loop alive…")
            time.sleep(2)
    threading.Thread(target=heartbeat, daemon=True).start()

    print("Before exec()")
    ret = app.exec()
    print(f"After exec() ret={ret}")

if __name__ == "__main__":
    if not getattr(sys, "_forestguard_started", False):
        sys._forestguard_started = True
        print("Launching ForestGuard UI…")
        main()
    else:
        print("ForestGuard UI already running — skipping second start.")
