from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QSplitter, QPlainTextEdit, QFrame, QProgressBar, QSizePolicy
)
from PySide6.QtCore import Qt, Signal, QObject
import sys

# ----- Fire-Risk segmented control -----
class FireRiskMeter(QWidget):
    changed = Signal(str)  # "Moderate" | "High" | "Extreme" | "Catastrophic"

    def __init__(self):
        super().__init__()
        self.setObjectName("FireRiskMeter")

        self.wrap = QFrame(objectName="riskWrap")
        row = QHBoxLayout(self.wrap)
        row.setContentsMargins(10,10,10,10)
        row.setSpacing(0)

        # Order + colors: Moderate(green), High(yellow), Extreme(orange), Catastrophic(red)
        self.levels = [
            ("Moderate", "green"),
            ("High", "yellow"),
            ("Extreme", "orange"),
            ("Catastrophic", "red"),
        ]

        self.btns = []
        for name, key in self.levels:
            b = QPushButton(name)
            b.setCheckable(True)
            b.setObjectName(f"risk_{key}")
            b.clicked.connect(lambda _, n=name: self.changed.emit(n))
            self.btns.append(b)
            row.addWidget(b)

        # default
        self.btns[0].setChecked(True)

        outer = QHBoxLayout(self)
        outer.setContentsMargins(0,0,0,0)
        outer.addWidget(self.wrap)

        # make the whole meter longer
        self.setMinimumWidth(540)  # << longer bar

        # segmented styles
        self.setStyleSheet("""
        #riskWrap { border:2px solid #2b2b2b; border-radius:16px; background:#fff; }
        #riskWrap QPushButton {
            border:0; padding:10px 24px; font-size:14px; min-width:120px;
        }
        #riskWrap QPushButton + QPushButton { border-left:1px solid #cfcfcf; }

        /* Checked colors per level */
        #riskWrap QPushButton:checked#risk_green  { background:#e6f5e6; }
        #riskWrap QPushButton:checked#risk_yellow { background:#fff7d1; }
        #riskWrap QPushButton:checked#risk_orange { background:#ffe5cc; }
        #riskWrap QPushButton:checked#risk_red    { background:#ffe0e0; }
        """)

    def setValue(self, name: str):
        names = [n for n, _ in self.levels]
        if name in names:
            self.btns[names.index(name)].setChecked(True)


# ----- App signals (for later ROS2 wiring) -----
class UiSignals(QObject):
    set_mode = Signal(str)
    fire_risk = Signal(str)
    battery = Signal(int)
    trees = Signal(int)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ForestGuard")
        self.sig = UiSignals()
        self._build_ui()
        self._wire()

        # initial state
        self.sig.set_mode.emit("Running")
        self.sig.fire_risk.emit("Moderate")  # default selection
        self.sig.battery.emit(63)
        self.sig.trees.emit(7)

    def _build_ui(self):
        root = QWidget(); self.setCentralWidget(root)

        # ======= TOP (two columns) =======
        topRow = QHBoxLayout()
        topRow.setContentsMargins(12,10,12,0)
        topRow.setSpacing(12)

        # -- left content card: camera / map / log --
        contentCard = QFrame(objectName="contentCard")
        contentCard.setStyleSheet("#contentCard{border:2px solid #2b2b2b; border-radius:18px; background:#fff;}")
        contentCard.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        cv = QVBoxLayout(contentCard)
        cv.setContentsMargins(12,12,12,12)
        cv.setSpacing(10)

        self.camera = QLabel("Camera"); self.camera.setAlignment(Qt.AlignCenter)
        self.map    = QLabel("Map");    self.map.setAlignment(Qt.AlignCenter)
        for w in (self.camera, self.map):
            w.setStyleSheet("border:2px solid #2b2b2b; background:#fbf7fb; font-size:18px;")
            w.setMinimumHeight(220)

        split = QSplitter(Qt.Vertical)
        split.setHandleWidth(6)
        split.setChildrenCollapsible(False)
        split.addWidget(self.camera)
        split.addWidget(self.map)
        split.setSizes([460, 310])
        cv.addWidget(split, 1)

        self.log = QPlainTextEdit()
        self.log.setPlaceholderText("Log Panel…")
        self.log.setFixedHeight(92)
        self.log.setStyleSheet("border:2px solid #2b2b2b; background:#ffffff;")
        cv.addWidget(self.log, 0)

        topRow.addWidget(contentCard, 7)

        # -- right control panel: centered, bigger buttons, more spacing --
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
        pv.setSpacing(24)  # << more space between buttons

        def mk(text, cls):
            b = QPushButton(text, self.panel)
            b.setProperty("class", cls)
            b.setMinimumHeight(54); b.setMaximumHeight(60)  # chunkier
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            return b

        self.b_home  = mk("Return / Home", "home")
        self.b_wp    = mk("Set way point", "wp")
        self.b_hover = mk("Stay / Hover", "hover")
        self.b_stop  = mk("E stop / Safety\nMode", "estop")
        self.b_multi = mk("Multi Button", "multi")
        self.b_power = mk("Off / On", "power")

        pv.addStretch(1)  # center the stack
        for b in (self.b_home, self.b_wp, self.b_hover, self.b_stop, self.b_multi, self.b_power):
            pv.addWidget(b)
        pv.addStretch(1)

        topRow.addWidget(self.panel, 3)

        # ======= FOOTER =======
        footer = QFrame(objectName="footerBar")
        footer.setStyleSheet("#footerBar{border:2px solid #2b2b2b; border-radius:16px; background:#fff;}")
        fl = QHBoxLayout(footer)
        fl.setContentsMargins(14,8,14,8)
        fl.setSpacing(14)

        # Status left
        self.mode_lbl = QLabel("Running")
        self.dot = QLabel(); self.dot.setFixedSize(14,14)
        self._set_dot("#9bef6d")
        status = QHBoxLayout()
        status.addWidget(QLabel("Status:")); status.addWidget(self.mode_lbl); status.addWidget(self.dot)
        statusW = QWidget(); statusW.setLayout(status)

        # Risk meter center (longer)
        self.risk = FireRiskMeter()

        # Trees + Battery right
        right = QHBoxLayout()
        self.trees_lbl = QLabel("Trees Scanned : 0")
        self.batt = QProgressBar(); self.batt.setRange(0,100); self.batt.setValue(0)
        self.batt.setFixedWidth(110); self.batt.setFormat("%p%")
        self.batt.setStyleSheet("QProgressBar{border:2px solid #2b2b2b; border-radius:12px; background:#fff;}")
        right.addWidget(self.trees_lbl); right.addSpacing(12); right.addWidget(self.batt)
        rightW = QWidget(); rightW.setLayout(right)

        # Put everything together vertically
        rootLayout = QVBoxLayout(root)
        rootLayout.setContentsMargins(0,0,0,0)
        rootLayout.setSpacing(10)
        rootLayout.addLayout(topRow, 1)

        footerRow = QHBoxLayout()
        footerRow.setContentsMargins(12,0,12,10)
        footerRow.addWidget(statusW, 0, Qt.AlignLeft)
        footerRow.addWidget(self.risk, 0, Qt.AlignHCenter)
        footerRow.addWidget(rightW, 0, Qt.AlignRight)

        footerContainer = QWidget()
        fcLay = QHBoxLayout(footerContainer)
        fcLay.setContentsMargins(0,0,0,0)
        fcLay.addWidget(footer)

        rootLayout.addLayout(footerRow, 0)

        self.resize(1240, 760)

    def _wire(self):
        # temporary logs; you’ll replace with ROS2 actions later
        self.b_home.clicked.connect(lambda: self._log("Return/Home clicked"))
        self.b_wp.clicked.connect(lambda: self._log("Set waypoint clicked"))
        self.b_hover.clicked.connect(lambda: self._log("Hover clicked"))
        self.b_stop.clicked.connect(lambda: self._log("E-stop clicked"))
        self.b_multi.clicked.connect(lambda: self._log("Multi clicked"))
        self.b_power.clicked.connect(lambda: self._log("Power toggled"))

        self.sig.set_mode.connect(self._apply_mode)
        self.sig.fire_risk.connect(self._apply_risk)
        self.sig.battery.connect(self.batt.setValue)
        self.sig.trees.connect(lambda n: self.trees_lbl.setText(f"Trees Scanned : {n}"))

    # --- helpers ---
    def _log(self, msg):
        self.log.appendPlainText(msg)

    def _set_dot(self, color):
        self.dot.setStyleSheet(f"background:{color}; border:1px solid #2b2b2b; border-radius:7px;")

    def _apply_mode(self, mode):
        self.mode_lbl.setText(mode)
        colors = {"Running":"#9bef6d","Hovering":"#9bef6d","Scanning":"#ffd966","Error":"#ff6464"}
        self._set_dot(colors.get(mode, "#cfcfcf"))
        self._log(f"Mode -> {mode}")

    def _apply_risk(self, risk):
        # Ensure the correct button is checked
        self.risk.setValue(risk)
        self._log(f"Fire risk -> {risk}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow(); w.show()
    sys.exit(app.exec())
