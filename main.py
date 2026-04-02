import sys, json, threading, math, csv, time, os

# Binary image frame delimiter (3 bytes) — separates header fields
FRAME_DELIM = b'\xAA\xBB\xCC'
from datetime import datetime
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGridLayout, QFrame, QSizePolicy, QFileDialog, QMessageBox
)
from PyQt6.QtCore import Qt, pyqtSignal, QObject, QTimer, QRectF
from PyQt6.QtGui import (
    QFont, QPixmap, QPainter, QColor, QPen, QBrush, QRadialGradient
)
import paho.mqtt.client as mqtt

# ═══════════════════════════════════════════════════════════
#  DATA LOGGER
# ═══════════════════════════════════════════════════════════

class TelemetryProcessor(QObject):
    """
    Processes incoming telemetry.
    'lat' and 'lng' are treated as local X / Y in metres (no GPS).
    Computes:
      · Cumulative Euclidean distance travelled
      · Bounding-box area of all visited positions
    """
    stats_signal = pyqtSignal(float, float)   # (dist_m, area_m2)

    def __init__(self):
        super().__init__()
        self._prev_x: float | None = None
        self._prev_y: float | None = None
        self._dist_m  = 0.0
        self._all_x: list[float] = []
        self._all_y: list[float] = []

    def process(self, x: float, y: float):
        # Distance
        if self._prev_x is not None:
            dx = x - self._prev_x
            dy = y - self._prev_y
            self._dist_m += math.sqrt(dx * dx + dy * dy)
        self._prev_x, self._prev_y = x, y

        # Bounding-box area
        self._all_x.append(x)
        self._all_y.append(y)
        if len(self._all_x) >= 2:
            w = max(self._all_x) - min(self._all_x)
            h = max(self._all_y) - min(self._all_y)
            area = w * h
        else:
            area = 0.0

        self.stats_signal.emit(self._dist_m, area)

    def reset(self):
        self._prev_x = self._prev_y = None
        self._dist_m = 0.0
        self._all_x.clear()
        self._all_y.clear()


# ═══════════════════════════════════════════════════════════
#  DATA LOGGER
# ═══════════════════════════════════════════════════════════

class DataLogger:
    """Logs every telemetry tick to a timestamped CSV file."""

    FIELDS = ["timestamp", "x", "y", "velocity", "altitude",
              "mode", "battery", "signal"]

    def __init__(self):
        fname = f"gcs_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self._path = os.path.join(os.path.dirname(os.path.abspath(__file__)), fname)
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self._file.flush()
        print(f"[LOG] Logging to {self._path}")

    def log(self, data: dict):
        row = {
            "timestamp": datetime.now().isoformat(),
            "x":         data.get("lat", ""),
            "y":         data.get("lng", ""),
            "velocity":  data.get("velocity", ""),
            "altitude":  data.get("altitude", ""),
            "mode":      data.get("mode", ""),
            "battery":   data.get("battery", ""),
            "signal":    data.get("signal", ""),
        }
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        self._file.close()
        print(f"[LOG] Closed log: {self._path}")


# ═══════════════════════════════════════════════════════════
#  MISSION MANAGER
# ═══════════════════════════════════════════════════════════

class MissionManager(QObject):
    """LOAD / SAVE waypoint missions as JSON files."""

    def __init__(self, mqtt_handler):
        super().__init__()
        self._mqtt = mqtt_handler
        self._waypoints: list[dict] = []

    def save(self, parent_widget):
        path, _ = QFileDialog.getSaveFileName(
            parent_widget, "Save Mission", "mission.json",
            "JSON Files (*.json)"
        )
        if not path:
            return
        mission = {"waypoints": self._waypoints}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(mission, f, indent=2)
        print(f"[MISSION] Saved {len(self._waypoints)} waypoints → {path}")

    def load(self, parent_widget):
        path, _ = QFileDialog.getOpenFileName(
            parent_widget, "Load Mission", "",
            "JSON Files (*.json)"
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._waypoints = data.get("waypoints", [])
            for wp in self._waypoints:
                self._mqtt.publish("/clientUAV/control", {"waypoint": wp})
            print(f"[MISSION] Loaded & sent {len(self._waypoints)} waypoints")
            QMessageBox.information(
                parent_widget, "Mission Loaded",
                f"Sent {len(self._waypoints)} waypoints to drone."
            )
        except Exception as e:
            QMessageBox.critical(parent_widget, "Load Error", str(e))

    def add_waypoint(self, x: float, y: float, alt: float = 0.0):
        self._waypoints.append({"lat": x, "lng": y, "alt": alt})


# ═══════════════════════════════════════════════════════════
#  MQTT HANDLER
# ═══════════════════════════════════════════════════════════

class MQTTHandler(QObject):
    telemetry_signal   = pyqtSignal(dict)
    image_signal       = pyqtSignal(bytes)
    connection_signal  = pyqtSignal(bool)
    failsafe_signal    = pyqtSignal(str)    # emits failsafe message or "NONE"

    BROKER = "test.mosquitto.org"
    PORT   = 1883

    # Individual telemetry topics → dict key
    TELE_TOPICS = {
        "/clientUAV/x_pos":      "lat",
        "/clientUAV/u_pos":      "lng",
        "/clientUAV/velocity": "velocity",
        "/clientUAV/altitude": "altitude",
        "/clientUAV/mode":     "mode",
        "/clientUAV/soc":  "battery",
        "/clientUAV/signal":   "signal",
    }

    def __init__(self):
        super().__init__()
        self.connected   = False
        self._tele_state: dict = {}              # accumulated per-field telemetry
        self._chunks: dict[int, dict[int, bytes]] = {}
        self._chunk_times: dict[int, float] = {} # img_no → first-chunk timestamp
        self._CHUNK_TIMEOUT = 60.0               # seconds before incomplete image discarded

        self._build_client()

        # Chunk-timeout watchdog — fires every 5 s
        self._watchdog = QTimer()
        self._watchdog.timeout.connect(self._expire_chunks)
        self._watchdog.start(5000)

    def _build_client(self):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect    = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message    = self._on_message
        self._try_connect()

    def _try_connect(self):
        def _attempt():
            while not self.connected:
                try:
                    self.client.connect(self.BROKER, self.PORT, 60)
                    self.client.loop_forever()
                except Exception as e:
                    print(f"[MQTT] Reconnect failed: {e}. Retrying in 5 s…")
                    time.sleep(5)
        threading.Thread(target=_attempt, daemon=True).start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            self.connected = True
            print("[MQTT] Connected")
            # Subscribe to each telemetry field topic individually
            for topic in self.TELE_TOPICS:
                client.subscribe(topic, qos=1)
            client.subscribe("/clientUAV/image",    qos=1)
            client.subscribe("/clientUAV/failsafe", qos=1)
            self.connection_signal.emit(True)
        else:
            print(f"[MQTT] Connect refused: {reason_code}")

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected = False
        print(f"[MQTT] Disconnected ({reason_code}). Auto-reconnect active…")
        self.connection_signal.emit(False)

    @staticmethod
    def _decode_frame(raw: bytes):
        """Parse binary image frame: DELIM|img_no|DELIM|chunk_no|DELIM|total|DELIM|payload"""
        parts = raw.split(FRAME_DELIM, 4)
        if len(parts) != 5 or parts[0] != b'':
            raise ValueError(f"Bad frame structure ({len(parts)} parts)")
        img_no     = int.from_bytes(parts[1], 'big')
        chunk_no   = int.from_bytes(parts[2], 'big')
        total_size = int.from_bytes(parts[3], 'big')
        payload    = parts[4]
        return img_no, chunk_no, total_size, payload

    def _on_message(self, client, userdata, msg):
        topic = msg.topic

        # ── Per-field telemetry ──────────────────────────
        if topic in self.TELE_TOPICS:
            try:
                key = self.TELE_TOPICS[topic]
                self._tele_state[key] = msg.payload.decode().strip()
                self.telemetry_signal.emit(dict(self._tele_state))
            except RuntimeError:
                pass
            except Exception as e:
                print(f"[MQTT] Telemetry parse error ({topic}): {e}")

        # ── Failsafe ──────────────────────────────────────
        elif topic == "/clientUAV/failsafe":
            try:
                self.failsafe_signal.emit(msg.payload.decode().strip())
            except RuntimeError:
                pass
            except Exception as e:
                print(f"[MQTT] Failsafe parse error: {e}")

        # ── Binary image frame ───────────────────────────
        elif topic == "/clientUAV/image":
            try:
                img_no, chunk_no, total, payload = self._decode_frame(msg.payload)

                if img_no not in self._chunks:
                    self._chunks[img_no]      = {}
                    self._chunk_times[img_no] = time.monotonic()

                self._chunks[img_no][chunk_no] = payload

                assembled = b"".join(
                    self._chunks[img_no][k] for k in sorted(self._chunks[img_no])
                )
                if len(assembled) >= total:
                    self.image_signal.emit(assembled)
                    del self._chunks[img_no]
                    del self._chunk_times[img_no]
                    print(f"[MQTT] Image {img_no} assembled ({total} bytes)")
            except Exception as e:
                print(f"[MQTT] Image frame error: {e}")

    def _expire_chunks(self):
        now     = time.monotonic()
        expired = [k for k, t in self._chunk_times.items()
                   if now - t > self._CHUNK_TIMEOUT]
        for k in expired:
            print(f"[MQTT] Image {k} timed out — discarding incomplete chunks")
            self._chunks.pop(k, None)
            self._chunk_times.pop(k, None)

    def publish(self, topic: str, payload: dict):
        if self.connected:
            self.client.publish(topic, json.dumps(payload), qos=1)
        else:
            print(f"[MQTT] Offline — cannot publish to {topic}")


# ═══════════════════════════════════════════════════════════
#  CUSTOM WIDGETS
# ═══════════════════════════════════════════════════════════

class GlassPanel(QFrame):
    def __init__(self, parent=None, accent="#00D4FF"):
        super().__init__(parent)
        self.accent = accent
        self.setObjectName("GlassPanel")

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        p.setBrush(QBrush(QColor(14, 20, 35, 210)))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(rect, 14, 14)
        pen = QPen(QColor(self.accent))
        pen.setWidth(1)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(rect, 14, 14)
        p.end()


class DPadWidget(QWidget):
    directionPressed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(180, 180)
        layout = QGridLayout(self)
        layout.setSpacing(4)
        layout.setContentsMargins(20, 20, 20, 20)
        for label, row, col, cmd in [
            ("▲", 0, 1, "UP"), ("◀", 1, 0, "LEFT"),
            ("▶", 1, 2, "RIGHT"), ("▼", 2, 1, "DOWN"),
        ]:
            btn = QPushButton(label)
            btn.setFixedSize(44, 44)
            btn.setObjectName("DPadBtn")
            btn.clicked.connect(lambda _, c=cmd: self.directionPressed.emit(c))
            layout.addWidget(btn, row, col, Qt.AlignmentFlag.AlignCenter)
        center = QLabel("●")
        center.setAlignment(Qt.AlignmentFlag.AlignCenter)
        center.setStyleSheet("color: #1a2a40; font-size: 20px;")
        layout.addWidget(center, 1, 1, Qt.AlignmentFlag.AlignCenter)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = self.rect().center()
        r = min(self.width(), self.height()) // 2 - 4
        glow = QRadialGradient(c.x(), c.y(), r)
        glow.setColorAt(0.7, QColor(0, 212, 255, 30))
        glow.setColorAt(1.0, QColor(0, 212, 255, 0))
        p.setBrush(QBrush(glow))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(c, r, r)
        pen = QPen(QColor("#00D4FF"))
        pen.setWidth(2)
        p.setPen(pen)
        p.setBrush(QBrush(QColor(10, 18, 30, 200)))
        p.drawEllipse(c, r - 1, r - 1)
        p.end()


class BatteryWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(90, 36)
        self._level = 100

    def setLevel(self, v: int):
        self._level = max(0, min(100, v))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h, x, y = self.width() - 10, self.height() - 8, 4, 4
        color = QColor("#00FF88") if self._level > 30 else QColor("#FF4444")
        pen = QPen(QColor("#405070")); pen.setWidth(2)
        p.setPen(pen)
        p.setBrush(QBrush(QColor(14, 20, 35)))
        p.drawRoundedRect(x, y, w, h, 4, 4)
        nub_h = 10
        p.setBrush(QBrush(QColor("#405070")))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(x + w, y + (h - nub_h) // 2, 5, nub_h, 2, 2)
        fill_w = int((w - 4) * self._level / 100)
        p.setBrush(QBrush(color))
        p.drawRoundedRect(x + 2, y + 2, fill_w, h - 4, 3, 3)
        p.end()


class SignalWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(50, 32)
        self._bars = 5

    def setBars(self, n: int):
        self._bars = max(0, min(5, n))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        base_y = self.height() - 4
        for i in range(5):
            bh = 4 + i * 5
            bx = 2 + i * 10
            by = base_y - bh
            color = QColor("#00D4FF") if i < self._bars else QColor("#1e2e40")
            p.setBrush(QBrush(color))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRoundedRect(bx, by, 7, bh, 2, 2)
        p.end()


class StatusDot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(14, 14)
        self._on       = False
        self._green    = False
        self._timer    = QTimer(self)
        self._timer.timeout.connect(self._blink)
        self._timer.start(600)

    def setConnected(self, connected: bool):
        self._green = connected
        self._on    = connected
        self.update()

    def _blink(self):
        if not self._green:
            self._on = not self._on
            self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = (QColor("#00FF88") if self._green
                 else (QColor("#FF6644") if self._on else QColor("#3a1510")))
        p.setBrush(QBrush(color))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(2, 2, 10, 10)
        p.end()


# ═══════════════════════════════════════════════════════════
#  MAIN WINDOW
# ═══════════════════════════════════════════════════════════

class DroneGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ISRO IoRC 2026 — Ground Control Station")
        self.setMinimumSize(1280, 720)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self._drag_pos = None

        # ── Backend ────────────────────────────────────────
        self.mqtt    = MQTTHandler()
        self.tele_p  = TelemetryProcessor()
        self.logger  = DataLogger()
        self._last_pixmap: QPixmap | None = None   # for SAVE button

        # ── Signals ────────────────────────────────────────
        self.mqtt.telemetry_signal.connect(self._on_telemetry)
        self.mqtt.image_signal.connect(self._on_image)
        self.mqtt.connection_signal.connect(self._on_connection)
        self.mqtt.failsafe_signal.connect(self._on_failsafe)
        self.tele_p.stats_signal.connect(self._on_stats)

        self._current_mode = "MANUAL"
        self._x   = 0.0
        self._y   = 0.0

        self.apply_global_styles()
        self.init_ui()

        # IST clock
        self._clk = QTimer(self)
        self._clk.timeout.connect(self._tick_clock)
        self._clk.start(1000)
        self._tick_clock()

    # ── Window drag ────────────────────────────────────────
    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self._drag_pos = e.globalPosition().toPoint()

    def mouseMoveEvent(self, e):
        if self._drag_pos and e.buttons() == Qt.MouseButton.LeftButton:
            self.move(self.pos() + e.globalPosition().toPoint() - self._drag_pos)
            self._drag_pos = e.globalPosition().toPoint()

    def mouseReleaseEvent(self, e):
        self._drag_pos = None

    def closeEvent(self, e):
        self.logger.close()
        super().closeEvent(e)

    # ── UI Build ───────────────────────────────────────────
    def init_ui(self):
        root = QWidget(); root.setObjectName("Root")
        self.setCentralWidget(root)
        rl = QVBoxLayout(root)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(0)
        rl.addWidget(self._build_titlebar())
        rl.addWidget(self._build_modebar())
        rl.addWidget(self._build_failsafe_banner())   # hidden until triggered
        body = QHBoxLayout()
        body.setContentsMargins(12, 8, 12, 8)
        body.setSpacing(10)
        body.addWidget(self._build_left_panel(),  0)
        body.addWidget(self._build_center_panel(), 1)
        body.addWidget(self._build_right_panel(),  0)
        rl.addLayout(body)
        rl.addWidget(self._build_statusbar())

    def _build_titlebar(self):
        bar = QWidget(); bar.setFixedHeight(46); bar.setObjectName("TitleBar")
        h = QHBoxLayout(bar); h.setContentsMargins(16, 0, 10, 0)
        logo = QLabel("◈"); logo.setObjectName("Logo")
        title = QLabel("GCS  <span style='color:#405070'>|</span>  ISRO IoRC 2026")
        title.setObjectName("TitleLabel"); title.setTextFormat(Qt.TextFormat.RichText)
        self._conn_dot   = StatusDot()
        self._conn_label = QLabel("CONNECTING…"); self._conn_label.setObjectName("ConnLabel")
        h.addWidget(logo); h.addSpacing(8); h.addWidget(title); h.addStretch()
        h.addWidget(self._conn_dot); h.addSpacing(6); h.addWidget(self._conn_label)
        h.addSpacing(16)
        for sym, slot in [("─", self.showMinimized), ("✕", self.close)]:
            btn = QPushButton(sym); btn.setObjectName("WinBtn")
            btn.setFixedSize(30, 30); btn.clicked.connect(slot); h.addWidget(btn)
        return bar

    def _build_modebar(self):
        bar = QWidget(); bar.setObjectName("ModeBar"); bar.setFixedHeight(52)
        h = QHBoxLayout(bar); h.setContentsMargins(20, 6, 20, 6); h.setSpacing(8)
        self._mode_btns = {}
        for mode, icon in [("MANUAL","🕹"), ("WAYPOINT","📍"), ("AUTO","🤖")]:
            btn = QPushButton(f"{icon}  {mode}")
            btn.setCheckable(True); btn.setObjectName("ModeBtn")
            btn.clicked.connect(lambda _, m=mode: self.set_mode(m))
            self._mode_btns[mode] = btn; h.addWidget(btn)
        self._mode_btns["MANUAL"].setChecked(True)
        h.addStretch()
        return bar

    def _build_failsafe_banner(self):
        """Full-width warning bar — hidden when all clear, blinking when failsafe active."""
        self._fs_bar = QWidget()
        self._fs_bar.setObjectName("FailsafeBar")
        self._fs_bar.setFixedHeight(44)
        self._fs_bar.hide()

        h = QHBoxLayout(self._fs_bar)
        h.setContentsMargins(20, 0, 20, 0)
        h.setSpacing(12)

        icon = QLabel("⚠")
        icon.setObjectName("FailsafeIcon")

        self._fs_text = QLabel("FAILSAFE ACTIVE")
        self._fs_text.setObjectName("FailsafeText")

        self._fs_time = QLabel("")
        self._fs_time.setObjectName("FailsafeTime")

        h.addWidget(icon)
        h.addWidget(self._fs_text)
        h.addStretch()
        h.addWidget(self._fs_time)

        # Blink timer
        self._fs_blink_on = True
        self._fs_blink_timer = QTimer(self)
        self._fs_blink_timer.timeout.connect(self._blink_failsafe)

        return self._fs_bar

    def _blink_failsafe(self):
        self._fs_blink_on = not self._fs_blink_on
        color = "#4A0000" if self._fs_blink_on else "#2A0000"
        self._fs_bar.setStyleSheet(
            f"#FailsafeBar {{ background-color: {color}; "
            f"border-bottom: 2px solid #FF3322; }}"
        )

    def _build_left_panel(self):
        panel = GlassPanel(accent="#00D4FF"); panel.setFixedWidth(220)
        v = QVBoxLayout(panel); v.setContentsMargins(16,16,16,16); v.setSpacing(12)
        lbl = QLabel("CONTROL PAD"); lbl.setObjectName("PanelTitle")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter); v.addWidget(lbl)
        self._dpad = DPadWidget()
        self._dpad.directionPressed.connect(self.send_control)
        v.addWidget(self._dpad, 0, Qt.AlignmentFlag.AlignCenter)
        v.addStretch()
        return panel

    def _build_center_panel(self):
        panel = GlassPanel(accent="#00FF88")
        v = QVBoxLayout(panel); v.setContentsMargins(24,20,24,20); v.setSpacing(10)

        # Status row
        sr = QHBoxLayout()
        dot = QLabel("●"); dot.setObjectName("StatusDot")
        self._status_lbl = QLabel("DRONE STATUS:  MANUAL")
        self._status_lbl.setObjectName("StatusLabel")
        sr.addStretch(); sr.addWidget(dot); sr.addSpacing(8)
        sr.addWidget(self._status_lbl); sr.addStretch()
        v.addLayout(sr)
        v.addWidget(self._separator())

        # Telemetry grid
        tg = QGridLayout(); tg.setSpacing(16)
        def cell(tag, init):
            t = QLabel(tag); t.setObjectName("TeleTag"); t.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val = QLabel(init); val.setObjectName("TeleVal"); val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            w = QWidget(); w.setObjectName("TeleCell")
            cv = QVBoxLayout(w); cv.setSpacing(4); cv.addWidget(t); cv.addWidget(val)
            return w, val
        c1, self._x_val   = cell("X (m)",    "---")
        c2, self._y_val   = cell("Y (m)",    "---")
        c3, self._vel_val = cell("VELOCITY", "---")
        c4, self._alt_val = cell("ALTITUDE", "---")
        for w, r, c in [(c1,0,0),(c2,0,1),(c3,1,0),(c4,1,1)]:
            tg.addWidget(w, r, c)
        v.addLayout(tg)
        v.addWidget(self._separator())

        # Image viewer
        img_title = QLabel("IMAGE VIEWER"); img_title.setObjectName("PanelTitle")
        img_title.setAlignment(Qt.AlignmentFlag.AlignLeft); v.addWidget(img_title)
        self._img_area = QLabel()
        self._img_area.setObjectName("CamArea")
        self._img_area.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._img_area.setText(
            "<span style='color:#1a3050;font-size:40px'>🖼</span><br>"
            "<span style='color:#1e3555;font-size:13px'>AWAITING IMAGE DATA</span>"
        )
        self._img_area.setTextFormat(Qt.TextFormat.RichText)
        self._img_area.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._img_area.setMinimumHeight(200)
        v.addWidget(self._img_area)
        v.addStretch()
        return panel

    def _build_right_panel(self):
        panel = GlassPanel(accent="#FFB300"); panel.setFixedWidth(200)
        v = QVBoxLayout(panel); v.setContentsMargins(16,16,16,16); v.setSpacing(14)

        v.addWidget(self._section_title("BATTERY"))
        br = QHBoxLayout()
        self._bat_widget = BatteryWidget()
        self._bat_label  = QLabel("--- %"); self._bat_label.setObjectName("StatVal")
        br.addWidget(self._bat_widget); br.addWidget(self._bat_label); br.addStretch()
        v.addLayout(br)
        v.addWidget(self._separator())

        v.addWidget(self._section_title("RC SIGNAL"))
        sgr = QHBoxLayout()
        self._sig_widget = SignalWidget()
        self._sig_label  = QLabel("---"); self._sig_label.setObjectName("StatVal")
        sgr.addWidget(self._sig_widget); sgr.addWidget(self._sig_label); sgr.addStretch()
        v.addLayout(sgr)
        v.addWidget(self._separator())

        v.addWidget(self._section_title("IMAGE"))
        self._btn_load = QPushButton("📂  LOAD IMG"); self._btn_load.setObjectName("ActionBtn")
        self._btn_save = QPushButton("💾  SAVE IMG"); self._btn_save.setObjectName("ActionBtn")
        self._btn_load.clicked.connect(self._load_image_file)
        self._btn_save.clicked.connect(self._save_image_file)
        v.addWidget(self._btn_load); v.addWidget(self._btn_save)
        v.addStretch()

        v.addWidget(self._separator())
        self._startstop_btn = QPushButton("▶   START")
        self._startstop_btn.setObjectName("StartBtn")
        self._startstop_btn.clicked.connect(self.toggle_mission)
        self._startstop_btn.setFixedHeight(52)
        v.addWidget(self._startstop_btn)
        return panel

    def _build_statusbar(self):
        bar = QWidget(); bar.setFixedHeight(36); bar.setObjectName("StatusBar")
        h = QHBoxLayout(bar); h.setContentsMargins(20,0,20,0); h.setSpacing(0)
        self._dist_lbl   = QLabel("DIST  0.0 m");   self._dist_lbl.setObjectName("BarItem")
        self._area_lbl   = QLabel("AREA  0.0 m²");  self._area_lbl.setObjectName("BarItem")
        self._mission_lbl = QLabel("MISSION ACTIVE"); self._mission_lbl.setObjectName("MissionLabel")
        self._time_lbl   = QLabel("--:-- IST");      self._time_lbl.setObjectName("BarItem")
        h.addWidget(self._dist_lbl); h.addSpacing(30)
        h.addWidget(self._area_lbl); h.addStretch()
        h.addWidget(self._mission_lbl); h.addStretch()
        h.addWidget(self._time_lbl)
        return bar

    # ── Helpers ────────────────────────────────────────────
    def _separator(self):
        ln = QFrame(); ln.setFrameShape(QFrame.Shape.HLine); ln.setObjectName("Sep")
        return ln

    def _section_title(self, t):
        lbl = QLabel(t); lbl.setObjectName("SectionTitle"); return lbl

    # ── Backend Slots ──────────────────────────────────────
    def _on_connection(self, connected: bool):
        self._conn_dot.setConnected(connected)
        if connected:
            self._conn_label.setText("CONNECTED")
            self._conn_label.setStyleSheet("color: #00FF88;")
        else:
            self._conn_label.setText("OFFLINE")
            self._conn_label.setStyleSheet("color: #FF6644;")

    def _on_telemetry(self, data: dict):
        x   = float(data.get("lat", self._x))
        y   = float(data.get("lng", self._y))
        vel = data.get("velocity", None)
        alt = data.get("altitude", None)
        bat = data.get("battery", None)
        sig = data.get("signal", None)
        mode = data.get("mode", None)

        self._x, self._y = x, y
        self._x_val.setText(f"{x:.3f}")
        self._y_val.setText(f"{y:.3f}")
        if vel is not None: self._vel_val.setText(f"{float(vel):.2f} m/s")
        if alt is not None: self._alt_val.setText(f"{float(alt):.1f} m")
        if bat is not None:
            b = int(bat)
            self._bat_widget.setLevel(b)
            self._bat_label.setText(f"{b} %")
        if sig is not None:
            s = int(sig)
            self._sig_widget.setBars(s)
            labels = {0:"NONE",1:"WEAK",2:"POOR",3:"FAIR",4:"GOOD",5:"STRONG"}
            self._sig_label.setText(labels.get(s, str(s)))
        if mode:
            self._apply_mode(mode)   # UI only — no re-publish

        # Feed telemetry processor & logger
        self.tele_p.process(x, y)
        self.logger.log(data)

    def _on_failsafe(self, msg: str):
        if msg.upper() == "NONE" or msg == "":
            self._fs_bar.hide()
            self._fs_blink_timer.stop()
        else:
            ts = datetime.now().strftime("%H:%M:%S")
            self._fs_text.setText(f"FAILSAFE — {msg}")
            self._fs_time.setText(ts)
            self._fs_bar.show()
            if not self._fs_blink_timer.isActive():
                self._fs_blink_timer.start(400)
            print(f"[FAILSAFE] {msg} @ {ts}")

    def _on_stats(self, dist_m: float, area_m2: float):
        self._dist_lbl.setText(f"DIST  {dist_m:.1f} m")
        self._area_lbl.setText(f"AREA  {area_m2:.1f} m²")

    def _on_image(self, image_bytes: bytes):
        px = QPixmap()
        px.loadFromData(image_bytes)
        self._last_pixmap = px
        self._img_area.setPixmap(
            px.scaled(
                self._img_area.width(), self._img_area.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
        )

    def _load_image_file(self):
        """Open a local image file and display it in the image viewer."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Image", "",
            "Images (*.png *.jpg *.jpeg *.bmp *.gif)"
        )
        if not path:
            return
        px = QPixmap(path)
        if px.isNull():
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "Load Error", f"Could not load image:\n{path}")
            return
        self._last_pixmap = px
        self._img_area.setPixmap(
            px.scaled(
                self._img_area.width(), self._img_area.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
        )
        print(f"[IMG] Loaded: {path}")

    def _save_image_file(self):
        """Save the currently displayed image to a file."""
        if self._last_pixmap is None or self._last_pixmap.isNull():
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.information(self, "Nothing to Save", "No image is currently displayed.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Image", "drone_image.png",
            "PNG Image (*.png);;JPEG Image (*.jpg)"
        )
        if not path:
            return
        if self._last_pixmap.save(path):
            print(f"[IMG] Saved: {path}")
        else:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "Save Error", f"Could not save to:\n{path}")

    def _tick_clock(self):
        self._time_lbl.setText(datetime.now().strftime("%H:%M:%S IST"))

    # ── Controls ───────────────────────────────────────────
    def _apply_mode(self, mode: str):
        """Update mode UI only — does NOT publish to MQTT (used for incoming telemetry)."""
        self._current_mode = mode
        for m, btn in self._mode_btns.items():
            btn.setChecked(m == mode)
        self._status_lbl.setText(f"DRONE STATUS:  {mode}")
        self._status_lbl.setStyleSheet("")

    def set_mode(self, mode: str):
        """User clicked a mode button — update UI AND publish command to drone."""
        self._apply_mode(mode)
        self.mqtt.publish("/clientUAV/control", {"mode": mode})

    def send_control(self, direction: str):
        self.mqtt.publish("/clientUAV/control", {"move": direction})

    def toggle_mission(self):
        if not hasattr(self, '_running'):
            self._running = False
        self._running = not self._running
        if self._running:
            self.mqtt.publish("/clientUAV/control", {"command": "START"})
            self._startstop_btn.setText("⏹   STOP")
            self._startstop_btn.setObjectName("StopBtn")
            self._status_lbl.setText(f"DRONE STATUS:  {self._current_mode}")
            self._status_lbl.setStyleSheet("")
        else:
            self.mqtt.publish("/clientUAV/control", {"command": "STOP"})
            self._startstop_btn.setText("▶   START")
            self._startstop_btn.setObjectName("StartBtn")
            self._status_lbl.setText("⏹  STOPPED")
            self._status_lbl.setStyleSheet("color: #FF4444;")
        # Force stylesheet refresh after objectName change
        self._startstop_btn.style().unpolish(self._startstop_btn)
        self._startstop_btn.style().polish(self._startstop_btn)

    # ── Stylesheet ─────────────────────────────────────────
    def apply_global_styles(self):
        self.setStyleSheet("""
#Root { background-color: #080D18; }
#TitleBar { background-color: #0D1525; border-bottom: 1px solid #1a2a40; }
#Logo { color: #00D4FF; font-size: 22px; }
#TitleLabel { color: #C0D0E0; font-size: 14px; font-weight: bold; letter-spacing: 1px; }
#ConnLabel  { font-size: 11px; color: #FFB300; }
#WinBtn { background: transparent; border: none; color: #405070; font-size: 14px; border-radius: 6px; }
#WinBtn:hover { background: #1a2a40; color: #E0E8F0; }
#ModeBar { background-color: #0A1220; border-bottom: 1px solid #152035; }
#ModeBtn {
    background-color: #0D1828; color: #5080A0;
    border: 1px solid #1e3050; border-radius: 8px;
    padding: 6px 20px; font-size: 12px; font-weight: bold;
    letter-spacing: 1px; min-width: 110px;
}
#ModeBtn:hover  { background-color: #132030; color: #80B0D0; border-color: #00D4FF; }
#ModeBtn:checked {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #003D55,stop:1 #005570);
    color: #00D4FF; border: 1px solid #00D4FF;
}
#GlassPanel { border-radius: 14px; }
#PanelTitle   { color: #3060A0; font-size: 10px; font-weight: bold; letter-spacing: 2px; }
#SectionTitle { color: #304060; font-size: 9px;  font-weight: bold; letter-spacing: 2px; }
#DPadBtn { background-color: #0D1828; color: #00D4FF; border: 1px solid #1a3050; border-radius: 8px; font-size: 16px; font-weight: bold; }
#DPadBtn:hover    { background-color: #003D55; border-color: #00D4FF; }
#DPadBtn:pressed  { background-color: #00688A; }
#ViewBtn { background-color: #0D1828; color: #3060A0; border: 1px solid #1a2840; border-radius: 8px; padding: 8px; font-size: 11px; font-weight: bold; }
#ViewBtn:checked { background-color: #0A2035; color: #00D4FF; border-color: #00D4FF; }
#ViewBtn:hover   { border-color: #00D4FF; color: #00D4FF; }
#StatusDot   { color: #00FF88; font-size: 12px; }
#StatusLabel { color: #00FF88; font-size: 18px; font-weight: bold; letter-spacing: 2px; }
#TeleCell { background-color: #0A1525; border: 1px solid #152030; border-radius: 10px; padding: 6px; }
#TeleTag  { color: #304560; font-size: 9px; font-weight: bold; letter-spacing: 2px; }
#TeleVal  { color: #00D4FF; font-size: 20px; font-weight: bold; }
#CamArea  { background-color: #080E1A; border: 1px solid #101e30; border-radius: 10px; }
#StatVal  { color: #C0D8F0; font-size: 12px; font-weight: bold; }
#ActionBtn { background-color: #0D1828; color: #3070A0; border: 1px solid #1a3050; border-radius: 8px; padding: 8px; font-size: 11px; font-weight: bold; }
#ActionBtn:hover { background-color: #0A2035; color: #00D4FF; border-color: #00D4FF; }
#StartBtn {
    background: qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #00451a,stop:1 #002610);
    color: #00FF88; border: 2px solid #00CC66; border-radius: 10px;
    font-size: 14px; font-weight: bold; letter-spacing: 2px;
}
#StartBtn:hover   { background: qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #006628,stop:1 #003815); color: #44FFaa; }
#StartBtn:pressed { background-color: #008833; }
#StopBtn {
    background: qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #6B0000,stop:1 #3D0000);
    color: #FF4444; border: 2px solid #FF2222; border-radius: 10px;
    font-size: 14px; font-weight: bold; letter-spacing: 2px;
}
#StopBtn:hover   { background: qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #880000,stop:1 #550000); color: #FF6666; }
#StopBtn:pressed { background-color: #AA0000; }
#Sep { color: #121e30; background: #121e30; max-height: 1px; border: none; }
#StatusBar { background-color: #0A1020; border-top: 1px solid #121e30; }
#BarItem      { color: #304560; font-size: 11px; font-weight: bold; letter-spacing: 1px; }
#MissionLabel { color: #00FF88; font-size: 10px; font-weight: bold; letter-spacing: 3px; }
#FailsafeBar  { background-color: #3D0000; border-bottom: 2px solid #FF3322; }
#FailsafeIcon { color: #FF4444; font-size: 22px; }
#FailsafeText { color: #FF4444; font-size: 13px; font-weight: bold; letter-spacing: 2px; }
#FailsafeTime { color: #FF8866; font-size: 11px; }
        """)


# ═══════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 10))
    window = DroneGCS()
    window.show()
    sys.exit(app.exec())