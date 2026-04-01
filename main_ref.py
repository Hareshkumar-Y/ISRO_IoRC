import sys
import json
import base64
import threading
from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QPixmap
import paho.mqtt.client as mqtt

# ================= MQTT HANDLER ================= #

class MQTTHandler(QObject):
    telemetry_signal = pyqtSignal(dict)
    image_signal = pyqtSignal(bytes)

    def __init__(self, broker="test.mosquitto.org", port=1883):
        super().__init__()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.image_buffer = bytearray()
        self.chunks = {}  # {img_no: {chunk_no: bytes}}
        self.expected_size = 0
        self.connected = False

        try:
            self.client.connect(broker, port, 60)
            threading.Thread(target=self.client.loop_forever, daemon=True).start()
        except Exception as e:
            print(f"[MQTT] Could not connect to broker ({broker}:{port}): {e}")
            print("[MQTT] Running in offline mode.")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            self.connected = True
            print("[MQTT] Connected to broker")
            client.subscribe("/uav/telemetry")
            client.subscribe("/uav/image")
        else:
            print(f"[MQTT] Connection failed, reason code: {reason_code}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic

        if topic == "/uav/telemetry":
            data = json.loads(msg.payload.decode())
            self.telemetry_signal.emit(data)

        elif topic == "/uav/image":
            try:
                data = json.loads(msg.payload.decode())
                img_no = data["img_no"]
                chunk_no = data["chunk_no"]
                total_size = data["total_img_size"]

                if img_no not in self.chunks:
                    self.chunks[img_no] = {}
                self.chunks[img_no][chunk_no] = base64.b64decode(data["payload"])

                # Assemble when all chunks received
                assembled = b"".join(
                    self.chunks[img_no][k]
                    for k in sorted(self.chunks[img_no])
                )
                if len(assembled) >= total_size:
                    self.image_signal.emit(assembled)
                    del self.chunks[img_no]
            except Exception as e:
                print(f"[MQTT] Image chunk error: {e}")

    def publish(self, topic, payload):
        self.client.publish(topic, json.dumps(payload))


# ================= UI ================= #

class DroneGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GCS - Drone Control")
        self.setGeometry(100, 100, 1200, 700)

        self.mqtt = MQTTHandler()
        self.mqtt.telemetry_signal.connect(self.update_telemetry)
        self.mqtt.image_signal.connect(self.show_image)

        self.init_ui()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        layout = QVBoxLayout()

        # ===== MODE SELECTOR ===== #
        mode_layout = QHBoxLayout()
        self.mode_buttons = {}

        for mode in ["MANUAL", "WAYPOINT", "AUTO", "RTH"]:
            btn = QPushButton(mode)
            btn.setCheckable(True)
            btn.clicked.connect(lambda _, m=mode: self.set_mode(m))
            self.mode_buttons[mode] = btn
            mode_layout.addWidget(btn)

        self.mode_buttons["MANUAL"].setChecked(True)
        layout.addLayout(mode_layout)

        # ===== MAIN PANEL ===== #
        center_layout = QHBoxLayout()

        # LEFT CONTROL PAD
        left_panel = QVBoxLayout()

        self.btn_up = QPushButton("↑")
        self.btn_down = QPushButton("↓")
        self.btn_left = QPushButton("←")
        self.btn_right = QPushButton("→")
        self.btn_view = QPushButton("VIEWING")

        self.btn_up.clicked.connect(lambda: self.send_control("UP"))
        self.btn_down.clicked.connect(lambda: self.send_control("DOWN"))
        self.btn_left.clicked.connect(lambda: self.send_control("LEFT"))
        self.btn_right.clicked.connect(lambda: self.send_control("RIGHT"))

        left_panel.addWidget(self.btn_up)
        left_panel.addWidget(self.btn_left)
        left_panel.addWidget(self.btn_right)
        left_panel.addWidget(self.btn_down)
        left_panel.addWidget(self.btn_view)

        # CENTER STATUS
        center_panel = QVBoxLayout()

        self.status_label = QLabel("DRONE STATUS: MANUAL")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.lat_label = QLabel("LAT: 0.0")
        self.lng_label = QLabel("LNG: 0.0")

        center_panel.addWidget(self.status_label)
        center_panel.addWidget(self.lat_label)
        center_panel.addWidget(self.lng_label)

        # RIGHT PANEL
        right_panel = QVBoxLayout()

        self.battery_label = QLabel("Battery: 85%")
        self.signal_label = QLabel("Signal: Strong")

        self.btn_load = QPushButton("LOAD")
        self.btn_save = QPushButton("SAVE")

        right_panel.addWidget(self.battery_label)
        right_panel.addWidget(self.signal_label)
        right_panel.addWidget(self.btn_load)
        right_panel.addWidget(self.btn_save)

        center_layout.addLayout(left_panel)
        center_layout.addLayout(center_panel)
        center_layout.addLayout(right_panel)

        layout.addLayout(center_layout)

        # ===== BOTTOM INFO ===== #
        self.info_label = QLabel("Dist: 0m | Area: 0 ha")
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.info_label)

        # ===== EMERGENCY BUTTON ===== #
        self.stop_button = QPushButton("STOP")
        self.stop_button.clicked.connect(self.emergency_stop)
        layout.addWidget(self.stop_button, alignment=Qt.AlignmentFlag.AlignRight)

        main_widget.setLayout(layout)

        self.apply_styles()

    # ================= FUNCTIONS ================= #

    def set_mode(self, mode):
        for m, btn in self.mode_buttons.items():
            btn.setChecked(m == mode)

        self.status_label.setText(f"DRONE STATUS: {mode}")
        self.mqtt.publish("/uav/control", {"mode": mode})

    def send_control(self, direction):
        self.mqtt.publish("/uav/control", {"move": direction})

    def emergency_stop(self):
        self.mqtt.publish("/uav/control", {"command": "STOP"})

    def update_telemetry(self, data):
        self.lat_label.setText(f"LAT: {data['lat']}")
        self.lng_label.setText(f"LNG: {data['lng']}")

    def show_image(self, image_bytes):
        pixmap = QPixmap()
        pixmap.loadFromData(image_bytes)

        dialog = QDialog(self)
        dialog.setWindowTitle("Drone Camera")

        label = QLabel()
        label.setPixmap(pixmap.scaled(400, 400, Qt.AspectRatioMode.KeepAspectRatio))

        layout = QVBoxLayout()
        layout.addWidget(label)
        dialog.setLayout(layout)

        dialog.exec()

    # ================= STYLES ================= #

    def apply_styles(self):
        self.setStyleSheet("""
        QWidget {
            background-color: #121212;
            color: #00FF88;
            font-size: 14px;
        }

        QPushButton {
            background-color: #1f1f1f;
            border: 2px solid #00FF88;
            border-radius: 10px;
            padding: 8px;
        }

        QPushButton:checked {
            background-color: #00FF88;
            color: black;
        }

        QPushButton:hover {
            background-color: #00cc66;
        }

        QLabel {
            font-size: 16px;
        }

        #stop {
            background-color: red;
        }
        """)


# ================= MAIN ================= #

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DroneGCS()
    window.show()
    sys.exit(app.exec())