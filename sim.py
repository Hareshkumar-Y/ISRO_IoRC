"""
sim.py — Pseudo drone simulator for ISRO IoRC GCS
──────────────────────────────────────────────────
Behaviour:
  · Starts PAUSED — press START on the GCS to begin telemetry
  · Mode is set by the GCS mode buttons (MANUAL / WAYPOINT / AUTO)
  · Responds to {"command":"START"} / {"command":"STOP"} on /uav/control
  · Responds to {"mode":"X"} on /uav/control — updates published mode

Usage:
    python sim.py                          # uses built-in test PNG
    python sim.py --image path/to/img.jpg  # sends a real image
    python sim.py --broker localhost
"""

import argparse, json, math, os, struct, time, zlib, threading
import paho.mqtt.client as mqtt

# ── Frame / topic config ─────────────────────────────────────
FRAME_DELIM  = b'\xAA\xBB\xCC'
CHUNK_SIZE   = 180          # payload bytes ≤ 180 → total frame ≤ 200 B
TELE_INTERVAL = 1.0

TELE_TOPICS = {
    "lat":      "/uav/lat",
    "lng":      "/uav/lng",
    "velocity": "/uav/velocity",
    "altitude": "/uav/altitude",
    "mode":     "/uav/mode",
    "battery":  "/uav/battery",
    "signal":   "/uav/signal",
}
IMAGE_TOPIC    = "/uav/image"
CONTROL_TOPIC  = "/uav/control"
FAILSAFE_TOPIC = "/uav/failsafe"


# ── Frame helpers ────────────────────────────────────────────

def _varint(n: int) -> bytes:
    for size in range(1, 5):
        try:
            return n.to_bytes(size, 'big')
        except OverflowError:
            continue
    raise ValueError(f"Integer too large: {n}")


def make_frame(img_no: int, chunk_no: int, total: int, payload: bytes) -> bytes:
    return (FRAME_DELIM + _varint(img_no)
            + FRAME_DELIM + _varint(chunk_no)
            + FRAME_DELIM + _varint(total)
            + FRAME_DELIM + payload)


# ── Test PNG generator ───────────────────────────────────────

def generate_test_png(width: int = 120, height: int = 90) -> bytes:
    def png_chunk(name: bytes, data: bytes) -> bytes:
        raw = name + data
        return struct.pack(">I", len(data)) + raw + struct.pack(">I", zlib.crc32(raw) & 0xFFFFFFFF)
    sig  = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    cx, cy = width // 2, height // 2
    raw = b""
    for y in range(height):
        raw += b"\x00"
        for x in range(width):
            r = int(8 + 12 * x / width)
            g = int(13 + 180 * y / height)
            b = int(24 + 100 * x / width)
            if abs(x - cx) <= 1 or abs(y - cy) <= 1:
                r, g, b = 0, 212, 255
            if (x < 8 or x > width - 9) and (y < 8 or y > height - 9):
                r, g, b = 0, 255, 136
            raw += bytes([r, g, b])
    return (sig + png_chunk(b"IHDR", ihdr)
            + png_chunk(b"IDAT", zlib.compress(raw, 6))
            + png_chunk(b"IEND", b""))


def resolve_image(path: str | None) -> bytes:
    if path and os.path.isfile(path):
        with open(path, "rb") as f:
            return f.read()
    if path:
        print(f"[SIM] WARNING: '{path}' not found — using generated PNG")
    else:
        print("[SIM] No image specified — using built-in test PNG (120×90)")
    return generate_test_png()


# ── Simulator state ──────────────────────────────────────────

class DroneSimulator:
    """
    Holds simulation state. Controlled externally by GCS via /uav/control.
    Starts in STOPPED state — GCS must send START.
    D-pad moves the drone 2 m per press (MANUAL mode only).
    """

    STEP = 2.0    # metres per D-pad press

    def __init__(self):
        self._running = False
        self._mode    = "MANUAL"
        self._dx      = 0.0      # accumulated D-pad X offset
        self._dy      = 0.0      # accumulated D-pad Y offset
        self._lock    = threading.Lock()

    def handle_control(self, payload: bytes):
        try:
            data = json.loads(payload.decode())
        except Exception:
            return

        cmd  = data.get("command", "")
        mode = data.get("mode",    "")
        move = data.get("move",    "")

        with self._lock:
            if cmd == "START":
                self._running = True
                print("[SIM] ▶ START received — telemetry running")
            elif cmd == "STOP":
                self._running = False
                print("[SIM] ⏹ STOP received — telemetry paused")

            if mode and mode in ("MANUAL", "WAYPOINT", "AUTO"):
                self._mode = mode
                print(f"[SIM] Mode → {self._mode}")

            if move:
                if self._mode == "MANUAL":
                    if   move == "UP":    self._dy -= self.STEP
                    elif move == "DOWN":  self._dy += self.STEP
                    elif move == "LEFT":  self._dx -= self.STEP
                    elif move == "RIGHT": self._dx += self.STEP
                    print(f"[SIM] D-pad {move} → offset ({self._dx:.1f}, {self._dy:.1f}) m")
                else:
                    print(f"[SIM] D-pad ignored — not in MANUAL mode ({self._mode})")

    def consume_offset(self) -> tuple[float, float]:
        """Return and reset the pending D-pad position offset."""
        with self._lock:
            dx, dy = self._dx, self._dy
            self._dx = self._dy = 0.0
            return dx, dy

    @property
    def running(self):
        with self._lock:
            return self._running

    @property
    def mode(self):
        with self._lock:
            return self._mode


# ── MQTT client ──────────────────────────────────────────────

def make_client(broker: str, port: int, sim: DroneSimulator) -> mqtt.Client:
    def on_connect(client, userdata, flags, rc, props):
        if rc == 0:
            print(f"[SIM] Connected to {broker}:{port}")
            client.subscribe(CONTROL_TOPIC, qos=1)
            print(f"[SIM] Subscribed to {CONTROL_TOPIC}")
        else:
            print(f"[SIM] Connect failed: {rc}")

    def on_message(client, userdata, msg):
        if msg.topic == CONTROL_TOPIC:
            sim.handle_control(msg.payload)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    client.loop_start()
    return client


# ── Telemetry loop ───────────────────────────────────────────

def run_telemetry(client: mqtt.Client, sim: DroneSimulator):
    """
    Publishes telemetry each second when running.
    Waits (polls every 100 ms) when stopped.
    XY traces a 50×50 m square. Battery drains only while running.
    """
    t        = 0.0
    battery  = 100.0
    side     = 50.0
    speed    = 5.0

    print("[SIM] Waiting for START from GCS…")

    while True:
        if not sim.running:
            time.sleep(0.1)
            continue

        # XY square trace + manual D-pad offset
        cycle = (speed * t) % (4 * side)
        if   cycle < side:       x, y = cycle,                   0.0
        elif cycle < 2 * side:   x, y = side,                    cycle - side
        elif cycle < 3 * side:   x, y = side - (cycle - 2*side), side
        else:                    x, y = 0.0,                      side - (cycle - 3*side)

        dx, dy = sim.consume_offset()   # apply D-pad nudge
        x = max(0.0, x + dx)
        y = max(0.0, y + dy)

        battery = max(0.0, battery - 0.3)
        signal  = 3 + int(2 * abs(math.sin(t * 0.3)))
        alt     = round(15.0 + 5.0 * math.sin(t * 0.2), 1)

        # ── Failsafe logic ─────────────────────────
        if battery < 20.0:
            failsafe = "LOW_BATTERY"
        elif 30 <= t < 38:           # simulated altitude breach at t=30s
            failsafe = "ALTITUDE_BREACH"
        elif 75 <= t < 80:           # simulated comms dropout at t=75s
            failsafe = "COMM_TIMEOUT"
        else:
            failsafe = "NONE"

        fields = {
            "lat":      str(round(x, 3)),
            "lng":      str(round(y, 3)),
            "velocity": str(speed),
            "altitude": str(alt),
            "mode":     sim.mode,
            "battery":  str(int(battery)),
            "signal":   str(min(5, signal)),
        }

        for key, topic in TELE_TOPICS.items():
            client.publish(topic, fields[key], qos=1)
        client.publish(FAILSAFE_TOPIC, failsafe, qos=1)

        fs_disp = f"  ⚠ {failsafe}" if failsafe != "NONE" else ""
        print(f"[SIM]  x={x:.1f}  y={y:.1f}  bat={int(battery)}%  "
              f"sig={signal}  alt={alt}m  mode={sim.mode}{fs_disp}")

        t += TELE_INTERVAL
        time.sleep(TELE_INTERVAL)


# ── Image sender ─────────────────────────────────────────────

def send_image(client: mqtt.Client, image_bytes: bytes, img_no: int = 1):
    total  = len(image_bytes)
    chunks = [image_bytes[i:i+CHUNK_SIZE] for i in range(0, total, CHUNK_SIZE)]
    print(f"[SIM] Sending image — {total} B / {len(chunks)} chunks")
    for chunk_no, chunk in enumerate(chunks):
        frame = make_frame(img_no, chunk_no, total, chunk)
        client.publish(IMAGE_TOPIC, frame, qos=1)
        print(f"[SIM] img chunk {chunk_no+1}/{len(chunks)} ({len(frame)} B)")
        time.sleep(0.01)
    print("[SIM] Image send complete.")


# ── Entry point ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ISRO IoRC Drone Simulator")
    parser.add_argument("--broker",    default="test.mosquitto.org")
    parser.add_argument("--port",      default=1883, type=int)
    parser.add_argument("--image",     default=None)
    parser.add_argument("--img-delay", default=3.0, type=float,
                        help="Seconds after START before sending image (default 3)")
    args = parser.parse_args()

    sim    = DroneSimulator()
    client = make_client(args.broker, args.port, sim)
    time.sleep(1.5)

    img_bytes = resolve_image(args.image)

    # Send image 3 s after drone starts (so GCS image viewer is already open)
    def _img_thread():
        while not sim.running:
            time.sleep(0.2)
        print(f"[SIM] Drone started — image in {args.img_delay}s…")
        time.sleep(args.img_delay)
        send_image(client, img_bytes)

    threading.Thread(target=_img_thread, daemon=True).start()

    try:
        run_telemetry(client, sim)
    except KeyboardInterrupt:
        print("\n[SIM] Stopped.")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
