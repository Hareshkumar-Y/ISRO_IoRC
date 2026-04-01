"""
sim.py — Pseudo drone simulator for ISRO IoRC GCS
──────────────────────────────────────────────────
Simulates a drone by publishing MQTT messages to the same broker
as main.py. Run this in a second terminal alongside the GCS app.

Usage:
    python sim.py                          # telemetry only
    python sim.py --image path/to/img.jpg  # telemetry + image
    python sim.py --broker localhost        # custom broker
"""

import argparse, base64, json, math, sys, time, struct, zlib, os
import paho.mqtt.client as mqtt

# ── Config ──────────────────────────────────────────────────
TELEMETRY_TOPIC = "/uav/telemetry"
IMAGE_TOPIC     = "/uav/image"
CHUNK_SIZE      = 180          # bytes per image chunk (≤200 per spec)
TELE_INTERVAL   = 1.0          # seconds between telemetry ticks

# ── MQTT setup ──────────────────────────────────────────────
def make_client(broker: str, port: int) -> mqtt.Client:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = lambda c, u, f, rc, p: print(
        f"[SIM] Connected to {broker}:{port}" if rc == 0
        else f"[SIM] Connect failed: {rc}"
    )
    client.connect(broker, port, 60)
    client.loop_start()
    return client

# ── Telemetry loop ──────────────────────────────────────────
def run_telemetry(client: mqtt.Client, duration: float | None = None):
    """
    Publishes telemetry every TELE_INTERVAL seconds.
    Simulates a drone flying in a square pattern (XY in metres).
    battery drains 0.5 % per tick; signal oscillates 3-5.
    """
    t         = 0.0
    battery   = 100.0
    modes     = ["MANUAL", "WAYPOINT", "AUTO", "MANUAL"]
    mode_idx  = 0
    side      = 50.0          # 50 m square sides
    speed     = 5.0           # m/s

    print("[SIM] Starting telemetry stream… (Ctrl+C to stop)")
    start = time.monotonic()

    while True:
        # XY position: trace a square
        cycle     = (speed * t) % (4 * side)
        if   cycle < side:       x, y = cycle,            0.0
        elif cycle < 2 * side:   x, y = side,             cycle - side
        elif cycle < 3 * side:   x, y = side - (cycle - 2*side), side
        else:                    x, y = 0.0,               side - (cycle - 3*side)

        battery = max(0.0, battery - 0.5)
        signal  = 3 + int(2 * abs(math.sin(t * 0.3)))    # 3-5
        if int(t) % 30 == 0:                              # mode change every 30 s
            mode_idx = (mode_idx + 1) % len(modes)

        payload = {
            "lat":      round(x, 3),
            "lng":      round(y, 3),
            "velocity": round(speed, 2),
            "altitude": round(15.0 + 5.0 * math.sin(t * 0.2), 1),
            "mode":     modes[mode_idx],
            "battery":  int(battery),
            "signal":   min(5, signal),
        }
        client.publish(TELEMETRY_TOPIC, json.dumps(payload), qos=1)
        print(f"[SIM] tele → x={x:.1f} y={y:.1f} bat={int(battery)}% sig={signal} mode={modes[mode_idx]}")

        t += TELE_INTERVAL
        if duration and (time.monotonic() - start) >= duration:
            break
        time.sleep(TELE_INTERVAL)

# ── Test image generator (stdlib only) ─────────────────────
def generate_test_png(width: int = 120, height: int = 90) -> bytes:
    """
    Creates a valid PNG in-memory using only struct + zlib.
    Draws a cyan/green gradient with a crosshair — no Pillow needed.
    """
    def png_chunk(name: bytes, data: bytes) -> bytes:
        raw   = name + data
        crc   = zlib.crc32(raw) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + raw + struct.pack(">I", crc)

    sig  = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)

    cx, cy = width // 2, height // 2
    raw = b""
    for y in range(height):
        raw += b"\x00"   # filter byte
        for x in range(width):
            # Gradient background
            r = int(8  + 12  * x / width)
            g = int(13 + 180 * y / height)
            b = int(24 + 100 * x / width)
            # Crosshair lines
            if abs(x - cx) <= 1 or abs(y - cy) <= 1:
                r, g, b = 0, 212, 255
            # Corner markers
            if (x < 8 or x > width - 9) and (y < 8 or y > height - 9):
                r, g, b = 0, 255, 136
            raw += bytes([r, g, b])

    idat = zlib.compress(raw, 6)
    return (sig
            + png_chunk(b"IHDR", ihdr)
            + png_chunk(b"IDAT", idat)
            + png_chunk(b"IEND", b""))


def resolve_image(path: str | None) -> bytes:
    """Return image bytes from path, or auto-generate a test PNG."""
    if path:
        if not os.path.isfile(path):
            print(f"[SIM] WARNING: '{path}' not found — generating test PNG instead")
        else:
            with open(path, "rb") as f:
                return f.read()
    print("[SIM] Generating built-in test PNG (120×90)…")
    return generate_test_png()


# ── Image sender ─────────────────────────────────────────────
def send_image(client: mqtt.Client, image_bytes: bytes, img_no: int = 1):
    """
    Splits image_bytes into CHUNK_SIZE-byte chunks and
    publishes them one by one to /uav/image.
    """
    total  = len(image_bytes)
    chunks = [image_bytes[i:i+CHUNK_SIZE] for i in range(0, total, CHUNK_SIZE)]
    print(f"[SIM] Sending image — {total} bytes / {len(chunks)} chunks")

    for chunk_no, chunk in enumerate(chunks):
        payload = {
            "img_no":         img_no,
            "chunk_no":       chunk_no,
            "total_img_size": total,
            "payload":        base64.b64encode(chunk).decode(),
        }
        client.publish(IMAGE_TOPIC, json.dumps(payload), qos=1)
        print(f"[SIM] img chunk {chunk_no+1}/{len(chunks)}")
        time.sleep(0.01)

    print("[SIM] Image sending complete.")

# ── Entry point ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="ISRO IoRC Drone Simulator")
    parser.add_argument("--broker", default="test.mosquitto.org", help="MQTT broker hostname")
    parser.add_argument("--port",   default=1883, type=int,       help="MQTT broker port")
    parser.add_argument("--image",  default=None,                 help="Path to image file to send")
    parser.add_argument("--img-delay", default=5.0, type=float,   help="Seconds before sending image (default 5)")
    args = parser.parse_args()

    client = make_client(args.broker, args.port)
    time.sleep(1.5)   # wait for connection

    if args.image is not None or True:   # always send an image in sim mode
        img_bytes = resolve_image(args.image)
        import threading
        def _delayed_image():
            print(f"[SIM] Waiting {args.img_delay}s before sending image…")
            time.sleep(args.img_delay)
            send_image(client, img_bytes)
        threading.Thread(target=_delayed_image, daemon=True).start()

    try:
        run_telemetry(client)
    except KeyboardInterrupt:
        print("\n[SIM] Stopped.")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
