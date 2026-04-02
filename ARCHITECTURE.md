# 🛰 GCS Architecture & Data Flow
### ISRO IoRC 2026 — Ground Control Station

---

## 1. System Overview

```
┌─────────────────────┐         MQTT          ┌──────────────────────┐
│   Ground Control    │◄──────────────────────►│   Drone / Simulator  │
│   Station (GCS)     │   test.mosquitto.org   │      (sim.py)        │
│    (main.py)        │       port 1883        │                      │
└─────────────────────┘                        └──────────────────────┘
```

Both sides connect to the **same public MQTT broker** (`test.mosquitto.org:1883`) over the internet.  
There is no direct link — all communication is brokered.

---

## 2. Components

### 2.1 `main.py` — Ground Control Station

| Class | Role |
|---|---|
| `MQTTHandler` | Connects to broker, subscribes to telemetry + image topics, publishes control commands |
| `TelemetryProcessor` | Computes cumulative distance and bounding-box area from XY positions |
| `DataLogger` | Writes every telemetry tick to a timestamped CSV file |
| `DroneGCS` (UI) | PyQt6 window — renders all data, forwards user input to `MQTTHandler` |
| Custom Widgets | `DPadWidget`, `BatteryWidget`, `SignalWidget`, `StatusDot`, `GlassPanel` |

### 2.2 `sim.py` — Drone Simulator

| Class/Function | Role |
|---|---|
| `DroneSimulator` | Holds drone state: running flag, mode, D-pad offset |
| `run_telemetry()` | Publishes each telemetry field every 1 second (only when running) |
| `send_image()` | Splits an image into binary frames and publishes them |
| `generate_test_png()` | Creates a 120×90 PNG using only stdlib (no Pillow) |

---

## 3. MQTT Topic Map

### 3.1 Telemetry — Drone → GCS

Each field has its **own dedicated topic**. Payload is a plain UTF-8 string (no JSON).

| Topic | Payload example | Description |
|---|---|---|
| `/uav/lat` | `"12.500"` | Local X position (metres) |
| `/uav/lng` | `"30.000"` | Local Y position (metres) |
| `/uav/velocity` | `"5.0"` | Speed in m/s |
| `/uav/altitude` | `"17.3"` | Height in metres |
| `/uav/mode` | `"AUTO"` | Current flight mode |
| `/uav/battery` | `"84"` | Battery percentage (0–100) |
| `/uav/signal` | `"4"` | RC signal bars (0–5) |

> QoS = 1 on all subscriptions (at-least-once delivery)

### 3.2 Image — Drone → GCS

| Topic | Payload |
|---|---|
| `/uav/image` | Binary frame (see §4) |

### 3.3 Control — GCS → Drone

| Topic | Payload (JSON) | Trigger |
|---|---|---|
| `/uav/control` | `{"command": "START"}` | ▶ START button pressed |
| `/uav/control` | `{"command": "STOP"}` | ⏹ STOP button pressed |
| `/uav/control` | `{"mode": "WAYPOINT"}` | Mode button clicked |
| `/uav/control` | `{"move": "UP"}` | D-pad pressed (MANUAL only) |

---

## 4. Binary Image Frame Format

Images are too large to send in one MQTT message (broker limit ~256 KB, but spec says ≤200 B per frame).  
Each image is split into **chunks of ≤180 bytes** and wrapped in a binary frame.

### Frame Structure

```
┌──────────┬───────────┬──────────┬──────────┬──────────┬────────────┬──────────┬─────────────┐
│ DELIM    │  img_no   │ DELIM    │ chunk_no │ DELIM    │ total_size │ DELIM    │   payload   │
│ 3 bytes  │ variable  │ 3 bytes  │ variable │ 3 bytes  │  variable  │ 3 bytes  │  ≤180 bytes │
└──────────┴───────────┴──────────┴──────────┴──────────┴────────────┴──────────┴─────────────┘
```

| Field | Encoding | Min size |
|---|---|---|
| `DELIM` | `0xAA 0xBB 0xCC` (literal 3 bytes) | 3 B |
| `img_no` | Big-endian unsigned integer | 1 B |
| `chunk_no` | Big-endian unsigned integer | 1 B |
| `total_size` | Big-endian unsigned integer | 1 B |
| `payload` | Raw image bytes (no base64) | 1 B |

**Maximum frame size: 200 bytes** — typical frame is 196 B (3+1+3+1+3+2+3+180).

### Parsing (GCS side)

```python
parts = raw_bytes.split(b'\xAA\xBB\xCC', maxsplit=4)
# parts[0] = b''          (empty — frame starts with DELIM)
# parts[1] = img_no bytes
# parts[2] = chunk_no bytes
# parts[3] = total_size bytes
# parts[4] = payload bytes  ← maxsplit=4 keeps payload intact even if it contains DELIM
```

### Reassembly

```
Chunk dict: { img_no → { chunk_no → payload_bytes } }

On each chunk received:
  Add payload to dict[img_no][chunk_no]
  assembled = join(dict[img_no][k] for k in sorted(...))
  if len(assembled) >= total_size:
      emit image_signal(assembled)
      delete dict[img_no]

Timeout: if image not complete within 60 s → discard and log warning
```

---

## 5. Telemetry Data Flow

```
sim.py                              test.mosquitto.org          main.py
──────                              ──────────────────          ───────

run_telemetry() every 1 s
  publish /uav/lat      "12.5" ──►  broker  ──────────────►  MQTTHandler._on_message()
  publish /uav/lng      "30.0" ──►  broker  ──────────────►    updates _tele_state dict
  publish /uav/velocity "5.0"  ──►  broker  ──────────────►    emits telemetry_signal(dict)
  publish /uav/battery  "84"   ──►  broker  ──────────────►
  ...                                                         DroneGCS._on_telemetry(data)
                                                                updates UI labels
                                                                feeds TelemetryProcessor
                                                                  → updates DIST / AREA
                                                                feeds DataLogger → CSV row
```

---

## 6. Control Data Flow

```
main.py (GCS UI)                    test.mosquitto.org          sim.py
────────────────                    ──────────────────          ──────

User clicks ▶ START
  mqtt.publish /uav/control ──────► broker ────────────────►  DroneSimulator.handle_control()
  {"command": "START"}                                            _running = True
                                                                  D-pad enabled

User clicks mode button
  mqtt.publish /uav/control ──────► broker ────────────────►  DroneSimulator.handle_control()
  {"mode": "WAYPOINT"}                                            _mode = "WAYPOINT"
                                                                  (auto-cycle disabled)

User presses D-pad (MANUAL only)
  mqtt.publish /uav/control ──────► broker ────────────────►  DroneSimulator.handle_control()
  {"move": "LEFT"}                                                _dx -= 2.0 m
                                                                  applied next telemetry tick
```

---

## 7. Image Data Flow

```
sim.py                                  broker              main.py
──────                                  ──────              ───────

After drone STARTs (3 s delay):
  read image bytes (file or generated PNG)
  split into 180-byte chunks
  
  for each chunk:
    frame = DELIM+varint(img_no)         publish          MQTTHandler._on_message()
            +DELIM+varint(chunk_no)  ──────────────────►  _decode_frame(raw)
            +DELIM+varint(total)                           store in _chunks[img_no][chunk_no]
            +DELIM+payload                                 
    sleep 10 ms                                            when len(assembled) >= total:
                                                             emit image_signal(bytes)
                                                             
                                                           DroneGCS._on_image(bytes)
                                                             QPixmap.loadFromData()
                                                             display in image viewer
```

---

## 8. GCS UI Layout

```
┌──────────────────────────────────────────────────────────────────────┐
│  ◈  GCS | ISRO IoRC 2026              ● CONNECTED        ─   ✕      │  ← Title bar
├──────────────────────────────────────────────────────────────────────┤
│  🕹 MANUAL    📍 WAYPOINT    🤖 AUTO                                  │  ← Mode bar
├─────────────┬────────────────────────────────┬───────────────────────┤
│             │  ● DRONE STATUS: MANUAL        │  BATTERY              │
│  CONTROL    │                                │  [████████░] 84%      │
│    PAD      │  X (m)   Y (m)                │                       │
│             │  12.500  30.000               │  RC SIGNAL            │
│    ▲        │                                │  ▐▐▐▐░  GOOD         │
│  ◄   ►      │  VELOCITY  ALTITUDE           │                       │
│    ▼        │  5.00 m/s  17.3 m            │  IMAGE                │
│             │                                │  📂 LOAD IMG         │
│             │  IMAGE VIEWER                  │  💾 SAVE IMG         │
│             │  ┌──────────────────────────┐  │                       │
│             │  │   (received drone image) │  │  ▶ START             │
│             │  └──────────────────────────┘  │                       │
├─────────────┴────────────────────────────────┴───────────────────────┤
│  DIST  0.0 m    AREA  0.0 m²          MISSION ACTIVE     09:48 IST  │  ← Status bar
└──────────────────────────────────────────────────────────────────────┘
```

---

## 9. File Structure

```
ISRO_IoRC/
├── main.py          — GCS application (PyQt6 UI + MQTT backend)
├── sim.py           — Drone simulator (publishes telemetry + images)
├── main_ref.py      — Original reference implementation
├── DroneGCS.spec    — PyInstaller spec file
├── dist/
│   └── DroneGCS.exe — Standalone executable
└── gcs_log_*.csv    — Telemetry logs (auto-generated per session)
```

---

## 10. Key Design Decisions

| Decision | Rationale |
|---|---|
| **Per-field MQTT topics** | Allows the drone to publish only changed fields; GCS accumulates state across topics |
| **Binary image frame** | No JSON/base64 overhead — raw bytes stay within 200 B frame limit |
| **Sim starts paused** | Drone does nothing until GCS operator explicitly sends START — safe default |
| **Mode feedback loop fix** | `_apply_mode()` updates UI only; `set_mode()` publishes — prevents infinite publish loop |
| **60 s chunk timeout** | Public broker is slow; 60 s gives enough headroom for large images |
| **DataLogger to CSV** | Every telemetry tick is recorded for post-mission analysis |
| **Frameless PyQt6 window** | Clean cockpit aesthetic; custom title bar with drag support |
