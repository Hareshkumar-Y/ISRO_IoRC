# 🚀 Ground Control Station (GCS) for Autonomous Drone

### ISRO IoRC 2026 Project

---

## 🧠 Overview

This project implements a **desktop-based Ground Control Station (GCS)** for an autonomous UAV.
The application acts as the **Home Station**, receiving real-time telemetry and image data from the drone via **MQTT**, and sending control commands back.

The system is designed for:

* Real-time monitoring 📡
* Remote control 🎮
* Visual data reconstruction 🖼
* Mission-level interaction 🧭

---

## ⚙️ Core Features

### 📡 1. MQTT Communication

* Protocol: MQTT
* Library: `paho-mqtt`
* Acts as:

  * Subscriber → Telemetry & Image data
  * Publisher → Control commands

---

### 📍 2. Telemetry Dashboard

Displays real-time:

* Latitude (LAT)
* Longitude (LNG)
* Velocity
* Drone Mode (MANUAL / AUTO / etc.)

---

### 🖼 3. Image Reconstruction System

* Images are transmitted as **fragmented chunks**
* Each chunk contains:

  * `img_no`
  * `chunk_no`
  * `total_img_size`
  * `payload` (≤ 200 bytes)

#### Reconstruction Flow:

1. Group chunks using `img_no`
2. Sort using `chunk_no`
3. Combine payloads
4. Validate using `total_img_size`
5. Decode and display image

---

### 🎮 4. Control System

UI-based controls send commands via MQTT:

* Flight Modes:

  * MANUAL
  * WAYPOINT
  * AUTO
  * RTH (Return to Home)
* Movement (Directional Pad):

  * UP / DOWN / LEFT / RIGHT
* Emergency:

  * STOP (critical override)

---

### 🧾 5. UI Design (Drone Cockpit Style)

The interface follows a **modern UAV dashboard layout**:

#### 🔝 Top Section

* Mode selector (MANUAL, WAYPOINT, AUTO, RTH)

#### 🎯 Center Panel

* Drone Status (highlighted)
* Live LAT / LNG

#### ⬇️ Bottom Info Bar

* Distance (Dist)
* Area Covered (Area)

#### ⬅️ Left Panel

* Directional control pad
* Viewing toggle

#### ➡️ Right Panel

* Battery %
* RC Signal Strength
* LOAD / SAVE buttons

#### 🔴 Emergency Control

* Large STOP button (bottom-right)

---

## 📡 MQTT Topic Structure

| Topic            | Direction | Description              |
| ---------------- | --------- | ------------------------ |
| `/uav/telemetry` | Sub       | Position & velocity data |
| `/uav/image`     | Sub       | Image chunks             |
| `/uav/control`   | Pub       | Commands to UAV          |

---

## 📦 Data Formats

### Telemetry संदेश (JSON)

```json
{
  "lat": 37.7749,
  "lng": -122.4194,
  "velocity": 5.2,
  "mode": "MANUAL"
}
```

---

### Image Chunk Format

```json
{
  "img_no": 1,
  "chunk_no": 3,
  "total_img_size": 20480,
  "payload": "base64_encoded_data"
}
```

---

## 🧱 Tech Stack

| Component     | Technology       |
| ------------- | ---------------- |
| Language      | Python           |
| GUI Framework | PyQt6            |
| Communication | MQTT (paho-mqtt) |
| Visualization | PyQt Widgets     |

---

## ▶️ How to Run

### 1. Install Dependencies

```bash
pip install PyQt6 paho-mqtt
```

---

### 2. Start MQTT Broker

Local:

```bash
mosquitto
```

OR use public broker:

```
test.mosquitto.org
```

---

### 3. Run Application

```bash
python main.py
```

---

## 🧪 Testing MQTT Data

### Publish Telemetry (example)

```bash
mosquitto_pub -h test.mosquitto.org -t /uav/telemetry -m '{"lat":12.9,"lng":80.2,"velocity":3.5}'
```

---

### Publish Image Chunk (example)

Use Python script or MQTT client to send base64 chunks.

---

## ⚠️ Engineering Considerations

### 🔥 Packet Loss Handling

* Use MQTT QoS (1 or 2)
* Implement chunk validation

---

### 🔥 Image Assembly Risks

* Handle missing chunks
* Add timeout for incomplete images

---

### 🔥 Latency Optimization

* Compress images (JPEG)
* Limit frame rate

---

### 🔥 Synchronization

* Use timestamps for all messages

---

## 🧭 Future Enhancements

* Live map integration (GPS plotting)
* Video streaming (instead of chunked images)
* Mission planning UI
* Autonomous path visualization

---

## 🛰 System Architecture

```
        UAV (Drone)
             │
             ▼
        MQTT Broker
             │
             ▼
   Ground Control Station (This App)
             │
             ▼
   Operator / User Interface
```

---

## 🏁 Goal

To create a **real-time, responsive, and intuitive ground station** that enables:

* Safe drone control
* Efficient monitoring
* Reliable data handling

---

## 💡 Note

This is a **prototype-focused implementation** built under time constraints.
Priority is given to:

> ✔ Functionality
> ✔ UI clarity
> ✔ Rapid deployment

---

## 🚀 Mission Status

> Ground station ready for deployment.
> Awaiting UAV handshake... 📡
