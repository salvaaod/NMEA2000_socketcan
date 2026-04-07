# NMEA2000 SocketCAN Simulator

A single-file Flask application that simulates two NMEA 2000 nodes and transmits their traffic through Linux SocketCAN interfaces.

- **Engine node** (configurable source address, engine telemetry PGNs, product info, heartbeat)
- **Switch node** (virtual switch bank status, own address claim/product info/heartbeat)

The app is intended for **bench and integration testing** of systems that consume NMEA 2000 / ISO11783-style CAN traffic.

---

## Purpose

This application helps you:

- Generate realistic NMEA 2000 frames for engine and switching scenarios.
- Exercise consumers with both **single-frame** and **fast-packet** PGNs.
- Test periodic transmission behavior at configurable rates.
- Validate request/response behavior for **ISO Request (PGN 59904)**.
- Send user-triggered switch control commands via **Group Function (PGN 126208)**.

---

## What the code does

The simulator is implemented in `nmea2000_simulator.py` and has four major parts:

1. **Configuration and message model**
   - `SimulatorConfig`: all UI-editable runtime parameters (addresses, engine metrics, switch identity, enable flags, interval).
   - `ProtocolMessage`: normalized message object before conversion into CAN frames.

2. **SocketCAN transport**
   - `SocketCANDevice` controls link setup (`ip link set ...`), opens `python-can` SocketCAN bus, sends frames with retry on TX buffer pressure (`ENOBUFS`), and receives frames.

3. **NMEA 2000 protocol assembly**
   - Utility builders create payloads for:
     - Address Claim (60928)
     - ISO Request (59904)
     - Product Information (126996)
     - Heartbeat (126993)
     - Engine Rapid Update (127488)
     - Engine Dynamic Parameters (127489)
     - Binary Switch Bank Status (127501)
     - Group Function switch command (126208)
   - Long payloads are fragmented with fast-packet sequencing.

4. **Flask web UI and control flow**
   - Browser UI (port `8080`) updates config, connects/disconnects CAN, sends once, starts/stops periodic sending, and toggles 12 switch buttons.
   - Background receiver thread listens for ISO Request frames and auto-responds when the requested PGN is currently enabled.

---

## Supported interfaces

The app maps UI index to interface name:

- `0` → `awlink0`
- `1` → `awlink1`

If an invalid index is entered, the code falls back to `0`.

---

## Dependencies

### System dependencies

- Linux kernel with CAN/SocketCAN support.
- `ip` command from `iproute2`.
- A CAN interface configured as `awlink0` and/or `awlink1` (real hardware or compatible virtual setup).

### Python dependencies

- Python 3.10+
- `Flask`
- `python-can`

Install Python packages:

```bash
pip install flask python-can
```

---

## Running the application

```bash
python3 nmea2000_simulator.py
```

Open:

- `http://localhost:8080`

> Note: Bringing CAN links down/up and setting bitrate usually requires root privileges or equivalent Linux capabilities.

---

## Web UI usage

### 1) Configure connection and timing

- Set **CAN interface index** (`0` or `1`).
- Set **Bitrate** (default `250000`).
- Set **Interval ms** for periodic sending.
- Click **Save Configuration**.

### 2) Connect to SocketCAN

- Click **Connect** to initialize the selected interface and open the CAN bus.
- Status text shows current connection state.

### 3) Control transmission

- **Send Once**: sends one full frame set based on enabled PGNs.
- **Start Periodic**: sends continuously using `interval_ms`.
- **Stop Periodic**: stops continuous transmission.
- **Disconnect**: stops workers and closes the CAN bus.

### 4) Toggle virtual switch channels

- 12 switch controls update the internal switch state.
- Each toggle sends a **PGN 126208 Group Function** command.
- When enabled, periodic/one-shot sets also include **PGN 127501 Binary Switch Bank Status** reflecting current states.

---

## Default behavior summary

With default enable flags, the simulator can publish:

- Engine node: Address Claim, ISO Request, Product Info, Heartbeat, Engine Rapid, Engine Dynamic.
- Switch node: Address Claim, Product Info, Heartbeat, Binary Switch Bank Status.

PGN enable/disable checkboxes in the UI directly control which messages are assembled and transmitted.

---

## Operational notes

- CAN IDs are built as 29-bit extended identifiers suitable for NMEA 2000 style PGN addressing.
- Fast-packet framing is applied for payloads larger than 8 bytes.
- Transmission pacing introduces short delays between frames to reduce TX queue overflow on constrained interfaces.
- RX handling is intentionally focused on ISO Request processing; this is primarily a transmit simulator.

---

## Quick start checklist

1. Ensure `awlink0` or `awlink1` exists and is usable.
2. Install dependencies.
3. Run `python3 nmea2000_simulator.py`.
4. Open `http://localhost:8080`.
5. Save config, connect, then use **Send Once** or **Start Periodic**.
