# NMEA2000 SocketCAN Simulator (Flask)

This project is a **Flask-based NMEA2000 simulator** that sends NMEA2000/ISO11783 traffic through Linux **SocketCAN**.

## What changed

- Removed the legacy ECan/`ECanVci.dll` transport code.
- Removed the Qt/Tk desktop workflow and replaced it with a **web UI using Flask**.
- Added SocketCAN interface selection by index:
  - `0` → `awlink0`
  - `1` → `awlink1`
- Kept simulator behavior similar to the previous app:
  - ISO Address Claim / ISO Request
  - Product Information / Heartbeat
  - Engine Rapid and Engine Dynamic PGNs
  - Virtual switch node identity and binary switch bank status
  - Group Function switch command writes (PGN 126208)
- Flask runs on **port 8080**.

## Requirements

- Linux with SocketCAN support
- Python 3.10+
- Root privileges (or equivalent capabilities) for `ip link set ...` and CAN interface configuration
- Python packages:

```bash
pip install flask python-can
```

## Run

```bash
python3 nmea2000_simulator.py
```

Then open:

- `http://localhost:8080`

## Using interface 0 or 1

In the web UI, set **CAN interface index**:

- `0` uses `awlink0`
- `1` uses `awlink1`

The app brings the selected interface down, tries to set bitrate, then brings it up before opening the SocketCAN bus.

## Controls in UI

- **Save Configuration**: updates all current fields and message enable flags.
- **Connect / Disconnect**: open/close SocketCAN on the selected interface.
- **Send Once**: transmits one full frame batch with the currently enabled PGNs.
- **Start Periodic / Stop Periodic**: controls continuous sending at `interval_ms`.
- **Binary switch buttons**: toggle switch state and send PGN 126208 command frames.

## Notes

- Fast-packet fragmentation is applied for payloads larger than 8 bytes.
- This simulator is transmit-focused and does not implement frame receive/parse UI.
