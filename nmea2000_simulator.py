#!/usr/bin/env python3
"""NMEA2000 simulator served with Flask and SocketCAN.

This application provides a browser UI (http://<host>:8080) to configure and send
NMEA2000 frames over Linux SocketCAN interfaces. It supports selecting interface
index 0 or 1, one-shot transmission, periodic transmission, and binary switch
command/status behavior similar to the old desktop application.
"""

from __future__ import annotations

import subprocess
import threading
import time
from dataclasses import dataclass, field
from errno import ENOBUFS
from typing import Any

import can
from flask import Flask, redirect, render_template_string, request, url_for

DEFAULT_CAN_INTERFACE_INDEX = 0
CAN_INTERFACE_MAP = {0: "awlink0", 1: "awlink1"}
DEFAULT_BITRATE = 250000

# Core NMEA2000 / ISO11783 PGNs used by this simulator.
PGN_ISO_REQUEST = 59904
PGN_ADDRESS_CLAIM = 60928
PGN_PRODUCT_INFO = 126996
PGN_HEARTBEAT = 126993
PGN_ENGINE_RAPID = 127488
PGN_ENGINE_DYNAMIC = 127489
PGN_GROUP_FUNCTION = 126208
PGN_BINARY_SWITCH_BANK_STATUS = 127501

GLOBAL_DESTINATION = 0xFF
DEFAULT_PRIORITY = 6


@dataclass
class ProtocolMessage:
    pgn: int
    data: bytes
    priority: int = DEFAULT_PRIORITY
    destination: int = GLOBAL_DESTINATION
    source_address: int | None = None


@dataclass
class SimulatorConfig:
    can_interface_index: int = DEFAULT_CAN_INTERFACE_INDEX
    bitrate: int = DEFAULT_BITRATE
    source_address: int = 0
    destination_address: int = 255
    engine_instance: int = 0
    device_name: int = 0x1F2000123456789A

    engine_speed_rpm: float = 750.0
    engine_boost_bar: float = 1.0
    trim_percent: float = 0.0
    oil_pressure_bar: float = 3.5
    oil_temp_c: float = 85.0
    coolant_temp_c: float = 78.0
    alternator_v: float = 13.8
    fuel_rate_lph: float = 12.0
    engine_hours_h: float = 500.0
    coolant_pressure_bar: float = 1.2
    fuel_pressure_psi: float = 43.5
    engine_load_percent: float = 35.0
    engine_torque_percent: float = 42.0
    iso_request_pgn: int = PGN_ADDRESS_CLAIM

    product_model: str = "GCAN Engine Sim"
    software_version: str = "1.0.0"
    model_version: str = "1.0"
    serial_code: str = "SIM-0001"

    switch_node_source_address: int = 100
    switch_node_device_name: int = 0x1F2000AA12345678
    switch_manufacturer_code: int = 176
    switch_product_model: str = "CKM12"
    switch_software_version: str = "2.02.08"
    switch_model_version: str = "Rev A"
    switch_serial_code: str = "1606029"

    interval_ms: int = 100
    binary_switch_bank_instance: int = 52

    address_claim_enabled: bool = True
    iso_request_enabled: bool = True
    product_info_enabled: bool = True
    heartbeat_enabled: bool = True
    engine_rapid_enabled: bool = True
    engine_dynamic_enabled: bool = True
    switch_address_claim_enabled: bool = True
    switch_product_info_enabled: bool = True
    switch_heartbeat_enabled: bool = True
    binary_switch_status_enabled: bool = True

    binary_switch_states: list[bool] = field(default_factory=lambda: [False] * 12)


class SocketCANDevice:
    """SocketCAN transport with interface setup/teardown helpers."""

    def __init__(self) -> None:
        self.bus: can.BusABC | None = None
        self.interface: str | None = None

    @staticmethod
    def _run_cmd(cmd: list[str], check: bool = True) -> None:
        subprocess.run(cmd, check=check, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def connect(self, interface: str, bitrate: int) -> None:
        self.disconnect(set_down=False)

        # Reset + set bitrate then bring interface up.
        self._run_cmd(["ip", "link", "set", interface, "down"], check=False)
        self._run_cmd(["ip", "link", "set", interface, "type", "can", "bitrate", str(bitrate)], check=False)
        self._run_cmd(["ip", "link", "set", interface, "up"], check=True)

        self.bus = can.Bus(interface="socketcan", channel=interface)
        self.interface = interface

    def disconnect(self, set_down: bool = True) -> None:
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

        if set_down and self.interface:
            self._run_cmd(["ip", "link", "set", self.interface, "down"], check=False)
        self.interface = None

    def send(self, frame_id: int, data: bytes) -> None:
        if not self.bus:
            raise RuntimeError("SocketCAN bus not connected")
        msg = can.Message(arbitration_id=frame_id, data=data, is_extended_id=True)
        retries = 5
        for attempt in range(retries):
            try:
                self.bus.send(msg, timeout=0.2)
                return
            except can.CanOperationError as exc:
                error_code = getattr(exc, "error_code", None)
                if error_code != ENOBUFS and "No buffer space available" not in str(exc):
                    raise
                if attempt == retries - 1:
                    raise
                # SocketCAN TX queue is full; wait briefly and retry.
                time.sleep(0.01)

    def recv(self, timeout: float = 0.0) -> can.Message | None:
        if not self.bus:
            return None
        return self.bus.recv(timeout=timeout)


# ===== Protocol utility functions =====

def nmea2000_id(priority: int, pgn: int, source_address: int, destination: int = GLOBAL_DESTINATION) -> int:
    pf = (pgn >> 8) & 0xFF
    if pf < 240:
        pgn_field = pgn & 0x3FF00
        ps = destination & 0xFF
    else:
        pgn_field = pgn & 0x3FFFF
        ps = pgn & 0xFF
    return (
        ((priority & 0x7) << 26)
        | ((pgn_field & 0x3FF00) << 8)
        | ((pf & 0xFF) << 16)
        | ((ps & 0xFF) << 8)
        | (source_address & 0xFF)
    )


def extract_pgn_and_destination(frame_id: int) -> tuple[int, int]:
    pf = (frame_id >> 16) & 0xFF
    ps = (frame_id >> 8) & 0xFF
    dp = (frame_id >> 24) & 0x01
    if pf < 240:
        pgn = (dp << 16) | (pf << 8)
        destination = ps
    else:
        pgn = (dp << 16) | (pf << 8) | ps
        destination = GLOBAL_DESTINATION
    return pgn, destination


def clamp_u16(value: float, scale: float, minimum: float = 0.0, maximum: float = 65535.0) -> int:
    scaled = int(round(value / scale))
    return max(int(minimum), min(int(maximum), scaled))


def le_u16(value: int) -> bytes:
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def build_address_claim(name: int) -> bytes:
    return name.to_bytes(8, byteorder="little", signed=False)


def set_name_manufacturer_code(name: int, manufacturer_code: int) -> int:
    code = max(0, min(0x7FF, int(manufacturer_code)))
    mask = 0x7FF << 21
    return (name & ~mask) | (code << 21)


def build_iso_request(requested_pgn: int) -> bytes:
    return bytes((requested_pgn & 0xFF, (requested_pgn >> 8) & 0xFF, (requested_pgn >> 16) & 0xFF)) + bytes((0xFF,) * 5)


def build_engine_rapid(engine_instance: int, engine_speed_rpm: float, engine_boost_bar: float, trim_percent: float) -> bytes:
    speed_raw = clamp_u16(engine_speed_rpm, 0.25)
    boost_pa = engine_boost_bar * 100000.0
    boost_raw = clamp_u16(boost_pa, 100.0)
    trim_raw = max(-125, min(125, int(round(trim_percent))))
    return bytes(
        (
            engine_instance & 0xFF,
            speed_raw & 0xFF,
            (speed_raw >> 8) & 0xFF,
            boost_raw & 0xFF,
            (boost_raw >> 8) & 0xFF,
            trim_raw & 0xFF,
            0xFF,
            0xFF,
        )
    )


def build_engine_dynamic(
    engine_instance: int,
    oil_pressure_bar: float,
    oil_temp_c: float,
    coolant_temp_c: float,
    alternator_voltage_v: float,
    fuel_rate_lph: float,
    total_engine_hours_h: float,
    coolant_pressure_bar: float,
    fuel_pressure_bar: float,
    engine_load_percent: float,
    engine_torque_percent: float,
) -> bytes:
    oil_p_raw = clamp_u16(oil_pressure_bar * 100000.0, 100.0)
    oil_t_raw = clamp_u16(oil_temp_c + 273.15, 0.1)
    coolant_t_raw = clamp_u16(coolant_temp_c + 273.15, 0.01)
    alt_v_raw = clamp_u16(alternator_voltage_v, 0.01)
    fuel_rate_raw = clamp_u16(fuel_rate_lph, 0.1)
    hours_raw = max(0, min(0xFFFFFFFF, int(round(total_engine_hours_h))))
    coolant_p_raw = clamp_u16(coolant_pressure_bar * 100000.0, 100.0)
    fuel_p_raw = clamp_u16(fuel_pressure_bar * 100000.0, 1000.0)
    load_raw = max(-125, min(125, int(round(engine_load_percent)))) & 0xFF
    torque_raw = max(-125, min(125, int(round(engine_torque_percent)))) & 0xFF

    data = bytearray()
    data.extend((engine_instance & 0xFF,))
    data.extend(le_u16(oil_p_raw))
    data.extend(le_u16(oil_t_raw))
    data.extend(le_u16(coolant_t_raw))
    data.extend(le_u16(alt_v_raw))
    data.extend(le_u16(fuel_rate_raw))
    data.extend(hours_raw.to_bytes(4, byteorder="little", signed=False))
    data.extend(le_u16(coolant_p_raw))
    data.extend(le_u16(fuel_p_raw))
    data.extend((0xFF,))
    data.extend(le_u16(0xFFFF))
    data.extend(le_u16(0xFFFF))
    data.extend((load_raw, torque_raw))
    return bytes(data)


def build_product_info_payload(model_id: str, software_version: str, model_version: str, serial_code: str) -> bytes:
    model = model_id[:32].ljust(32, "\x00").encode("ascii", errors="ignore")
    software = software_version[:32].ljust(32, "\x00").encode("ascii", errors="ignore")
    version = model_version[:32].ljust(32, "\x00").encode("ascii", errors="ignore")
    serial = serial_code[:32].ljust(32, "\x00").encode("ascii", errors="ignore")
    n2k_version = (2100).to_bytes(2, byteorder="little")
    product_code = (1001).to_bytes(2, byteorder="little")
    cert_level = bytes((1,))
    load_equivalency = bytes((1,))
    return n2k_version + product_code + model + software + version + serial + cert_level + load_equivalency


def build_heartbeat_payload(interval_ms: int, sequence_counter: int) -> bytes:
    return int(max(0, min(0xFFFF, interval_ms))).to_bytes(2, byteorder="little", signed=False) + bytes((sequence_counter & 0xFF,)) + bytes((0xFF,) * 5)


def _pack_2bit_values(values: list[int], output_len: int) -> bytes:
    packed = bytearray((0x00,) * output_len)
    for index, value in enumerate(values):
        bit_pos = index * 2
        byte_index = bit_pos // 8
        shift = bit_pos % 8
        if byte_index >= output_len:
            break
        packed[byte_index] |= (value & 0x03) << shift
    return bytes(packed)


def build_binary_switch_bank_status(bank_instance: int, switch_states: list[bool]) -> bytes:
    states_2bit = [(1 if state else 0) for state in switch_states[:12]]
    if len(states_2bit) < 28:
        states_2bit.extend([3] * (28 - len(states_2bit)))
    packed_states = _pack_2bit_values(states_2bit[:28], 7)
    return bytes((bank_instance & 0xFF,)) + packed_states


def build_group_function_binary_switch_command(bank_instance: int, switch_number: int, state_on: bool) -> bytes:
    return bytes(
        (
            1,
            PGN_BINARY_SWITCH_BANK_STATUS & 0xFF,
            (PGN_BINARY_SWITCH_BANK_STATUS >> 8) & 0xFF,
            (PGN_BINARY_SWITCH_BANK_STATUS >> 16) & 0xFF,
            3,
            bank_instance & 0xFF,
            max(1, min(12, switch_number)) & 0xFF,
            1 if state_on else 0,
        )
    )


def split_fast_packet(payload: bytes, sequence_id: int) -> list[bytes]:
    if len(payload) <= 8:
        return [payload]
    sid = sequence_id & 0x07
    frame_index = 0
    cursor = 0
    frames: list[bytes] = []

    first_chunk = payload[cursor : cursor + 6]
    cursor += len(first_chunk)
    frames.append(bytes(((sid << 5) | frame_index, len(payload))) + first_chunk)
    frame_index += 1

    while cursor < len(payload):
        chunk = payload[cursor : cursor + 7]
        cursor += len(chunk)
        frames.append(bytes((((sid << 5) | frame_index),)) + chunk)
        frame_index += 1
    return frames


class SimulatorService:
    def __init__(self) -> None:
        self.config = SimulatorConfig()
        self.device = SocketCANDevice()
        self.lock = threading.Lock()
        self.fast_packet_sequence = 0
        self.engine_heartbeat_sequence = 0
        self.switch_heartbeat_sequence = 0
        self.connected = False
        self.periodic_running = False
        self.periodic_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.rx_stop_event = threading.Event()
        self.rx_thread: threading.Thread | None = None
        self.status = "Disconnected"

    def _source_address(self) -> int:
        return max(0, min(251, self.config.source_address))

    def _switch_source_address(self) -> int:
        return max(0, min(251, self.config.switch_node_source_address))

    def _destination(self) -> int:
        return max(0, min(255, self.config.destination_address))

    def _switch_device_name(self) -> int:
        return set_name_manufacturer_code(self.config.switch_node_device_name, self.config.switch_manufacturer_code)

    def _binary_switch_bank_instance(self) -> int:
        return max(0, min(255, self.config.binary_switch_bank_instance))

    def connect(self) -> None:
        idx = 0 if self.config.can_interface_index not in CAN_INTERFACE_MAP else self.config.can_interface_index
        interface = CAN_INTERFACE_MAP[idx]
        self.device.connect(interface, self.config.bitrate)
        self.connected = True
        self._start_receiver()
        self.status = f"Connected to {interface} @ {self.config.bitrate} bps"

    def disconnect(self) -> None:
        self.stop_periodic()
        self._stop_receiver()
        self.device.disconnect(set_down=True)
        self.connected = False
        self.status = "Disconnected"

    def _start_receiver(self) -> None:
        self.rx_stop_event.clear()
        self.rx_thread = threading.Thread(target=self._receiver_worker, daemon=True)
        self.rx_thread.start()

    def _stop_receiver(self) -> None:
        self.rx_stop_event.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        self.rx_thread = None

    def current_messages(self) -> list[ProtocolMessage]:
        cfg = self.config
        destination = self._destination()
        engine_instance = max(0, min(255, cfg.engine_instance))

        messages: list[ProtocolMessage] = []
        if cfg.address_claim_enabled:
            messages.append(ProtocolMessage(PGN_ADDRESS_CLAIM, build_address_claim(cfg.device_name), 6, GLOBAL_DESTINATION, self._source_address()))
        if cfg.iso_request_enabled:
            request_pgn = max(0, min(0x3FFFF, cfg.iso_request_pgn))
            messages.append(ProtocolMessage(PGN_ISO_REQUEST, build_iso_request(request_pgn), 6, destination))
        if cfg.product_info_enabled:
            messages.append(
                ProtocolMessage(
                    PGN_PRODUCT_INFO,
                    build_product_info_payload(cfg.product_model, cfg.software_version, cfg.model_version, cfg.serial_code),
                    6,
                    GLOBAL_DESTINATION,
                )
            )
        if cfg.heartbeat_enabled:
            payload = build_heartbeat_payload(max(0, cfg.interval_ms), self.engine_heartbeat_sequence)
            self.engine_heartbeat_sequence = (self.engine_heartbeat_sequence + 1) & 0xFF
            messages.append(ProtocolMessage(PGN_HEARTBEAT, payload, 7, GLOBAL_DESTINATION, self._source_address()))
        if cfg.engine_rapid_enabled:
            messages.append(
                ProtocolMessage(
                    PGN_ENGINE_RAPID,
                    build_engine_rapid(engine_instance, cfg.engine_speed_rpm, cfg.engine_boost_bar, cfg.trim_percent),
                    2,
                    GLOBAL_DESTINATION,
                    self._source_address(),
                )
            )
        if cfg.engine_dynamic_enabled:
            messages.append(
                ProtocolMessage(
                    PGN_ENGINE_DYNAMIC,
                    build_engine_dynamic(
                        engine_instance,
                        cfg.oil_pressure_bar,
                        cfg.oil_temp_c,
                        cfg.coolant_temp_c,
                        cfg.alternator_v,
                        cfg.fuel_rate_lph,
                        cfg.engine_hours_h,
                        cfg.coolant_pressure_bar,
                        cfg.fuel_pressure_psi * 0.0689476,
                        cfg.engine_load_percent,
                        cfg.engine_torque_percent,
                    ),
                    2,
                    GLOBAL_DESTINATION,
                    self._source_address(),
                )
            )
        if cfg.switch_address_claim_enabled:
            messages.append(ProtocolMessage(PGN_ADDRESS_CLAIM, build_address_claim(self._switch_device_name()), 6, GLOBAL_DESTINATION, self._switch_source_address()))
        if cfg.switch_product_info_enabled:
            messages.append(
                ProtocolMessage(
                    PGN_PRODUCT_INFO,
                    build_product_info_payload(
                        cfg.switch_product_model,
                        cfg.switch_software_version,
                        cfg.switch_model_version,
                        cfg.switch_serial_code,
                    ),
                    6,
                    GLOBAL_DESTINATION,
                    self._switch_source_address(),
                )
            )
        if cfg.switch_heartbeat_enabled:
            payload = build_heartbeat_payload(max(0, cfg.interval_ms), self.switch_heartbeat_sequence)
            self.switch_heartbeat_sequence = (self.switch_heartbeat_sequence + 1) & 0xFF
            messages.append(ProtocolMessage(PGN_HEARTBEAT, payload, 7, GLOBAL_DESTINATION, self._switch_source_address()))
        if cfg.binary_switch_status_enabled:
            messages.append(
                ProtocolMessage(
                    PGN_BINARY_SWITCH_BANK_STATUS,
                    build_binary_switch_bank_status(self._binary_switch_bank_instance(), cfg.binary_switch_states),
                    3,
                    GLOBAL_DESTINATION,
                    self._switch_source_address(),
                )
            )
        return messages

    def _expand_protocol_message(self, message: ProtocolMessage) -> list[tuple[int, bytes]]:
        source = self._source_address() if message.source_address is None else message.source_address
        frame_id = nmea2000_id(message.priority, message.pgn, source, message.destination)
        if len(message.data) <= 8:
            return [(frame_id, message.data)]
        frames = split_fast_packet(message.data, self.fast_packet_sequence)
        self.fast_packet_sequence = (self.fast_packet_sequence + 1) & 0x07
        return [(frame_id, frame.ljust(8, b"\xFF")) for frame in frames]

    def _messages_for_requested_pgn(self, requested_pgn: int) -> list[ProtocolMessage]:
        return [msg for msg in self.current_messages() if msg.pgn == requested_pgn]

    def _handle_iso_request(self, destination: int, data: bytes) -> None:
        if len(data) < 3:
            return
        requested_pgn = data[0] | (data[1] << 8) | (data[2] << 16)
        engine_src = self._source_address()
        switch_src = self._switch_source_address()
        if destination not in (GLOBAL_DESTINATION, engine_src, switch_src):
            return
        for message in self._messages_for_requested_pgn(requested_pgn):
            for frame_id, frame_data in self._expand_protocol_message(message):
                self.device.send(frame_id, frame_data)
                time.sleep(0.001)

    def _receiver_worker(self) -> None:
        while not self.rx_stop_event.is_set():
            try:
                msg = self.device.recv(timeout=0.1)
                if msg is None or not msg.is_extended_id:
                    continue
                pgn, destination = extract_pgn_and_destination(msg.arbitration_id)
                if pgn == PGN_ISO_REQUEST:
                    with self.lock:
                        self._handle_iso_request(destination, bytes(msg.data))
            except Exception as exc:
                self.status = f"Receive error: {exc}"
                return

    def current_frames(self) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        for message in self.current_messages():
            frames.extend(self._expand_protocol_message(message))
        return frames

    def send_once(self) -> None:
        if not self.connected:
            raise RuntimeError("Not connected")
        frames = self.current_frames()
        for index, (frame_id, data) in enumerate(frames):
            self.device.send(frame_id, data)
            # Short pacing prevents bursts of fast-packet frames from overflowing
            # small kernel TX queues on low-end interfaces.
            if index != len(frames) - 1:
                time.sleep(0.001)

    def send_switch_command(self, switch_number: int, state_on: bool) -> None:
        if not self.connected:
            return
        payload = build_group_function_binary_switch_command(self._binary_switch_bank_instance(), switch_number, state_on)
        frame_id = nmea2000_id(DEFAULT_PRIORITY, PGN_GROUP_FUNCTION, self._source_address(), self._destination())
        self.device.send(frame_id, payload)

    def _periodic_worker(self) -> None:
        while not self.stop_event.is_set():
            start = time.time()
            try:
                with self.lock:
                    self.send_once()
            except Exception as exc:
                self.status = f"Periodic send error: {exc}"
                self.periodic_running = False
                return
            elapsed_ms = int((time.time() - start) * 1000)
            wait_ms = max(10, self.config.interval_ms) - elapsed_ms
            if self.stop_event.wait(max(0.01, wait_ms / 1000.0)):
                break

    def start_periodic(self) -> None:
        if not self.connected or self.periodic_running:
            return
        self.stop_event.clear()
        self.periodic_running = True
        self.periodic_thread = threading.Thread(target=self._periodic_worker, daemon=True)
        self.periodic_thread.start()
        self.status = "Periodic sending started"

    def stop_periodic(self) -> None:
        if not self.periodic_running:
            return
        self.stop_event.set()
        if self.periodic_thread and self.periodic_thread.is_alive():
            self.periodic_thread.join(timeout=1.0)
        self.periodic_running = False
        self.periodic_thread = None
        self.status = "Periodic sending stopped"


app = Flask(__name__)
service = SimulatorService()

HTML_TEMPLATE = """
<!doctype html>
<html>
<head>
  <meta charset=\"utf-8\" />
  <title>NMEA2000 SocketCAN Simulator</title>
  <style>
    body { font-family: sans-serif; margin: 1rem 2rem; }
    fieldset { margin-bottom: 1rem; }
    label { display:inline-block; min-width:220px; }
    .grid { display:grid; grid-template-columns:repeat(4, minmax(240px, 1fr)); gap: 1rem; }
    .switches form { display:inline-block; margin:2px; }
  </style>
</head>
<body>
  <h2>NMEA2000 SocketCAN Simulator (Flask)</h2>
  <p><strong>Status:</strong> {{ status }}</p>

  <form method=\"post\" action=\"{{ url_for('update') }}\">
    <div class=\"grid\">
      <fieldset>
        <legend>Connection</legend>
        <label>CAN interface index (0/1)</label><input name=\"can_interface_index\" value=\"{{ cfg.can_interface_index }}\" /><br/>
        <label>Bitrate</label><input name=\"bitrate\" value=\"{{ cfg.bitrate }}\" /><br/>
        <label>Interval ms</label><input name=\"interval_ms\" value=\"{{ cfg.interval_ms }}\" /><br/>
      </fieldset>

      <fieldset>
        <legend>Node addressing</legend>
        <label>Source address</label><input name=\"source_address\" value=\"{{ cfg.source_address }}\" /><br/>
        <label>Destination</label><input name=\"destination_address\" value=\"{{ cfg.destination_address }}\" /><br/>
        <label>Engine instance</label><input name=\"engine_instance\" value=\"{{ cfg.engine_instance }}\" /><br/>
        <label>Device NAME</label><input name=\"device_name\" value=\"{{ '0x%X' % cfg.device_name }}\" /><br/>
      </fieldset>

      <fieldset>
        <legend>Engine values</legend>
        <label>Speed rpm</label><input name=\"engine_speed_rpm\" value=\"{{ cfg.engine_speed_rpm }}\" /><br/>
        <label>Boost bar</label><input name=\"engine_boost_bar\" value=\"{{ cfg.engine_boost_bar }}\" /><br/>
        <label>Trim %</label><input name=\"trim_percent\" value=\"{{ cfg.trim_percent }}\" /><br/>
        <label>Oil pressure bar</label><input name=\"oil_pressure_bar\" value=\"{{ cfg.oil_pressure_bar }}\" /><br/>
        <label>Oil temp C</label><input name=\"oil_temp_c\" value=\"{{ cfg.oil_temp_c }}\" /><br/>
        <label>Coolant temp C</label><input name=\"coolant_temp_c\" value=\"{{ cfg.coolant_temp_c }}\" /><br/>
        <label>Alternator V</label><input name=\"alternator_v\" value=\"{{ cfg.alternator_v }}\" /><br/>
        <label>Fuel rate L/h</label><input name=\"fuel_rate_lph\" value=\"{{ cfg.fuel_rate_lph }}\" /><br/>
        <label>Engine hours</label><input name=\"engine_hours_h\" value=\"{{ cfg.engine_hours_h }}\" /><br/>
        <label>Coolant pressure bar</label><input name=\"coolant_pressure_bar\" value=\"{{ cfg.coolant_pressure_bar }}\" /><br/>
        <label>Fuel pressure PSI</label><input name=\"fuel_pressure_psi\" value=\"{{ cfg.fuel_pressure_psi }}\" /><br/>
        <label>Engine load %</label><input name=\"engine_load_percent\" value=\"{{ cfg.engine_load_percent }}\" /><br/>
        <label>Engine torque %</label><input name=\"engine_torque_percent\" value=\"{{ cfg.engine_torque_percent }}\" /><br/>
      </fieldset>

      <fieldset>
        <legend>Products + switch node</legend>
        <label>ISO request PGN</label><input name=\"iso_request_pgn\" value=\"{{ cfg.iso_request_pgn }}\" /><br/>
        <label>Product model</label><input name=\"product_model\" value=\"{{ cfg.product_model }}\" /><br/>
        <label>Software version</label><input name=\"software_version\" value=\"{{ cfg.software_version }}\" /><br/>
        <label>Model version</label><input name=\"model_version\" value=\"{{ cfg.model_version }}\" /><br/>
        <label>Serial code</label><input name=\"serial_code\" value=\"{{ cfg.serial_code }}\" /><br/>
        <hr/>
        <label>Switch source address</label><input name=\"switch_node_source_address\" value=\"{{ cfg.switch_node_source_address }}\" /><br/>
        <label>Switch NAME</label><input name=\"switch_node_device_name\" value=\"{{ '0x%X' % cfg.switch_node_device_name }}\" /><br/>
        <label>Switch manufacturer code</label><input name=\"switch_manufacturer_code\" value=\"{{ cfg.switch_manufacturer_code }}\" /><br/>
        <label>Switch model</label><input name=\"switch_product_model\" value=\"{{ cfg.switch_product_model }}\" /><br/>
        <label>Switch software</label><input name=\"switch_software_version\" value=\"{{ cfg.switch_software_version }}\" /><br/>
        <label>Switch model version</label><input name=\"switch_model_version\" value=\"{{ cfg.switch_model_version }}\" /><br/>
        <label>Switch serial</label><input name=\"switch_serial_code\" value=\"{{ cfg.switch_serial_code }}\" /><br/>
        <label>Bank instance</label><input name=\"binary_switch_bank_instance\" value=\"{{ cfg.binary_switch_bank_instance }}\" /><br/>
      </fieldset>
    </div>

    <fieldset>
      <legend>Enabled data PGNs</legend>
      <p>Protocol PGNs are always enabled.</p>
      {% for field, label in toggles %}
      <label><input type=\"checkbox\" name=\"{{ field }}\" {% if getattr(cfg, field) %}checked{% endif %}/> {{ label }}</label>
      {% endfor %}
    </fieldset>

    <button type=\"submit\">Save Configuration</button>
  </form>

  <p>
    <a href=\"{{ url_for('action', name='connect') }}\">Connect</a> |
    <a href=\"{{ url_for('action', name='disconnect') }}\">Disconnect</a> |
    <a href=\"{{ url_for('action', name='send_once') }}\">Send Once</a> |
    <a href=\"{{ url_for('action', name='start_periodic') }}\">Start Periodic</a> |
    <a href=\"{{ url_for('action', name='stop_periodic') }}\">Stop Periodic</a>
  </p>

  <fieldset class=\"switches\">
    <legend>Binary switches (toggle sends PGN 126208 command)</legend>
    {% for idx, state in switches %}
      <form method=\"post\" action=\"{{ url_for('set_switch', switch_no=idx) }}\">
        <input type=\"hidden\" name=\"state\" value=\"{{ '0' if state else '1' }}\" />
        <button type=\"submit\">SW {{ idx }}: {{ 'ON' if state else 'OFF' }}</button>
      </form>
    {% endfor %}
  </fieldset>
</body>
</html>
"""


TOGGLE_FIELDS = [
    ("engine_rapid_enabled", "Engine Rapid PGN 127488"),
    ("engine_dynamic_enabled", "Engine Dynamic PGN 127489"),
    ("binary_switch_status_enabled", "Binary Switch Bank Status PGN 127501"),
]

PROTOCOL_ALWAYS_ENABLED_FIELDS = [
    "address_claim_enabled",
    "iso_request_enabled",
    "product_info_enabled",
    "heartbeat_enabled",
    "switch_address_claim_enabled",
    "switch_product_info_enabled",
    "switch_heartbeat_enabled",
]


def parse_int(value: str, default: int = 0) -> int:
    try:
        text = value.strip()
        return int(text, 16) if text.lower().startswith("0x") else int(float(text))
    except Exception:
        return default


def parse_float(value: str, default: float = 0.0) -> float:
    try:
        return float(value.strip())
    except Exception:
        return default


@app.get("/")
def index() -> str:
    with service.lock:
        cfg = service.config
        switches = list(enumerate(cfg.binary_switch_states, start=1))
        return render_template_string(
            HTML_TEMPLATE,
            cfg=cfg,
            status=service.status,
            toggles=TOGGLE_FIELDS,
            switches=switches,
            getattr=getattr,
        )


@app.post("/update")
def update() -> Any:
    form = request.form
    with service.lock:
        cfg = service.config
        cfg.can_interface_index = max(0, min(1, parse_int(form.get("can_interface_index", "0"), cfg.can_interface_index)))
        cfg.bitrate = max(10000, parse_int(form.get("bitrate", str(cfg.bitrate)), cfg.bitrate))
        cfg.interval_ms = max(10, parse_int(form.get("interval_ms", str(cfg.interval_ms)), cfg.interval_ms))
        cfg.source_address = max(0, min(251, parse_int(form.get("source_address", str(cfg.source_address)), cfg.source_address)))
        cfg.destination_address = max(0, min(255, parse_int(form.get("destination_address", str(cfg.destination_address)), cfg.destination_address)))
        cfg.engine_instance = max(0, min(255, parse_int(form.get("engine_instance", str(cfg.engine_instance)), cfg.engine_instance)))
        cfg.device_name = parse_int(form.get("device_name", hex(cfg.device_name)), cfg.device_name)

        float_fields = [
            "engine_speed_rpm", "engine_boost_bar", "trim_percent", "oil_pressure_bar", "oil_temp_c", "coolant_temp_c",
            "alternator_v", "fuel_rate_lph", "engine_hours_h", "coolant_pressure_bar", "fuel_pressure_psi",
            "engine_load_percent", "engine_torque_percent",
        ]
        for field in float_fields:
            setattr(cfg, field, parse_float(form.get(field, str(getattr(cfg, field))), getattr(cfg, field)))

        int_fields = [
            "iso_request_pgn", "switch_node_source_address", "switch_node_device_name", "switch_manufacturer_code",
            "binary_switch_bank_instance",
        ]
        for field in int_fields:
            setattr(cfg, field, parse_int(form.get(field, str(getattr(cfg, field))), getattr(cfg, field)))

        str_fields = [
            "product_model", "software_version", "model_version", "serial_code", "switch_product_model",
            "switch_software_version", "switch_model_version", "switch_serial_code",
        ]
        for field in str_fields:
            setattr(cfg, field, form.get(field, getattr(cfg, field)))

        for field, _ in TOGGLE_FIELDS:
            setattr(cfg, field, field in form)
        for field in PROTOCOL_ALWAYS_ENABLED_FIELDS:
            setattr(cfg, field, True)

        service.status = "Configuration updated"
    return redirect(url_for("index"))


@app.get("/action/<name>")
def action(name: str) -> Any:
    try:
        with service.lock:
            if name == "connect":
                service.connect()
            elif name == "disconnect":
                service.disconnect()
            elif name == "send_once":
                service.send_once()
                service.status = "Sent one batch of frames"
            elif name == "start_periodic":
                service.start_periodic()
            elif name == "stop_periodic":
                service.stop_periodic()
            else:
                service.status = f"Unknown action: {name}"
    except Exception as exc:
        service.status = f"Action '{name}' failed: {exc}"
    return redirect(url_for("index"))


@app.post("/switch/<int:switch_no>")
def set_switch(switch_no: int) -> Any:
    switch_index = max(1, min(12, switch_no)) - 1
    state = request.form.get("state", "0") == "1"
    with service.lock:
        service.config.binary_switch_states[switch_index] = state
        service.send_switch_command(switch_index + 1, state)
        service.status = f"Switch {switch_index + 1} set to {'ON' if state else 'OFF'}"
    return redirect(url_for("index"))


def main() -> None:
    # Flask HTTP server must run on port 8080 per project requirement.
    app.run(host="0.0.0.0", port=8080, debug=False)


if __name__ == "__main__":
    main()
