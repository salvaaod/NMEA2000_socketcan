import ctypes
import os
import platform
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk

USBCAN_II = 4
DEFAULT_DEVICE_TYPE = USBCAN_II
DEFAULT_DEVICE_INDEX = 0
DEFAULT_CAN_INDEX = 0
DEFAULT_DLL_NAME = "ECanVci.dll"
TIMING0_250K = 0x01
TIMING1_250K = 0x1C

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
class DeviceConfig:
    dll_path: str
    device_type: int
    device_index: int
    can_index: int
    timing0: int
    timing1: int


@dataclass
class ProtocolMessage:
    pgn: int
    data: bytes
    priority: int = DEFAULT_PRIORITY
    destination: int = GLOBAL_DESTINATION
    source_address: int | None = None


class CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),
        ("TimeStamp", ctypes.c_uint),
        ("TimeFlag", ctypes.c_ubyte),
        ("SendType", ctypes.c_ubyte),
        ("RemoteFlag", ctypes.c_ubyte),
        ("ExternFlag", ctypes.c_ubyte),
        ("DataLen", ctypes.c_ubyte),
        ("Data", ctypes.c_ubyte * 8),
        ("Reserved", ctypes.c_ubyte * 3),
    ]


class INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint),
        ("AccMask", ctypes.c_uint),
        ("Reserved", ctypes.c_uint),
        ("Filter", ctypes.c_ubyte),
        ("Timing0", ctypes.c_ubyte),
        ("Timing1", ctypes.c_ubyte),
        ("Mode", ctypes.c_ubyte),
    ]


class USBCANDevice:
    def __init__(self, config: DeviceConfig) -> None:
        self.config = config
        self.dll = ctypes.WinDLL(config.dll_path)
        self._bind_functions()

    def _bind_functions(self) -> None:
        self.dll.OpenDevice.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.dll.OpenDevice.restype = ctypes.c_uint
        self.dll.CloseDevice.argtypes = [ctypes.c_uint, ctypes.c_uint]
        self.dll.CloseDevice.restype = ctypes.c_uint
        self.dll.InitCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(INIT_CONFIG)]
        self.dll.InitCAN.restype = ctypes.c_uint
        self.dll.StartCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.dll.StartCAN.restype = ctypes.c_uint
        self.dll.Transmit.argtypes = [
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.POINTER(CAN_OBJ),
            ctypes.c_ulong,
        ]
        self.dll.Transmit.restype = ctypes.c_ulong

    def open(self) -> None:
        result = self.dll.OpenDevice(self.config.device_type, self.config.device_index, 0)
        if result == 0:
            raise RuntimeError("OpenDevice failed.")
        init_config = INIT_CONFIG(
            AccCode=0,
            AccMask=0xFFFFFFFF,
            Reserved=0,
            Filter=0,
            Timing0=self.config.timing0,
            Timing1=self.config.timing1,
            Mode=0,
        )
        if self.dll.InitCAN(
            self.config.device_type,
            self.config.device_index,
            self.config.can_index,
            ctypes.byref(init_config),
        ) == 0:
            raise RuntimeError("InitCAN failed.")
        if self.dll.StartCAN(self.config.device_type, self.config.device_index, self.config.can_index) == 0:
            raise RuntimeError("StartCAN failed.")

    def close(self) -> None:
        self.dll.CloseDevice(self.config.device_type, self.config.device_index)

    def send(self, frame_id: int, data: bytes) -> int:
        can_obj = CAN_OBJ()
        can_obj.ID = frame_id
        can_obj.TimeStamp = 0
        can_obj.TimeFlag = 0
        can_obj.SendType = 0
        can_obj.RemoteFlag = 0
        can_obj.ExternFlag = 1
        can_obj.DataLen = len(data)
        for index, value in enumerate(data):
            can_obj.Data[index] = value
        return int(
            self.dll.Transmit(
                self.config.device_type,
                self.config.device_index,
                self.config.can_index,
                ctypes.byref(can_obj),
                1,
            )
        )


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


def clamp_u16(value: float, scale: float, minimum: float = 0.0, maximum: float = 65535.0) -> int:
    scaled = int(round(value / scale))
    return max(int(minimum), min(int(maximum), scaled))


def le_u16(value: int) -> bytes:
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def build_address_claim(name: int) -> bytes:
    return name.to_bytes(8, byteorder="little", signed=False)


def set_name_manufacturer_code(name: int, manufacturer_code: int) -> int:
    # NMEA2000 NAME manufacturer code occupies 11 bits at bit position 21.
    code = max(0, min(0x7FF, int(manufacturer_code)))
    mask = 0x7FF << 21
    return (name & ~mask) | (code << 21)


def build_iso_request(requested_pgn: int) -> bytes:
    return bytes((requested_pgn & 0xFF, (requested_pgn >> 8) & 0xFF, (requested_pgn >> 16) & 0xFF)) + bytes((0xFF,) * 5)


def build_engine_rapid(engine_instance: int, engine_speed_rpm: float, engine_boost_bar: float, trim_percent: float) -> bytes:
    # PGN 127488 per NMEA2000: speed=0.25 rpm/bit, boost=100 Pa/bit, trim=1 %/bit (int8)
    speed_raw = clamp_u16(engine_speed_rpm, 0.25)
    boost_pa = engine_boost_bar * 100000.0
    boost_raw = clamp_u16(boost_pa, 100.0)
    trim_raw = max(-125, min(125, int(round(trim_percent))))
    trim_encoded = trim_raw & 0xFF
    return bytes(
        (
            engine_instance & 0xFF,
            speed_raw & 0xFF,
            (speed_raw >> 8) & 0xFF,
            boost_raw & 0xFF,
            (boost_raw >> 8) & 0xFF,
            trim_encoded,
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
    # PGN 127489 per NMEA2000:
    # Oil/Coolant pressure: 100 Pa/bit, Fuel pressure: 1000 Pa/bit
    # Oil temp: 0.1 K/bit, Coolant temp: 0.01 K/bit
    # Alternator: 0.01 V/bit, Fuel rate: 0.1 L/h/bit
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
    # Byte layout after fuel pressure is:
    # Reserved (1), Engine Discrete Status 1 (2), Engine Discrete Status 2 (2),
    # Percent Engine Load (1), Percent Engine Torque (1)
    data.extend((0xFF,))
    data.extend(le_u16(0xFFFF))
    data.extend(le_u16(0xFFFF))
    data.extend((load_raw, torque_raw))
    return bytes(data)


def build_product_info_payload(model_id: str, software_version: str, model_version: str, serial_code: str) -> bytes:
    # Simplified NMEA2000 Product Information payload (fast packet).
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
    return (
        int(max(0, min(0xFFFF, interval_ms))).to_bytes(2, byteorder="little", signed=False)
        + bytes((sequence_counter & 0xFF,))
        + bytes((0xFF,) * 5)
    )


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
    # PGN 127501: 1 byte bank instance + 28 x 2-bit switch status fields.
    # Status encoding used here: 0=Off, 1=On, 3=Unavailable.
    states_2bit = [(1 if state else 0) for state in switch_states[:12]]
    if len(states_2bit) < 28:
        states_2bit.extend([3] * (28 - len(states_2bit)))
    packed_states = _pack_2bit_values(states_2bit[:28], 7)
    return bytes((bank_instance & 0xFF,)) + packed_states


def build_group_function_binary_switch_command(bank_instance: int, switch_number: int, state_on: bool) -> bytes:
    # Simplified PGN 126208 (Command Group Function) command payload:
    # [FunctionCode=1, PGN(127501), NumberOfParams=3, BankInstance, SwitchNumber(1..12), Status(0/1)]
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

    first_room = 6
    first_chunk = payload[cursor : cursor + first_room]
    cursor += len(first_chunk)
    first_frame = bytes(((sid << 5) | frame_index, len(payload))) + first_chunk
    frames.append(first_frame)
    frame_index += 1

    while cursor < len(payload):
        chunk = payload[cursor : cursor + 7]
        cursor += len(chunk)
        frame = bytes((((sid << 5) | frame_index),)) + chunk
        frames.append(frame)
        frame_index += 1
    return frames


class SimulatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("NMEA2000 Engine Simulator (USB GCAN)")
        self.device: USBCANDevice | None = None
        self.send_job: str | None = None
        self.is_connected = False
        self.fast_packet_sequence = 0
        self.engine_heartbeat_sequence = 0
        self.switch_heartbeat_sequence = 0
        self.binary_switch_states = [False] * 12
        self.switch_buttons: list[ttk.Button] = []
        self._build_ui()

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        self.status_text = tk.StringVar(value="Status: Disconnected")
        ttk.Label(main, textvariable=self.status_text).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))

        ttk.Label(main, text="DLL path").grid(row=1, column=0, sticky="w")
        self.dll_path = tk.StringVar(value=DEFAULT_DLL_NAME)
        ttk.Entry(main, textvariable=self.dll_path).grid(row=1, column=1, columnspan=3, sticky="ew")

        ttk.Label(main, text="Source address").grid(row=2, column=0, sticky="w")
        self.source_address = tk.StringVar(value="0")
        ttk.Entry(main, textvariable=self.source_address).grid(row=2, column=1, sticky="ew")

        ttk.Label(main, text="Destination").grid(row=2, column=2, sticky="w")
        self.destination_address = tk.StringVar(value="255")
        ttk.Entry(main, textvariable=self.destination_address).grid(row=2, column=3, sticky="ew")

        ttk.Label(main, text="Engine instance").grid(row=3, column=0, sticky="w")
        self.engine_instance = tk.StringVar(value="0")
        ttk.Entry(main, textvariable=self.engine_instance).grid(row=3, column=1, sticky="ew")

        ttk.Label(main, text="Device NAME (hex)").grid(row=3, column=2, sticky="w")
        self.device_name = tk.StringVar(value="0x1F2000123456789A")
        ttk.Entry(main, textvariable=self.device_name).grid(row=3, column=3, sticky="ew")

        row = 4
        self.engine_speed_rpm = self._add_field(main, row, "Engine speed rpm", "750")
        self.engine_boost_bar = self._add_field(main, row, "Boost pressure bar", "1.0", col=2)
        row += 1
        self.trim_percent = self._add_field(main, row, "Engine trim %", "0")
        self.oil_pressure_bar = self._add_field(main, row, "Oil pressure bar", "3.5", col=2)
        row += 1
        self.oil_temp_c = self._add_field(main, row, "Oil temp °C", "85")
        self.coolant_temp_c = self._add_field(main, row, "Coolant temp °C", "78", col=2)
        row += 1
        self.alternator_v = self._add_field(main, row, "Alternator V", "13.8")
        self.fuel_rate_lph = self._add_field(main, row, "Fuel rate L/h", "12", col=2)
        row += 1
        self.engine_hours_h = self._add_field(main, row, "Engine hours", "500")
        self.coolant_pressure_bar = self._add_field(main, row, "Coolant pressure bar", "1.2", col=2)
        row += 1
        self.fuel_pressure_psi = self._add_field(main, row, "Fuel pressure PSI", "43.5")
        self.engine_load_percent = self._add_field(main, row, "Engine load %", "35", col=2)
        row += 1
        self.engine_torque_percent = self._add_field(main, row, "Engine torque %", "42")
        self.iso_request_pgn = self._add_field(main, row, "ISO request PGN", str(PGN_ADDRESS_CLAIM), col=2)
        row += 1

        ttk.Label(main, text="Product model").grid(row=row, column=0, sticky="w")
        self.product_model = tk.StringVar(value="GCAN Engine Sim")
        ttk.Entry(main, textvariable=self.product_model).grid(row=row, column=1, sticky="ew")
        ttk.Label(main, text="Software version").grid(row=row, column=2, sticky="w")
        self.software_version = tk.StringVar(value="1.0.0")
        ttk.Entry(main, textvariable=self.software_version).grid(row=row, column=3, sticky="ew")
        row += 1

        ttk.Label(main, text="Model version").grid(row=row, column=0, sticky="w")
        self.model_version = tk.StringVar(value="1.0")
        ttk.Entry(main, textvariable=self.model_version).grid(row=row, column=1, sticky="ew")
        ttk.Label(main, text="Serial code").grid(row=row, column=2, sticky="w")
        self.serial_code = tk.StringVar(value="SIM-0001")
        ttk.Entry(main, textvariable=self.serial_code).grid(row=row, column=3, sticky="ew")

        row += 1
        ttk.Label(main, text="Interval ms").grid(row=row, column=2, sticky="w")
        self.interval_ms = tk.IntVar(value=100)
        ttk.Entry(main, textvariable=self.interval_ms).grid(row=row, column=3, sticky="ew")
        row += 1

        switch_node = ttk.LabelFrame(main, text="Virtual switch node identity (2nd device)", padding=8)
        switch_node.grid(row=row, column=0, columnspan=4, sticky="ew", pady=(2, 6))
        ttk.Label(switch_node, text="Switch node source").grid(row=0, column=0, sticky="w")
        self.switch_node_source_address = tk.StringVar(value="100")
        ttk.Entry(switch_node, textvariable=self.switch_node_source_address, width=10).grid(row=0, column=1, sticky="w")
        ttk.Label(switch_node, text="Switch node NAME (hex)").grid(row=0, column=2, sticky="w")
        self.switch_node_device_name = tk.StringVar(value="0x1F2000AA12345678")
        ttk.Entry(switch_node, textvariable=self.switch_node_device_name).grid(row=0, column=3, sticky="ew")
        ttk.Label(switch_node, text="Switch manufacturer code").grid(row=1, column=0, sticky="w")
        self.switch_manufacturer_code = tk.StringVar(value="176")
        ttk.Entry(switch_node, textvariable=self.switch_manufacturer_code, width=10).grid(row=1, column=1, sticky="w")

        ttk.Label(switch_node, text="Switch model").grid(row=1, column=2, sticky="w")
        self.switch_product_model = tk.StringVar(value="CKM12")
        ttk.Entry(switch_node, textvariable=self.switch_product_model, width=18).grid(row=1, column=3, sticky="ew")
        ttk.Label(switch_node, text="Switch software").grid(row=2, column=0, sticky="w")
        self.switch_software_version = tk.StringVar(value="2.02.08")
        ttk.Entry(switch_node, textvariable=self.switch_software_version, width=18).grid(row=2, column=1, sticky="ew")

        ttk.Label(switch_node, text="Switch model version").grid(row=2, column=2, sticky="w")
        self.switch_model_version = tk.StringVar(value="Rev A")
        ttk.Entry(switch_node, textvariable=self.switch_model_version, width=18).grid(row=2, column=3, sticky="ew")
        ttk.Label(switch_node, text="Switch serial").grid(row=3, column=0, sticky="w")
        self.switch_serial_code = tk.StringVar(value="1606029")
        ttk.Entry(switch_node, textvariable=self.switch_serial_code, width=18).grid(row=3, column=1, sticky="ew")
        row += 1

        enabled = ttk.LabelFrame(main, text="Enabled messages", padding=8)
        enabled.grid(row=row, column=0, columnspan=4, sticky="ew", pady=(8, 6))
        self.address_claim_enabled = tk.BooleanVar(value=True)
        self.iso_request_enabled = tk.BooleanVar(value=True)
        self.product_info_enabled = tk.BooleanVar(value=True)
        self.heartbeat_enabled = tk.BooleanVar(value=True)
        self.engine_rapid_enabled = tk.BooleanVar(value=True)
        self.engine_dynamic_enabled = tk.BooleanVar(value=True)
        self.switch_address_claim_enabled = tk.BooleanVar(value=True)
        self.switch_product_info_enabled = tk.BooleanVar(value=True)
        self.switch_heartbeat_enabled = tk.BooleanVar(value=True)
        self.binary_switch_status_enabled = tk.BooleanVar(value=True)
        ttk.Checkbutton(enabled, text="ISO Address Claim", variable=self.address_claim_enabled).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(enabled, text="ISO Request", variable=self.iso_request_enabled).grid(row=0, column=1, sticky="w")
        ttk.Checkbutton(enabled, text="Product Info", variable=self.product_info_enabled).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(enabled, text="Heartbeat", variable=self.heartbeat_enabled).grid(row=1, column=1, sticky="w")
        ttk.Checkbutton(enabled, text="Engine Rapid PGN 127488", variable=self.engine_rapid_enabled).grid(row=2, column=0, sticky="w")
        ttk.Checkbutton(enabled, text="Engine Dynamic PGN 127489", variable=self.engine_dynamic_enabled).grid(row=2, column=1, sticky="w")
        ttk.Checkbutton(enabled, text="Switch node Address Claim", variable=self.switch_address_claim_enabled).grid(
            row=3, column=0, sticky="w"
        )
        ttk.Checkbutton(enabled, text="Switch node Product Info", variable=self.switch_product_info_enabled).grid(
            row=3, column=1, sticky="w"
        )
        ttk.Checkbutton(enabled, text="Switch node Heartbeat", variable=self.switch_heartbeat_enabled).grid(
            row=4, column=0, sticky="w"
        )
        ttk.Checkbutton(
            enabled, text="Binary Switch Bank Status PGN 127501", variable=self.binary_switch_status_enabled
        ).grid(row=4, column=1, columnspan=2, sticky="w")
        row += 1

        switch_frame = ttk.LabelFrame(main, text="Binary Switch Bank (1-12 pushbuttons)", padding=8)
        switch_frame.grid(row=row, column=0, columnspan=4, sticky="ew", pady=(4, 6))
        ttk.Label(switch_frame, text="Bank instance").grid(row=0, column=0, sticky="w", padx=(0, 4))
        self.binary_switch_bank_instance = tk.StringVar(value="52")
        ttk.Entry(switch_frame, textvariable=self.binary_switch_bank_instance, width=8).grid(row=0, column=1, sticky="w")
        ttk.Label(switch_frame, text="Write with PGN 126208 on button press").grid(row=0, column=2, columnspan=4, sticky="w")
        for index in range(12):
            button = ttk.Button(
                switch_frame,
                text=f"SW {index + 1}: RELEASED",
                width=12,
            )
            button.bind("<ButtonPress-1>", lambda _event, switch_no=index + 1: self.on_switch_press(switch_no))
            button.bind("<ButtonRelease-1>", lambda _event, switch_no=index + 1: self.on_switch_release(switch_no))
            button.grid(row=1 + (index // 6), column=index % 6, padx=2, pady=2, sticky="ew")
            self.switch_buttons.append(button)
        row += 1

        buttons = ttk.Frame(main)
        buttons.grid(row=row, column=0, columnspan=4, pady=8, sticky="ew")
        self.connect_button = ttk.Button(buttons, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=0, padx=4)
        self.disconnect_button = ttk.Button(buttons, text="Disconnect", command=self.disconnect)
        self.disconnect_button.grid(row=0, column=1, padx=4)
        self.send_once_button = ttk.Button(buttons, text="Send Once", command=self.send_once)
        self.send_once_button.grid(row=0, column=2, padx=4)
        self.start_button = ttk.Button(buttons, text="Start Periodic", command=self.start_periodic)
        self.start_button.grid(row=0, column=3, padx=4)
        self.stop_button = ttk.Button(buttons, text="Stop Periodic", command=self.stop_periodic)
        self.stop_button.grid(row=0, column=4, padx=4)

        self._update_button_states()
        self._refresh_switch_button_labels()

    def _add_field(self, parent: ttk.Frame, row: int, label: str, default: str, col: int = 0) -> tk.StringVar:
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="w")
        value = tk.StringVar(value=default)
        ttk.Entry(parent, textvariable=value).grid(row=row, column=col + 1, sticky="ew")
        return value

    def _as_int(self, value: str, default: int = 0) -> int:
        try:
            if value.strip().lower().startswith("0x"):
                return int(value.strip(), 16)
            return int(float(value.strip()))
        except ValueError:
            return default

    def _as_float(self, value: str, default: float = 0.0) -> float:
        try:
            return float(value.strip())
        except ValueError:
            return default

    def _source_address(self) -> int:
        return max(0, min(251, self._as_int(self.source_address.get(), 0)))

    def _switch_source_address(self) -> int:
        return max(0, min(251, self._as_int(self.switch_node_source_address.get(), 100)))

    def _destination(self) -> int:
        return max(0, min(255, self._as_int(self.destination_address.get(), 255)))

    def _device_name(self) -> int:
        value = self.device_name.get().strip()
        try:
            return int(value, 16) if value.lower().startswith("0x") else int(value)
        except ValueError:
            return 0x1F2000123456789A

    def _switch_device_name(self) -> int:
        value = self.switch_node_device_name.get().strip()
        try:
            name = int(value, 16) if value.lower().startswith("0x") else int(value)
        except ValueError:
            name = 0x1F2000AA12345678
        manufacturer = self._as_int(self.switch_manufacturer_code.get(), 176)
        return set_name_manufacturer_code(name, manufacturer)

    def _binary_switch_bank_instance(self) -> int:
        return max(0, min(255, self._as_int(self.binary_switch_bank_instance.get(), 52)))

    def _refresh_switch_button_labels(self) -> None:
        for index, button in enumerate(self.switch_buttons, start=1):
            state_text = "PRESSED" if self.binary_switch_states[index - 1] else "RELEASED"
            button.configure(text=f"SW {index}: {state_text}")

    def _send_switch_command(self, switch_number: int, state_on: bool) -> None:
        if not self.device:
            return
        payload = build_group_function_binary_switch_command(
            self._binary_switch_bank_instance(),
            switch_number,
            state_on,
        )
        frame_id = nmea2000_id(DEFAULT_PRIORITY, PGN_GROUP_FUNCTION, self._source_address(), self._destination())
        self.device.send(frame_id, payload)

    def on_switch_press(self, switch_number: int) -> None:
        switch_index = max(1, min(12, switch_number)) - 1
        if self.binary_switch_states[switch_index]:
            return
        self.binary_switch_states[switch_index] = True
        self._refresh_switch_button_labels()
        self._send_switch_command(switch_number, True)

    def on_switch_release(self, switch_number: int) -> None:
        switch_index = max(1, min(12, switch_number)) - 1
        if not self.binary_switch_states[switch_index]:
            return
        self.binary_switch_states[switch_index] = False
        self._refresh_switch_button_labels()
        self._send_switch_command(switch_number, False)

    def resolve_dll_path(self) -> str:
        path = self.dll_path.get().strip() or DEFAULT_DLL_NAME
        return os.path.abspath(path)

    def connect(self) -> None:
        if platform.system() != "Windows":
            messagebox.showerror("Unsupported OS", "This simulator requires Windows because it loads ECanVci.dll.")
            return
        try:
            config = DeviceConfig(
                dll_path=self.resolve_dll_path(),
                device_type=DEFAULT_DEVICE_TYPE,
                device_index=DEFAULT_DEVICE_INDEX,
                can_index=DEFAULT_CAN_INDEX,
                timing0=TIMING0_250K,
                timing1=TIMING1_250K,
            )
            self.device = USBCANDevice(config)
            self.device.open()
            self.is_connected = True
            self.status_text.set(f"Status: Connected ({config.dll_path})")
            self.send_once()
        except Exception as exc:
            self.device = None
            self.is_connected = False
            messagebox.showerror("Connection error", str(exc))
        self._update_button_states()

    def disconnect(self) -> None:
        self.stop_periodic()
        if self.device:
            try:
                self.device.close()
            except Exception:
                pass
        self.device = None
        self.is_connected = False
        self.status_text.set("Status: Disconnected")
        self._update_button_states()

    def _send_protocol_messages(self) -> None:
        if not self.device:
            return
        for frame_id, data in self.current_frames():
            self.device.send(frame_id, data)

    def send_once(self) -> None:
        self._send_protocol_messages()

    def start_periodic(self) -> None:
        if not self.device or self.send_job is not None:
            return
        self._schedule_send()
        self._update_button_states()

    def stop_periodic(self) -> None:
        if self.send_job is not None:
            self.root.after_cancel(self.send_job)
            self.send_job = None
        self._update_button_states()

    def _schedule_send(self) -> None:
        try:
            interval = max(10, int(self.interval_ms.get()))
        except tk.TclError:
            interval = 100
        self.send_job = self.root.after(interval, self._send_and_reschedule)

    def _send_and_reschedule(self) -> None:
        self._send_protocol_messages()
        self._schedule_send()

    def _expand_protocol_message(self, message: ProtocolMessage) -> list[tuple[int, bytes]]:
        source = self._source_address() if message.source_address is None else message.source_address
        if len(message.data) <= 8:
            frame_id = nmea2000_id(message.priority, message.pgn, source, message.destination)
            return [(frame_id, message.data)]

        frames = split_fast_packet(message.data, self.fast_packet_sequence)
        self.fast_packet_sequence = (self.fast_packet_sequence + 1) & 0x07
        frame_id = nmea2000_id(message.priority, message.pgn, source, message.destination)
        return [(frame_id, frame.ljust(8, b"\xFF")) for frame in frames]

    def current_messages(self) -> list[ProtocolMessage]:
        destination = self._destination()
        engine_instance = max(0, min(255, self._as_int(self.engine_instance.get(), 0)))

        messages: list[ProtocolMessage] = []
        if self.address_claim_enabled.get():
            messages.append(
                ProtocolMessage(PGN_ADDRESS_CLAIM, build_address_claim(self._device_name()), 6, GLOBAL_DESTINATION, self._source_address())
            )
        if self.iso_request_enabled.get():
            request_pgn = max(0, min(0x3FFFF, self._as_int(self.iso_request_pgn.get(), PGN_ADDRESS_CLAIM)))
            messages.append(ProtocolMessage(PGN_ISO_REQUEST, build_iso_request(request_pgn), 6, destination))
        if self.product_info_enabled.get():
            payload = build_product_info_payload(
                self.product_model.get(),
                self.software_version.get(),
                self.model_version.get(),
                self.serial_code.get(),
            )
            messages.append(ProtocolMessage(PGN_PRODUCT_INFO, payload, 6, GLOBAL_DESTINATION))
        if self.heartbeat_enabled.get():
            heartbeat_interval = max(0, self._as_int(str(self.interval_ms.get()), 100))
            payload = build_heartbeat_payload(heartbeat_interval, self.engine_heartbeat_sequence)
            self.engine_heartbeat_sequence = (self.engine_heartbeat_sequence + 1) & 0xFF
            messages.append(ProtocolMessage(PGN_HEARTBEAT, payload, 7, GLOBAL_DESTINATION, self._source_address()))
        if self.engine_rapid_enabled.get():
            messages.append(
                ProtocolMessage(
                    PGN_ENGINE_RAPID,
                    build_engine_rapid(
                        engine_instance,
                        self._as_float(self.engine_speed_rpm.get(), 0.0),
                        self._as_float(self.engine_boost_bar.get(), 1.0),
                        self._as_float(self.trim_percent.get(), 0.0),
                    ),
                    2,
                    GLOBAL_DESTINATION,
                    self._source_address(),
                )
            )
        if self.engine_dynamic_enabled.get():
            messages.append(
                ProtocolMessage(
                    PGN_ENGINE_DYNAMIC,
                    build_engine_dynamic(
                        engine_instance,
                        self._as_float(self.oil_pressure_bar.get(), 0.0),
                        self._as_float(self.oil_temp_c.get(), 0.0),
                        self._as_float(self.coolant_temp_c.get(), 0.0),
                        self._as_float(self.alternator_v.get(), 0.0),
                        self._as_float(self.fuel_rate_lph.get(), 0.0),
                        self._as_float(self.engine_hours_h.get(), 0.0),
                        self._as_float(self.coolant_pressure_bar.get(), 0.0),
                        self._as_float(self.fuel_pressure_psi.get(), 0.0) * 0.0689476,
                        self._as_float(self.engine_load_percent.get(), 0.0),
                        self._as_float(self.engine_torque_percent.get(), 0.0),
                    ),
                    2,
                    GLOBAL_DESTINATION,
                    self._source_address(),
                )
            )
        if self.switch_address_claim_enabled.get():
            messages.append(
                ProtocolMessage(
                    PGN_ADDRESS_CLAIM,
                    build_address_claim(self._switch_device_name()),
                    6,
                    GLOBAL_DESTINATION,
                    self._switch_source_address(),
                )
            )
        if self.switch_product_info_enabled.get():
            payload = build_product_info_payload(
                self.switch_product_model.get(),
                self.switch_software_version.get(),
                self.switch_model_version.get(),
                self.switch_serial_code.get(),
            )
            messages.append(ProtocolMessage(PGN_PRODUCT_INFO, payload, 6, GLOBAL_DESTINATION, self._switch_source_address()))
        if self.switch_heartbeat_enabled.get():
            heartbeat_interval = max(0, self._as_int(str(self.interval_ms.get()), 100))
            payload = build_heartbeat_payload(heartbeat_interval, self.switch_heartbeat_sequence)
            self.switch_heartbeat_sequence = (self.switch_heartbeat_sequence + 1) & 0xFF
            messages.append(ProtocolMessage(PGN_HEARTBEAT, payload, 7, GLOBAL_DESTINATION, self._switch_source_address()))
        if self.binary_switch_status_enabled.get():
            messages.append(
                ProtocolMessage(
                    PGN_BINARY_SWITCH_BANK_STATUS,
                    build_binary_switch_bank_status(self._binary_switch_bank_instance(), self.binary_switch_states),
                    3,
                    GLOBAL_DESTINATION,
                    self._switch_source_address(),
                )
            )
        return messages

    def current_frames(self) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        for message in self.current_messages():
            frames.extend(self._expand_protocol_message(message))
        return frames

    def _update_button_states(self) -> None:
        if self.is_connected:
            self.connect_button.state(["disabled"])
            self.disconnect_button.state(["!disabled"])
            self.send_once_button.state(["!disabled"])
            if self.send_job is None:
                self.start_button.state(["!disabled"])
                self.stop_button.state(["disabled"])
            else:
                self.start_button.state(["disabled"])
                self.stop_button.state(["!disabled"])
        else:
            self.connect_button.state(["!disabled"])
            self.disconnect_button.state(["disabled"])
            self.send_once_button.state(["disabled"])
            self.start_button.state(["disabled"])
            self.stop_button.state(["disabled"])


def main() -> None:
    root = tk.Tk()
    SimulatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
