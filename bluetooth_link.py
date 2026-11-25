"""Windows Classic Bluetooth SPP discovery, PIN pairing, and RFCOMM socket transport."""
from collections import deque
import ctypes
from ctypes import wintypes
from dataclasses import dataclass
import os
from queue import Empty, Queue
import re
import socket
from threading import Event, Lock, Thread
from time import monotonic

from serial.tools import list_ports

_BT_DLL = None
_BT_INIT_TRIED = False


@dataclass
class BluetoothDevice:
    name: str
    address: str
    authenticated: bool
    remembered: bool
    connected: bool
    com_port: str | None = None


def format_bt_address(raw_u64: int) -> str:
    return ":".join(f"{(raw_u64 >> (8 * i)) & 0xFF:02X}" for i in reversed(range(6)))


def parse_bt_address(address: str) -> int:
    cleaned = re.sub(r"[^0-9A-Fa-f]", "", str(address or ""))
    if len(cleaned) != 12:
        raise ValueError(f"无效蓝牙 MAC 地址：{address}")
    return int(cleaned, 16)


def extract_bt_mac_from_hwid(hwid: str) -> str | None:
    """Extract normalized AA:BB:CC:DD:EE:FF from Windows BTHENUM HWID, ignoring incoming 000000000000."""
    text = str(hwid or "")
    if "BTHENUM\\" not in text.upper():
        return None
    m = re.search(r"&([0-9A-Fa-f]{12})_", text)
    if not m:
        return None
    raw = m.group(1).upper()
    if raw == "000000000000":
        return None
    return ":".join(raw[i : i + 2] for i in range(0, 12, 2))


def map_bluetooth_com_ports(ports=None) -> dict[str, str]:
    """Return {MAC_ADDRESS: COM_PORT} for paired outgoing Bluetooth SPP virtual ports."""
    mapping: dict[str, str] = {}
    try:
        items = list(ports) if ports is not None else list(list_ports.comports())
    except OSError:
        return mapping
    for p in items:
        hwid = getattr(p, "hwid", "") or ""
        mac = extract_bt_mac_from_hwid(hwid)
        dev = getattr(p, "device", "") or ""
        if mac and dev and mac not in mapping:
            mapping[mac] = dev
    return mapping


class _SYSTEMTIME(ctypes.Structure):
    _fields_ = [
        ("wYear", wintypes.WORD),
        ("wMonth", wintypes.WORD),
        ("wDayOfWeek", wintypes.WORD),
        ("wDay", wintypes.WORD),
        ("wHour", wintypes.WORD),
        ("wMinute", wintypes.WORD),
        ("wSecond", wintypes.WORD),
        ("wMilliseconds", wintypes.WORD),
    ]


class _BLUETOOTH_DEVICE_SEARCH_PARAMS(ctypes.Structure):
    _fields_ = [
        ("dwSize", wintypes.DWORD),
        ("fReturnAuthenticated", wintypes.BOOL),
        ("fReturnRemembered", wintypes.BOOL),
        ("fReturnUnknown", wintypes.BOOL),
        ("fReturnConnected", wintypes.BOOL),
        ("fIssueInquiry", wintypes.BOOL),
        ("cTimeoutMultiplier", ctypes.c_ubyte),
        ("hRadio", wintypes.HANDLE),
    ]


class _BLUETOOTH_DEVICE_INFO(ctypes.Structure):
    _fields_ = [
        ("dwSize", wintypes.DWORD),
        ("Address", ctypes.c_uint64),
        ("ulClassofDevice", wintypes.ULONG),
        ("fConnected", wintypes.BOOL),
        ("fRemembered", wintypes.BOOL),
        ("fAuthenticated", wintypes.BOOL),
        ("stLastSeen", _SYSTEMTIME),
        ("stLastUsed", _SYSTEMTIME),
        ("szName", wintypes.WCHAR * 248),
    ]


def _get_bt_dll():
    global _BT_DLL, _BT_INIT_TRIED
    if _BT_INIT_TRIED:
        return _BT_DLL
    _BT_INIT_TRIED = True
    if os.name != "nt":
        return None
    for dll_name in ("bthprops.cpl", "BluetoothAPIs.dll"):
        try:
            dll = ctypes.WinDLL(dll_name)
        except OSError:
            continue
        if not hasattr(dll, "BluetoothFindFirstDevice"):
            continue
        dll.BluetoothFindFirstDevice.restype = wintypes.HANDLE
        dll.BluetoothFindFirstDevice.argtypes = [
            ctypes.POINTER(_BLUETOOTH_DEVICE_SEARCH_PARAMS),
            ctypes.POINTER(_BLUETOOTH_DEVICE_INFO),
        ]
        dll.BluetoothFindNextDevice.restype = wintypes.BOOL
        dll.BluetoothFindNextDevice.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(_BLUETOOTH_DEVICE_INFO),
        ]
        dll.BluetoothFindDeviceClose.restype = wintypes.BOOL
        dll.BluetoothFindDeviceClose.argtypes = [wintypes.HANDLE]
        if hasattr(dll, "BluetoothAuthenticateDevice"):
            dll.BluetoothAuthenticateDevice.restype = wintypes.DWORD
            dll.BluetoothAuthenticateDevice.argtypes = [
                wintypes.HWND,
                wintypes.HANDLE,
                ctypes.POINTER(_BLUETOOTH_DEVICE_INFO),
                wintypes.LPWSTR,
                wintypes.ULONG,
            ]
        _BT_DLL = dll
        break
    return _BT_DLL


def discover_bluetooth_devices(inquiry: bool = True, timeout_units: int = 3) -> list[BluetoothDevice]:
    """Enumerate cached and optionally live-inquired Classic Bluetooth devices on Windows."""
    dll = _get_bt_dll()
    if dll is None:
        return []

    com_map = map_bluetooth_com_ports()
    sp = _BLUETOOTH_DEVICE_SEARCH_PARAMS()
    sp.dwSize = ctypes.sizeof(sp)
    sp.fReturnAuthenticated = True
    sp.fReturnRemembered = True
    sp.fReturnUnknown = True
    sp.fReturnConnected = True
    sp.fIssueInquiry = bool(inquiry)
    sp.cTimeoutMultiplier = max(1, min(15, int(timeout_units)))
    sp.hRadio = None

    di = _BLUETOOTH_DEVICE_INFO()
    di.dwSize = ctypes.sizeof(di)

    handle = dll.BluetoothFindFirstDevice(ctypes.byref(sp), ctypes.byref(di))
    if not handle:
        return []

    by_mac: dict[str, BluetoothDevice] = {}
    try:
        while True:
            mac = format_bt_address(di.Address)
            clean_name = (di.szName or "").replace("\r", "").replace("\n", "").strip() or "未知设备"
            by_mac[mac] = BluetoothDevice(
                name=clean_name,
                address=mac,
                authenticated=bool(di.fAuthenticated),
                remembered=bool(di.fRemembered),
                connected=bool(di.fConnected),
                com_port=com_map.get(mac),
            )
            di = _BLUETOOTH_DEVICE_INFO()
            di.dwSize = ctypes.sizeof(di)
            if not dll.BluetoothFindNextDevice(handle, ctypes.byref(di)):
                break
    finally:
        dll.BluetoothFindDeviceClose(handle)

    def sort_key(dev: BluetoothDevice):
        low = dev.name.lower()
        is_serial_bt = any(tok in low for tok in ("hc-05", "hc-06", "spp", "car", "blue", "esp32", "bt", "jd"))
        return (
            0 if is_serial_bt else 1,
            0 if dev.com_port else 1,
            0 if dev.authenticated else 1,
            dev.name.lower(),
            dev.address,
        )

    return sorted(by_mac.values(), key=sort_key)


def pair_bluetooth_device(address: str, pin: str = "1234", hwnd: int = 0) -> tuple[bool, str]:
    """Initiate Windows Bluetooth pairing with optional PIN code (e.g. '1234' for HC-05)."""
    dll = _get_bt_dll()
    if dll is None or not hasattr(dll, "BluetoothAuthenticateDevice"):
        return False, "当前系统不支持 Windows 蓝牙配对 API（请在系统设置配对）"
    try:
        addr_u64 = parse_bt_address(address)
    except ValueError as exc:
        return False, str(exc)

    di = _BLUETOOTH_DEVICE_INFO()
    di.dwSize = ctypes.sizeof(di)
    di.Address = addr_u64

    clean_pin = str(pin or "").strip()
    if clean_pin:
        buf = ctypes.create_unicode_buffer(clean_pin)
        rc = int(dll.BluetoothAuthenticateDevice(wintypes.HWND(hwnd or 0), None, ctypes.byref(di), buf, len(clean_pin)))
    else:
        rc = int(dll.BluetoothAuthenticateDevice(wintypes.HWND(hwnd or 0), None, ctypes.byref(di), None, 0))

    if rc == 0:
        return True, f"已与 {address} 配对成功（可在列表点【蓝牙直连】或使用映射 COM 口）"
    reasons = {
        259: "目标设备未响应或已超出通信距离（请确认模块红灯快闪处于可发现状态）",
        1167: "本机蓝牙适配器未开启或设备已离线",
        1223: "用户取消了系统配对窗口",
        1244: "PIN 配对码错误或设备拒绝认证（HC-05 默认 1234 / 0000）",
    }
    return False, reasons.get(rc, f"配对失败 (Windows 错误码 {rc})")


def open_windows_bluetooth_settings() -> None:
    if os.name == "nt":
        os.startfile("ms-settings:bluetooth")


class BluetoothRfcommLink(Thread):
    """Background Classic Bluetooth SPP (RFCOMM) link matching SerialLink's queue/event interface."""

    def __init__(self, address: str, channel: int = 1, name: str = "", timeout: float = 5.0):
        super().__init__(daemon=True)
        self.address = address
        self.channel = int(channel)
        self.device_name = name or address
        self.timeout = float(timeout)
        self.closed = Event()
        self.closing_after_flush = Event()
        self.out: Queue[bytes] = Queue()
        self.events: deque = deque(maxlen=1500)
        self.dropped = 0
        self.lock = Lock()

    def event(self, kind: str, data):
        with self.lock:
            if len(self.events) == self.events.maxlen:
                self.dropped += 1
            self.events.append((kind, monotonic(), data))

    def send(self, data: bytes):
        if data:
            self.out.put(bytes(data))

    def take(self):
        with self.lock:
            items = list(self.events)
            dropped = self.dropped
            self.events.clear()
            self.dropped = 0
        return items, dropped

    def flush_and_close(self, final_data: bytes = b"", timeout: float = 0.25):
        if final_data:
            self.out.put(bytes(final_data))
        self.closing_after_flush.set()
        deadline = monotonic() + max(0.05, float(timeout))
        while monotonic() < deadline and not self.out.empty() and self.is_alive():
            self.join(0.02)
        self.closed.set()
        self.join(max(0.05, min(0.25, float(timeout))))

    def run(self):
        if not hasattr(socket, "AF_BLUETOOTH") or not hasattr(socket, "BTPROTO_RFCOMM"):
            self.event("error", "当前 Python 运行环境不支持 AF_BLUETOOTH RFCOMM 直连")
            self.event("closed", None)
            return
        sock = None
        try:
            sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            sock.settimeout(self.timeout)
            sock.connect((self.address, self.channel))
            sock.settimeout(0.03)
            label = f"{self.device_name} ({self.address})"
            self.event("open", f"蓝牙 {label}")
            while not self.closed.is_set():
                try:
                    while True:
                        chunk = self.out.get_nowait()
                        if chunk:
                            sock.sendall(chunk)
                            self.event("tx", chunk)
                except Empty:
                    pass
                if self.closing_after_flush.is_set() and self.out.empty():
                    break
                try:
                    data = sock.recv(1024)
                    if not data:
                        self.event("error", "蓝牙对端已断开连接")
                        break
                    self.event("rx", data)
                except TimeoutError:
                    pass
                except OSError as exc:
                    if not self.closed.is_set():
                        self.event("error", f"蓝牙接收中断：{exc}")
                    break
        except OSError as exc:
            self.event("error", f"蓝牙连接失败 ({self.address})：{exc}")
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
            self.event("closed", None)
