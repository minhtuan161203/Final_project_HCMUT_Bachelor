"""Background serial transport worker."""

from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time

from qt_compat import QtCore, Signal
from protocol import format_hex

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - handled gracefully at runtime
    serial = None
    list_ports = None


@dataclass(slots=True)
class PortDescriptor:
    port_name: str
    description: str
    hwid: str
    vid: int | None
    pid: int | None

    @property
    def display_name(self) -> str:
        if self.vid is not None and self.pid is not None:
            return f"{self.port_name} - {self.description} [{self.vid:04X}:{self.pid:04X}]"
        return f"{self.port_name} - {self.description}"


def list_serial_ports() -> list[PortDescriptor]:
    if list_ports is None:
        return []

    ports: list[PortDescriptor] = []
    for port in list_ports.comports():
        ports.append(
            PortDescriptor(
                port_name=port.device,
                description=port.description or "Unknown device",
                hwid=port.hwid,
                vid=port.vid,
                pid=port.pid,
            )
        )
    return ports


class SerialWorker(QtCore.QThread):
    frame_received = Signal(object)
    status_changed = Signal(bool, str)
    log_message = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, parser, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self._stop_event = threading.Event()
        self._open_request: tuple[str, int] | None = None
        self._close_requested = False
        self._lock = threading.Lock()
        self._tx_queue: queue.Queue[bytes] = queue.Queue()
        self._parser = parser
        self._serial_port = None

    def request_open(self, port_name: str, baudrate: int = 115200) -> None:
        with self._lock:
            self._open_request = (port_name, baudrate)
            self._close_requested = False
        if not self.isRunning():
            self.start()

    def request_close(self) -> None:
        with self._lock:
            self._close_requested = True

    def queue_tx(self, data: bytes) -> None:
        self._tx_queue.put(bytes(data))

    def stop(self) -> None:
        self._stop_event.set()
        self.request_close()
        self.wait(1500)

    def run(self) -> None:
        if serial is None:
            self.error_occurred.emit("Missing dependency: pyserial")
            return

        while not self._stop_event.is_set():
            self._process_connection_requests()

            if self._serial_port is not None and self._serial_port.is_open:
                self._drain_tx_queue()
                self._read_rx_bytes()
            else:
                time.sleep(0.05)

        self._close_port()

    def _process_connection_requests(self) -> None:
        open_request = None
        close_requested = False
        with self._lock:
            if self._open_request is not None:
                open_request = self._open_request
                self._open_request = None
            close_requested = self._close_requested
            self._close_requested = False

        if close_requested:
            self._close_port()
        if open_request is not None:
            self._open_port(*open_request)

    def _open_port(self, port_name: str, baudrate: int) -> None:
        self._close_port()
        try:
            self._serial_port = serial.Serial(
                port=port_name,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                timeout=0.02,
                write_timeout=0.2,
            )
            self._serial_port.reset_input_buffer()
            self._serial_port.reset_output_buffer()
        except Exception as exc:  # pragma: no cover - depends on host machine
            self._serial_port = None
            self.error_occurred.emit(f"Open port failed: {exc}")
            self.status_changed.emit(False, "")
            return

        self._parser.reset()
        self.log_message.emit(f"Connected to {port_name} @ {baudrate}")
        self.status_changed.emit(True, port_name)

    def _close_port(self) -> None:
        if self._serial_port is None:
            return

        port_name = self._serial_port.port
        try:
            if self._serial_port.is_open:
                self._serial_port.close()
        except Exception:
            pass
        self._serial_port = None
        self._parser.reset()
        self.log_message.emit(f"Disconnected from {port_name}")
        self.status_changed.emit(False, "")

    def _drain_tx_queue(self) -> None:
        while True:
            try:
                payload = self._tx_queue.get_nowait()
            except queue.Empty:
                break

            try:
                self._serial_port.write(payload)
                self._serial_port.flush()
                self.log_message.emit(f"TX {format_hex(payload)}")
            except Exception as exc:  # pragma: no cover - depends on host machine
                self.error_occurred.emit(f"Write failed: {exc}")
                self._close_port()
                break

    def _read_rx_bytes(self) -> None:
        try:
            waiting = self._serial_port.in_waiting
            data = self._serial_port.read(waiting or 1)
        except Exception as exc:  # pragma: no cover - depends on host machine
            self.error_occurred.emit(f"Read failed: {exc}")
            self._close_port()
            return

        if not data:
            return

        for frame in self._parser.feed(data):
            self.log_message.emit(f"RX {format_hex(frame.raw)}")
            self.frame_received.emit(frame)
