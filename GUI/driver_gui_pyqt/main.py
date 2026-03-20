"""Basic desktop GUI for ASD04 USB communication."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import struct
import sys
import time

from qt_compat import QtCore, QtGui, QtWidgets, QT_API
from protocol import (
    ACK_ERROR,
    ACK_NOERROR,
    Command,
    DRIVER_PARAMETER_NAMES,
    MOTOR_PARAMETER_NAMES,
    ParsedFrame,
    UpdateCode,
    FrameStreamParser,
    build_command_frame,
    build_parameter_write_chunks,
    parse_error_payload,
    parse_monitor_payload,
    parse_parameter_chunk,
)
from transport import PortDescriptor, SerialWorker, list_serial_ports


KNOWN_DEVICE_VID = 1100
KNOWN_DEVICE_PID = 22336
TREND_BUFFER_CAPACITY = 4000
TREND_REFRESH_INTERVAL_MS = 33

TREND_SERIES_META = {
    "phase_u": {"label": "Iu", "unit": "A", "color": "#1f77b4"},
    "phase_v": {"label": "Iv", "unit": "A", "color": "#2ca02c"},
    "phase_w": {"label": "Iw", "unit": "A", "color": "#d62728"},
    "id_current": {"label": "Id", "unit": "A", "color": "#9467bd"},
    "iq_current": {"label": "Iq", "unit": "A", "color": "#ff7f0e"},
    "vd": {"label": "Vd", "unit": "V", "color": "#8c564b"},
    "vq": {"label": "Vq", "unit": "V", "color": "#e377c2"},
    "act_speed": {"label": "Act Speed", "unit": "rpm", "color": "#17becf"},
    "cmd_speed": {"label": "Cmd Speed", "unit": "rpm", "color": "#bcbd22"},
    "speed_error": {"label": "Speed Error", "unit": "rpm", "color": "#7f7f7f"},
}
TREND_EMA_ALPHA = {
    "phase_u": 0.18,
    "phase_v": 0.18,
    "phase_w": 0.18,
    "id_current": 0.18,
    "iq_current": 0.18,
    "vd": 0.22,
    "vq": 0.22,
    "act_speed": 0.25,
    "cmd_speed": None,
    "speed_error": 0.22,
}


@dataclass(slots=True)
class PendingFrame:
    frame: bytes
    description: str
    command: int
    quiet: bool = False


class CircularSeriesBuffer:
    def __init__(
        self,
        series_keys: list[str],
        capacity: int = TREND_BUFFER_CAPACITY,
        ema_alpha: dict[str, float | None] | None = None,
    ) -> None:
        self._series_keys = list(series_keys)
        self._capacity = max(100, capacity)
        self._timestamps = [0.0] * self._capacity
        self._values = {
            key: [0.0] * self._capacity for key in self._series_keys
        }
        self._ema_alpha = ema_alpha or {}
        self._last_filtered: dict[str, float | None] = {
            key: None for key in self._series_keys
        }
        self._write_index = 0
        self._count = 0

    def clear(self) -> None:
        self._write_index = 0
        self._count = 0
        self._last_filtered = {key: None for key in self._series_keys}

    def append_sample(self, timestamp_s: float, values: dict[str, float]) -> None:
        index = self._write_index
        self._timestamps[index] = timestamp_s
        for key in self._series_keys:
            raw_value = float(values.get(key, 0.0))
            alpha = self._ema_alpha.get(key)
            if alpha is None:
                filtered = raw_value
            else:
                last = self._last_filtered[key]
                filtered = raw_value if last is None else (last + alpha * (raw_value - last))
                self._last_filtered[key] = filtered
            self._values[key][index] = filtered

        self._write_index = (self._write_index + 1) % self._capacity
        self._count = min(self._count + 1, self._capacity)

    def latest_timestamp(self) -> float | None:
        if self._count == 0:
            return None
        latest_index = (self._write_index - 1) % self._capacity
        return self._timestamps[latest_index]

    def windowed_series(
        self, series_keys: list[str], start_time_s: float, end_time_s: float
    ) -> tuple[list[float], dict[str, list[float]]]:
        timestamps: list[float] = []
        values = {key: [] for key in series_keys}
        if self._count == 0:
            return timestamps, values

        first_index = (self._write_index - self._count) % self._capacity
        for offset in range(self._count):
            index = (first_index + offset) % self._capacity
            timestamp = self._timestamps[index]
            if timestamp < start_time_s:
                continue
            if timestamp > end_time_s:
                break
            timestamps.append(timestamp)
            for key in series_keys:
                values[key].append(self._values[key][index])

        return timestamps, values


class ScadaTrendView(QtWidgets.QWidget):
    def __init__(
        self,
        title: str,
        trend_buffer: CircularSeriesBuffer,
        series_keys: list[str],
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._title = title
        self._buffer = trend_buffer
        self._series_keys = list(series_keys)
        self._time_window_s = 10.0
        self._paused = False
        self._paused_anchor_s: float | None = None
        self._auto_scale = True
        self._frozen_range = (-1.0, 1.0)
        self._render_times: list[float] = []
        self._render_values: dict[str, list[float]] = {key: [] for key in self._series_keys}
        self._render_anchor_s = 0.0
        self._plot_rect = QtCore.QRectF()
        self._hover_index: int | None = None
        self.setMinimumHeight(220)
        self.setMouseTracking(True)

    def clear(self) -> None:
        self._render_times.clear()
        self._render_values = {key: [] for key in self._series_keys}
        self._hover_index = None
        self.update()

    def set_paused(self, paused: bool) -> None:
        self._paused = paused
        if paused:
            self._paused_anchor_s = self._buffer.latest_timestamp()
        else:
            self._paused_anchor_s = None

    def set_auto_scale(self, enabled: bool) -> None:
        self._auto_scale = enabled
        if not enabled:
            self._frozen_range = self._calculate_y_range()

    def set_time_window_s(self, value: float) -> None:
        self._time_window_s = max(1.0, float(value))

    def refresh_from_buffer(self) -> None:
        anchor_s = self._buffer.latest_timestamp()
        if anchor_s is None:
            self.clear()
            return

        if self._paused:
            if self._paused_anchor_s is None:
                self._paused_anchor_s = anchor_s
            anchor_s = self._paused_anchor_s
        else:
            self._paused_anchor_s = None

        start_s = anchor_s - self._time_window_s
        self._render_anchor_s = anchor_s
        self._render_times, self._render_values = self._buffer.windowed_series(
            self._series_keys, start_s, anchor_s
        )
        if self._auto_scale:
            self._frozen_range = self._calculate_y_range()
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        super().paintEvent(event)

        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        rect = self.rect().adjusted(8, 8, -8, -8)
        painter.fillRect(rect, self.palette().base())

        title_rect = QtCore.QRect(rect.left(), rect.top(), rect.width(), 20)
        painter.setPen(self.palette().text().color())
        painter.drawText(
            title_rect,
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
            self._title,
        )

        legend_y = title_rect.bottom() + 8
        legend_x = rect.left()
        for key in self._series_keys:
            color = QtGui.QColor(TREND_SERIES_META[key]["color"])
            painter.fillRect(QtCore.QRect(legend_x, legend_y, 10, 10), color)
            painter.drawText(
                QtCore.QRect(legend_x + 14, legend_y - 3, 90, 18),
                QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
                TREND_SERIES_META[key]["label"],
            )
            legend_x += 95

        plot_rect = QtCore.QRectF(rect.left(), legend_y + 20, rect.width(), rect.height() - 54)
        if plot_rect.height() <= 0:
            return
        self._plot_rect = plot_rect

        border_pen = QtGui.QPen(self.palette().mid().color())
        painter.setPen(border_pen)
        painter.drawRect(plot_rect)

        if not self._render_times:
            painter.drawText(
                plot_rect.toRect(),
                QtCore.Qt.AlignmentFlag.AlignCenter,
                "Waiting for monitor data...",
            )
            return

        y_min, y_max = self._frozen_range
        y_span = max(y_max - y_min, 0.1)

        grid_pen = QtGui.QPen(self.palette().midlight().color())
        grid_pen.setStyle(QtCore.Qt.PenStyle.DashLine)
        painter.setPen(grid_pen)
        for grid_index in range(1, 5):
            y = plot_rect.top() + (plot_rect.height() * grid_index / 5.0)
            painter.drawLine(
                QtCore.QPointF(plot_rect.left(), y),
                QtCore.QPointF(plot_rect.right(), y),
            )

        value_text_rect = QtCore.QRectF(plot_rect.right() - 80, plot_rect.top() + 4, 76, 16)
        painter.setPen(self.palette().text().color())
        painter.drawText(value_text_rect, QtCore.Qt.AlignmentFlag.AlignRight, f"{y_max:.2f}")
        value_text_rect.moveBottom(plot_rect.bottom() - 4)
        painter.drawText(value_text_rect, QtCore.Qt.AlignmentFlag.AlignRight, f"{y_min:.2f}")

        time_labels = [0.0, self._time_window_s / 2.0, self._time_window_s]
        for offset_s in time_labels:
            ratio = offset_s / self._time_window_s if self._time_window_s > 0 else 0.0
            x = plot_rect.right() - (plot_rect.width() * ratio)
            painter.drawLine(
                QtCore.QPointF(x, plot_rect.bottom()),
                QtCore.QPointF(x, plot_rect.bottom() + 4),
            )
            label = "now" if offset_s == 0.0 else f"-{offset_s:.0f}s"
            painter.drawText(
                QtCore.QRectF(x - 22, plot_rect.bottom() + 6, 44, 14),
                QtCore.Qt.AlignmentFlag.AlignCenter,
                label,
            )

        for key in self._series_keys:
            series = self._render_values[key]
            if len(series) < 2:
                continue

            path = QtGui.QPainterPath()
            for index, value in enumerate(series):
                time_ratio = (
                    (self._render_times[index] - (self._render_anchor_s - self._time_window_s))
                    / self._time_window_s
                )
                time_ratio = min(max(time_ratio, 0.0), 1.0)
                x = plot_rect.left() + (plot_rect.width() * time_ratio)
                y = plot_rect.bottom() - (((value - y_min) / y_span) * plot_rect.height())
                point = QtCore.QPointF(x, y)
                if index == 0:
                    path.moveTo(point)
                else:
                    path.lineTo(point)

            series_pen = QtGui.QPen(QtGui.QColor(TREND_SERIES_META[key]["color"]), 1.8)
            painter.setPen(series_pen)
            painter.drawPath(path)

        if self._hover_index is not None and 0 <= self._hover_index < len(self._render_times):
            hover_time = self._render_times[self._hover_index]
            ratio = (
                (hover_time - (self._render_anchor_s - self._time_window_s)) / self._time_window_s
            )
            x = plot_rect.left() + (plot_rect.width() * ratio)
            cursor_pen = QtGui.QPen(QtGui.QColor("#888888"))
            cursor_pen.setStyle(QtCore.Qt.PenStyle.DashLine)
            painter.setPen(cursor_pen)
            painter.drawLine(QtCore.QPointF(x, plot_rect.top()), QtCore.QPointF(x, plot_rect.bottom()))

            for key in self._series_keys:
                value = self._render_values[key][self._hover_index]
                y = plot_rect.bottom() - (((value - y_min) / y_span) * plot_rect.height())
                painter.setBrush(QtGui.QBrush(QtGui.QColor(TREND_SERIES_META[key]["color"])))
                painter.setPen(QtCore.Qt.PenStyle.NoPen)
                painter.drawEllipse(QtCore.QPointF(x, y), 3.0, 3.0)

    def _calculate_y_range(self) -> tuple[float, float]:
        all_values = [
            value
            for key in self._series_keys
            for value in self._render_values.get(key, [])
        ]
        if not all_values:
            return (-1.0, 1.0)

        y_min = min(all_values)
        y_max = max(all_values)
        if abs(y_max - y_min) < 0.05:
            center = (y_max + y_min) * 0.5
            return (center - 0.5, center + 0.5)
        margin = (y_max - y_min) * 0.12
        return (y_min - margin, y_max + margin)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if not self._render_times or not self._plot_rect.contains(event.position()):
            self._hover_index = None
            QtWidgets.QToolTip.hideText()
            self.update()
            return

        ratio = (event.position().x() - self._plot_rect.left()) / self._plot_rect.width()
        ratio = min(max(ratio, 0.0), 1.0)
        target_time = (self._render_anchor_s - self._time_window_s) + (ratio * self._time_window_s)
        nearest_index = min(
            range(len(self._render_times)),
            key=lambda index: abs(self._render_times[index] - target_time),
        )
        self._hover_index = nearest_index
        age_s = self._render_anchor_s - self._render_times[nearest_index]
        tooltip_lines = [f"t = -{age_s:.3f}s"]
        for key in self._series_keys:
            value = self._render_values[key][nearest_index]
            tooltip_lines.append(
                f"{TREND_SERIES_META[key]['label']}: {value:.3f} {TREND_SERIES_META[key]['unit']}"
            )
        QtWidgets.QToolTip.showText(
            event.globalPosition().toPoint(),
            "\n".join(tooltip_lines),
            self,
        )
        self.update()

    def leaveEvent(self, event: QtCore.QEvent) -> None:  # noqa: N802
        self._hover_index = None
        QtWidgets.QToolTip.hideText()
        self.update()
        super().leaveEvent(event)


class ScadaTrendPanel(QtWidgets.QWidget):
    def __init__(
        self,
        title: str,
        trend_buffer: CircularSeriesBuffer,
        series_keys: list[str],
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        toolbar = QtWidgets.QHBoxLayout()

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.setCheckable(True)
        self.auto_scale_checkbox = QtWidgets.QCheckBox("Auto-scale Y")
        self.auto_scale_checkbox.setChecked(True)
        self.time_window_combo = QtWidgets.QComboBox()
        self.time_window_combo.addItems(["5 s", "10 s", "20 s", "30 s"])
        self.time_window_combo.setCurrentText("10 s")

        toolbar.addWidget(self.pause_button)
        toolbar.addWidget(self.auto_scale_checkbox)
        toolbar.addWidget(QtWidgets.QLabel("Window"))
        toolbar.addWidget(self.time_window_combo)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.plot_view = ScadaTrendView(title, trend_buffer, series_keys, self)
        layout.addWidget(self.plot_view)

        self.pause_button.toggled.connect(self._handle_pause_toggled)
        self.auto_scale_checkbox.toggled.connect(self.plot_view.set_auto_scale)
        self.time_window_combo.currentTextChanged.connect(self._handle_window_changed)

    def _handle_pause_toggled(self, checked: bool) -> None:
        self.pause_button.setText("Resume" if checked else "Pause")
        self.plot_view.set_paused(checked)

    def _handle_window_changed(self, text: str) -> None:
        value = float(text.split()[0])
        self.plot_view.set_time_window_s(value)

    def refresh(self) -> None:
        self.plot_view.refresh_from_buffer()

    def clear(self) -> None:
        self.plot_view.clear()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ASD04 Driver GUI (Python)")
        self.resize(1280, 840)

        self._worker = SerialWorker(FrameStreamParser(), self)
        self._connected = False
        self._current_port = ""
        self._pending_frames: deque[PendingFrame] = deque()
        self._awaiting_ack = False
        self._last_sent: PendingFrame | None = None
        self._port_descriptors: list[PortDescriptor] = []
        self._latest_monitor_snapshot = None
        self._trend_buffer = CircularSeriesBuffer(
            list(TREND_SERIES_META.keys()),
            capacity=TREND_BUFFER_CAPACITY,
            ema_alpha=TREND_EMA_ALPHA,
        )
        self._trend_panels: list[ScadaTrendPanel] = []

        self._ack_timer = QtCore.QTimer(self)
        self._ack_timer.setSingleShot(True)
        self._ack_timer.timeout.connect(self._on_ack_timeout)

        self._monitor_timer = QtCore.QTimer(self)
        self._monitor_timer.setInterval(250)
        self._monitor_timer.timeout.connect(self._queue_monitor_poll)

        self._trend_refresh_timer = QtCore.QTimer(self)
        self._trend_refresh_timer.setInterval(TREND_REFRESH_INTERVAL_MS)
        self._trend_refresh_timer.timeout.connect(self._refresh_scada_ui)

        self._build_ui()
        self._connect_signals()
        self._refresh_ports()
        self._monitor_timer.start()
        self._trend_refresh_timer.start()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)

        main_layout.addWidget(self._build_connection_group())
        content_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal, self)
        content_splitter.addWidget(self._build_monitor_group())
        content_splitter.addWidget(self._build_quick_command_group())
        content_splitter.setStretchFactor(0, 3)
        content_splitter.setStretchFactor(1, 2)
        main_layout.addWidget(content_splitter, 1)

        self._parameter_window = self._build_parameter_window()
        self._trend_window = self._build_trend_window()
        self._log_window = self._build_log_window()

        self.statusBar().showMessage(f"Ready ({QT_API})")

    def _build_connection_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Connection")
        layout = QtWidgets.QGridLayout(box)

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports_button = QtWidgets.QPushButton("Refresh Ports")
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.disconnect_button.setEnabled(False)

        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")

        self.auto_poll_checkbox = QtWidgets.QCheckBox("Auto Monitor Poll")
        self.auto_poll_checkbox.setChecked(False)
        self.monitor_rate_combo = QtWidgets.QComboBox()
        self.monitor_rate_combo.addItem("4 Hz", 250)
        self.monitor_rate_combo.addItem("10 Hz", 100)
        self.monitor_rate_combo.addItem("20 Hz", 50)
        self.monitor_rate_combo.addItem("40 Hz", 25)
        self.monitor_rate_combo.setCurrentIndex(2)

        self.device_hint_label = QtWidgets.QLabel(
            f"Known device VID:PID = {KNOWN_DEVICE_VID:04X}:{KNOWN_DEVICE_PID:04X}"
        )
        self.connection_state_label = QtWidgets.QLabel("Disconnected")
        self.open_parameters_button = QtWidgets.QPushButton("Parameters...")
        self.open_trends_button = QtWidgets.QPushButton("Trends...")
        self.open_log_button = QtWidgets.QPushButton("Log...")

        layout.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        layout.addWidget(self.port_combo, 0, 1)
        layout.addWidget(self.refresh_ports_button, 0, 2)
        layout.addWidget(QtWidgets.QLabel("Baud"), 0, 3)
        layout.addWidget(self.baud_combo, 0, 4)
        layout.addWidget(self.connect_button, 0, 5)
        layout.addWidget(self.disconnect_button, 0, 6)
        layout.addWidget(QtWidgets.QLabel("Poll Rate"), 0, 7)
        layout.addWidget(self.monitor_rate_combo, 0, 8)
        layout.addWidget(self.auto_poll_checkbox, 1, 0, 1, 2)
        layout.addWidget(self.device_hint_label, 1, 2, 1, 3)
        layout.addWidget(self.open_parameters_button, 1, 5)
        layout.addWidget(self.open_trends_button, 1, 6)
        layout.addWidget(self.open_log_button, 1, 7)
        layout.addWidget(self.connection_state_label, 1, 8)
        return box

    def _build_monitor_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Monitor")
        layout = QtWidgets.QVBoxLayout(box)
        tabs = QtWidgets.QTabWidget()
        self.monitor_labels: dict[str, QtWidgets.QLabel] = {}

        basic_fields = [
            ("enable_run", "Enable Run"),
            ("run_mode", "Run Mode"),
            ("fault_occurred", "Fault"),
            ("vdc", "Vdc"),
            ("temperature", "Temperature"),
            ("motor_power", "Motor Power"),
            ("cmd_speed", "Cmd Speed"),
            ("act_speed", "Act Speed"),
            ("speed_error", "Speed Error"),
            ("cmd_position", "Cmd Position"),
            ("act_position", "Act Position"),
            ("position_error", "Position Error"),
        ]
        electrical_fields = [
            ("id_ref", "Id Ref"),
            ("iq_ref", "Iq Ref"),
            ("phase_u", "Iu"),
            ("phase_v", "Iv"),
            ("phase_w", "Iw"),
            ("id_current", "Id"),
            ("iq_current", "Iq"),
            ("vd", "Vd"),
            ("vq", "Vq"),
            ("v_phase_u", "Vu"),
            ("v_phase_v", "Vv"),
            ("v_phase_w", "Vw"),
        ]
        debug_fields = [
            ("theta", "Theta"),
            ("vf_frequency", "V/F Freq"),
            ("vf_voltage", "V/F Volt"),
        ]

        tabs.addTab(self._build_monitor_fields_widget(basic_fields), "Basic")
        tabs.addTab(self._build_monitor_fields_widget(electrical_fields), "Electrical")
        tabs.addTab(self._build_monitor_fields_widget(debug_fields), "Debug")

        trend_note = QtWidgets.QLabel(
            "Full-size trend charts are available in the separate Trends window."
        )
        trend_note.setWordWrap(True)
        open_trends_inline_button = QtWidgets.QPushButton("Open Trends Window")
        open_trends_inline_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._trend_window)
        )

        layout.addWidget(tabs)
        layout.addWidget(trend_note)
        layout.addWidget(open_trends_inline_button, 0, QtCore.Qt.AlignmentFlag.AlignRight)
        return box

    def _build_trend_window(self) -> QtWidgets.QDialog:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Trend Charts")
        dialog.resize(1220, 860)
        dialog.setModal(False)

        layout = QtWidgets.QVBoxLayout(dialog)
        tabs = QtWidgets.QTabWidget(dialog)

        self.phase_current_panel = ScadaTrendPanel(
            "Phase Currents",
            self._trend_buffer,
            ["phase_u", "phase_v", "phase_w"],
        )
        self.dq_current_panel = ScadaTrendPanel(
            "D/Q Currents",
            self._trend_buffer,
            ["id_current", "iq_current"],
        )
        self.vdq_voltage_panel = ScadaTrendPanel(
            "D/Q Voltages",
            self._trend_buffer,
            ["vd", "vq"],
        )
        self.speed_panel = ScadaTrendPanel(
            "Speed Trend",
            self._trend_buffer,
            ["act_speed", "cmd_speed", "speed_error"],
        )

        tabs.addTab(self.phase_current_panel, "Phase Currents")
        tabs.addTab(self.dq_current_panel, "D/Q Currents")
        tabs.addTab(self.vdq_voltage_panel, "Vd/Vq")
        tabs.addTab(self.speed_panel, "Speed")

        toolbar = QtWidgets.QHBoxLayout()
        clear_trend_button = QtWidgets.QPushButton("Clear History")
        clear_trend_button.clicked.connect(self._clear_trend_history)
        trend_hint = QtWidgets.QLabel(
            "Use Pause to inspect data, Auto-scale Y to freeze the range, and hover to read point values."
        )
        trend_hint.setWordWrap(True)
        toolbar.addWidget(clear_trend_button)
        toolbar.addWidget(trend_hint, 1)

        layout.addLayout(toolbar)
        layout.addWidget(tabs, 1)

        self._trend_panels = [
            self.phase_current_panel,
            self.dq_current_panel,
            self.vdq_voltage_panel,
            self.speed_panel,
        ]
        return dialog

    def _build_quick_command_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Quick Commands")
        layout = QtWidgets.QVBoxLayout(box)

        button_grid = QtWidgets.QGridLayout()
        self.servo_on_button = QtWidgets.QPushButton("Servo ON")
        self.servo_off_button = QtWidgets.QPushButton("Servo OFF")
        self.ack_fault_button = QtWidgets.QPushButton("ACK Fault")
        self.save_flash_button = QtWidgets.QPushButton("Write To Flash")
        self.refresh_monitor_button = QtWidgets.QPushButton("Refresh Monitor")

        button_grid.addWidget(self.servo_on_button, 0, 0)
        button_grid.addWidget(self.servo_off_button, 0, 1)
        button_grid.addWidget(self.ack_fault_button, 0, 2)
        button_grid.addWidget(self.save_flash_button, 0, 3)
        button_grid.addWidget(self.refresh_monitor_button, 1, 0, 1, 2)
        layout.addLayout(button_grid)

        vf_group = QtWidgets.QGroupBox("Open Loop V/F Test")
        vf_layout = QtWidgets.QGridLayout(vf_group)
        self.vf_frequency_spin = QtWidgets.QDoubleSpinBox()
        self.vf_frequency_spin.setRange(0.0, 50.0)
        self.vf_frequency_spin.setDecimals(2)
        self.vf_frequency_spin.setSingleStep(0.50)
        self.vf_frequency_spin.setSuffix(" Hz")
        self.vf_voltage_spin = QtWidgets.QDoubleSpinBox()
        self.vf_voltage_spin.setRange(0.0, 40.0)
        self.vf_voltage_spin.setDecimals(2)
        self.vf_voltage_spin.setSingleStep(1.0)
        self.vf_voltage_spin.setSuffix(" V")
        self.start_vf_button = QtWidgets.QPushButton("Start V/F")
        self.stop_vf_button = QtWidgets.QPushButton("Stop V/F")
        vf_note = QtWidgets.QLabel(
            "Electrical frequency and voltage reference are clamped in firmware for safe open-loop testing."
        )
        vf_note.setWordWrap(True)
        vf_layout.addWidget(QtWidgets.QLabel("Electrical Frequency"), 0, 0)
        vf_layout.addWidget(self.vf_frequency_spin, 0, 1)
        vf_layout.addWidget(QtWidgets.QLabel("Voltage Reference"), 1, 0)
        vf_layout.addWidget(self.vf_voltage_spin, 1, 1)
        vf_layout.addWidget(self.start_vf_button, 2, 0)
        vf_layout.addWidget(self.stop_vf_button, 2, 1)
        vf_layout.addWidget(vf_note, 3, 0, 1, 2)
        layout.addWidget(vf_group)

        raw_group = QtWidgets.QGroupBox("Raw Command")
        raw_layout = QtWidgets.QGridLayout(raw_group)
        self.command_combo = QtWidgets.QComboBox()
        for command in Command:
            self.command_combo.addItem(f"0x{command.value:02X} {command.name}", command.value)
        self.payload_edit = QtWidgets.QLineEdit()
        self.payload_edit.setPlaceholderText("Hex payload, example: 10 00 00 80 3F")
        self.send_raw_button = QtWidgets.QPushButton("Send Raw")
        raw_layout.addWidget(QtWidgets.QLabel("Command"), 0, 0)
        raw_layout.addWidget(self.command_combo, 0, 1)
        raw_layout.addWidget(QtWidgets.QLabel("Payload"), 1, 0)
        raw_layout.addWidget(self.payload_edit, 1, 1)
        raw_layout.addWidget(self.send_raw_button, 1, 2)
        layout.addWidget(raw_group)

        info_group = QtWidgets.QGroupBox("Notes")
        info_layout = QtWidgets.QVBoxLayout(info_group)
        info_label = QtWidgets.QLabel(
            "Basic GUI features:\n"
            "- COM connect / disconnect\n"
            "- monitor polling\n"
            "- servo on / off and fault ack\n"
            "- parameters and communication log are available from the top toolbar\n"
            "- raw command testing\n"
            "- SCADA-style current, voltage, and speed trend charts"
        )
        info_label.setWordWrap(True)
        info_layout.addWidget(info_label)
        layout.addWidget(info_group)

        layout.addStretch(1)
        return box

    def _build_monitor_fields_widget(
        self, field_names: list[tuple[str, str]], columns: int = 2
    ) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(widget)
        for index, (key, label) in enumerate(field_names):
            value_label = QtWidgets.QLabel("-")
            value_label.setTextInteractionFlags(
                QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
            )
            self.monitor_labels[key] = value_label
            row = index // columns
            col = (index % columns) * 2
            layout.addWidget(QtWidgets.QLabel(label), row, col)
            layout.addWidget(value_label, row, col + 1)
        for column in range(columns):
            layout.setColumnStretch(column * 2 + 1, 1)
        layout.setRowStretch((len(field_names) // columns) + 1, 1)
        return widget

    def _build_parameter_tabs(self) -> QtWidgets.QTabWidget:
        tabs = QtWidgets.QTabWidget()
        self.driver_table = self._create_parameter_table(DRIVER_PARAMETER_NAMES)
        self.motor_table = self._create_parameter_table(MOTOR_PARAMETER_NAMES)
        tabs.addTab(self.driver_table, "Driver Parameters")
        tabs.addTab(self.motor_table, "Motor Parameters")
        return tabs

    def _build_parameter_window(self) -> QtWidgets.QDialog:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Parameters")
        dialog.resize(920, 700)
        dialog.setModal(False)

        layout = QtWidgets.QVBoxLayout(dialog)
        toolbar = QtWidgets.QHBoxLayout()
        self.read_driver_button = QtWidgets.QPushButton("Read Driver Params")
        self.read_motor_button = QtWidgets.QPushButton("Read Motor Params")
        self.write_driver_button = QtWidgets.QPushButton("Write Driver Params")
        self.write_motor_button = QtWidgets.QPushButton("Write Motor Params")
        toolbar.addWidget(self.read_driver_button)
        toolbar.addWidget(self.read_motor_button)
        toolbar.addWidget(self.write_driver_button)
        toolbar.addWidget(self.write_motor_button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)
        layout.addWidget(self._build_parameter_tabs())
        return dialog

    def _create_parameter_table(self, names: list[str]) -> QtWidgets.QTableWidget:
        table = QtWidgets.QTableWidget(len(names), 3)
        table.setHorizontalHeaderLabels(["Index", "Name", "Value"])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)

        for index, name in enumerate(names):
            index_item = QtWidgets.QTableWidgetItem(str(index))
            index_item.setFlags(index_item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            name_item = QtWidgets.QTableWidgetItem(name)
            name_item.setFlags(name_item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            value_item = QtWidgets.QTableWidgetItem("0.0")
            table.setItem(index, 0, index_item)
            table.setItem(index, 1, name_item)
            table.setItem(index, 2, value_item)

        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeMode.Stretch)
        return table

    def _build_log_window(self) -> QtWidgets.QDialog:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Communication Log")
        dialog.resize(780, 520)
        dialog.setModal(False)
        layout = QtWidgets.QVBoxLayout(dialog)

        log_toolbar = QtWidgets.QHBoxLayout()
        self.clear_log_button = QtWidgets.QPushButton("Clear Log")
        log_toolbar.addWidget(self.clear_log_button)
        log_toolbar.addStretch(1)

        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)

        layout.addLayout(log_toolbar)
        layout.addWidget(self.log_edit)
        return dialog

    def _connect_signals(self) -> None:
        self.refresh_ports_button.clicked.connect(self._refresh_ports)
        self.connect_button.clicked.connect(self._connect_selected_port)
        self.disconnect_button.clicked.connect(self._disconnect_port)
        self.open_parameters_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._parameter_window)
        )
        self.open_trends_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._trend_window)
        )
        self.open_log_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._log_window)
        )
        self.monitor_rate_combo.currentIndexChanged.connect(self._update_monitor_poll_interval)
        self.refresh_monitor_button.clicked.connect(self._request_monitor_once)
        self.start_vf_button.clicked.connect(self._start_open_loop_vf)
        self.stop_vf_button.clicked.connect(
            lambda: self._enqueue_command(
                Command.CMD_STOP_OPEN_LOOP_VF, b"", "Stop Open Loop V/F"
            )
        )
        self.servo_on_button.clicked.connect(
            lambda: self._enqueue_command(Command.CMD_SERVO_ON, b"", "Servo ON")
        )
        self.servo_off_button.clicked.connect(
            lambda: self._enqueue_command(Command.CMD_SERVO_OFF, b"", "Servo OFF")
        )
        self.ack_fault_button.clicked.connect(
            lambda: self._enqueue_command(Command.CMD_ACK_FAULT, b"", "ACK Fault")
        )
        self.save_flash_button.clicked.connect(
            lambda: self._enqueue_command(Command.CMD_WRITE_TO_FLASH, b"", "Write To Flash")
        )
        self.read_driver_button.clicked.connect(self._read_driver_parameters)
        self.read_motor_button.clicked.connect(self._read_motor_parameters)
        self.write_driver_button.clicked.connect(self._write_driver_parameters)
        self.write_motor_button.clicked.connect(self._write_motor_parameters)
        self.send_raw_button.clicked.connect(self._send_raw_command)
        self.clear_log_button.clicked.connect(self.log_edit.clear)

        self._worker.frame_received.connect(self._handle_frame)
        self._worker.status_changed.connect(self._handle_connection_status)
        self._worker.log_message.connect(self._handle_worker_log)
        self._worker.error_occurred.connect(self._handle_worker_error)
        self._update_monitor_poll_interval()

    def _show_auxiliary_window(self, window: QtWidgets.QDialog) -> None:
        window.show()
        window.raise_()
        window.activateWindow()

    def _update_monitor_poll_interval(self) -> None:
        interval_ms = int(self.monitor_rate_combo.currentData() or 250)
        self._monitor_timer.setInterval(interval_ms)

    def _refresh_ports(self) -> None:
        self._port_descriptors = list_serial_ports()
        self.port_combo.clear()
        for descriptor in self._port_descriptors:
            self.port_combo.addItem(descriptor.display_name, descriptor.port_name)

        if not self._port_descriptors:
            self.port_combo.addItem("No serial ports detected", "")
            return

        preferred_index = 0
        for index, descriptor in enumerate(self._port_descriptors):
            if descriptor.vid == KNOWN_DEVICE_VID and descriptor.pid == KNOWN_DEVICE_PID:
                preferred_index = index
                break
        self.port_combo.setCurrentIndex(preferred_index)

    def _connect_selected_port(self) -> None:
        port_name = self.port_combo.currentData()
        if not port_name:
            self._append_log("No serial port selected")
            return
        baudrate = int(self.baud_combo.currentText())
        self._pending_frames.clear()
        self._awaiting_ack = False
        self._last_sent = None
        self._ack_timer.stop()
        self._worker.request_open(port_name, baudrate)

    def _disconnect_port(self) -> None:
        self._worker.request_close()

    def _handle_connection_status(self, connected: bool, port_name: str) -> None:
        self._connected = connected
        self._current_port = port_name
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)

        if connected:
            self.connection_state_label.setText(f"Connected: {port_name}")
            self.statusBar().showMessage(f"Connected to {port_name}")
            self._clear_trend_history()
            self._try_send_next()
        else:
            self.connection_state_label.setText("Disconnected")
            self.statusBar().showMessage("Disconnected")
            self._pending_frames.clear()
            self._awaiting_ack = False
            self._last_sent = None
            self._latest_monitor_snapshot = None
            self._ack_timer.stop()
            self._clear_trend_history()

    def _handle_worker_log(self, message: str) -> None:
        if message.startswith("TX 02 16 01 1C"):
            return
        if message.startswith("RX 02 16 2A 35"):
            return
        if message.startswith("TX ") and self._last_sent and self._last_sent.quiet:
            return
        if message.startswith("RX ") and self._last_sent and self._last_sent.quiet:
            return
        self._append_log(message)

    def _handle_worker_error(self, message: str) -> None:
        self._append_log(f"ERROR {message}")
        QtWidgets.QMessageBox.warning(self, "Communication Error", message)

    def _append_log(self, message: str) -> None:
        timestamp = QtCore.QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        self.log_edit.appendPlainText(f"[{timestamp}] {message}")

    def _enqueue_command(
        self, command: Command | int, payload: bytes, description: str, quiet: bool = False
    ) -> None:
        frame = build_command_frame(int(command), payload)
        self._pending_frames.append(
            PendingFrame(frame=frame, description=description, command=int(command), quiet=quiet)
        )
        self._try_send_next()

    def _try_send_next(self) -> None:
        if not self._connected or self._awaiting_ack or not self._pending_frames:
            return

        self._last_sent = self._pending_frames.popleft()
        self._worker.queue_tx(self._last_sent.frame)
        self._awaiting_ack = True
        self._ack_timer.start(300)
        if not self._last_sent.quiet:
            self._append_log(
                f"SEND {self._last_sent.description} (0x{self._last_sent.command:02X})"
            )

    def _on_ack_timeout(self) -> None:
        if self._last_sent is not None:
            self._append_log(
                f"ACK timeout for {self._last_sent.description} "
                f"(0x{self._last_sent.command:02X})"
            )
        self._awaiting_ack = False
        self._last_sent = None
        self._try_send_next()

    def _queue_monitor_poll(self) -> None:
        if not self._connected or not self.auto_poll_checkbox.isChecked():
            return
        if self._awaiting_ack or self._pending_frames:
            return
        self._enqueue_command(
            Command.CMD_UPDATE_MONITOR,
            b"",
            "Monitor Poll",
            quiet=True,
        )

    def _request_monitor_once(self) -> None:
        self._enqueue_command(Command.CMD_UPDATE_MONITOR, b"", "Monitor Poll")

    def _read_driver_parameters(self) -> None:
        self._show_auxiliary_window(self._parameter_window)
        self._enqueue_command(
            Command.CMD_READ_DRIVER,
            bytes([len(DRIVER_PARAMETER_NAMES)]),
            "Read Driver Parameters",
        )

    def _read_motor_parameters(self) -> None:
        self._show_auxiliary_window(self._parameter_window)
        self._enqueue_command(
            Command.CMD_READ_MOTOR,
            bytes([len(MOTOR_PARAMETER_NAMES)]),
            "Read Motor Parameters",
        )

    def _write_driver_parameters(self) -> None:
        try:
            values = self._table_values(self.driver_table)
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid Driver Parameters", str(exc))
            return

        for payload in build_parameter_write_chunks(values, chunk_size=16):
            self._enqueue_command(
                Command.CMD_WRITE_DRIVER,
                payload,
                "Write Driver Parameters",
            )

    def _write_motor_parameters(self) -> None:
        try:
            values = self._table_values(self.motor_table)
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid Motor Parameters", str(exc))
            return

        chunks = build_parameter_write_chunks(values, chunk_size=20)
        for index, payload in enumerate(chunks, start=1):
            self._enqueue_command(
                Command.CMD_WRITE_MOTOR,
                payload,
                f"Write Motor Parameters chunk {index}/{len(chunks)}",
            )

    def _table_values(self, table: QtWidgets.QTableWidget) -> list[float]:
        values: list[float] = []
        for row in range(table.rowCount()):
            item = table.item(row, 2)
            text = item.text().strip() if item is not None else ""
            try:
                values.append(float(text))
            except ValueError as exc:
                raise ValueError(f"Row {row}: invalid float '{text}'") from exc
        return values

    def _start_open_loop_vf(self) -> None:
        payload = struct.pack(
            "<ff",
            float(self.vf_frequency_spin.value()),
            float(self.vf_voltage_spin.value()),
        )
        self._enqueue_command(
            Command.CMD_START_OPEN_LOOP_VF,
            payload,
            (
                f"Start Open Loop V/F "
                f"({self.vf_frequency_spin.value():.2f} Hz, "
                f"{self.vf_voltage_spin.value():.2f} V)"
            ),
        )

    def _send_raw_command(self) -> None:
        command_value = int(self.command_combo.currentData())
        payload_text = self.payload_edit.text().strip()
        try:
            payload = self._parse_hex_payload(payload_text)
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid Payload", str(exc))
            return
        self._enqueue_command(command_value, payload, "Raw Command")

    def _parse_hex_payload(self, text: str) -> bytes:
        if not text:
            return b""
        cleaned = text.replace(",", " ")
        if " " not in cleaned and len(cleaned) % 2 == 0:
            return bytes.fromhex(cleaned)
        return bytes(int(token, 16) for token in cleaned.split())

    def _handle_frame(self, frame: ParsedFrame) -> None:
        if frame.code == ACK_NOERROR:
            echoed_command = frame.payload[0] if frame.payload else None
            if not (self._last_sent and self._last_sent.quiet):
                if echoed_command is None:
                    self._append_log("ACK received")
                else:
                    self._append_log(f"ACK received for command 0x{echoed_command:02X}")
            self._ack_timer.stop()
            self._awaiting_ack = False
            self._last_sent = None
            self._try_send_next()
            return

        if frame.code == ACK_ERROR:
            self._append_log("Driver returned ACK_ERROR")
            self._ack_timer.stop()
            self._awaiting_ack = False
            self._last_sent = None
            self._try_send_next()
            return

        if not frame.is_syn or frame.subcommand is None:
            return

        body = frame.payload[1:]
        subcommand = frame.subcommand
        if subcommand == UpdateCode.CMD_MONITOR_DATA:
            try:
                snapshot = parse_monitor_payload(body)
            except ValueError as exc:
                self._append_log(f"Monitor parse error: {exc}")
                return
            self._latest_monitor_snapshot = snapshot
            self._append_snapshot_to_trend_buffer(snapshot)
            self._update_monitor(snapshot)
            return

        if subcommand == UpdateCode.MTR_CODE_ERROR:
            try:
                snapshot = parse_error_payload(body)
            except ValueError as exc:
                self._append_log(f"Error payload parse error: {exc}")
                return
            self._update_error_snapshot(snapshot)
            return

        if subcommand == UpdateCode.CMD_READ_DRIVER_DATA:
            entries = parse_parameter_chunk(body)
            self._update_parameter_table(self.driver_table, entries)
            self._append_log(f"Driver parameter chunk received ({len(entries)} items)")
            return

        if subcommand == UpdateCode.CMD_READ_MOTOR_DATA:
            entries = parse_parameter_chunk(body)
            self._update_parameter_table(self.motor_table, entries)
            self._append_log(f"Motor parameter chunk received ({len(entries)} items)")
            return

        self._append_log(f"SYN packet received: subcommand 0x{subcommand:02X}, {len(body)} bytes")

    def _update_parameter_table(
        self, table: QtWidgets.QTableWidget, entries: dict[int, float]
    ) -> None:
        for index, value in entries.items():
            if 0 <= index < table.rowCount():
                item = table.item(index, 2)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    table.setItem(index, 2, item)
                item.setText(f"{value:.6f}")

    def _clear_trend_history(self) -> None:
        self._trend_buffer.clear()
        for panel in self._trend_panels:
            panel.clear()

    def _append_snapshot_to_trend_buffer(self, snapshot) -> None:
        self._trend_buffer.append_sample(
            time.monotonic(),
            {
                "phase_u": snapshot.phase_u,
                "phase_v": snapshot.phase_v,
                "phase_w": snapshot.phase_w,
                "id_current": snapshot.id_current,
                "iq_current": snapshot.iq_current,
                "vd": snapshot.vd,
                "vq": snapshot.vq,
                "act_speed": snapshot.act_speed,
                "cmd_speed": snapshot.cmd_speed,
                "speed_error": snapshot.speed_error,
            },
        )

    def _append_error_snapshot_to_trend_buffer(self, snapshot) -> None:
        last_monitor = self._latest_monitor_snapshot
        self._trend_buffer.append_sample(
            time.monotonic(),
            {
                "phase_u": snapshot.phase_u,
                "phase_v": snapshot.phase_v,
                "phase_w": snapshot.phase_w,
                "id_current": (
                    float(last_monitor.id_current) if last_monitor is not None else 0.0
                ),
                "iq_current": (
                    float(last_monitor.iq_current) if last_monitor is not None else 0.0
                ),
                "vd": float(last_monitor.vd) if last_monitor is not None else 0.0,
                "vq": float(last_monitor.vq) if last_monitor is not None else 0.0,
                "act_speed": snapshot.act_speed,
                "cmd_speed": snapshot.cmd_speed,
                "speed_error": snapshot.speed_error,
            },
        )

    def _refresh_scada_ui(self) -> None:
        for panel in self._trend_panels:
            panel.refresh()

    def _run_mode_text(self, run_mode: int) -> str:
        if run_mode == 1:
            return "Open Loop V/F"
        if run_mode == 0:
            return "FOC"
        return f"Unknown ({run_mode})"

    def _update_monitor(self, snapshot) -> None:
        self.monitor_labels["enable_run"].setText("ON" if snapshot.enable_run else "OFF")
        self.monitor_labels["run_mode"].setText(self._run_mode_text(snapshot.run_mode))
        self.monitor_labels["fault_occurred"].setText(f"0x{snapshot.fault_occurred:04X}")
        self.monitor_labels["vdc"].setText(f"{snapshot.vdc:.3f} V")
        self.monitor_labels["temperature"].setText(f"{snapshot.temperature:.3f} C")
        self.monitor_labels["motor_power"].setText(str(snapshot.motor_power))
        self.monitor_labels["cmd_speed"].setText(f"{snapshot.cmd_speed:.3f}")
        self.monitor_labels["act_speed"].setText(f"{snapshot.act_speed:.3f}")
        self.monitor_labels["speed_error"].setText(f"{snapshot.speed_error:.3f}")
        self.monitor_labels["cmd_position"].setText(f"{snapshot.cmd_position:.3f}")
        self.monitor_labels["act_position"].setText(f"{snapshot.act_position:.3f}")
        self.monitor_labels["position_error"].setText(f"{snapshot.position_error:.3f}")
        self.monitor_labels["id_ref"].setText(f"{snapshot.id_ref:.3f} A")
        self.monitor_labels["iq_ref"].setText(f"{snapshot.iq_ref:.3f} A")
        self.monitor_labels["phase_u"].setText(f"{snapshot.phase_u:.3f} A")
        self.monitor_labels["phase_v"].setText(f"{snapshot.phase_v:.3f} A")
        self.monitor_labels["phase_w"].setText(f"{snapshot.phase_w:.3f} A")
        self.monitor_labels["id_current"].setText(f"{snapshot.id_current:.3f} A")
        self.monitor_labels["iq_current"].setText(f"{snapshot.iq_current:.3f} A")
        self.monitor_labels["vd"].setText(f"{snapshot.vd:.3f} V")
        self.monitor_labels["vq"].setText(f"{snapshot.vq:.3f} V")
        self.monitor_labels["v_phase_u"].setText(f"{snapshot.v_phase_u:.3f} V")
        self.monitor_labels["v_phase_v"].setText(f"{snapshot.v_phase_v:.3f} V")
        self.monitor_labels["v_phase_w"].setText(f"{snapshot.v_phase_w:.3f} V")
        self.monitor_labels["theta"].setText(f"{snapshot.theta:.3f} rad")
        self.monitor_labels["vf_frequency"].setText(f"{snapshot.vf_frequency:.3f} Hz")
        self.monitor_labels["vf_voltage"].setText(f"{snapshot.vf_voltage:.3f} V")

    def _update_error_snapshot(self, snapshot) -> None:
        self._append_error_snapshot_to_trend_buffer(snapshot)
        self.monitor_labels["enable_run"].setText("OFF")
        self.monitor_labels["run_mode"].setText("FAULT")
        self.monitor_labels["vdc"].setText(f"{snapshot.vdc:.3f} V")
        self.monitor_labels["temperature"].setText(f"{snapshot.temperature:.3f} C")
        self.monitor_labels["cmd_speed"].setText(f"{snapshot.cmd_speed:.3f}")
        self.monitor_labels["act_speed"].setText(f"{snapshot.act_speed:.3f}")
        self.monitor_labels["speed_error"].setText(f"{snapshot.speed_error:.3f}")
        self.monitor_labels["cmd_position"].setText(f"{snapshot.cmd_position:.3f}")
        self.monitor_labels["act_position"].setText(f"{snapshot.act_position:.3f}")
        self.monitor_labels["position_error"].setText(f"{snapshot.position_error:.3f}")
        self.monitor_labels["iq_ref"].setText(f"{snapshot.iq_ref:.3f} A")
        self.monitor_labels["motor_power"].setText(str(snapshot.motor_power))
        self.monitor_labels["fault_occurred"].setText(f"0x{snapshot.fault_code:04X}")
        self.monitor_labels["phase_u"].setText(f"{snapshot.phase_u:.3f} A")
        self.monitor_labels["phase_v"].setText(f"{snapshot.phase_v:.3f} A")
        self.monitor_labels["phase_w"].setText(f"{snapshot.phase_w:.3f} A")
        self._append_log(
            "Driver error packet: "
            f"fault=0x{snapshot.fault_code:04X}, "
            f"phase_u={snapshot.phase_u:.3f}, "
            f"phase_v={snapshot.phase_v:.3f}, "
            f"phase_w={snapshot.phase_w:.3f}"
        )

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        self._worker.stop()
        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
