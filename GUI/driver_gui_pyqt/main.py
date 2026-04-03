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
    CONTROL_TIMING_MODE_16KHZ,
    CONTROL_TIMING_MODE_3KHZ,
    CURRENT_LOOP_FREQUENCY_HZ,
    CURRENT_TUNING_TOTAL_SAMPLES,
    DEFAULT_MOTOR_MAXIMUM_POWER,
    DEFAULT_MOTOR_MAXIMUM_VOLTAGE,
    DEFAULT_MOTOR_PEAK_CURRENT_RMS,
    DEFAULT_MOTOR_POLE_PAIRS,
    DEFAULT_MOTOR_RATED_CURRENT_RMS,
    DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ,
    DEFAULT_MOTOR_RATED_SPEED_RPM,
    ID_SQUARE_ANGLE_TEST_MINUS_90,
    ID_SQUARE_ANGLE_TEST_NONE,
    ID_SQUARE_ANGLE_TEST_PLUS_90,
    ID_SQUARE_ANGLE_TEST_PLUS_180,
    Command,
    DRIVER_PARAMETER_NAMES,
    MOTOR_PARAMETER_NAMES,
    ParsedFrame,
    TraceChunk,
    TRACE_TOTAL_SAMPLES,
    UpdateCode,
    FrameStreamParser,
    build_command_frame,
    build_parameter_write_chunks,
    decode_fault_flags,
    format_fault_text,
    parse_current_tuning_payload,
    parse_error_payload,
    parse_monitor_payload,
    parse_parameter_chunk,
    parse_trace_payload,
)
from transport import PortDescriptor, SerialWorker, list_serial_ports


KNOWN_DEVICE_VID = 1100
KNOWN_DEVICE_PID = 22336
TREND_BUFFER_CAPACITY = 4000
TREND_REFRESH_INTERVAL_MS = 33
TRACE_CAPTURE_REFRESH_INTERVAL_MS = 100

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

DEFAULT_DRIVER_PARAMETER_VALUES: dict[int, float] = {
    12: DEFAULT_MOTOR_RATED_SPEED_RPM,
}

DEFAULT_MOTOR_PARAMETER_VALUES: dict[int, float] = {
    0: DEFAULT_MOTOR_RATED_CURRENT_RMS,
    1: DEFAULT_MOTOR_PEAK_CURRENT_RMS,
    9: DEFAULT_MOTOR_MAXIMUM_POWER,
    10: DEFAULT_MOTOR_MAXIMUM_VOLTAGE,
    11: DEFAULT_MOTOR_POLE_PAIRS,
    13: DEFAULT_MOTOR_RATED_SPEED_RPM,
    19: 1.0,
}

TRACE_CHANNEL_META = {
    0: {"label": "Unused", "unit": "", "color": "#aaaaaa"},
    1: {"label": "Cmd Speed", "unit": "rpm", "color": "#bcbd22"},
    2: {"label": "Act Speed", "unit": "rpm", "color": "#17becf"},
    3: {"label": "Iq Ref", "unit": "A", "color": "#ff7f0e"},
    4: {"label": "Iq", "unit": "A", "color": "#d62728"},
    5: {"label": "Iu", "unit": "A", "color": "#1f77b4"},
    6: {"label": "Iv", "unit": "A", "color": "#2ca02c"},
    7: {"label": "Iw", "unit": "A", "color": "#9467bd"},
    8: {"label": "Vdc", "unit": "V", "color": "#8c564b"},
    9: {"label": "Temp", "unit": "C", "color": "#e377c2"},
    10: {"label": "Pos Err", "unit": "cnt", "color": "#7f7f7f"},
    11: {"label": "Id", "unit": "A", "color": "#393b79"},
    12: {"label": "Id Ref", "unit": "A", "color": "#637939"},
    13: {"label": "Vd", "unit": "V", "color": "#843c39"},
    14: {"label": "Vq", "unit": "V", "color": "#7b4173"},
}

TRACE_PRESETS = {
    "Id Tuning": [12, 11, 13, 14],
    "Current Loop": [3, 4, 11, 14],
    "Phase Currents": [3, 5, 6, 7],
    "Speed Loop": [1, 2, 3, 4],
    "Voltage Debug": [11, 12, 13, 14],
}


@dataclass(slots=True)
class ScopeCaptureState:
    title: str
    sample_period_s: float
    total_samples: int
    series_defs: list[dict[str, str]]
    series_data: list[list[float]]
    source_indices: list[int] | None = None
    active: bool = False
    received_samples: int = 0


def _empty_scope_state(title: str) -> ScopeCaptureState:
    return ScopeCaptureState(
        title=title,
        sample_period_s=1.0 / CURRENT_LOOP_FREQUENCY_HZ,
        total_samples=0,
        series_defs=[],
        series_data=[],
        source_indices=[],
        active=False,
        received_samples=0,
    )


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


class ScopeCaptureView(QtWidgets.QWidget):
    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._title = "Capture"
        self._sample_period_s = 1.0 / CURRENT_LOOP_FREQUENCY_HZ
        self._series_defs: list[dict[str, str]] = []
        self._series_data: list[list[float]] = []
        self._plot_rect = QtCore.QRectF()
        self._hover_index: int | None = None
        self._auto_scale = True
        self._frozen_range = (-1.0, 1.0)
        self._status_text = "Waiting for capture..."
        self.setMinimumHeight(340)
        self.setMouseTracking(True)

    def clear(self) -> None:
        self._series_defs = []
        self._series_data = []
        self._hover_index = None
        self._status_text = "Waiting for capture..."
        self.update()

    def set_auto_scale(self, enabled: bool) -> None:
        self._auto_scale = enabled
        if not enabled:
            self._frozen_range = self._calculate_y_range()
        self.update()

    def set_capture(
        self,
        title: str,
        series_defs: list[dict[str, str]],
        series_data: list[list[float]],
        sample_period_s: float,
        status_text: str,
    ) -> None:
        self._title = title
        self._series_defs = list(series_defs)
        self._series_data = [list(series) for series in series_data]
        self._sample_period_s = max(1e-6, float(sample_period_s))
        self._status_text = status_text
        if self._auto_scale:
            self._frozen_range = self._calculate_y_range()
        self.update()

    def _calculate_y_range(self) -> tuple[float, float]:
        all_values = [value for series in self._series_data for value in series]
        if not all_values:
            return (-1.0, 1.0)
        y_min = min(all_values)
        y_max = max(all_values)
        if abs(y_max - y_min) < 0.05:
            center = 0.5 * (y_max + y_min)
            return (center - 0.5, center + 0.5)
        margin = (y_max - y_min) * 0.12
        return (y_min - margin, y_max + margin)

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        rect = self.rect().adjusted(8, 8, -8, -8)
        painter.fillRect(rect, self.palette().base())
        painter.setPen(self.palette().text().color())
        painter.drawText(
            QtCore.QRect(rect.left(), rect.top(), rect.width(), 20),
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
            self._title,
        )

        legend_y = rect.top() + 28
        legend_x = rect.left()
        for series_def in self._series_defs:
            painter.fillRect(
                QtCore.QRect(legend_x, legend_y, 10, 10),
                QtGui.QColor(series_def["color"]),
            )
            painter.drawText(
                QtCore.QRect(legend_x + 14, legend_y - 3, 120, 18),
                QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
                series_def["label"],
            )
            legend_x += 124

        plot_rect = QtCore.QRectF(rect.left(), legend_y + 22, rect.width(), rect.height() - 64)
        self._plot_rect = plot_rect
        painter.setPen(QtGui.QPen(self.palette().mid().color()))
        painter.drawRect(plot_rect)

        if not self._series_data or not self._series_data[0]:
            painter.drawText(plot_rect.toRect(), QtCore.Qt.AlignmentFlag.AlignCenter, self._status_text)
            return

        y_min, y_max = self._frozen_range
        y_span = max(y_max - y_min, 0.1)
        sample_count = len(self._series_data[0])
        total_ms = (sample_count - 1) * self._sample_period_s * 1000.0

        grid_pen = QtGui.QPen(self.palette().midlight().color())
        grid_pen.setStyle(QtCore.Qt.PenStyle.DashLine)
        painter.setPen(grid_pen)
        for grid_index in range(1, 5):
            y = plot_rect.top() + (plot_rect.height() * grid_index / 5.0)
            painter.drawLine(QtCore.QPointF(plot_rect.left(), y), QtCore.QPointF(plot_rect.right(), y))

        painter.setPen(self.palette().text().color())
        painter.drawText(QtCore.QRectF(plot_rect.right() - 90, plot_rect.top() + 4, 86, 16), QtCore.Qt.AlignmentFlag.AlignRight, f"{y_max:.3f}")
        painter.drawText(QtCore.QRectF(plot_rect.right() - 90, plot_rect.bottom() - 18, 86, 16), QtCore.Qt.AlignmentFlag.AlignRight, f"{y_min:.3f}")

        for marker_ratio in (0.0, 0.5, 1.0):
            x = plot_rect.left() + plot_rect.width() * marker_ratio
            marker_ms = total_ms * marker_ratio
            painter.drawLine(QtCore.QPointF(x, plot_rect.bottom()), QtCore.QPointF(x, plot_rect.bottom() + 4))
            painter.drawText(QtCore.QRectF(x - 28, plot_rect.bottom() + 6, 56, 14), QtCore.Qt.AlignmentFlag.AlignCenter, f"{marker_ms:.1f} ms")

        for series_index, series in enumerate(self._series_data):
            if len(series) < 2:
                continue
            path = QtGui.QPainterPath()
            for index, value in enumerate(series):
                ratio = index / max(sample_count - 1, 1)
                x = plot_rect.left() + plot_rect.width() * ratio
                y = plot_rect.bottom() - (((value - y_min) / y_span) * plot_rect.height())
                point = QtCore.QPointF(x, y)
                if index == 0:
                    path.moveTo(point)
                else:
                    path.lineTo(point)
            painter.setPen(QtGui.QPen(QtGui.QColor(self._series_defs[series_index]["color"]), 1.8))
            painter.drawPath(path)

        if self._hover_index is not None and 0 <= self._hover_index < sample_count:
            ratio = self._hover_index / max(sample_count - 1, 1)
            x = plot_rect.left() + plot_rect.width() * ratio
            cursor_pen = QtGui.QPen(QtGui.QColor("#888888"))
            cursor_pen.setStyle(QtCore.Qt.PenStyle.DashLine)
            painter.setPen(cursor_pen)
            painter.drawLine(QtCore.QPointF(x, plot_rect.top()), QtCore.QPointF(x, plot_rect.bottom()))

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if not self._series_data or not self._series_data[0] or not self._plot_rect.contains(event.position()):
            self._hover_index = None
            QtWidgets.QToolTip.hideText()
            self.update()
            return

        sample_count = len(self._series_data[0])
        ratio = (event.position().x() - self._plot_rect.left()) / max(self._plot_rect.width(), 1.0)
        ratio = min(max(ratio, 0.0), 1.0)
        self._hover_index = min(int(round(ratio * max(sample_count - 1, 0))), sample_count - 1)
        time_ms = self._hover_index * self._sample_period_s * 1000.0
        tooltip_lines = [f"t = {time_ms:.3f} ms"]
        for series_def, series in zip(self._series_defs, self._series_data):
            tooltip_lines.append(f"{series_def['label']}: {series[self._hover_index]:.4f} {series_def['unit']}")
        QtWidgets.QToolTip.showText(event.globalPosition().toPoint(), "\n".join(tooltip_lines), self)
        self.update()

    def leaveEvent(self, event: QtCore.QEvent) -> None:  # noqa: N802
        self._hover_index = None
        QtWidgets.QToolTip.hideText()
        self.update()
        super().leaveEvent(event)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ASD04 Driver GUI (Python)")
        self.resize(1220, 800)

        self._worker = SerialWorker(FrameStreamParser(), self)
        self._connected = False
        self._current_port = ""
        self._pending_frames: deque[PendingFrame] = deque()
        self._awaiting_ack = False
        self._last_sent: PendingFrame | None = None
        self._port_descriptors: list[PortDescriptor] = []
        self._latest_monitor_snapshot = None
        self._active_timing_mode = CONTROL_TIMING_MODE_3KHZ
        self._active_control_loop_hz = 3000.0
        self._active_speed_loop_hz = 1500.0
        self._trend_buffer = CircularSeriesBuffer(
            list(TREND_SERIES_META.keys()),
            capacity=TREND_BUFFER_CAPACITY,
            ema_alpha=TREND_EMA_ALPHA,
        )
        self._trend_panels: list[ScadaTrendPanel] = []
        self._current_tuning_capture = _empty_scope_state("Current Tuning Response")
        self._trace_capture = _empty_scope_state("Firmware Trace Scope")
        self._active_trace_target: str | None = None

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
        self._set_active_timing_profile(self._active_timing_mode)
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
        content_splitter.setStretchFactor(0, 5)
        content_splitter.setStretchFactor(1, 2)
        main_layout.addWidget(content_splitter, 1)

        self._parameter_window = self._build_parameter_window()
        self._trend_window = self._build_trend_window()
        self._tuning_window = self._build_tuning_window()
        self._debug_terminal_window = self._build_debug_terminal_window()
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
        self.timing_mode_combo = QtWidgets.QComboBox()
        self.timing_mode_combo.addItem("3 kHz ISR", CONTROL_TIMING_MODE_3KHZ)
        self.timing_mode_combo.addItem("16 kHz ISR", CONTROL_TIMING_MODE_16KHZ)
        self.timing_mode_combo.setCurrentIndex(0)
        self.apply_timing_mode_button = QtWidgets.QPushButton("Apply Timing")

        self.device_hint_label = QtWidgets.QLabel(
            f"Known device VID:PID = {KNOWN_DEVICE_VID:04X}:{KNOWN_DEVICE_PID:04X}"
        )
        self.connection_state_label = QtWidgets.QLabel("Disconnected")
        self.timing_mode_state_label = QtWidgets.QLabel("Active: 3 kHz (3000 Hz)")
        self.open_parameters_button = QtWidgets.QPushButton("Parameters...")
        self.open_trends_button = QtWidgets.QPushButton("Trends...")
        self.open_tuning_button = QtWidgets.QPushButton("Tuning / Scope...")
        self.open_debug_terminal_button = QtWidgets.QPushButton("Debug Terminal...")
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
        layout.addWidget(QtWidgets.QLabel("Timing"), 0, 9)
        layout.addWidget(self.timing_mode_combo, 0, 10)
        layout.addWidget(self.apply_timing_mode_button, 0, 11)
        layout.addWidget(self.auto_poll_checkbox, 1, 0, 1, 2)
        layout.addWidget(self.device_hint_label, 1, 2, 1, 3)
        layout.addWidget(self.timing_mode_state_label, 1, 5, 1, 2)
        layout.addWidget(self.open_parameters_button, 1, 7)
        layout.addWidget(self.open_trends_button, 1, 8)
        layout.addWidget(self.open_tuning_button, 1, 9)
        layout.addWidget(self.open_debug_terminal_button, 1, 10)
        layout.addWidget(self.open_log_button, 1, 11)
        layout.addWidget(self.connection_state_label, 1, 12)
        return box

    def _build_monitor_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Monitor")
        layout = QtWidgets.QVBoxLayout(box)
        self.monitor_labels: dict[str, QtWidgets.QLabel] = {}

        status_fields = [
            ("enable_run", "Enable Run"),
            ("run_mode", "Run Mode"),
            ("control_timing_mode", "Timing Mode"),
            ("effective_loop_hz", "Loop Freq"),
            ("fault_occurred", "Fault"),
            ("vdc", "Vdc"),
            ("temperature", "Temperature"),
            ("motor_power", "Motor Power"),
        ]
        motion_fields = [
            ("cmd_speed", "Cmd Speed"),
            ("act_speed", "Act Speed"),
            ("speed_error", "Speed Error"),
            ("cmd_position", "Cmd Position"),
            ("act_position", "Act Position"),
            ("position_error", "Position Error"),
        ]
        current_fields = [
            ("id_ref", "Id Ref"),
            ("iq_ref", "Iq Ref"),
            ("phase_u", "Iu"),
            ("phase_v", "Iv"),
            ("phase_w", "Iw"),
            ("id_current", "Id"),
            ("iq_current", "Iq"),
        ]
        voltage_fields = [
            ("vd", "Vd"),
            ("vq", "Vq"),
            ("v_phase_u", "Vu"),
            ("v_phase_v", "Vv"),
            ("v_phase_w", "Vw"),
            ("theta", "Theta"),
            ("vf_frequency", "V/F Freq"),
            ("vf_voltage", "V/F Volt"),
        ]

        dashboard_widget = QtWidgets.QWidget()
        dashboard_layout = QtWidgets.QGridLayout(dashboard_widget)
        dashboard_layout.setContentsMargins(0, 0, 0, 0)
        dashboard_layout.setHorizontalSpacing(10)
        dashboard_layout.setVerticalSpacing(10)
        dashboard_layout.addWidget(
            self._build_monitor_dashboard_section("Status", status_fields, columns=2), 0, 0
        )
        dashboard_layout.addWidget(
            self._build_monitor_dashboard_section("Motion", motion_fields, columns=2), 0, 1
        )
        dashboard_layout.addWidget(
            self._build_monitor_dashboard_section("Currents", current_fields, columns=2), 1, 0
        )
        dashboard_layout.addWidget(
            self._build_monitor_dashboard_section("Voltage && Debug", voltage_fields, columns=2),
            1,
            1,
        )
        dashboard_layout.setColumnStretch(0, 1)
        dashboard_layout.setColumnStretch(1, 1)
        dashboard_layout.setRowStretch(2, 1)

        trend_note = QtWidgets.QLabel(
            "Full-size trend charts are available in the separate Trends window."
        )
        trend_note.setWordWrap(True)
        open_trends_inline_button = QtWidgets.QPushButton("Open Trends Window")
        open_trends_inline_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._trend_window)
        )

        layout.addWidget(dashboard_widget, 1)
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

    def _build_tuning_window(self) -> QtWidgets.QDialog:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Current Tuning / Scope")
        dialog.resize(1280, 900)
        dialog.setModal(False)

        layout = QtWidgets.QVBoxLayout(dialog)
        tabs = QtWidgets.QTabWidget(dialog)
        tabs.addTab(self._build_current_tuning_tab(), "Current Tuning")
        tabs.addTab(self._build_trace_scope_tab(), "Trace Scope")
        layout.addWidget(tabs)
        return dialog

    def _build_current_tuning_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        control_group = QtWidgets.QGroupBox("Id Square-Wave Current Loop Tuning")
        control_layout = QtWidgets.QGridLayout(control_group)
        self.ctuning_ref1_spin = QtWidgets.QDoubleSpinBox()
        self.ctuning_ref1_spin.setRange(0.0, 20.0)
        self.ctuning_ref1_spin.setDecimals(3)
        self.ctuning_ref1_spin.setValue(0.5)
        self.ctuning_ref1_spin.setSuffix(" A")
        self.ctuning_ref2_spin = QtWidgets.QDoubleSpinBox()
        self.ctuning_ref2_spin.setRange(0.5, 50.0)
        self.ctuning_ref2_spin.setDecimals(3)
        self.ctuning_ref2_spin.setValue(10.0)
        self.ctuning_ref2_spin.setSuffix(" Hz")
        self.ctuning_rate_combo = QtWidgets.QComboBox()
        self.ctuning_rate_combo.addItem("16 kHz", 0)
        self.ctuning_rate_combo.addItem("8 kHz", 1)
        self.ctuning_rate_combo.addItem("4 kHz", 3)
        self.ctuning_rate_combo.addItem("2 kHz", 7)
        self.ctuning_rate_combo.addItem("1 kHz", 15)
        self.ctuning_rate_combo.setCurrentIndex(2)
        self.ctuning_kp_spin = QtWidgets.QDoubleSpinBox()
        self.ctuning_kp_spin.setRange(0.0, 10000.0)
        self.ctuning_kp_spin.setDecimals(5)
        self.ctuning_kp_spin.setSingleStep(0.001)
        self.ctuning_ki_spin = QtWidgets.QDoubleSpinBox()
        self.ctuning_ki_spin.setRange(0.0, 10000.0)
        self.ctuning_ki_spin.setDecimals(5)
        self.ctuning_ki_spin.setSingleStep(0.001)
        self.ctuning_angle_test_combo = QtWidgets.QComboBox()
        self.ctuning_angle_test_combo.addItem("None", ID_SQUARE_ANGLE_TEST_NONE)
        self.ctuning_angle_test_combo.addItem("+90e", ID_SQUARE_ANGLE_TEST_PLUS_90)
        self.ctuning_angle_test_combo.addItem("-90e", ID_SQUARE_ANGLE_TEST_MINUS_90)
        self.ctuning_angle_test_combo.addItem("+180e", ID_SQUARE_ANGLE_TEST_PLUS_180)
        self.ctuning_angle_test_combo.setToolTip(
            "Apply a temporary electrical angle shift only during Id tuning after alignment. Use this to test whether the dq frame is off by +90e, -90e, or +180e."
        )
        self.ctuning_invert_current_checkbox = QtWidgets.QCheckBox("Invert Current")
        self.ctuning_invert_current_checkbox.setToolTip(
            "Invert U/V current polarity only during Id tuning."
        )
        self.ctuning_swap_uv_checkbox = QtWidgets.QCheckBox("Swap U/V")
        self.ctuning_swap_uv_checkbox.setToolTip(
            "Swap U/V current channels only during Id tuning."
        )
        ctuning_test_flags_widget = QtWidgets.QWidget()
        ctuning_test_flags_layout = QtWidgets.QHBoxLayout(ctuning_test_flags_widget)
        ctuning_test_flags_layout.setContentsMargins(0, 0, 0, 0)
        ctuning_test_flags_layout.setSpacing(10)
        ctuning_test_flags_layout.addWidget(QtWidgets.QLabel("Angle Test"))
        ctuning_test_flags_layout.addWidget(self.ctuning_angle_test_combo)
        ctuning_test_flags_layout.addWidget(self.ctuning_invert_current_checkbox)
        ctuning_test_flags_layout.addWidget(self.ctuning_swap_uv_checkbox)
        ctuning_test_flags_layout.addStretch(1)

        self.apply_ctuning_button = QtWidgets.QPushButton("Apply Id Tune Config")
        self.start_ctuning_button = QtWidgets.QPushButton("Start Id Tune + Scope")
        self.stop_ctuning_button = QtWidgets.QPushButton("Stop Id Tune")
        self.ctuning_status_label = QtWidgets.QLabel("Idle")

        control_layout.addWidget(QtWidgets.QLabel("Id Amplitude"), 0, 0)
        control_layout.addWidget(self.ctuning_ref1_spin, 0, 1)
        control_layout.addWidget(QtWidgets.QLabel("Square Frequency"), 0, 2)
        control_layout.addWidget(self.ctuning_ref2_spin, 0, 3)
        control_layout.addWidget(QtWidgets.QLabel("Capture Rate"), 1, 0)
        control_layout.addWidget(self.ctuning_rate_combo, 1, 1)
        control_layout.addWidget(QtWidgets.QLabel("Current Kp"), 1, 2)
        control_layout.addWidget(self.ctuning_kp_spin, 1, 3)
        control_layout.addWidget(QtWidgets.QLabel("Current Ki"), 2, 0)
        control_layout.addWidget(self.ctuning_ki_spin, 2, 1)
        control_layout.addWidget(ctuning_test_flags_widget, 2, 2, 1, 2)
        self.ctuning_window_label = QtWidgets.QLabel("")
        control_layout.addWidget(QtWidgets.QLabel("Capture Window"), 3, 2)
        control_layout.addWidget(self.ctuning_window_label, 3, 3)
        control_layout.addWidget(self.apply_ctuning_button, 4, 0, 1, 2)
        control_layout.addWidget(self.start_ctuning_button, 4, 2)
        control_layout.addWidget(self.stop_ctuning_button, 4, 3)
        control_layout.addWidget(QtWidgets.QLabel("Status"), 5, 0)
        control_layout.addWidget(self.ctuning_status_label, 5, 1, 1, 3)

        scope_toolbar = QtWidgets.QHBoxLayout()
        self.ctuning_auto_scale_checkbox = QtWidgets.QCheckBox("Auto-scale Y")
        self.ctuning_auto_scale_checkbox.setChecked(True)
        self.ctuning_clear_button = QtWidgets.QPushButton("Clear Capture")
        scope_hint = QtWidgets.QLabel(
            "Lock the rotor mechanically before use. Start from 5-10% rated current, keep the square-wave frequency low, and only increase amplitude after the trace is stable. Use a lower capture rate if you want a longer time window to see more than one pulse. Firmware clamps Id amplitude, limits the current-loop voltage to a safe fraction of Vdc, keeps Iq at zero, and records Id_ref / Id / Vd / Vq using the active timing profile."
        )
        scope_hint.setWordWrap(True)
        scope_toolbar.addWidget(self.ctuning_auto_scale_checkbox)
        scope_toolbar.addWidget(self.ctuning_clear_button)
        scope_toolbar.addWidget(scope_hint, 1)

        self.ctuning_scope_view = ScopeCaptureView()
        self._update_current_tuning_capture_window_label()

        layout.addWidget(control_group)
        layout.addLayout(scope_toolbar)
        layout.addWidget(self.ctuning_scope_view, 1)
        return widget

    def _build_trace_scope_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        control_group = QtWidgets.QGroupBox("Firmware Trace Capture")
        control_layout = QtWidgets.QGridLayout(control_group)

        self.trace_preset_combo = QtWidgets.QComboBox()
        self.trace_preset_combo.addItems(list(TRACE_PRESETS.keys()))
        self.trace_preset_combo.setCurrentText("Current Loop")
        self.trace_mode_combo = QtWidgets.QComboBox()
        self.trace_mode_combo.addItem("Single Shot", 0)
        self.trace_mode_combo.addItem("Continuous", 1)
        self.trace_rate_combo = QtWidgets.QComboBox()
        self.trace_rate_combo.addItem("16 kHz (every ISR)", 0)
        self.trace_rate_combo.addItem("8 kHz", 1)
        self.trace_rate_combo.addItem("4 kHz", 3)
        self.trace_rate_combo.addItem("2 kHz", 7)
        self.trace_rate_combo.addItem("1 kHz", 15)

        self.trace_channel_combos: list[QtWidgets.QComboBox] = []
        for _ in range(4):
            combo = QtWidgets.QComboBox()
            for code, meta in TRACE_CHANNEL_META.items():
                combo.addItem(f"{code:02d} - {meta['label']}", code)
            self.trace_channel_combos.append(combo)

        self.trace_start_button = QtWidgets.QPushButton("Start Scope Capture")
        self.trace_stop_button = QtWidgets.QPushButton("Stop Scope Capture")
        self.trace_clear_button = QtWidgets.QPushButton("Clear Capture")
        self.trace_auto_scale_checkbox = QtWidgets.QCheckBox("Auto-scale Y")
        self.trace_auto_scale_checkbox.setChecked(True)
        self.trace_status_label = QtWidgets.QLabel("Idle")

        control_layout.addWidget(QtWidgets.QLabel("Preset"), 0, 0)
        control_layout.addWidget(self.trace_preset_combo, 0, 1)
        control_layout.addWidget(QtWidgets.QLabel("Mode"), 0, 2)
        control_layout.addWidget(self.trace_mode_combo, 0, 3)
        control_layout.addWidget(QtWidgets.QLabel("Sample Rate"), 1, 0)
        control_layout.addWidget(self.trace_rate_combo, 1, 1)
        for index, combo in enumerate(self.trace_channel_combos):
            row = 2 + index // 2
            col = (index % 2) * 2
            control_layout.addWidget(QtWidgets.QLabel(f"CH{index + 1}"), row, col)
            control_layout.addWidget(combo, row, col + 1)
        control_layout.addWidget(self.trace_start_button, 4, 0)
        control_layout.addWidget(self.trace_stop_button, 4, 1)
        control_layout.addWidget(self.trace_clear_button, 4, 2)
        control_layout.addWidget(self.trace_auto_scale_checkbox, 4, 3)
        control_layout.addWidget(QtWidgets.QLabel("Status"), 5, 0)
        control_layout.addWidget(self.trace_status_label, 5, 1, 1, 3)

        self.trace_scope_view = ScopeCaptureView()
        layout.addWidget(control_group)
        layout.addWidget(self.trace_scope_view, 1)

        self._apply_trace_preset(self.trace_preset_combo.currentText())
        return widget

    def _build_quick_command_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Controls")
        layout = QtWidgets.QVBoxLayout(box)

        self.servo_on_button = QtWidgets.QPushButton("Servo ON")
        self.servo_off_button = QtWidgets.QPushButton("Servo OFF")
        self.ack_fault_button = QtWidgets.QPushButton("ACK Fault")
        self.save_flash_button = QtWidgets.QPushButton("Write To Flash")
        self.refresh_monitor_button = QtWidgets.QPushButton("Refresh Monitor")

        tabs = QtWidgets.QTabWidget()

        drive_tab = QtWidgets.QWidget()
        drive_layout = QtWidgets.QVBoxLayout(drive_tab)
        button_grid = QtWidgets.QGridLayout()
        button_grid.addWidget(self.servo_on_button, 0, 0)
        button_grid.addWidget(self.servo_off_button, 0, 1)
        button_grid.addWidget(self.ack_fault_button, 0, 2)
        button_grid.addWidget(self.save_flash_button, 0, 3)
        button_grid.addWidget(self.refresh_monitor_button, 1, 0, 1, 2)
        drive_layout.addLayout(button_grid)
        drive_hint = QtWidgets.QLabel(
            "Parameters, trends, tuning/scope, and communication log are available from the top toolbar."
        )
        drive_hint.setWordWrap(True)
        drive_layout.addWidget(drive_hint)
        drive_layout.addStretch(1)

        vf_tab = QtWidgets.QWidget()
        vf_layout = QtWidgets.QGridLayout(vf_tab)
        self.vf_frequency_spin = QtWidgets.QDoubleSpinBox()
        self.vf_frequency_spin.setRange(0.0, DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ)
        self.vf_frequency_spin.setDecimals(2)
        self.vf_frequency_spin.setSingleStep(0.50)
        self.vf_frequency_spin.setSuffix(" Hz")
        self.vf_voltage_spin = QtWidgets.QDoubleSpinBox()
        self.vf_voltage_spin.setRange(0.0, DEFAULT_MOTOR_MAXIMUM_VOLTAGE)
        self.vf_voltage_spin.setDecimals(2)
        self.vf_voltage_spin.setSingleStep(1.0)
        self.vf_voltage_spin.setSuffix(" V")
        self.start_vf_button = QtWidgets.QPushButton("Start V/F")
        self.stop_vf_button = QtWidgets.QPushButton("Stop V/F")
        vf_note = QtWidgets.QLabel(
            "Default motor profile: 4 pole pairs, 3000 rpm, 200 Hz electrical, 91 V, 1.6 A, 200 W. Open-loop commands are still clamped in firmware for safe testing."
        )
        vf_note.setWordWrap(True)
        vf_layout.addWidget(QtWidgets.QLabel("Electrical Frequency"), 0, 0)
        vf_layout.addWidget(self.vf_frequency_spin, 0, 1)
        vf_layout.addWidget(QtWidgets.QLabel("Voltage Reference"), 1, 0)
        vf_layout.addWidget(self.vf_voltage_spin, 1, 1)
        vf_layout.addWidget(self.start_vf_button, 2, 0)
        vf_layout.addWidget(self.stop_vf_button, 2, 1)
        vf_layout.addWidget(vf_note, 3, 0, 1, 2)
        vf_layout.setRowStretch(4, 1)

        tabs.addTab(drive_tab, "Drive")
        tabs.addTab(vf_tab, "Open Loop V/F")
        layout.addWidget(tabs)
        return box

    def _build_monitor_dashboard_section(
        self, title: str, field_names: list[tuple[str, str]], columns: int = 2
    ) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QGridLayout(box)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)
        for index, (key, label) in enumerate(field_names):
            value_label = QtWidgets.QLabel("-")
            value_label.setMinimumWidth(110)
            value_label.setTextInteractionFlags(
                QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
            )
            self.monitor_labels[key] = value_label
            if key == "fault_occurred":
                value_label.setWordWrap(True)
            row = index // columns
            col = (index % columns) * 2
            layout.addWidget(QtWidgets.QLabel(label), row, col)
            layout.addWidget(value_label, row, col + 1)
        for column in range(columns):
            layout.setColumnStretch(column * 2 + 1, 1)
        return box

    def _build_parameter_tabs(self) -> QtWidgets.QTabWidget:
        tabs = QtWidgets.QTabWidget()
        self.driver_table = self._create_parameter_table(DRIVER_PARAMETER_NAMES)
        self.motor_table = self._create_parameter_table(MOTOR_PARAMETER_NAMES)
        tabs.addTab(self.driver_table, "Driver Parameters")
        tabs.addTab(self.motor_table, "Motor Parameters")
        self._populate_default_parameter_tables()
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

    def _populate_default_parameter_tables(self) -> None:
        self._apply_default_values_to_table(self.driver_table, DEFAULT_DRIVER_PARAMETER_VALUES)
        self._apply_default_values_to_table(self.motor_table, DEFAULT_MOTOR_PARAMETER_VALUES)

    @staticmethod
    def _apply_default_values_to_table(
        table: QtWidgets.QTableWidget, defaults: dict[int, float]
    ) -> None:
        for row, value in defaults.items():
            item = table.item(row, 2)
            if item is not None:
                item.setText(f"{value:.6f}")

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

    def _build_debug_terminal_window(self) -> QtWidgets.QDialog:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Drive Debug Terminal")
        dialog.resize(760, 560)
        dialog.setModal(False)
        layout = QtWidgets.QVBoxLayout(dialog)

        toolbar = QtWidgets.QHBoxLayout()
        self.refresh_debug_terminal_button = QtWidgets.QPushButton("Refresh Snapshot")
        self.copy_debug_terminal_button = QtWidgets.QPushButton("Copy Snapshot")
        toolbar.addWidget(self.refresh_debug_terminal_button)
        toolbar.addWidget(self.copy_debug_terminal_button)
        toolbar.addStretch(1)

        self.debug_terminal_edit = QtWidgets.QPlainTextEdit()
        self.debug_terminal_edit.setReadOnly(True)
        self.debug_terminal_edit.setLineWrapMode(
            QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap
        )
        fixed_font = QtGui.QFontDatabase.systemFont(
            QtGui.QFontDatabase.SystemFont.FixedFont
        )
        self.debug_terminal_edit.setFont(fixed_font)
        self.debug_terminal_edit.setPlainText(
            "Waiting for monitor data...\n\n"
            "Connect the driver, press Refresh Snapshot, then copy the text here."
        )

        layout.addLayout(toolbar)
        layout.addWidget(self.debug_terminal_edit)
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
        self.open_tuning_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._tuning_window)
        )
        self.open_debug_terminal_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._debug_terminal_window)
        )
        self.open_log_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._log_window)
        )
        self.refresh_debug_terminal_button.clicked.connect(self._request_monitor_once)
        self.copy_debug_terminal_button.clicked.connect(self._copy_debug_terminal_text)
        self.monitor_rate_combo.currentIndexChanged.connect(self._update_monitor_poll_interval)
        self.apply_timing_mode_button.clicked.connect(self._apply_control_timing_mode)
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
        self.clear_log_button.clicked.connect(self.log_edit.clear)
        self.apply_ctuning_button.clicked.connect(self._apply_current_tuning_parameters)
        self.start_ctuning_button.clicked.connect(self._start_current_tuning)
        self.stop_ctuning_button.clicked.connect(self._stop_current_tuning)
        self.ctuning_clear_button.clicked.connect(self._clear_current_tuning_capture)
        self.ctuning_auto_scale_checkbox.toggled.connect(self.ctuning_scope_view.set_auto_scale)
        self.ctuning_rate_combo.currentIndexChanged.connect(
            self._update_current_tuning_capture_window_label
        )
        self.trace_preset_combo.currentTextChanged.connect(self._apply_trace_preset)
        self.trace_start_button.clicked.connect(self._start_trace_capture)
        self.trace_stop_button.clicked.connect(self._stop_trace_capture)
        self.trace_clear_button.clicked.connect(self._clear_trace_capture)
        self.trace_auto_scale_checkbox.toggled.connect(self.trace_scope_view.set_auto_scale)

        self._worker.frame_received.connect(self._handle_frame)
        self._worker.status_changed.connect(self._handle_connection_status)
        self._worker.log_message.connect(self._handle_worker_log)
        self._worker.error_occurred.connect(self._handle_worker_error)
        self._update_monitor_poll_interval()

    def _show_auxiliary_window(self, window: QtWidgets.QDialog) -> None:
        window.show()
        window.raise_()
        window.activateWindow()

    def _timing_mode_text(self, mode: int) -> str:
        if int(mode) == CONTROL_TIMING_MODE_3KHZ:
            return "3 kHz"
        return "16 kHz"

    def _format_capture_rate_label(self, decimation: int, loop_hz: float) -> str:
        rate_hz = loop_hz / float(decimation + 1)
        if rate_hz >= 1000.0:
            text = f"{rate_hz / 1000.0:.3g} kHz"
        else:
            text = f"{rate_hz:.3g} Hz"
        if decimation == 0:
            text += " (every ISR)"
        return text

    def _refresh_capture_rate_combos(self) -> None:
        combos = [self.ctuning_rate_combo, self.trace_rate_combo]
        decimations = [0, 1, 3, 7, 15]
        for combo in combos:
            current_data = int(combo.currentData() or 0)
            combo.blockSignals(True)
            combo.clear()
            for decimation in decimations:
                combo.addItem(
                    self._format_capture_rate_label(decimation, self._active_control_loop_hz),
                    decimation,
                )
            index = combo.findData(current_data)
            combo.setCurrentIndex(index if index >= 0 else 0)
            combo.blockSignals(False)

        self._update_current_tuning_capture_window_label()

    def _set_active_timing_profile(
        self,
        mode: int,
        current_loop_hz: float | None = None,
        speed_loop_hz: float | None = None,
        *,
        update_combo: bool = True,
    ) -> None:
        mode = CONTROL_TIMING_MODE_3KHZ if int(mode) == CONTROL_TIMING_MODE_3KHZ else CONTROL_TIMING_MODE_16KHZ
        if current_loop_hz is None or current_loop_hz <= 1.0:
            current_loop_hz = 3000.0 if mode == CONTROL_TIMING_MODE_3KHZ else 16000.0
        if speed_loop_hz is None or speed_loop_hz <= 1.0:
            speed_loop_hz = current_loop_hz * 0.5

        self._active_timing_mode = mode
        self._active_control_loop_hz = float(current_loop_hz)
        self._active_speed_loop_hz = float(speed_loop_hz)

        if hasattr(self, "timing_mode_combo") and update_combo:
            index = self.timing_mode_combo.findData(mode)
            if index >= 0:
                self.timing_mode_combo.blockSignals(True)
                self.timing_mode_combo.setCurrentIndex(index)
                self.timing_mode_combo.blockSignals(False)

        if hasattr(self, "timing_mode_state_label"):
            self.timing_mode_state_label.setText(
                f"Active: {self._timing_mode_text(mode)} ({self._active_control_loop_hz:.0f} Hz)"
            )

        if hasattr(self, "ctuning_rate_combo") and hasattr(self, "trace_rate_combo"):
            self._refresh_capture_rate_combos()

    def _update_monitor_poll_interval(self) -> None:
        interval_ms = int(self.monitor_rate_combo.currentData() or 250)
        self._monitor_timer.setInterval(interval_ms)

    def _trace_sample_period_s(self) -> float:
        decimation = int(self.trace_rate_combo.currentData() or 0)
        return (decimation + 1) / max(self._active_control_loop_hz, 1.0)

    def _current_tuning_sample_period_s(self) -> float:
        decimation = int(self.ctuning_rate_combo.currentData() or 0)
        return (decimation + 1) / max(self._active_control_loop_hz, 1.0)

    def _update_current_tuning_capture_window_label(self) -> None:
        window_ms = TRACE_TOTAL_SAMPLES * self._current_tuning_sample_period_s() * 1000.0
        self.ctuning_window_label.setText(f"{window_ms:.1f} ms")

    def _apply_trace_preset(self, preset_name: str) -> None:
        channels = TRACE_PRESETS.get(preset_name)
        if channels is None:
            return
        for combo, channel_code in zip(self.trace_channel_combos, channels):
            combo.setCurrentIndex(combo.findData(channel_code))

    def _clear_current_tuning_capture(self) -> None:
        self._current_tuning_capture = _empty_scope_state("Current Tuning Response")
        self.ctuning_scope_view.clear()
        self.ctuning_status_label.setText("Idle")
        if self._active_trace_target == "current_tuning":
            self._active_trace_target = None

    def _clear_trace_capture(self) -> None:
        self._trace_capture = _empty_scope_state("Firmware Trace Scope")
        self.trace_scope_view.clear()
        self.trace_status_label.setText("Idle")
        if self._active_trace_target == "trace":
            self._active_trace_target = None

    def _apply_current_tuning_parameters(self) -> None:
        payload = struct.pack(
            "<4fBBB",
            float(self.ctuning_ref1_spin.value()),
            float(self.ctuning_ref2_spin.value()),
            float(self.ctuning_kp_spin.value()),
            float(self.ctuning_ki_spin.value()),
            int(self.ctuning_angle_test_combo.currentData() or 0),
            1 if self.ctuning_invert_current_checkbox.isChecked() else 0,
            1 if self.ctuning_swap_uv_checkbox.isChecked() else 0,
        )
        self._enqueue_command(
            Command.CMD_APPLY_ID_SQUARE_TUNING,
            payload,
            "Apply Id Square Tuning",
        )

    def _start_current_tuning(self) -> None:
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None and (
            last_monitor.enable_run or abs(float(last_monitor.act_speed)) > 1.0
        ):
            QtWidgets.QMessageBox.warning(
                self,
                "Current Tuning Safety",
                "Stop the drive and lock the rotor before starting Id tuning. This mode is intended for a stationary rotor with Iq = 0.",
            )
            return
        self._apply_current_tuning_parameters()
        self._clear_current_tuning_capture()
        sample_period_s = self._current_tuning_sample_period_s()
        decimation = int(self.ctuning_rate_combo.currentData() or 0)
        self._current_tuning_capture = ScopeCaptureState(
            title="Id Square-Wave Tuning Response",
            sample_period_s=sample_period_s,
            total_samples=TRACE_TOTAL_SAMPLES,
            series_defs=[
                dict(TRACE_CHANNEL_META[12]),
                dict(TRACE_CHANNEL_META[11]),
                dict(TRACE_CHANNEL_META[13]),
                dict(TRACE_CHANNEL_META[14]),
            ],
            series_data=[[], [], [], []],
            source_indices=[0, 1, 2, 3],
            active=True,
            received_samples=0,
        )
        self._active_trace_target = "current_tuning"
        window_ms = TRACE_TOTAL_SAMPLES * sample_period_s * 1000.0
        self.ctuning_status_label.setText(
            f"Arming trace and starting Id square-wave tuning ({window_ms:.1f} ms window)..."
        )
        trace_payload = bytes([1, 0, 12, 11, 13, 14, decimation])
        self._enqueue_command(Command.CMD_APPLY_TRACE, trace_payload, "Start Id Tuning Trace")
        self._enqueue_command(
            Command.CMD_START_ID_SQUARE_TUNING,
            b"",
            "Start Id Square-Wave Tuning",
        )

    def _stop_current_tuning(self) -> None:
        self._enqueue_command(
            Command.CMD_STOP_ID_SQUARE_TUNING,
            b"",
            "Stop Id Square-Wave Tuning",
        )
        self._enqueue_command(Command.CMD_APPLY_TRACE, bytes([0, 0, 0, 0, 0, 0, 0]), "Stop Id Tuning Trace")
        self._current_tuning_capture.active = False
        self.ctuning_status_label.setText("Stopped")
        if self._active_trace_target == "current_tuning":
            self._active_trace_target = None

    def _start_trace_capture(self) -> None:
        channel_codes = [int(combo.currentData()) for combo in self.trace_channel_combos]
        payload = bytes(
            [
                1,
                int(self.trace_mode_combo.currentData()),
                channel_codes[0],
                channel_codes[1],
                channel_codes[2],
                channel_codes[3],
                int(self.trace_rate_combo.currentData()),
            ]
        )

        source_indices = [index for index, channel_code in enumerate(channel_codes) if channel_code != 0]
        series_defs = [dict(TRACE_CHANNEL_META[channel_codes[index]]) for index in source_indices]
        if not series_defs:
            QtWidgets.QMessageBox.warning(self, "Invalid Trace Setup", "Select at least one trace channel.")
            return

        self._clear_trace_capture()
        self._trace_capture = ScopeCaptureState(
            title="Firmware Trace Scope",
            sample_period_s=self._trace_sample_period_s(),
            total_samples=TRACE_TOTAL_SAMPLES,
            series_defs=series_defs,
            series_data=[[] for _ in series_defs],
            source_indices=source_indices,
            active=True,
            received_samples=0,
        )
        self._active_trace_target = "trace"
        duration_ms = self._trace_capture.total_samples * self._trace_capture.sample_period_s * 1000.0
        self.trace_status_label.setText(
            f"Capturing {self.trace_capture_rate_text()} for ~{duration_ms:.1f} ms"
        )
        self._enqueue_command(Command.CMD_APPLY_TRACE, payload, "Start Firmware Trace")

    def trace_capture_rate_text(self) -> str:
        return self.trace_rate_combo.currentText()

    def _stop_trace_capture(self) -> None:
        self._enqueue_command(Command.CMD_APPLY_TRACE, bytes([0, 0, 0, 0, 0, 0, 0]), "Stop Firmware Trace")
        self._trace_capture.active = False
        self.trace_status_label.setText("Stopped")
        if self._active_trace_target == "trace":
            self._active_trace_target = None

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
            self._clear_current_tuning_capture()
            self._clear_trace_capture()
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
            self._clear_current_tuning_capture()
            self._clear_trace_capture()
            self._clear_monitor_display()
            self._refresh_debug_terminal(None)

    def _handle_worker_log(self, message: str) -> None:
        if message.startswith("TX 02 16 01 1C"):
            return
        if message.startswith("RX 02 16 2A 35"):
            return
        if message.startswith("RX 02 16 A3 34"):
            return
        if message.startswith("RX 02 16 A4 30"):
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

    def _copy_debug_terminal_text(self) -> None:
        QtWidgets.QApplication.clipboard().setText(self.debug_terminal_edit.toPlainText())
        self.statusBar().showMessage("Debug snapshot copied", 2000)

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

    def _apply_control_timing_mode(self) -> None:
        selected_mode = int(self.timing_mode_combo.currentData() or CONTROL_TIMING_MODE_16KHZ)
        payload = bytes([selected_mode & 0xFF])
        self._enqueue_command(
            Command.CMD_SET_CONTROL_TIMING_MODE,
            payload,
            f"Set Control Timing Mode ({self._timing_mode_text(selected_mode)})",
        )
        self._set_active_timing_profile(selected_mode, update_combo=False)
        self._request_monitor_once()

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
            self._set_active_timing_profile(
                snapshot.control_timing_mode,
                snapshot.control_loop_frequency_hz,
                snapshot.speed_loop_frequency_hz,
            )
            self._append_snapshot_to_trend_buffer(snapshot)
            self._update_monitor(snapshot)
            self._refresh_debug_terminal(snapshot)
            return

        if subcommand == UpdateCode.MTR_CODE_ERROR:
            try:
                snapshot = parse_error_payload(body)
            except ValueError as exc:
                self._append_log(f"Error payload parse error: {exc}")
                return
            self._update_error_snapshot(snapshot)
            return

        if subcommand == UpdateCode.CMD_CTUNNING_DATA:
            try:
                chunk = parse_current_tuning_payload(body)
            except ValueError as exc:
                self._append_log(f"Current tuning parse error: {exc}")
                return
            self._handle_current_tuning_chunk(chunk)
            return

        if subcommand == UpdateCode.CMD_TRACE_DATA:
            try:
                chunk = parse_trace_payload(body)
            except ValueError as exc:
                self._append_log(f"Trace parse error: {exc}")
                return
            self._handle_trace_chunk(chunk)
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

    def _update_scope_view(
        self,
        view: ScopeCaptureView,
        capture: ScopeCaptureState,
        status_prefix: str,
    ) -> None:
        expected = capture.total_samples or max((len(series) for series in capture.series_data), default=0)
        status_text = f"{status_prefix}: {capture.received_samples}/{expected} samples"
        if capture.series_defs and capture.series_data:
            view.set_capture(
                capture.title,
                capture.series_defs,
                capture.series_data,
                capture.sample_period_s,
                status_text,
            )

    def _handle_current_tuning_chunk(self, chunk) -> None:
        if not self._current_tuning_capture.series_defs:
            self._current_tuning_capture = ScopeCaptureState(
                title="Current Tuning Response",
                sample_period_s=1.0 / max(self._active_control_loop_hz, 1.0),
                total_samples=CURRENT_TUNING_TOTAL_SAMPLES,
                series_defs=[
                    {"label": "Reference", "unit": "A", "color": "#ff7f0e"},
                    {"label": "Feedback", "unit": "A", "color": "#1f77b4"},
                ],
                series_data=[[], []],
                source_indices=[0, 1],
                active=True,
                received_samples=0,
            )

        if chunk.chunk_index == 0:
            self._current_tuning_capture.series_data = [[], []]
            self._current_tuning_capture.received_samples = 0

        self._current_tuning_capture.series_data[0].extend(chunk.reference)
        self._current_tuning_capture.series_data[1].extend(chunk.feedback)
        self._current_tuning_capture.received_samples = len(self._current_tuning_capture.series_data[0])
        self._current_tuning_capture.active = self._current_tuning_capture.received_samples < self._current_tuning_capture.total_samples
        if self._current_tuning_capture.active:
            self.ctuning_status_label.setText(
                f"Received {self._current_tuning_capture.received_samples}/{self._current_tuning_capture.total_samples} samples"
            )
        else:
            self.ctuning_status_label.setText("Capture complete")
        self._update_scope_view(self.ctuning_scope_view, self._current_tuning_capture, "Current tuning")

    def _handle_trace_chunk(self, chunk: TraceChunk) -> None:
        capture_kind = self._active_trace_target
        if capture_kind == "current_tuning":
            capture = self._current_tuning_capture
            view = self.ctuning_scope_view
            status_label = self.ctuning_status_label
        else:
            capture = self._trace_capture
            view = self.trace_scope_view
            status_label = self.trace_status_label

        if not capture.series_defs:
            status_label.setText("Trace chunk received without an active trace setup")
            return

        source_indices = capture.source_indices or list(range(len(capture.series_defs)))
        active_series_count = len(source_indices)
        if chunk.chunk_index == 0:
            capture.series_data = [[] for _ in range(active_series_count)]
            capture.received_samples = 0

        for series_index, source_index in enumerate(source_indices):
            capture.series_data[series_index].extend(chunk.channels[source_index])
        capture.received_samples = len(capture.series_data[0])
        capture.active = capture.received_samples < capture.total_samples

        continuous_mode = False
        if capture_kind == "trace":
            continuous_mode = int(self.trace_mode_combo.currentData()) == 1

        if capture.active or continuous_mode:
            status_label.setText(
                f"Received {capture.received_samples}/{capture.total_samples} samples"
            )
        else:
            status_label.setText("Capture complete")
            if capture_kind == "current_tuning":
                self._active_trace_target = None
        prefix = "Id tuning" if capture_kind == "current_tuning" else "Trace"
        self._update_scope_view(view, capture, prefix)

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

    def _format_debug_terminal_text(self, snapshot) -> str:
        timestamp = QtCore.QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")
        expected_delta = float(snapshot.debug_expected_delta_pos_sync)
        actual_delta = float(snapshot.debug_delta_pos)
        delta_ratio = actual_delta / expected_delta if abs(expected_delta) > 1e-6 else 0.0
        delta_ratio_avg = (
            float(snapshot.debug_delta_pos_avg) / expected_delta
            if abs(expected_delta) > 1e-6
            else 0.0
        )
        sync_rpm = float(snapshot.debug_open_loop_sync_rpm_cmd)
        speed_ratio = float(snapshot.act_speed) / sync_rpm if abs(sync_rpm) > 1e-6 else 0.0
        speed_ratio_raw_avg = (
            float(snapshot.debug_speed_raw_rpm_avg) / sync_rpm
            if abs(sync_rpm) > 1e-6
            else 0.0
        )
        theta_deg = float(snapshot.theta) * 180.0 / 3.141592653589793
        mechanical_deg = (
            float(snapshot.debug_mechanical_angle_rad) * 180.0 / 3.141592653589793
        )
        electrical_deg = (
            float(snapshot.debug_electrical_angle_rad) * 180.0 / 3.141592653589793
        )

        lines = [
            f"Timestamp: {timestamp}",
            f"Port: {self._current_port or '-'}",
            f"RunMode: {self._run_mode_text(snapshot.run_mode)}",
            f"ControlTimingMode: {self._timing_mode_text(snapshot.control_timing_mode)}",
            f"ControlLoopHz: {snapshot.control_loop_frequency_hz:.6f}",
            f"SpeedLoopHz: {snapshot.speed_loop_frequency_hz:.6f}",
            f"EnableRun: {'ON' if snapshot.enable_run else 'OFF'}",
            f"Fault: {format_fault_text(snapshot.fault_occurred)}",
            "",
            "[OpenLoopCommand]",
            f"OpenLoopElectricalHzCmd: {snapshot.debug_open_loop_electrical_hz_cmd:.6f}",
            f"OpenLoopSyncRpmCmd: {snapshot.debug_open_loop_sync_rpm_cmd:.6f}",
            f"ExpectedDeltaPosSync: {snapshot.debug_expected_delta_pos_sync:.6f}",
            f"IsrDeltaCycles: {snapshot.debug_isr_delta_cycles}",
            f"IsrPeriodUs: {snapshot.debug_isr_period_us:.6f}",
            f"IsrFrequencyHz: {snapshot.debug_isr_frequency_hz:.6f}",
            f"IsrMeasureOnlyMode: {snapshot.debug_isr_measure_only_mode}",
            f"IsrMeasureEdgeFrequencyHz: {snapshot.debug_isr_measure_edge_frequency_hz:.6f}",
            "",
            "[ObservedFeedback]",
            f"ActSpeedFilteredRpm: {snapshot.act_speed:.6f}",
            f"SpeedRawRpm: {snapshot.debug_speed_raw_rpm:.6f}",
            f"SpeedRawRpmAvg: {snapshot.debug_speed_raw_rpm_avg:.6f}",
            f"ObservedElectricalHz: {snapshot.debug_observed_electrical_hz:.6f}",
            f"ObservedElectricalHzAvg: {snapshot.debug_observed_electrical_hz_avg:.6f}",
            f"DeltaPos: {snapshot.debug_delta_pos}",
            f"DeltaPosAvg: {snapshot.debug_delta_pos_avg:.6f}",
            f"EncSingleTurn: {snapshot.debug_enc_single_turn}",
            f"EncoderTurns: {snapshot.debug_encoder_turns:.6f}",
            "",
            "[Angles]",
            f"ThetaRad: {snapshot.theta:.6f}",
            f"ThetaDeg: {theta_deg:.6f}",
            f"MechanicalAngleRad: {snapshot.debug_mechanical_angle_rad:.6f}",
            f"MechanicalAngleDeg: {mechanical_deg:.6f}",
            f"ElectricalAngleRad: {snapshot.debug_electrical_angle_rad:.6f}",
            f"ElectricalAngleDeg: {electrical_deg:.6f}",
            "",
            "[Ratios]",
            f"DeltaPosRatioActualOverExpected: {delta_ratio:.6f}",
            f"DeltaPosAvgRatioOverExpected: {delta_ratio_avg:.6f}",
            f"SpeedRatioActualOverSync: {speed_ratio:.6f}",
            f"SpeedRawAvgRatioOverSync: {speed_ratio_raw_avg:.6f}",
            "",
            "[Monitor]",
            f"Vdc: {snapshot.vdc:.6f}",
            f"Temperature: {snapshot.temperature:.6f}",
            f"IdRef: {snapshot.id_ref:.6f}",
            f"IqRef: {snapshot.iq_ref:.6f}",
            f"Iu: {snapshot.phase_u:.6f}",
            f"Iv: {snapshot.phase_v:.6f}",
            f"Iw: {snapshot.phase_w:.6f}",
            f"Id: {snapshot.id_current:.6f}",
            f"Iq: {snapshot.iq_current:.6f}",
            f"Vd: {snapshot.vd:.6f}",
            f"Vq: {snapshot.vq:.6f}",
            f"ActPosition: {snapshot.act_position:.6f}",
        ]
        return "\n".join(lines)

    def _refresh_debug_terminal(self, snapshot) -> None:
        if snapshot is None:
            self.debug_terminal_edit.setPlainText(
                "Waiting for monitor data...\n\n"
                "Connect the driver, press Refresh Snapshot, then copy the text here."
            )
            return
        self.debug_terminal_edit.setPlainText(self._format_debug_terminal_text(snapshot))

    def _clear_monitor_display(self) -> None:
        for label in self.monitor_labels.values():
            label.setText("-")
            label.setToolTip("")
            label.setStyleSheet("")
        self.monitor_labels["enable_run"].setText("OFF")
        self.monitor_labels["run_mode"].setText("Idle")
        self.monitor_labels["control_timing_mode"].setText(self._timing_mode_text(self._active_timing_mode))
        self.monitor_labels["effective_loop_hz"].setText(f"{self._active_control_loop_hz:.0f} Hz")
        self.monitor_labels["fault_occurred"].setText("OK")
        self.monitor_labels["fault_occurred"].setStyleSheet(
            "color: #6aa84f; font-weight: 600;"
        )

    def _fault_summary_text(self, fault_code: int) -> str:
        labels = decode_fault_flags(int(fault_code))
        return ", ".join(labels)

    def _set_fault_label(self, fault_code: int) -> None:
        label = self.monitor_labels["fault_occurred"]
        summary = self._fault_summary_text(fault_code)
        label.setText(summary)
        label.setToolTip(format_fault_text(fault_code))
        if fault_code == 0:
            label.setStyleSheet("color: #6aa84f; font-weight: 600;")
        else:
            label.setStyleSheet("color: #d9534f; font-weight: 600;")

    def _update_monitor(self, snapshot) -> None:
        self.monitor_labels["enable_run"].setText("ON" if snapshot.enable_run else "OFF")
        self.monitor_labels["run_mode"].setText(self._run_mode_text(snapshot.run_mode))
        self.monitor_labels["control_timing_mode"].setText(self._timing_mode_text(snapshot.control_timing_mode))
        self.monitor_labels["effective_loop_hz"].setText(f"{snapshot.control_loop_frequency_hz:.0f} Hz")
        self._set_fault_label(snapshot.fault_occurred)
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
        self.monitor_labels["control_timing_mode"].setText(self._timing_mode_text(self._active_timing_mode))
        self.monitor_labels["effective_loop_hz"].setText(f"{self._active_control_loop_hz:.0f} Hz")
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
        self._set_fault_label(snapshot.fault_code)
        self.monitor_labels["phase_u"].setText(f"{snapshot.phase_u:.3f} A")
        self.monitor_labels["phase_v"].setText(f"{snapshot.phase_v:.3f} A")
        self.monitor_labels["phase_w"].setText(f"{snapshot.phase_w:.3f} A")
        self._append_log(
            "Driver error packet: "
            f"{format_fault_text(snapshot.fault_code)}, "
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
