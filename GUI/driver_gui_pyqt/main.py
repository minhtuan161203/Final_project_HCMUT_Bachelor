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
    AutoTuneChunk,
    CONTROL_TIMING_MODE_16KHZ,
    CURRENT_CALIB_STATUS_DONE,
    CURRENT_CALIB_STATUS_IDLE,
    CURRENT_CALIB_STATUS_OFFSET_INVALID,
    CURRENT_CALIB_STATUS_RUNNING,
    CURRENT_CALIB_STATUS_TIMEOUT,
    ENCODER_ALIGNMENT_POLICY_MANUAL_SAVE,
    ENCODER_ALIGNMENT_POLICY_POWER_ON,
    ENCODER_ALIGNMENT_STATUS_DONE,
    ENCODER_ALIGNMENT_STATUS_FAULT,
    ENCODER_ALIGNMENT_STATUS_IDLE,
    ENCODER_ALIGNMENT_STATUS_REQUESTED,
    ENCODER_ALIGNMENT_STATUS_RUNNING,
    CURRENT_LOOP_FREQUENCY_HZ,
    FOC_DIRECTION_TEST_DONE_FLIPPED,
    FOC_DIRECTION_TEST_DONE_OK,
    FOC_DIRECTION_TEST_FAULT,
    FOC_DIRECTION_TEST_IDLE,
    FOC_DIRECTION_TEST_INCONCLUSIVE,
    FOC_DIRECTION_TEST_RUNNING,
    ID_SQUARE_ANGLE_TEST_NONE,
    ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD,
    ID_SQUARE_TUNING_MODE_SQUARE_WAVE,
    MOTOR_AUTOTUNE_CHART_LS,
    MOTOR_AUTOTUNE_CHART_NONE,
    MOTOR_AUTOTUNE_CHART_RS,
    MOTOR_AUTOTUNE_ERROR_INVALID_CONFIG,
    MOTOR_AUTOTUNE_ERROR_NONE,
    MOTOR_AUTOTUNE_ERROR_OVERCURRENT,
    MOTOR_AUTOTUNE_ERROR_SIGNAL,
    MOTOR_AUTOTUNE_ERROR_STALL,
    MOTOR_AUTOTUNE_STATE_DONE,
    MOTOR_AUTOTUNE_STATE_ERROR,
    MOTOR_AUTOTUNE_STATE_FLUX,
    MOTOR_AUTOTUNE_STATE_IDLE,
    MOTOR_AUTOTUNE_STATE_LS,
    MOTOR_AUTOTUNE_STATE_RS,
    CURRENT_TUNING_TOTAL_SAMPLES,
    DEFAULT_MOTOR_MAXIMUM_POWER,
    DEFAULT_MOTOR_MAXIMUM_VOLTAGE,
    DEFAULT_MOTOR_PEAK_CURRENT_RMS,
    DEFAULT_MOTOR_POLE_PAIRS,
    DEFAULT_MOTOR_RATED_CURRENT_RMS,
    DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ,
    DEFAULT_MOTOR_RATED_SPEED_RPM,
    POSITION_CONTROL_MODE,
    RUN_MODE_ALIGNMENT_ONLY,
    RUN_MODE_AUTOTUNE,
    RUN_MODE_FOC,
    RUN_MODE_OPEN_LOOP_VF,
    SPEED_CONTROL_MODE,
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
    parse_autotune_payload,
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
    "id_ref": {"label": "Id Ref", "unit": "A", "color": "#8dd3c7"},
    "id_current": {"label": "Id", "unit": "A", "color": "#9467bd"},
    "iq_ref": {"label": "Iq Ref", "unit": "A", "color": "#fdb462"},
    "iq_current": {"label": "Iq", "unit": "A", "color": "#ff7f0e"},
    "vd": {"label": "Vd", "unit": "V", "color": "#8c564b"},
    "vq": {"label": "Vq", "unit": "V", "color": "#e377c2"},
    "act_speed": {"label": "Act Speed", "unit": "rpm", "color": "#17becf"},
    "cmd_speed": {"label": "Cmd Speed", "unit": "rpm", "color": "#bcbd22"},
    "speed_error": {"label": "Speed Error", "unit": "rpm", "color": "#7f7f7f"},
}
TREND_EMA_ALPHA = {
    "phase_u": None,
    "phase_v": None,
    "phase_w": None,
    "id_ref": None,
    "id_current": 0.18,
    "iq_ref": None,
    "iq_current": 0.18,
    "vd": 0.22,
    "vq": 0.22,
    "act_speed": 0.25,
    "cmd_speed": None,
    "speed_error": 0.22,
}

DEFAULT_DRIVER_PARAMETER_VALUES: dict[int, float] = {
    1: float(SPEED_CONTROL_MODE),
    2: 0.05,
    5: 0.02,
    6: 5.0,
    10: 250.0,
    11: 250.0,
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

DRIVER_PARAM_CONTROL_MODE = 1
DRIVER_PARAM_POSITION_P_GAIN = 2
DRIVER_PARAM_SPEED_P_GAIN = 5
DRIVER_PARAM_SPEED_I_GAIN = 6
DRIVER_PARAM_ACCEL_TIME_MS = 10
DRIVER_PARAM_DECEL_TIME_MS = 11
DRIVER_PARAM_MAXIMUM_SPEED = 12
MOTOR_PARAM_CURRENT_P_GAIN = MOTOR_PARAMETER_NAMES.index("MOTOR_CURRENT_P_GAIN")
MOTOR_PARAM_CURRENT_I_GAIN = MOTOR_PARAMETER_NAMES.index("MOTOR_CURRENT_I_GAIN")

TRACE_PRESETS = {
    "Encoder Alignment": [12, 11, 4, 14],
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


@dataclass(slots=True)
class AlarmHistoryEntry:
    timestamp_text: str
    severity_text: str
    source_text: str
    fault_code: int
    summary_text: str


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
        self.setWindowTitle("ASD04 Servo Commissioning Console")
        self.resize(1220, 800)

        self._worker = SerialWorker(FrameStreamParser(), self)
        self._connected = False
        self._current_port = ""
        self._pending_frames: deque[PendingFrame] = deque()
        self._awaiting_ack = False
        self._last_sent: PendingFrame | None = None
        self._port_descriptors: list[PortDescriptor] = []
        self._latest_monitor_snapshot = None
        self._active_timing_mode = CONTROL_TIMING_MODE_16KHZ
        self._active_control_loop_hz = 16000.0
        self._active_speed_loop_hz = 8000.0
        self._capture_rate_combo_loop_hz: float | None = None
        self._trend_buffer = CircularSeriesBuffer(
            list(TREND_SERIES_META.keys()),
            capacity=TREND_BUFFER_CAPACITY,
            ema_alpha=TREND_EMA_ALPHA,
        )
        self._trend_panels: list[ScadaTrendPanel] = []
        self._current_tuning_capture = _empty_scope_state("Current Tuning Response")
        self._autotune_capture = _empty_scope_state("Motor Auto-Tune Charts")
        self._trace_capture = _empty_scope_state("Firmware Trace Scope")
        self._active_trace_target: str | None = None
        self._alarm_history: list[AlarmHistoryEntry] = []
        self._active_fault_code = 0
        self._active_alarm_summary_text = "No active alarms"
        self._active_alarm_detail_text = "No active alarms"
        self._active_alarm_source_text = "-"
        self._active_alarm_severity_text = "OK"
        self._foc_speed_target_rpm = 300.0
        self._foc_position_speed_limit_rpm = DEFAULT_MOTOR_RATED_SPEED_RPM
        self._last_foc_mode_ui = SPEED_CONTROL_MODE

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
        main_layout.addWidget(self._build_alarm_banner())
        content_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal, self)
        content_splitter.addWidget(self._build_monitor_tabs())
        content_splitter.addWidget(self._build_quick_command_group())
        content_splitter.setStretchFactor(0, 5)
        content_splitter.setStretchFactor(1, 2)
        main_layout.addWidget(content_splitter, 1)

        self._trend_window = self._build_trend_window()
        self._tuning_window = self._build_tuning_window()
        self._debug_terminal_window = self._build_debug_terminal_window()
        self._log_window = self._build_log_window()
        self._load_foc_controls_from_driver_table()

        self.statusBar().showMessage(f"Ready ({QT_API})")

    def _build_alarm_banner(self) -> QtWidgets.QFrame:
        frame = QtWidgets.QFrame(self)
        frame.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(10, 8, 10, 8)

        self.alarm_banner_title_label = QtWidgets.QLabel("Alarm Status")
        self.alarm_banner_title_label.setStyleSheet("font-weight: 700;")
        self.alarm_banner_status_label = QtWidgets.QLabel("No active alarms")
        self.alarm_banner_detail_label = QtWidgets.QLabel("Drive protection is idle.")
        self.alarm_banner_detail_label.setWordWrap(True)
        self.alarm_banner_reset_button = QtWidgets.QPushButton("Reset Alarm")

        layout.addWidget(self.alarm_banner_title_label)
        layout.addWidget(self.alarm_banner_status_label)
        layout.addWidget(self.alarm_banner_detail_label, 1)
        layout.addWidget(self.alarm_banner_reset_button)

        self._set_alarm_banner_state(0, "No active alarms", "No active alarms", "OK")
        return frame

    def _build_connection_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Connection")
        layout = QtWidgets.QGridLayout(box)

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports_button = QtWidgets.QPushButton("Refresh Ports")
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.setCheckable(True)
        self.connect_button.setStyleSheet("background-color: #2e7d32; color: white; font-weight: 600;")

        self.auto_poll_checkbox = QtWidgets.QCheckBox("Auto Monitor")
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
        self.connection_state_label.setStyleSheet("color: #c62828; font-weight: 600;")
        self.timing_mode_state_label = QtWidgets.QLabel("Fixed: 16 kHz (16000 Hz)")
        self.open_trends_button = QtWidgets.QPushButton("Trend Charts...")
        self.open_tuning_button = QtWidgets.QPushButton("Commissioning...")
        self.open_debug_terminal_button = QtWidgets.QPushButton("Snapshot...")
        self.open_log_button = QtWidgets.QPushButton("Log...")

        layout.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        layout.addWidget(self.port_combo, 0, 1)
        layout.addWidget(self.refresh_ports_button, 0, 2)
        layout.addWidget(self.connect_button, 0, 3)
        layout.addWidget(self.connection_state_label, 0, 4)
        layout.addWidget(self.auto_poll_checkbox, 0, 5)
        layout.addWidget(QtWidgets.QLabel("Rate"), 0, 6)
        layout.addWidget(self.monitor_rate_combo, 0, 7)
        layout.addWidget(QtWidgets.QLabel("Timing"), 0, 8)
        layout.addWidget(self.timing_mode_state_label, 0, 9, 1, 2)
        layout.addWidget(self.device_hint_label, 1, 0, 1, 5)
        layout.addWidget(self.open_trends_button, 1, 6)
        layout.addWidget(self.open_tuning_button, 1, 7)
        layout.addWidget(self.open_debug_terminal_button, 1, 8)
        layout.addWidget(self.open_log_button, 1, 9)
        return box

    def _build_monitor_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Drive Monitor")
        layout = QtWidgets.QVBoxLayout(box)
        self.monitor_labels: dict[str, QtWidgets.QLabel] = {}

        status_fields = [
            ("enable_run", "Servo"),
            ("run_mode", "Run Mode"),
            ("control_timing_mode", "Timing"),
            ("effective_loop_hz", "Loop Freq"),
            ("fault_occurred", "Fault"),
            ("vdc", "Vdc"),
            ("temperature", "Temp"),
            ("calibration_status", "Cal Status"),
            ("adc_offset_ia", "ADC Offset Ia"),
            ("adc_offset_ib", "ADC Offset Ib"),
            ("alignment_status", "Align Status"),
            ("alignment_offset", "Enc Offset"),
            ("alignment_save", "Align Save"),
        ]
        motion_fields = [
            ("cmd_speed", "Cmd Speed"),
            ("act_speed", "Act Speed"),
            ("speed_error", "Speed Error"),
            ("act_position", "Act Position"),
            ("position_error", "Position Error"),
        ]
        current_fields = [
            ("id_ref", "Id Ref"),
            ("id_current", "Id"),
            ("iq_ref", "Iq Ref"),
            ("iq_current", "Iq"),
            ("phase_u", "Iu"),
            ("phase_v", "Iv"),
            ("phase_w", "Iw"),
        ]
        voltage_fields = [
            ("vd", "Vd"),
            ("vq", "Vq"),
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
            self._build_monitor_dashboard_section("Voltages", voltage_fields, columns=2),
            1,
            1,
        )
        dashboard_layout.setColumnStretch(0, 1)
        dashboard_layout.setColumnStretch(1, 1)
        dashboard_layout.setRowStretch(2, 1)

        trend_note = QtWidgets.QLabel(
            "Main monitor shows the commissioning essentials. Use Trend Charts and Snapshot for deeper traces and debug detail."
        )
        trend_note.setWordWrap(True)
        open_trends_inline_button = QtWidgets.QPushButton("Open Trend Charts")
        open_trends_inline_button.clicked.connect(
            lambda: self._show_auxiliary_window(self._trend_window)
        )

        layout.addWidget(dashboard_widget, 1)
        layout.addWidget(trend_note)
        layout.addWidget(open_trends_inline_button, 0, QtCore.Qt.AlignmentFlag.AlignRight)
        return box

    def _build_monitor_tabs(self) -> QtWidgets.QTabWidget:
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._build_monitor_group(), "Driver Monitor")
        tabs.addTab(self._build_parameter_panel(), "Parameters")
        return tabs

    def _build_trend_window(self) -> QtWidgets.QDialog:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Trend Charts")
        dialog.resize(1220, 860)
        self._configure_auxiliary_window(dialog)

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
            ["id_ref", "id_current", "iq_ref", "iq_current"],
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
        dialog.setWindowTitle("Commissioning / Scope")
        dialog.resize(1280, 920)
        self._configure_auxiliary_window(dialog)

        layout = QtWidgets.QVBoxLayout(dialog)
        tabs = QtWidgets.QTabWidget(dialog)
        tabs.addTab(self._build_id_tuning_tab(), "Id Tuning")
        tabs.addTab(self._build_autotune_tab(), "Motor Auto-Tune")
        tabs.addTab(self._build_trace_scope_tab(), "Trace Scope")
        layout.addWidget(tabs)
        return dialog

    def _build_current_tuning_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        alignment_group = QtWidgets.QGroupBox("Encoder Alignment")
        alignment_layout = QtWidgets.QGridLayout(alignment_group)
        self.alignment_policy_value_label = QtWidgets.QLabel("Waiting for monitor data")
        self.alignment_status_value_label = QtWidgets.QLabel("Idle")
        self.alignment_active_offset_value_label = QtWidgets.QLabel("0 counts")
        self.alignment_captured_offset_value_label = QtWidgets.QLabel("0 counts")
        self.alignment_save_state_value_label = QtWidgets.QLabel("No pending save")
        self.run_alignment_button = QtWidgets.QPushButton("Run Encoder Alignment Only")
        self.save_alignment_flash_button = QtWidgets.QPushButton("Save Offset to Flash")
        alignment_hint = QtWidgets.QLabel(
            "Use Encoder Alignment as an explicit commissioning step. For incremental encoders, run alignment again after every power-up before closing the loop. Absolute encoders can usually reuse the stored offset immediately and only need alignment when commissioning or after mechanical changes."
        )
        alignment_hint.setWordWrap(True)
        alignment_layout.addWidget(QtWidgets.QLabel("Policy"), 0, 0)
        alignment_layout.addWidget(self.alignment_policy_value_label, 0, 1)
        alignment_layout.addWidget(QtWidgets.QLabel("Status"), 0, 2)
        alignment_layout.addWidget(self.alignment_status_value_label, 0, 3)
        alignment_layout.addWidget(QtWidgets.QLabel("Active Offset"), 1, 0)
        alignment_layout.addWidget(self.alignment_active_offset_value_label, 1, 1)
        alignment_layout.addWidget(QtWidgets.QLabel("Captured Offset"), 1, 2)
        alignment_layout.addWidget(self.alignment_captured_offset_value_label, 1, 3)
        alignment_layout.addWidget(QtWidgets.QLabel("Flash Save"), 2, 0)
        alignment_layout.addWidget(self.alignment_save_state_value_label, 2, 1, 1, 3)
        alignment_layout.addWidget(self.run_alignment_button, 3, 0, 1, 2)
        alignment_layout.addWidget(self.save_alignment_flash_button, 3, 2, 1, 2)
        alignment_layout.addWidget(alignment_hint, 4, 0, 1, 4)
        self._refresh_alignment_panel(None)

        layout.addWidget(alignment_group)
        layout.addStretch(1)
        return widget

    def _build_id_tuning_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        control_group = QtWidgets.QGroupBox("Id Current Loop Tuning")
        control_layout = QtWidgets.QGridLayout(control_group)

        self.ctuning_ref1_spin = QtWidgets.QDoubleSpinBox()
        self.ctuning_ref1_spin.setRange(0.0, 20.0)
        self.ctuning_ref1_spin.setDecimals(3)
        self.ctuning_ref1_spin.setValue(0.3)
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
        self.ctuning_kp_spin.setValue(5.0)

        self.ctuning_ki_spin = QtWidgets.QDoubleSpinBox()
        self.ctuning_ki_spin.setRange(0.0, 10000.0)
        self.ctuning_ki_spin.setDecimals(5)
        self.ctuning_ki_spin.setSingleStep(0.001)
        self.ctuning_ki_spin.setValue(100.0)

        self.ctuning_theta_mode_value_label = QtWidgets.QLabel("Forced = 0 rad")
        self.ctuning_theta_mode_value_label.setToolTip(
            "Id tuning runs in a fixed electrical frame after encoder alignment."
        )
        self.ctuning_current_polarity_value_label = QtWidgets.QLabel("Forced by driver")
        self.ctuning_current_polarity_value_label.setToolTip(
            "Current polarity inversion is a fixed hardware characteristic on this driver."
        )
        self.ctuning_foc_gain_scale_spin = QtWidgets.QDoubleSpinBox()
        self.ctuning_foc_gain_scale_spin.setRange(0.10, 1.50)
        self.ctuning_foc_gain_scale_spin.setDecimals(2)
        self.ctuning_foc_gain_scale_spin.setSingleStep(0.05)
        self.ctuning_foc_gain_scale_spin.setValue(0.70)
        self.ctuning_foc_gain_scale_spin.setSuffix(" x")

        self.ctuning_window_label = QtWidgets.QLabel("")
        self.ctuning_status_label = QtWidgets.QLabel("Ready")
        self.ctuning_hint_label = QtWidgets.QLabel(
            "Flow: confirm Encoder Alignment is done in Driver Monitor, lock the rotor mechanically, then use a small Id square-wave to tune the d-axis PI. During this test theta is forced to 0 and q-axis is isolated. After the trace is acceptable, copy the same gains into both Id/Iq current PI loops for FOC with a user-defined scale."
        )
        self.ctuning_hint_label.setWordWrap(True)

        self.apply_ctuning_gains_to_foc_button = QtWidgets.QPushButton("Apply Current PI To FOC (Id + Iq)")
        self.start_ctuning_button = QtWidgets.QPushButton("Start Id Tune + Scope")
        self.stop_ctuning_button = QtWidgets.QPushButton("Stop")

        control_layout.addWidget(QtWidgets.QLabel("Current Level"), 0, 0)
        control_layout.addWidget(self.ctuning_ref1_spin, 0, 1)
        control_layout.addWidget(QtWidgets.QLabel("Square Frequency"), 0, 2)
        control_layout.addWidget(self.ctuning_ref2_spin, 0, 3)
        control_layout.addWidget(QtWidgets.QLabel("Capture Rate"), 0, 4)
        control_layout.addWidget(self.ctuning_rate_combo, 0, 5)
        control_layout.addWidget(QtWidgets.QLabel("Current Kp"), 1, 0)
        control_layout.addWidget(self.ctuning_kp_spin, 1, 1)
        control_layout.addWidget(QtWidgets.QLabel("Current Ki"), 1, 2)
        control_layout.addWidget(self.ctuning_ki_spin, 1, 3)
        control_layout.addWidget(QtWidgets.QLabel("Capture Window"), 1, 4)
        control_layout.addWidget(self.ctuning_window_label, 1, 5)
        control_layout.addWidget(QtWidgets.QLabel("Tuning Theta"), 2, 0)
        control_layout.addWidget(self.ctuning_theta_mode_value_label, 2, 1)
        control_layout.addWidget(QtWidgets.QLabel("Current Polarity"), 2, 2)
        control_layout.addWidget(self.ctuning_current_polarity_value_label, 2, 3)
        control_layout.addWidget(QtWidgets.QLabel("FOC Gain Scale"), 3, 0)
        control_layout.addWidget(self.ctuning_foc_gain_scale_spin, 3, 1)
        control_layout.addWidget(self.ctuning_hint_label, 4, 0, 1, 6)
        control_layout.addWidget(QtWidgets.QLabel("Status"), 5, 0)
        control_layout.addWidget(self.ctuning_status_label, 5, 1, 1, 5)
        control_layout.addWidget(self.apply_ctuning_gains_to_foc_button, 6, 0, 1, 3)
        control_layout.addWidget(self.start_ctuning_button, 6, 3, 1, 2)
        control_layout.addWidget(self.stop_ctuning_button, 6, 5, 1, 1)

        scope_toolbar = QtWidgets.QHBoxLayout()
        self.ctuning_auto_scale_checkbox = QtWidgets.QCheckBox("Auto-scale Y")
        self.ctuning_auto_scale_checkbox.setChecked(True)
        self.ctuning_clear_button = QtWidgets.QPushButton("Clear Capture")
        scope_hint = QtWidgets.QLabel(
            "Captured channels: Id Ref, Id, Vd, Vq. Goal: Id tracks the square wave cleanly while the motor stays locked and Iq stays small."
        )
        scope_hint.setWordWrap(True)
        scope_toolbar.addWidget(self.ctuning_auto_scale_checkbox)
        scope_toolbar.addWidget(self.ctuning_clear_button)
        scope_toolbar.addWidget(scope_hint, 1)

        self.ctuning_current_scope_view = ScopeCaptureView()
        self.ctuning_current_scope_view.setMinimumHeight(250)
        self.ctuning_voltage_scope_view = ScopeCaptureView()
        self.ctuning_voltage_scope_view.setMinimumHeight(250)
        self.ctuning_scope_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        self.ctuning_scope_splitter.addWidget(self.ctuning_current_scope_view)
        self.ctuning_scope_splitter.addWidget(self.ctuning_voltage_scope_view)
        self.ctuning_scope_splitter.setChildrenCollapsible(False)
        self.ctuning_scope_splitter.setStretchFactor(0, 1)
        self.ctuning_scope_splitter.setStretchFactor(1, 1)
        self._update_current_tuning_capture_window_label()

        layout.addWidget(control_group)
        layout.addLayout(scope_toolbar)
        layout.addWidget(self.ctuning_scope_splitter, 1)
        return widget

    def _build_autotune_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        config_group = QtWidgets.QGroupBox("PMSM Auto-Tuning")
        config_layout = QtWidgets.QGridLayout(config_group)

        self.autotune_rs_low_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_rs_low_spin.setRange(0.05, 20.0)
        self.autotune_rs_low_spin.setDecimals(3)
        self.autotune_rs_low_spin.setValue(0.20)
        self.autotune_rs_low_spin.setSuffix(" A")

        self.autotune_rs_high_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_rs_high_spin.setRange(0.10, 20.0)
        self.autotune_rs_high_spin.setDecimals(3)
        self.autotune_rs_high_spin.setValue(0.45)
        self.autotune_rs_high_spin.setSuffix(" A")

        self.autotune_ls_voltage_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_ls_voltage_spin.setRange(0.25, DEFAULT_MOTOR_MAXIMUM_VOLTAGE)
        self.autotune_ls_voltage_spin.setDecimals(3)
        self.autotune_ls_voltage_spin.setValue(2.0)
        self.autotune_ls_voltage_spin.setSuffix(" V")

        self.autotune_flux_frequency_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_flux_frequency_spin.setRange(1.0, DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ)
        self.autotune_flux_frequency_spin.setDecimals(3)
        self.autotune_flux_frequency_spin.setValue(15.0)
        self.autotune_flux_frequency_spin.setSuffix(" Hz")

        self.autotune_flux_voltage_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_flux_voltage_spin.setRange(1.0, DEFAULT_MOTOR_MAXIMUM_VOLTAGE)
        self.autotune_flux_voltage_spin.setDecimals(3)
        self.autotune_flux_voltage_spin.setValue(6.0)
        self.autotune_flux_voltage_spin.setSuffix(" V")

        self.autotune_current_bw_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_current_bw_spin.setRange(1.0, 2000.0)
        self.autotune_current_bw_spin.setDecimals(3)
        self.autotune_current_bw_spin.setValue(200.0)
        self.autotune_current_bw_spin.setSuffix(" Hz")

        self.autotune_speed_bw_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_speed_bw_spin.setRange(0.1, 500.0)
        self.autotune_speed_bw_spin.setDecimals(3)
        self.autotune_speed_bw_spin.setValue(20.0)
        self.autotune_speed_bw_spin.setSuffix(" Hz")

        self.autotune_position_bw_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_position_bw_spin.setRange(0.05, 100.0)
        self.autotune_position_bw_spin.setDecimals(3)
        self.autotune_position_bw_spin.setValue(5.0)
        self.autotune_position_bw_spin.setSuffix(" Hz")

        self.autotune_start_button = QtWidgets.QPushButton("Start Auto-Tune")
        self.autotune_stop_button = QtWidgets.QPushButton("Stop Auto-Tune")
        self.autotune_apply_button = QtWidgets.QPushButton("Apply Estimated Gains")
        self.autotune_progress_bar = QtWidgets.QProgressBar()
        self.autotune_progress_bar.setRange(0, 100)
        self.autotune_progress_bar.setValue(0)

        self.autotune_state_value_label = QtWidgets.QLabel("Idle")
        self.autotune_error_value_label = QtWidgets.QLabel("No error")
        self.autotune_chart_state_label = QtWidgets.QLabel("Waiting for capture")
        self.autotune_rs_value_label = QtWidgets.QLabel("-")
        self.autotune_ls_value_label = QtWidgets.QLabel("-")
        self.autotune_ke_value_label = QtWidgets.QLabel("-")
        self.autotune_flux_value_label = QtWidgets.QLabel("-")
        self.autotune_pp_value_label = QtWidgets.QLabel("-")
        self.autotune_current_gain_value_label = QtWidgets.QLabel("-")
        self.autotune_speed_gain_value_label = QtWidgets.QLabel("-")
        self.autotune_position_gain_value_label = QtWidgets.QLabel("-")

        autotune_hint = QtWidgets.QLabel(
            "Workflow: keep the rotor mechanically locked for Rs/Ls, then let the motor run unloaded during the open-loop flux stage. The module now uses the fixed 16 kHz runtime profile reported by firmware."
        )
        autotune_hint.setWordWrap(True)

        config_layout.addWidget(QtWidgets.QLabel("Rs Low Current"), 0, 0)
        config_layout.addWidget(self.autotune_rs_low_spin, 0, 1)
        config_layout.addWidget(QtWidgets.QLabel("Rs High Current"), 0, 2)
        config_layout.addWidget(self.autotune_rs_high_spin, 0, 3)
        config_layout.addWidget(QtWidgets.QLabel("Ls Step Voltage"), 1, 0)
        config_layout.addWidget(self.autotune_ls_voltage_spin, 1, 1)
        config_layout.addWidget(QtWidgets.QLabel("Flux Frequency"), 1, 2)
        config_layout.addWidget(self.autotune_flux_frequency_spin, 1, 3)
        config_layout.addWidget(QtWidgets.QLabel("Flux Voltage"), 2, 0)
        config_layout.addWidget(self.autotune_flux_voltage_spin, 2, 1)
        config_layout.addWidget(QtWidgets.QLabel("Current Bandwidth"), 2, 2)
        config_layout.addWidget(self.autotune_current_bw_spin, 2, 3)
        config_layout.addWidget(QtWidgets.QLabel("Speed Bandwidth"), 3, 0)
        config_layout.addWidget(self.autotune_speed_bw_spin, 3, 1)
        config_layout.addWidget(QtWidgets.QLabel("Position Bandwidth"), 3, 2)
        config_layout.addWidget(self.autotune_position_bw_spin, 3, 3)
        config_layout.addWidget(self.autotune_start_button, 4, 0)
        config_layout.addWidget(self.autotune_stop_button, 4, 1)
        config_layout.addWidget(self.autotune_apply_button, 4, 2, 1, 2)
        config_layout.addWidget(QtWidgets.QLabel("Progress"), 5, 0)
        config_layout.addWidget(self.autotune_progress_bar, 5, 1, 1, 3)
        config_layout.addWidget(autotune_hint, 6, 0, 1, 4)

        results_group = QtWidgets.QGroupBox("Measured Results")
        results_layout = QtWidgets.QGridLayout(results_group)
        results_layout.addWidget(QtWidgets.QLabel("State"), 0, 0)
        results_layout.addWidget(self.autotune_state_value_label, 0, 1)
        results_layout.addWidget(QtWidgets.QLabel("Error"), 0, 2)
        results_layout.addWidget(self.autotune_error_value_label, 0, 3)
        results_layout.addWidget(QtWidgets.QLabel("Chart"), 1, 0)
        results_layout.addWidget(self.autotune_chart_state_label, 1, 1, 1, 3)
        results_layout.addWidget(QtWidgets.QLabel("Rs"), 2, 0)
        results_layout.addWidget(self.autotune_rs_value_label, 2, 1)
        results_layout.addWidget(QtWidgets.QLabel("Ls"), 2, 2)
        results_layout.addWidget(self.autotune_ls_value_label, 2, 3)
        results_layout.addWidget(QtWidgets.QLabel("Ke"), 3, 0)
        results_layout.addWidget(self.autotune_ke_value_label, 3, 1)
        results_layout.addWidget(QtWidgets.QLabel("Flux"), 3, 2)
        results_layout.addWidget(self.autotune_flux_value_label, 3, 3)
        results_layout.addWidget(QtWidgets.QLabel("Pole Pairs"), 4, 0)
        results_layout.addWidget(self.autotune_pp_value_label, 4, 1)
        results_layout.addWidget(QtWidgets.QLabel("Current PI"), 4, 2)
        results_layout.addWidget(self.autotune_current_gain_value_label, 4, 3)
        results_layout.addWidget(QtWidgets.QLabel("Speed PI"), 5, 0)
        results_layout.addWidget(self.autotune_speed_gain_value_label, 5, 1)
        results_layout.addWidget(QtWidgets.QLabel("Position P"), 5, 2)
        results_layout.addWidget(self.autotune_position_gain_value_label, 5, 3)

        scope_toolbar = QtWidgets.QHBoxLayout()
        self.autotune_auto_scale_checkbox = QtWidgets.QCheckBox("Auto-scale Y")
        self.autotune_auto_scale_checkbox.setChecked(True)
        self.autotune_clear_button = QtWidgets.QPushButton("Clear Capture")
        scope_note = QtWidgets.QLabel(
            "Rs chart shows Id/Vd during the two-current steady-state test. Ls chart shows the direct d-axis voltage step response used to estimate di/dt with the active loop timing."
        )
        scope_note.setWordWrap(True)
        scope_toolbar.addWidget(self.autotune_auto_scale_checkbox)
        scope_toolbar.addWidget(self.autotune_clear_button)
        scope_toolbar.addWidget(scope_note, 1)

        self.autotune_scope_view = ScopeCaptureView()
        self._refresh_autotune_panel(None)

        layout.addWidget(config_group)
        layout.addWidget(results_group)
        layout.addLayout(scope_toolbar)
        layout.addWidget(self.autotune_scope_view, 1)
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
        self.ack_fault_button = QtWidgets.QPushButton("Reset Alarm")
        self.ack_fault_button.setEnabled(False)
        self.save_flash_button = QtWidgets.QPushButton("Save Params to Flash")
        self.refresh_monitor_button = QtWidgets.QPushButton("Refresh Monitor")

        self.quick_command_tabs = QtWidgets.QTabWidget()

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
            "Servo ON only performs the safe arm sequence with no motion command. Watch Driver Monitor for current calibration and encoder alignment status, then use the FOC Control tab to start closed-loop motion when the drive is ready."
        )
        drive_hint.setWordWrap(True)
        drive_layout.addWidget(drive_hint)
        drive_layout.addStretch(1)

        foc_tab = self._build_foc_control_tab()

        vf_tab = QtWidgets.QWidget()
        vf_layout = QtWidgets.QGridLayout(vf_tab)
        self.vf_frequency_spin = QtWidgets.QDoubleSpinBox()
        self.vf_frequency_spin.setRange(
            -DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ,
            DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ,
        )
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
            "Default motor profile: 4 pole pairs, 3000 rpm, 200 Hz electrical, 91 V, 1.6 A, 200 W. Open-loop electrical frequency now accepts both positive and negative commands for direction/sign checks, while voltage remains firmware-clamped for safe testing. Trend Charts are low-rate monitor views; use Trace Scope with the 'Phase Currents' preset when you need waveform-level Iu/Iv/Iw detail."
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

        self.alarm_tab = self._build_alarm_tab()

        self.quick_command_tabs.addTab(drive_tab, "Drive")
        self.quick_command_tabs.addTab(foc_tab, "FOC Control")
        self.quick_command_tabs.addTab(vf_tab, "Open Loop V/F")
        self.quick_command_tabs.addTab(self.alarm_tab, "Alarms")
        layout.addWidget(self.quick_command_tabs)
        return box

    def _build_foc_control_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        summary_group = QtWidgets.QGroupBox("FOC Cascaded Control")
        summary_layout = QtWidgets.QGridLayout(summary_group)

        self.foc_mode_combo = QtWidgets.QComboBox()
        self.foc_mode_combo.addItem("Speed Mode", SPEED_CONTROL_MODE)
        self.foc_mode_combo.addItem("Position Mode", POSITION_CONTROL_MODE)

        self.foc_target_speed_label = QtWidgets.QLabel("Target Speed")
        self.foc_target_speed_spin = QtWidgets.QDoubleSpinBox()
        self.foc_target_speed_spin.setRange(-2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM, 2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM)
        self.foc_target_speed_spin.setDecimals(2)
        self.foc_target_speed_spin.setSingleStep(50.0)
        self.foc_target_speed_spin.setSuffix(" rpm")

        self.foc_target_position_label = QtWidgets.QLabel("Target Position")
        self.foc_target_position_spin = QtWidgets.QDoubleSpinBox()
        self.foc_target_position_spin.setRange(-100000000.0, 100000000.0)
        self.foc_target_position_spin.setDecimals(1)
        self.foc_target_position_spin.setSingleStep(1000.0)
        self.foc_target_position_spin.setSuffix(" cnt")

        self.foc_position_kp_label = QtWidgets.QLabel("Position Kp")
        self.foc_position_kp_spin = QtWidgets.QDoubleSpinBox()
        self.foc_position_kp_spin.setRange(0.0, 1000.0)
        self.foc_position_kp_spin.setDecimals(5)
        self.foc_position_kp_spin.setSingleStep(0.01)
        self.foc_position_kp_spin.setValue(0.05)

        self.foc_speed_kp_spin = QtWidgets.QDoubleSpinBox()
        self.foc_speed_kp_spin.setRange(0.0, 1000.0)
        self.foc_speed_kp_spin.setDecimals(5)
        self.foc_speed_kp_spin.setSingleStep(0.01)
        self.foc_speed_kp_spin.setValue(0.02)

        self.foc_speed_ki_spin = QtWidgets.QDoubleSpinBox()
        self.foc_speed_ki_spin.setRange(0.0, 10000.0)
        self.foc_speed_ki_spin.setDecimals(5)
        self.foc_speed_ki_spin.setSingleStep(0.10)
        self.foc_speed_ki_spin.setValue(5.0)

        self.foc_debug_angle_label = QtWidgets.QLabel("FOC Frame")
        self.foc_debug_angle_value_label = QtWidgets.QLabel("None (raw theta)")
        self.foc_debug_angle_value_label.setStyleSheet("font-weight: 600;")
        self.foc_debug_angle_value_label.setToolTip(
            "Runtime FOC applies no extra electrical frame offset. The controller uses raw theta from the encoder offset path as-is."
        )

        self.foc_accel_spin = QtWidgets.QDoubleSpinBox()
        self.foc_accel_spin.setRange(1.0, 100000.0)
        self.foc_accel_spin.setDecimals(1)
        self.foc_accel_spin.setSingleStep(25.0)
        self.foc_accel_spin.setSuffix(" ms")
        self.foc_accel_spin.setValue(250.0)

        self.foc_decel_spin = QtWidgets.QDoubleSpinBox()
        self.foc_decel_spin.setRange(1.0, 100000.0)
        self.foc_decel_spin.setDecimals(1)
        self.foc_decel_spin.setSingleStep(25.0)
        self.foc_decel_spin.setSuffix(" ms")
        self.foc_decel_spin.setValue(250.0)

        self.foc_status_value_label = QtWidgets.QLabel("Idle")
        self.foc_status_value_label.setStyleSheet("font-weight: 600;")
        self.foc_direction_test_status_label = QtWidgets.QLabel("Not run")
        self.foc_live_summary_label = QtWidgets.QLabel(
            "Speed Mode is ready. Enter target speed and gains, then press Start FOC."
        )
        self.foc_live_summary_label.setWordWrap(True)

        self.start_foc_direction_test_button = QtWidgets.QPushButton("Run Direction Test")
        self.start_foc_angle_fit_button = QtWidgets.QPushButton("Run Angle Fit")
        self.start_foc_button = QtWidgets.QPushButton("Start FOC")
        self.stop_foc_button = QtWidgets.QPushButton("Stop FOC")

        summary_layout.addWidget(QtWidgets.QLabel("Mode"), 0, 0)
        summary_layout.addWidget(self.foc_mode_combo, 0, 1)
        summary_layout.addWidget(self.foc_target_speed_label, 0, 2)
        summary_layout.addWidget(self.foc_target_speed_spin, 0, 3)
        summary_layout.addWidget(self.foc_target_position_label, 1, 0)
        summary_layout.addWidget(self.foc_target_position_spin, 1, 1)
        summary_layout.addWidget(self.foc_position_kp_label, 1, 2)
        summary_layout.addWidget(self.foc_position_kp_spin, 1, 3)
        summary_layout.addWidget(QtWidgets.QLabel("Speed Kp"), 2, 0)
        summary_layout.addWidget(self.foc_speed_kp_spin, 2, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Speed Ki"), 2, 2)
        summary_layout.addWidget(self.foc_speed_ki_spin, 2, 3)
        summary_layout.addWidget(self.foc_debug_angle_label, 3, 0)
        summary_layout.addWidget(self.foc_debug_angle_value_label, 3, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Acceleration"), 4, 0)
        summary_layout.addWidget(self.foc_accel_spin, 4, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Deceleration"), 4, 2)
        summary_layout.addWidget(self.foc_decel_spin, 4, 3)
        summary_layout.addWidget(QtWidgets.QLabel("Status"), 5, 0)
        summary_layout.addWidget(self.foc_status_value_label, 5, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Dir Test"), 5, 2)
        summary_layout.addWidget(self.foc_direction_test_status_label, 5, 3)
        summary_layout.addWidget(self.foc_live_summary_label, 6, 0, 1, 4)
        summary_layout.addWidget(self.start_foc_direction_test_button, 7, 0, 1, 2)
        summary_layout.addWidget(self.start_foc_angle_fit_button, 7, 2, 1, 2)
        summary_layout.addWidget(self.start_foc_button, 8, 0, 1, 2)
        summary_layout.addWidget(self.stop_foc_button, 8, 2, 1, 2)

        note_group = QtWidgets.QGroupBox("How It Works")
        note_layout = QtWidgets.QVBoxLayout(note_group)
        self.foc_mode_description_label = QtWidgets.QLabel()
        self.foc_mode_description_label.setWordWrap(True)
        self.foc_servo_note_label = QtWidgets.QLabel(
            "Servo ON only runs the safe arm sequence: current-sensor offset calibration, zero-current references, and PWM enable without any motion target. Watch Driver Monitor for encoder alignment status, then Start FOC sends the selected mode, target, gains, and ramp limits to the firmware; runtime FOC now uses raw theta with no extra electrical frame offset."
        )
        self.foc_servo_note_label.setWordWrap(True)
        note_layout.addWidget(self.foc_mode_description_label)
        note_layout.addWidget(self.foc_servo_note_label)

        layout.addWidget(summary_group)
        layout.addWidget(note_group)
        layout.addStretch(1)
        self._update_foc_mode_ui()
        return widget

    def _build_alarm_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        summary_group = QtWidgets.QGroupBox("Active Alarm")
        summary_layout = QtWidgets.QGridLayout(summary_group)
        self.alarm_active_status_value_label = QtWidgets.QLabel("No active alarms")
        self.alarm_active_status_value_label.setWordWrap(True)
        self.alarm_active_severity_value_label = QtWidgets.QLabel("OK")
        self.alarm_active_source_value_label = QtWidgets.QLabel("-")
        self.alarm_active_code_value_label = QtWidgets.QLabel("0x0000")
        self.alarm_reset_button = QtWidgets.QPushButton("Reset Alarm")
        self.alarm_clear_history_button = QtWidgets.QPushButton("Clear History")

        summary_layout.addWidget(QtWidgets.QLabel("Summary"), 0, 0)
        summary_layout.addWidget(self.alarm_active_status_value_label, 0, 1, 1, 3)
        summary_layout.addWidget(QtWidgets.QLabel("Severity"), 1, 0)
        summary_layout.addWidget(self.alarm_active_severity_value_label, 1, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Source"), 1, 2)
        summary_layout.addWidget(self.alarm_active_source_value_label, 1, 3)
        summary_layout.addWidget(QtWidgets.QLabel("Fault Code"), 2, 0)
        summary_layout.addWidget(self.alarm_active_code_value_label, 2, 1)
        summary_layout.addWidget(self.alarm_reset_button, 2, 2)
        summary_layout.addWidget(self.alarm_clear_history_button, 2, 3)

        history_group = QtWidgets.QGroupBox("Alarm History")
        history_layout = QtWidgets.QVBoxLayout(history_group)
        history_hint = QtWidgets.QLabel(
            "History stores only fault transitions, so repeated monitor polls do not spam duplicate alarms. Critical alarms are shown in red, warnings in amber, and clear events in green."
        )
        history_hint.setWordWrap(True)
        self.alarm_history_table = QtWidgets.QTableWidget(0, 5)
        self.alarm_history_table.setHorizontalHeaderLabels(
            ["Time", "Severity", "Source", "Code", "Summary"]
        )
        self.alarm_history_table.verticalHeader().setVisible(False)
        self.alarm_history_table.setAlternatingRowColors(True)
        self.alarm_history_table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.alarm_history_table.setEditTriggers(
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self.alarm_history_table.horizontalHeader().setSectionResizeMode(
            0,
            QtWidgets.QHeaderView.ResizeMode.ResizeToContents,
        )
        self.alarm_history_table.horizontalHeader().setSectionResizeMode(
            1,
            QtWidgets.QHeaderView.ResizeMode.ResizeToContents,
        )
        self.alarm_history_table.horizontalHeader().setSectionResizeMode(
            2,
            QtWidgets.QHeaderView.ResizeMode.ResizeToContents,
        )
        self.alarm_history_table.horizontalHeader().setSectionResizeMode(
            3,
            QtWidgets.QHeaderView.ResizeMode.ResizeToContents,
        )
        self.alarm_history_table.horizontalHeader().setSectionResizeMode(
            4,
            QtWidgets.QHeaderView.ResizeMode.Stretch,
        )
        history_layout.addWidget(history_hint)
        history_layout.addWidget(self.alarm_history_table, 1)

        layout.addWidget(summary_group)
        layout.addWidget(history_group, 1)
        self._set_alarm_panel_state(
            0,
            "No active alarms",
            "No active alarms",
            "OK",
            "-",
        )
        return widget

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

    def _build_parameter_panel(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        toolbar = QtWidgets.QHBoxLayout()
        self.read_driver_button = QtWidgets.QPushButton("Read Driver Params")
        self.read_motor_button = QtWidgets.QPushButton("Read Motor Params")
        self.write_driver_button = QtWidgets.QPushButton("Write Driver Params")
        self.write_motor_button = QtWidgets.QPushButton("Write Motor Params")
        self.parameter_status_label = QtWidgets.QLabel("Parameter tools are ready.")
        toolbar.addWidget(self.read_driver_button)
        toolbar.addWidget(self.read_motor_button)
        toolbar.addWidget(self.write_driver_button)
        toolbar.addWidget(self.write_motor_button)
        toolbar.addStretch(1)
        toolbar.addWidget(self.parameter_status_label)
        self.parameter_status_label.setStyleSheet("color: #bbbbbb; font-weight: 600;")
        layout.addLayout(toolbar)
        layout.addWidget(self._build_parameter_tabs())
        return widget

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
        self._configure_auxiliary_window(dialog)
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
        dialog.setWindowTitle("Drive Snapshot")
        dialog.resize(760, 560)
        self._configure_auxiliary_window(dialog)
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
        self.connect_button.clicked.connect(self._toggle_connection)
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
        self.refresh_monitor_button.clicked.connect(self._request_monitor_once)
        self.foc_mode_combo.currentIndexChanged.connect(self._update_foc_mode_ui)
        self.foc_target_speed_spin.valueChanged.connect(self._on_foc_target_speed_value_changed)
        self.start_foc_direction_test_button.clicked.connect(self._start_foc_direction_test)
        self.start_foc_angle_fit_button.clicked.connect(self._start_foc_angle_fit)
        self.start_foc_button.clicked.connect(self._start_foc_control)
        self.stop_foc_button.clicked.connect(self._stop_foc_control)
        self.start_vf_button.clicked.connect(self._start_open_loop_vf)
        self.stop_vf_button.clicked.connect(
            lambda: self._enqueue_command(
                Command.CMD_STOP_OPEN_LOOP_VF, b"", "Stop Open Loop V/F"
            )
        )
        self.servo_on_button.clicked.connect(self._handle_servo_on)
        self.servo_off_button.clicked.connect(self._handle_servo_off)
        self.ack_fault_button.clicked.connect(self._reset_alarm)
        self.alarm_banner_reset_button.clicked.connect(self._reset_alarm)
        self.alarm_reset_button.clicked.connect(self._reset_alarm)
        self.alarm_clear_history_button.clicked.connect(self._clear_alarm_history)
        self.save_flash_button.clicked.connect(
            lambda: self._enqueue_command(Command.CMD_WRITE_TO_FLASH, b"", "Save Parameters to Flash")
        )
        self.autotune_start_button.clicked.connect(self._start_motor_autotune)
        self.autotune_stop_button.clicked.connect(self._stop_motor_autotune)
        self.autotune_apply_button.clicked.connect(self._apply_motor_autotune_estimates)
        self.autotune_clear_button.clicked.connect(self._clear_autotune_capture)
        self.autotune_auto_scale_checkbox.toggled.connect(self.autotune_scope_view.set_auto_scale)
        self.read_driver_button.clicked.connect(self._read_driver_parameters)
        self.read_motor_button.clicked.connect(self._read_motor_parameters)
        self.write_driver_button.clicked.connect(self._write_driver_parameters)
        self.write_motor_button.clicked.connect(self._write_motor_parameters)
        self.clear_log_button.clicked.connect(self.log_edit.clear)
        self.apply_ctuning_gains_to_foc_button.clicked.connect(self._apply_current_tuning_gains_to_foc)
        self.start_ctuning_button.clicked.connect(self._start_current_tuning)
        self.stop_ctuning_button.clicked.connect(self._stop_current_tuning)
        self.ctuning_clear_button.clicked.connect(self._clear_current_tuning_capture)
        self.ctuning_auto_scale_checkbox.toggled.connect(self.ctuning_current_scope_view.set_auto_scale)
        self.ctuning_auto_scale_checkbox.toggled.connect(self.ctuning_voltage_scope_view.set_auto_scale)
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

    def _configure_auxiliary_window(self, dialog: QtWidgets.QDialog) -> None:
        dialog.setModal(False)
        dialog.setWindowModality(QtCore.Qt.WindowModality.NonModal)
        dialog.setWindowFlag(QtCore.Qt.WindowType.Window, True)

    def _show_auxiliary_window(self, window: QtWidgets.QDialog) -> None:
        if window.isMinimized():
            window.showNormal()
        window.show()
        window.raise_()
        window.activateWindow()

    def _timing_mode_text(self, mode: int) -> str:
        _ = mode
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

    def _refresh_capture_rate_combos(self, *, force: bool = False) -> None:
        combos = [self.ctuning_rate_combo, self.trace_rate_combo]
        decimations = [0, 1, 3, 7, 15]
        target_loop_hz = float(self._active_control_loop_hz)
        if (
            not force
            and self._capture_rate_combo_loop_hz is not None
            and abs(self._capture_rate_combo_loop_hz - target_loop_hz) < 0.5
        ):
            self._update_current_tuning_capture_window_label()
            return

        for combo in combos:
            current_data = int(combo.currentData() or 0)
            combo.blockSignals(True)
            combo.clear()
            for decimation in decimations:
                combo.addItem(
                    self._format_capture_rate_label(decimation, target_loop_hz),
                    decimation,
                )
            index = combo.findData(current_data)
            combo.setCurrentIndex(index if index >= 0 else 0)
            combo.blockSignals(False)

        self._capture_rate_combo_loop_hz = target_loop_hz
        self._update_current_tuning_capture_window_label()

    def _set_active_timing_profile(
        self,
        mode: int,
        current_loop_hz: float | None = None,
        speed_loop_hz: float | None = None,
        *,
        update_combo: bool = True,
    ) -> None:
        _ = update_combo
        mode = CONTROL_TIMING_MODE_16KHZ
        if current_loop_hz is None or current_loop_hz <= 1.0:
            current_loop_hz = 16000.0
        if speed_loop_hz is None or speed_loop_hz <= 1.0:
            speed_loop_hz = current_loop_hz * 0.5

        self._active_timing_mode = mode
        self._active_control_loop_hz = float(current_loop_hz)
        self._active_speed_loop_hz = float(speed_loop_hz)

        if hasattr(self, "timing_mode_state_label"):
            self.timing_mode_state_label.setText(
                f"Fixed: {self._timing_mode_text(mode)} ({self._active_control_loop_hz:.0f} Hz)"
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

    def _table_float_value(
        self, table: QtWidgets.QTableWidget, row: int, fallback: float
    ) -> float:
        item = table.item(row, 2)
        if item is None:
            return fallback
        try:
            return float(item.text().strip())
        except ValueError:
            return fallback

    def _set_table_float_value(
        self, table: QtWidgets.QTableWidget, row: int, value: float
    ) -> None:
        if not (0 <= row < table.rowCount()):
            return
        item = table.item(row, 2)
        if item is None:
            item = QtWidgets.QTableWidgetItem()
            table.setItem(row, 2, item)
        item.setText(f"{float(value):.6f}")

    def _current_foc_mode(self) -> int:
        return int(self.foc_mode_combo.currentData() or SPEED_CONTROL_MODE)

    def _load_foc_controls_from_driver_table(self) -> None:
        if not hasattr(self, "driver_table"):
            return
        mode = int(
            round(
                self._table_float_value(
                    self.driver_table,
                    DRIVER_PARAM_CONTROL_MODE,
                    float(SPEED_CONTROL_MODE),
                )
            )
        )
        if mode not in (SPEED_CONTROL_MODE, POSITION_CONTROL_MODE):
            mode = SPEED_CONTROL_MODE
        combo_index = self.foc_mode_combo.findData(mode)
        if combo_index >= 0:
            self.foc_mode_combo.blockSignals(True)
            self.foc_mode_combo.setCurrentIndex(combo_index)
            self.foc_mode_combo.blockSignals(False)

        self.foc_position_kp_spin.setValue(
            self._table_float_value(self.driver_table, DRIVER_PARAM_POSITION_P_GAIN, 0.05)
        )
        self.foc_speed_kp_spin.setValue(
            self._table_float_value(self.driver_table, DRIVER_PARAM_SPEED_P_GAIN, 0.02)
        )
        self.foc_speed_ki_spin.setValue(
            self._table_float_value(self.driver_table, DRIVER_PARAM_SPEED_I_GAIN, 5.0)
        )
        self.foc_accel_spin.setValue(
            self._table_float_value(self.driver_table, DRIVER_PARAM_ACCEL_TIME_MS, 250.0)
        )
        self.foc_decel_spin.setValue(
            self._table_float_value(self.driver_table, DRIVER_PARAM_DECEL_TIME_MS, 250.0)
        )
        if self._foc_speed_target_rpm <= 1.0:
            self._foc_speed_target_rpm = 300.0
        self._update_foc_mode_ui()

    def _sync_foc_controls_to_driver_table(self) -> None:
        if not hasattr(self, "driver_table"):
            return
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_CONTROL_MODE,
            float(self._current_foc_mode()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_POSITION_P_GAIN,
            float(self.foc_position_kp_spin.value()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_SPEED_P_GAIN,
            float(self.foc_speed_kp_spin.value()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_SPEED_I_GAIN,
            float(self.foc_speed_ki_spin.value()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_ACCEL_TIME_MS,
            float(self.foc_accel_spin.value()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_DECEL_TIME_MS,
            float(self.foc_decel_spin.value()),
        )

    def _update_foc_mode_ui(self) -> None:
        position_mode = self._current_foc_mode() == POSITION_CONTROL_MODE
        target_value = (
            abs(self._foc_position_speed_limit_rpm) if position_mode else self._foc_speed_target_rpm
        )
        self.foc_target_speed_spin.blockSignals(True)
        self.foc_target_speed_spin.setValue(target_value)
        self.foc_target_speed_spin.blockSignals(False)
        self._last_foc_mode_ui = POSITION_CONTROL_MODE if position_mode else SPEED_CONTROL_MODE
        self.foc_target_speed_label.setText("Speed Limit" if position_mode else "Target Speed")
        self.foc_target_position_label.setEnabled(position_mode)
        self.foc_target_position_spin.setEnabled(position_mode)
        self.foc_position_kp_label.setEnabled(position_mode)
        self.foc_position_kp_spin.setEnabled(position_mode)
        if position_mode:
            self.foc_mode_description_label.setText(
                "Position Mode: the outer position loop compares Target Position with actual position, converts the error into a commanded speed, then the speed loop generates Iq. The speed field above is used as the motion speed limit, not as a direct speed command."
            )
            if not self._connected:
                self.foc_live_summary_label.setText(
                    "Position Mode is ready. Enter target position, speed limit, and gains, then press Start FOC."
                )
        else:
            self.foc_mode_description_label.setText(
                "Speed Mode: the position loop is bypassed. Target Speed is sent directly into the speed loop, which generates Iq for the inner current loop."
            )
            if not self._connected:
                self.foc_live_summary_label.setText(
                    "Speed Mode is ready. Enter target speed and gains, then press Start FOC."
                )
        self._refresh_foc_control_panel(self._latest_monitor_snapshot)

    def _on_foc_target_speed_value_changed(self, value: float) -> None:
        if self._current_foc_mode() == POSITION_CONTROL_MODE:
            self._foc_position_speed_limit_rpm = abs(float(value))
        else:
            self._foc_speed_target_rpm = float(value)

    def _refresh_foc_control_panel(self, snapshot) -> None:
        if not hasattr(self, "foc_status_value_label"):
            return
        preview_mode = self._current_foc_mode()
        mode_text = "Position Mode" if preview_mode == POSITION_CONTROL_MODE else "Speed Mode"
        debug_suffix = " Electrical frame: none (raw theta)."
        if snapshot is None:
            self.foc_status_value_label.setText("Idle")
            self.foc_direction_test_status_label.setText("Not run")
            self.foc_live_summary_label.setText(
                f"{mode_text} is ready. Servo ON runs the arm sequence. Watch Driver Monitor for Cal Status and Align Status, then press Start FOC when the drive is ready.{debug_suffix}"
            )
            return

        direction_status = int(
            getattr(snapshot, "foc_direction_test_status", FOC_DIRECTION_TEST_IDLE)
        )
        direction_open_loop_delta = int(
            getattr(snapshot, "foc_direction_test_open_loop_delta_pos", 0)
        )
        direction_foc_delta = int(
            getattr(snapshot, "foc_direction_test_foc_delta_pos", 0)
        )
        if direction_status == FOC_DIRECTION_TEST_RUNNING:
            self.foc_direction_test_status_label.setText("Running...")
            self.foc_status_value_label.setText("Direction test running")
            self.foc_live_summary_label.setText(
                f"Open-loop + delta {direction_open_loop_delta:+d} cnt | "
                f"FOC +Iq delta {direction_foc_delta:+d} cnt. "
                "Firmware is comparing positive open-loop rotation against a very small +Iq on the raw runtime theta path with no extra frame offset. If the signs disagree, encoder direction will be flipped and alignment must be rerun."
            )
            return
        if direction_status == FOC_DIRECTION_TEST_DONE_OK:
            self.foc_direction_test_status_label.setText("Matched open-loop +")
        elif direction_status == FOC_DIRECTION_TEST_DONE_FLIPPED:
            self.foc_direction_test_status_label.setText("Flipped / rerun align")
        elif direction_status == FOC_DIRECTION_TEST_INCONCLUSIVE:
            self.foc_direction_test_status_label.setText("Inconclusive")
        elif direction_status == FOC_DIRECTION_TEST_FAULT:
            self.foc_direction_test_status_label.setText("Fault")
        else:
            self.foc_direction_test_status_label.setText("Idle")

        run_mode = getattr(snapshot, "run_mode", None)
        enable_run = bool(getattr(snapshot, "enable_run", False))
        if run_mode is None:
            runtime_mode = int(
                round(
                    self._table_float_value(
                        self.driver_table,
                        DRIVER_PARAM_CONTROL_MODE,
                        float(preview_mode),
                    )
                )
            )
            mode = (
                POSITION_CONTROL_MODE
                if runtime_mode == POSITION_CONTROL_MODE
                else SPEED_CONTROL_MODE
            )
            if mode == POSITION_CONTROL_MODE:
                self.foc_status_value_label.setText("Fault / stopped")
                self.foc_live_summary_label.setText(
                    f"Last position target {snapshot.cmd_position:.1f} cnt | "
                    f"Act {snapshot.act_position:.1f} cnt | "
                    f"Error {snapshot.position_error:.1f} cnt"
                )
            else:
                self.foc_status_value_label.setText("Fault / stopped")
                self.foc_live_summary_label.setText(
                    f"Last speed target {snapshot.cmd_speed:.1f} rpm | "
                    f"Act {snapshot.act_speed:.1f} rpm | "
                    f"Error {snapshot.speed_error:.1f} rpm"
                )
            return

        if int(run_mode) != RUN_MODE_FOC:
            servo_text = "Servo ON" if enable_run else "Servo OFF"
            self.foc_status_value_label.setText(f"{servo_text} | Idle")
            if direction_status == FOC_DIRECTION_TEST_DONE_OK:
                self.foc_live_summary_label.setText(
                    "Direction test passed: positive FOC Iq now matches positive open-loop rotation. Keep an eye on Driver Monitor and start FOC once Align Status stays Done."
                )
            elif direction_status == FOC_DIRECTION_TEST_DONE_FLIPPED:
                self.foc_live_summary_label.setText(
                    "Direction test found a sign mismatch and flipped the encoder direction chain to match open-loop positive rotation. Toggle Servo ON again and wait for Driver Monitor Align Status = Done before Start FOC."
                )
            elif direction_status == FOC_DIRECTION_TEST_INCONCLUSIVE:
                self.foc_live_summary_label.setText(
                    "Direction test could not measure a reliable FOC direction response. Check that the rotor can move freely; if it can, this usually points to weak torque production or runtime theta still being off enough that FOC does not move decisively."
                )
            elif direction_status == FOC_DIRECTION_TEST_FAULT:
                self.foc_live_summary_label.setText(
                    "Direction test faulted before it could finish. Clear alarms, reduce commissioning aggression, and rerun the test."
                )
            else:
                self.foc_live_summary_label.setText(
                    f"{mode_text} is configured. Servo ON keeps the drive armed at zero references only. Watch Driver Monitor for Cal Status and Align Status, then press Start FOC to run the selected setpoint.{debug_suffix}"
                )
            return

        runtime_mode = int(
            round(
                self._table_float_value(
                    self.driver_table,
                    DRIVER_PARAM_CONTROL_MODE,
                    float(preview_mode),
                )
            )
        )
        mode = POSITION_CONTROL_MODE if runtime_mode == POSITION_CONTROL_MODE else SPEED_CONTROL_MODE
        if mode == POSITION_CONTROL_MODE:
            self.foc_status_value_label.setText("FOC Position Running")
            self.foc_live_summary_label.setText(
                f"Target {snapshot.cmd_position:.1f} cnt | "
                f"Act {snapshot.act_position:.1f} cnt | "
                f"Error {snapshot.position_error:.1f} cnt | "
                f"Cmd Speed {snapshot.cmd_speed:.1f} rpm"
            )
        else:
            self.foc_status_value_label.setText("FOC Speed Running")
            self.foc_live_summary_label.setText(
                f"Target {snapshot.cmd_speed:.1f} rpm | "
                f"Actual {snapshot.act_speed:.1f} rpm | "
                f"Error {snapshot.speed_error:.1f} rpm"
            )

    def _handle_servo_on(self) -> None:
        self.auto_poll_checkbox.setChecked(True)
        self.foc_status_value_label.setText("Servo ON starting...")
        self.foc_live_summary_label.setText(
            "Running Servo ON sequence: ADC offset calibration first, then encoder alignment. Watch Driver Monitor for Cal Status and Align Status to reach Done."
        )
        self._enqueue_command(Command.CMD_SERVO_ON, b"", "Servo ON")
        self._request_monitor_once()

    def _handle_servo_off(self) -> None:
        self.auto_poll_checkbox.setChecked(True)
        self._enqueue_command(Command.CMD_SERVO_OFF, b"", "Servo OFF")
        self._request_monitor_once()

    def _start_foc_direction_test(self) -> None:
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None:
            alignment_policy = int(last_monitor.debug_alignment_policy)
            alignment_status = int(last_monitor.debug_alignment_status)
            if (
                alignment_policy == ENCODER_ALIGNMENT_POLICY_POWER_ON
                and alignment_status != ENCODER_ALIGNMENT_STATUS_DONE
            ):
                QtWidgets.QMessageBox.warning(
                    self,
                    "Direction Test Blocked",
                    "This incremental encoder profile aligns during Servo ON after every power-up. Toggle Servo ON and wait until Driver Monitor shows Align Status = Done, then run the FOC direction test.",
                )
                return

        self.auto_poll_checkbox.setChecked(True)
        self.foc_status_value_label.setText("Direction test starting...")
        self.foc_direction_test_status_label.setText("Starting...")
        self.foc_live_summary_label.setText(
            "Running a short open-loop vs FOC sign comparison. The firmware compares positive open-loop rotation against positive FOC Iq and flips encoder direction automatically if they disagree."
        )
        self._enqueue_command(
            Command.CMD_START_FOC_DIRECTION_TEST,
            b"",
            "Run FOC Direction Test",
        )
        self._request_monitor_once()

    def _start_foc_angle_fit(self) -> None:
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None:
            alignment_policy = int(last_monitor.debug_alignment_policy)
            alignment_status = int(last_monitor.debug_alignment_status)
            if (
                alignment_policy == ENCODER_ALIGNMENT_POLICY_POWER_ON
                and alignment_status != ENCODER_ALIGNMENT_STATUS_DONE
            ):
                QtWidgets.QMessageBox.warning(
                    self,
                    "Angle Fit Blocked",
                    "Toggle Servo ON and wait until Driver Monitor shows Align Status = Done, then run Angle Fit.",
                )
                return

        self.auto_poll_checkbox.setChecked(True)
        self.foc_status_value_label.setText("Angle fit starting...")
        self.foc_direction_test_status_label.setText("Angle fit")
        self.foc_live_summary_label.setText(
            "Running a short angle fit sweep around the current raw theta / encoder offset path to maximize q-axis torque. Watch Enc Offset in Driver Monitor; it should update when the fit completes."
        )
        self._enqueue_command(
            Command.CMD_START_FOC_ANGLE_FIT,
            b"",
            "Run FOC Angle Fit",
        )
        self._request_monitor_once()


    def _start_foc_control(self) -> None:
        mode = self._current_foc_mode()
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None:
            alignment_policy = int(last_monitor.debug_alignment_policy)
            alignment_status = int(last_monitor.debug_alignment_status)
            if (
                alignment_policy == ENCODER_ALIGNMENT_POLICY_POWER_ON
                and alignment_status != ENCODER_ALIGNMENT_STATUS_DONE
            ):
                QtWidgets.QMessageBox.warning(
                    self,
                    "FOC Start Blocked",
                    "This incremental encoder profile aligns during Servo ON after every power-up. Toggle Servo ON and wait until Driver Monitor shows Align Status = Done, then start FOC.",
                )
                return

        self.auto_poll_checkbox.setChecked(True)
        if mode == POSITION_CONTROL_MODE:
            self._foc_position_speed_limit_rpm = abs(float(self.foc_target_speed_spin.value()))
        else:
            self._foc_speed_target_rpm = float(self.foc_target_speed_spin.value())
        self._sync_foc_controls_to_driver_table()

        if mode == POSITION_CONTROL_MODE:
            payload = struct.pack(
                "<7fBB",
                float(self.foc_target_position_spin.value()),
                float(self._foc_position_speed_limit_rpm),
                float(self.foc_position_kp_spin.value()),
                float(self.foc_speed_kp_spin.value()),
                float(self.foc_speed_ki_spin.value()),
                float(self.foc_accel_spin.value()),
                float(self.foc_decel_spin.value()),
                ID_SQUARE_ANGLE_TEST_NONE,
                0,
            )
            description = (
                f"Start FOC Position Mode "
                f"(target={self.foc_target_position_spin.value():.1f} cnt, "
                f"limit={self._foc_position_speed_limit_rpm:.1f} rpm, "
                "frame=none)"
            )
            command = Command.CMD_START_POSITIONCONTROL
        else:
            payload = struct.pack(
                "<5fBB",
                float(self._foc_speed_target_rpm),
                float(self.foc_speed_kp_spin.value()),
                float(self.foc_speed_ki_spin.value()),
                float(self.foc_accel_spin.value()),
                float(self.foc_decel_spin.value()),
                ID_SQUARE_ANGLE_TEST_NONE,
                0,
            )
            description = (
                f"Start FOC Speed Mode "
                f"(target={self._foc_speed_target_rpm:.1f} rpm, "
                "frame=none)"
            )
            command = Command.CMD_START_SPEEDCONTROL

        self.foc_status_value_label.setText("Starting...")
        self._enqueue_command(command, payload, description)
        self._request_monitor_once()

    def _stop_foc_control(self) -> None:
        mode = self._current_foc_mode()
        command = (
            Command.CMD_STOP_POSITIONCONTROL
            if mode == POSITION_CONTROL_MODE
            else Command.CMD_STOP_SPEEDCONTROL
        )
        self._enqueue_command(command, b"", "Stop FOC")
        self.foc_status_value_label.setText("Stopping...")
        self._request_monitor_once()

    def _update_current_tuning_capture_window_label(self) -> None:
        window_ms = TRACE_TOTAL_SAMPLES * self._current_tuning_sample_period_s() * 1000.0
        self.ctuning_window_label.setText(f"{window_ms:.1f} ms")

    def _build_current_tuning_payload(self, tuning_mode: int | None = None) -> bytes:
        selected_mode = ID_SQUARE_TUNING_MODE_SQUARE_WAVE if tuning_mode is None else int(tuning_mode)
        electrical_angle_test = ID_SQUARE_ANGLE_TEST_NONE
        invert_current = 0
        return struct.pack(
            "<4fBBBB",
            float(self.ctuning_ref1_spin.value()),
            float(self.ctuning_ref2_spin.value()),
            float(self.ctuning_kp_spin.value()),
            float(self.ctuning_ki_spin.value()),
            electrical_angle_test,
            invert_current,
            0,
            selected_mode,
        )

    def _alignment_policy_text(self, policy: int) -> str:
        if int(policy) == ENCODER_ALIGNMENT_POLICY_MANUAL_SAVE:
            return "Manual save (absolute encoder)"
        if int(policy) == ENCODER_ALIGNMENT_POLICY_POWER_ON:
            return "Auto align on servo start (incremental encoder)"
        return f"Unknown ({policy})"

    def _alignment_status_text(self, status: int) -> str:
        if int(status) == ENCODER_ALIGNMENT_STATUS_IDLE:
            return "Idle"
        if int(status) == ENCODER_ALIGNMENT_STATUS_REQUESTED:
            return "Requested"
        if int(status) == ENCODER_ALIGNMENT_STATUS_RUNNING:
            return "Running"
        if int(status) == ENCODER_ALIGNMENT_STATUS_DONE:
            return "Done"
        if int(status) == ENCODER_ALIGNMENT_STATUS_FAULT:
            return "Fault"
        return f"Unknown ({status})"

    def _alignment_save_state_text(self, policy: int, status: int, needs_flash_save: int) -> str:
        if int(needs_flash_save) != 0:
            return "Pending flash save"
        if int(policy) == ENCODER_ALIGNMENT_POLICY_POWER_ON:
            return "No flash save required"
        if int(status) == ENCODER_ALIGNMENT_STATUS_DONE:
            return "Saved or ready to reuse"
        return "Waiting for commissioning alignment"

    def _adc_offset_text(self, raw_offset: int) -> str:
        value = int(raw_offset) & 0xFFFF
        if value == 0x7FFF:
            return "0x7FFF (default)"
        return f"{value:d}"

    def _adc_offset_tooltip(self, raw_offset: int) -> str:
        value = int(raw_offset) & 0xFFFF
        if value == 0x7FFF:
            return "Default sentinel. Current-sensor offset calibration has not completed yet."
        return f"Raw ADC zero-current offset: {value:d} (0x{value:04X})"

    def _calibration_status_text(self, status: int) -> str:
        code = int(status)
        if code == CURRENT_CALIB_STATUS_IDLE:
            return "Idle"
        if code == CURRENT_CALIB_STATUS_RUNNING:
            return "Running"
        if code == CURRENT_CALIB_STATUS_DONE:
            return "Done"
        if code == CURRENT_CALIB_STATUS_TIMEOUT:
            return "Fault: Timeout"
        if code == CURRENT_CALIB_STATUS_OFFSET_INVALID:
            return "Fault: Offset Invalid"
        return f"Unknown ({code})"

    def _refresh_alignment_panel(self, snapshot) -> None:
        if snapshot is None:
            if hasattr(self, "alignment_policy_value_label"):
                self.alignment_policy_value_label.setText("Waiting for monitor data")
                self.alignment_status_value_label.setText("Idle")
                self.alignment_active_offset_value_label.setText("0 counts")
                self.alignment_captured_offset_value_label.setText("0 counts")
                self.alignment_save_state_value_label.setText("No pending save")
                self.save_alignment_flash_button.setEnabled(False)
                self.run_alignment_button.setEnabled(True)
            self._set_monitor_value("alignment_status", "Idle")
            self._set_monitor_value("alignment_offset", "0 counts")
            self._set_monitor_value("alignment_save", "No pending save")
            return

        policy = int(snapshot.debug_alignment_policy)
        status = int(snapshot.debug_alignment_status)
        needs_flash_save = int(snapshot.debug_alignment_needs_flash_save)
        save_state_text = self._alignment_save_state_text(policy, status, needs_flash_save)
        if hasattr(self, "alignment_policy_value_label"):
            self.alignment_policy_value_label.setText(self._alignment_policy_text(policy))
            self.alignment_status_value_label.setText(self._alignment_status_text(status))
            self.alignment_active_offset_value_label.setText(f"{snapshot.debug_encoder_offset_counts:d} counts")
            self.alignment_captured_offset_value_label.setText(
                f"{snapshot.debug_alignment_captured_offset_counts:d} counts"
            )
            self.alignment_save_state_value_label.setText(save_state_text)
            self.run_alignment_button.setEnabled(status != ENCODER_ALIGNMENT_STATUS_RUNNING)
            self.save_alignment_flash_button.setEnabled(
                (status != ENCODER_ALIGNMENT_STATUS_RUNNING)
                and (
                    (needs_flash_save != 0)
                    or (
                        (policy == ENCODER_ALIGNMENT_POLICY_MANUAL_SAVE)
                        and (status == ENCODER_ALIGNMENT_STATUS_DONE)
                    )
                )
            )
        self._set_monitor_value("alignment_status", self._alignment_status_text(status))
        self._set_monitor_value("alignment_offset", f"{snapshot.debug_encoder_offset_counts:d} counts")
        self._set_monitor_value("alignment_save", save_state_text)

    def _apply_trace_preset(self, preset_name: str) -> None:
        channels = TRACE_PRESETS.get(preset_name)
        if channels is None:
            return
        for combo, channel_code in zip(self.trace_channel_combos, channels):
            combo.setCurrentIndex(combo.findData(channel_code))

    def _clear_current_tuning_capture(self) -> None:
        self._current_tuning_capture = _empty_scope_state("Commissioning Scope")
        self.ctuning_current_scope_view.clear()
        self.ctuning_voltage_scope_view.clear()
        self.ctuning_status_label.setText("Ready")
        if self._active_trace_target == "current_tuning":
            self._active_trace_target = None

    def _clear_trace_capture(self) -> None:
        self._trace_capture = _empty_scope_state("Firmware Trace Scope")
        self.trace_scope_view.clear()
        self.trace_status_label.setText("Idle")
        if self._active_trace_target == "trace":
            self._active_trace_target = None

    def _queue_current_tuning_setup(
        self,
        description: str = "Configure Id Square Tuning",
        tuning_mode: int | None = None,
    ) -> None:
        self._enqueue_command(
            Command.CMD_APPLY_ID_SQUARE_TUNING,
            self._build_current_tuning_payload(tuning_mode),
            description,
        )

    def _apply_current_tuning_gains_to_foc(self) -> None:
        apply_scale = float(self.ctuning_foc_gain_scale_spin.value())
        current_kp = float(self.ctuning_kp_spin.value()) * apply_scale
        current_ki = float(self.ctuning_ki_spin.value()) * apply_scale

        self._set_table_float_value(self.motor_table, MOTOR_PARAM_CURRENT_P_GAIN, current_kp)
        self._set_table_float_value(self.motor_table, MOTOR_PARAM_CURRENT_I_GAIN, current_ki)

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
                f"Apply Id Gains To FOC chunk {index}/{len(chunks)}",
            )

        self.ctuning_status_label.setText(
            f"Applied scaled current PI to FOC Id/Iq: scale={apply_scale:.2f}x, Kp={current_kp:.4f}, Ki={current_ki:.4f}"
        )
        self._request_monitor_once()

    def _start_encoder_alignment_only(self) -> None:
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None and (
            last_monitor.enable_run or abs(float(last_monitor.act_speed)) > 1.0
        ):
            QtWidgets.QMessageBox.warning(
                self,
                "Encoder Alignment Safety",
                "Stop the drive and lock the rotor before running encoder alignment. This commissioning step assumes the motor is stationary while the driver captures the electrical-zero offset.",
            )
            return

        self.auto_poll_checkbox.setChecked(True)

        self._queue_current_tuning_setup(
            "Configure Encoder Alignment Setup",
            ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD,
        )
        self._enqueue_command(
            Command.CMD_START_ENCODER_ALIGNMENT,
            b"",
            "Run Encoder Alignment Only",
        )
        self._request_monitor_once()

    def _start_current_tuning(self) -> None:
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None and (
            last_monitor.enable_run or abs(float(last_monitor.act_speed)) > 1.0
        ):
            QtWidgets.QMessageBox.warning(
                self,
                "Current Tuning Safety",
                "Stop the drive and lock the rotor before starting Id tuning. This commissioning step assumes a stationary rotor with Iq close to zero.",
            )
            return

        if last_monitor is None or int(last_monitor.debug_alignment_status) != ENCODER_ALIGNMENT_STATUS_DONE:
            QtWidgets.QMessageBox.warning(
                self,
                "Id Tuning Flow",
                "Servo ON first, then wait until Driver Monitor shows Align Status = Done before starting Id tuning.",
            )
            return

        self.auto_poll_checkbox.setChecked(True)
        channel_codes = [12, 11, 13, 14]
        title = "Id Square-Wave Tuning Response"
        start_description = "Start Id Square-Wave Tuning"
        trace_description = "Start Id Tuning Trace"

        self._queue_current_tuning_setup("Configure Id Square Tuning")
        self._clear_current_tuning_capture()
        sample_period_s = self._current_tuning_sample_period_s()
        decimation = int(self.ctuning_rate_combo.currentData() or 0)
        self._current_tuning_capture = ScopeCaptureState(
            title=title,
            sample_period_s=sample_period_s,
            total_samples=TRACE_TOTAL_SAMPLES,
            series_defs=[dict(TRACE_CHANNEL_META[channel]) for channel in channel_codes],
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
        trace_payload = bytes([1, 0, *channel_codes, decimation])
        self._enqueue_command(Command.CMD_APPLY_TRACE, trace_payload, trace_description)
        self._enqueue_command(
            Command.CMD_START_ID_SQUARE_TUNING,
            b"",
            start_description,
        )

    def _stop_current_tuning(self) -> None:
        self._enqueue_command(
            Command.CMD_STOP_ID_SQUARE_TUNING,
            b"",
            "Stop Alignment / Id Test",
        )
        self._enqueue_command(
            Command.CMD_APPLY_TRACE,
            bytes([0, 0, 0, 0, 0, 0, 0]),
            "Stop Current Tuning Trace",
        )
        self._current_tuning_capture.active = False
        self.ctuning_status_label.setText("Stopped")
        if self._active_trace_target == "current_tuning":
            self._active_trace_target = None

    def _build_autotune_payload(self) -> bytes:
        return struct.pack(
            "<8f",
            float(self.autotune_rs_low_spin.value()),
            float(self.autotune_rs_high_spin.value()),
            float(self.autotune_ls_voltage_spin.value()),
            float(self.autotune_flux_frequency_spin.value()),
            float(self.autotune_flux_voltage_spin.value()),
            float(self.autotune_current_bw_spin.value()),
            float(self.autotune_speed_bw_spin.value()),
            float(self.autotune_position_bw_spin.value()),
        )

    def _autotune_state_text(self, state: int) -> str:
        if int(state) == MOTOR_AUTOTUNE_STATE_IDLE:
            return "Idle"
        if int(state) == MOTOR_AUTOTUNE_STATE_RS:
            return "Rs Measurement"
        if int(state) == MOTOR_AUTOTUNE_STATE_LS:
            return "Ls Measurement"
        if int(state) == MOTOR_AUTOTUNE_STATE_FLUX:
            return "Flux / Pole Pairs"
        if int(state) == MOTOR_AUTOTUNE_STATE_DONE:
            return "Done"
        if int(state) == MOTOR_AUTOTUNE_STATE_ERROR:
            return "Error"
        return f"Unknown ({state})"

    def _autotune_error_text(self, error: int) -> str:
        if int(error) == MOTOR_AUTOTUNE_ERROR_NONE:
            return "No error"
        if int(error) == MOTOR_AUTOTUNE_ERROR_OVERCURRENT:
            return "Overcurrent during tuning"
        if int(error) == MOTOR_AUTOTUNE_ERROR_STALL:
            return "Stall / no motion in open-loop stage"
        if int(error) == MOTOR_AUTOTUNE_ERROR_INVALID_CONFIG:
            return "Invalid configuration"
        if int(error) == MOTOR_AUTOTUNE_ERROR_SIGNAL:
            return "Signal quality / estimation failure"
        return f"Unknown ({error})"

    def _autotune_chart_title(self, stage: int) -> str:
        if int(stage) == MOTOR_AUTOTUNE_CHART_RS:
            return "Rs Calibration Chart"
        if int(stage) == MOTOR_AUTOTUNE_CHART_LS:
            return "Ls Step Response"
        return "Motor Auto-Tune Charts"

    def _autotune_chart_series_defs(self, stage: int) -> list[dict[str, str]]:
        if int(stage) == MOTOR_AUTOTUNE_CHART_RS:
            return [
                {"label": "Id", "unit": "A", "color": "#1f77b4"},
                {"label": "Vd", "unit": "V", "color": "#8c564b"},
                {"label": "Id Ref", "unit": "A", "color": "#bcbd22"},
            ]
        if int(stage) == MOTOR_AUTOTUNE_CHART_LS:
            return [
                {"label": "Id", "unit": "A", "color": "#1f77b4"},
                {"label": "Vd Step", "unit": "V", "color": "#d62728"},
                {"label": "Marker", "unit": "", "color": "#7f7f7f"},
            ]
        return []

    def _refresh_autotune_panel(self, snapshot) -> None:
        if not hasattr(self, "autotune_state_value_label"):
            return
        if snapshot is None:
            self.autotune_state_value_label.setText("Idle")
            self.autotune_error_value_label.setText("No error")
            self.autotune_chart_state_label.setText("Waiting for capture")
            self.autotune_progress_bar.setValue(0)
            self.autotune_rs_value_label.setText("-")
            self.autotune_ls_value_label.setText("-")
            self.autotune_ke_value_label.setText("-")
            self.autotune_flux_value_label.setText("-")
            self.autotune_pp_value_label.setText("-")
            self.autotune_current_gain_value_label.setText("-")
            self.autotune_speed_gain_value_label.setText("-")
            self.autotune_position_gain_value_label.setText("-")
            self.autotune_apply_button.setEnabled(False)
            return

        self.autotune_state_value_label.setText(
            self._autotune_state_text(snapshot.autotune_state)
        )
        self.autotune_error_value_label.setText(
            self._autotune_error_text(snapshot.autotune_error)
        )
        self.autotune_progress_bar.setValue(int(snapshot.autotune_progress_percent))
        self.autotune_chart_state_label.setText(
            "Chart ready for host download"
            if int(snapshot.autotune_data_ready) != 0
            else "No pending chart transfer"
        )
        self.autotune_rs_value_label.setText(f"{snapshot.autotune_measured_rs:.6f} ohm")
        self.autotune_ls_value_label.setText(f"{snapshot.autotune_measured_ls * 1e6:.2f} uH")
        self.autotune_ke_value_label.setText(
            f"{snapshot.autotune_measured_ke:.6f} V/(rad/s)"
        )
        self.autotune_flux_value_label.setText(f"{snapshot.autotune_measured_flux:.6f} Wb")
        self.autotune_pp_value_label.setText(f"{snapshot.autotune_measured_pole_pairs:.2f}")
        self.autotune_current_gain_value_label.setText(
            f"Kp={snapshot.autotune_current_kp:.4f}, Ki={snapshot.autotune_current_ki:.4f}"
        )
        self.autotune_speed_gain_value_label.setText(
            f"Kp={snapshot.autotune_speed_kp:.4f}, Ki={snapshot.autotune_speed_ki:.4f}"
        )
        self.autotune_position_gain_value_label.setText(
            f"Kp={snapshot.autotune_position_kp:.4f}"
        )
        self.autotune_apply_button.setEnabled(
            int(snapshot.autotune_state) == MOTOR_AUTOTUNE_STATE_DONE
        )

    def _clear_autotune_capture(self) -> None:
        self._autotune_capture = _empty_scope_state("Motor Auto-Tune Charts")
        self.autotune_scope_view.clear()
        self.autotune_chart_state_label.setText("Waiting for capture")
        if self._active_trace_target == "autotune":
            self._active_trace_target = None

    def _start_motor_autotune(self) -> None:
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None and last_monitor.enable_run:
            QtWidgets.QMessageBox.warning(
                self,
                "Auto-Tune Safety",
                "Stop the drive before starting auto-tune. Rs/Ls assume a locked rotor, and the flux stage should only run after the electrical-zero setup is ready.",
            )
            return

        self._clear_autotune_capture()
        self.autotune_chart_state_label.setText("Starting auto-tune...")
        self._enqueue_command(
            Command.CMD_START_AUTOTUNING_T,
            self._build_autotune_payload(),
            "Start Motor Auto-Tune",
        )

    def _stop_motor_autotune(self) -> None:
        self._enqueue_command(
            Command.CMD_STOP_AUTOTUNING_T,
            b"",
            "Stop Motor Auto-Tune",
        )
        self.autotune_chart_state_label.setText("Stopped")

    def _apply_motor_autotune_estimates(self) -> None:
        self._enqueue_command(
            Command.CMD_UPDATE_TUNING_GAIN,
            b"",
            "Apply Auto-Tune Estimates",
        )

    def _handle_autotune_chunk(self, chunk: AutoTuneChunk) -> None:
        stage = int(chunk.stage)
        if (
            self._autotune_capture.total_samples != chunk.total_samples
            or self._autotune_capture.title != self._autotune_chart_title(stage)
        ):
            self._autotune_capture = ScopeCaptureState(
                title=self._autotune_chart_title(stage),
                sample_period_s=chunk.sample_period_s,
                total_samples=chunk.total_samples,
                series_defs=self._autotune_chart_series_defs(stage),
                series_data=[[], [], []],
                source_indices=[0, 1, 2],
                active=True,
                received_samples=0,
            )
            self._active_trace_target = "autotune"

        if chunk.sample_start == 0:
            self._autotune_capture.series_data = [[], [], []]
            self._autotune_capture.received_samples = 0

        self._autotune_capture.series_data[0].extend(chunk.primary)
        self._autotune_capture.series_data[1].extend(chunk.secondary)
        self._autotune_capture.series_data[2].extend(chunk.tertiary)
        self._autotune_capture.received_samples = len(self._autotune_capture.series_data[0])
        self._autotune_capture.active = (
            self._autotune_capture.received_samples < self._autotune_capture.total_samples
        )
        self.autotune_chart_state_label.setText(
            f"Received {self._autotune_capture.received_samples}/{self._autotune_capture.total_samples} chart samples"
        )
        self._update_scope_view(
            self.autotune_scope_view,
            self._autotune_capture,
            "Auto-tune chart",
        )

        if (
            self._autotune_capture.received_samples >= self._autotune_capture.total_samples
            and self._latest_monitor_snapshot is not None
            and int(self._latest_monitor_snapshot.autotune_data_ready) != 0
        ):
            self.autotune_chart_state_label.setText("Chart complete, continuing auto-tune...")
            self._enqueue_command(
                Command.CMD_CONTINUE_AUTO_TUNING_STATE,
                b"",
                "Continue Auto-Tune Stage",
                quiet=True,
            )

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
            self.connect_button.blockSignals(True)
            self.connect_button.setChecked(False)
            self.connect_button.blockSignals(False)
            return
        baudrate = 115200
        self._pending_frames.clear()
        self._awaiting_ack = False
        self._last_sent = None
        self._ack_timer.stop()
        self._worker.request_open(port_name, baudrate)

    def _disconnect_port(self) -> None:
        self._worker.request_close()

    def _toggle_connection(self) -> None:
        if self._connected:
            self._disconnect_port()
        else:
            self._connect_selected_port()

    def _handle_connection_status(self, connected: bool, port_name: str) -> None:
        self._connected = connected
        self._current_port = port_name
        self.port_combo.setEnabled(not connected)
        self.refresh_ports_button.setEnabled(not connected)
        self.connect_button.blockSignals(True)
        self.connect_button.setChecked(connected)
        self.connect_button.blockSignals(False)

        if connected:
            self.connection_state_label.setText(f"Connected: {port_name}")
            self.connection_state_label.setStyleSheet("color: #2e7d32; font-weight: 600;")
            self.connect_button.setText("Disconnect")
            self.connect_button.setStyleSheet(
                "background-color: #c62828; color: white; font-weight: 600;"
            )
            self.statusBar().showMessage(f"Connected to {port_name}")
            self._clear_trend_history()
            self._clear_current_tuning_capture()
            self._clear_trace_capture()
            self._try_send_next()
        else:
            self.connection_state_label.setText("Disconnected")
            self.connection_state_label.setStyleSheet("color: #c62828; font-weight: 600;")
            self.connect_button.setText("Connect")
            self.connect_button.setStyleSheet(
                "background-color: #2e7d32; color: white; font-weight: 600;"
            )
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

    def _reset_alarm(self) -> None:
        self._enqueue_command(Command.CMD_ACK_FAULT, b"", "Reset Alarm")
        self.statusBar().showMessage(
            "Reset Alarm sent. Protection stays active until firmware reports the fault is gone.",
            4000,
        )
        self._request_monitor_once()

    def _clear_alarm_history(self) -> None:
        self._alarm_history.clear()
        self.alarm_history_table.setRowCount(0)
        self.statusBar().showMessage("Alarm history cleared", 2500)

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
            if self._last_sent.command == Command.CMD_READ_DRIVER:
                self._set_parameter_status("Driver parameter read timed out.", "error")
            elif self._last_sent.command == Command.CMD_READ_MOTOR:
                self._set_parameter_status("Motor parameter read timed out.", "error")
            elif self._last_sent.command == Command.CMD_WRITE_DRIVER:
                self._set_parameter_status("Driver parameter write timed out.", "error")
            elif self._last_sent.command == Command.CMD_WRITE_MOTOR:
                self._set_parameter_status("Motor parameter write timed out.", "error")
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

    def _set_parameter_status(self, text: str, severity: str = "info") -> None:
        if not hasattr(self, "parameter_status_label"):
            return
        palette = {
            "info": "#bbbbbb",
            "ok": "#2e7d32",
            "warn": "#b26a00",
            "error": "#c62828",
        }
        color = palette.get(severity, palette["info"])
        self.parameter_status_label.setText(text)
        self.parameter_status_label.setStyleSheet(f"color: {color}; font-weight: 600;")

    def _read_driver_parameters(self) -> None:
        self._set_parameter_status("Reading driver parameters...", "info")
        self._enqueue_command(
            Command.CMD_READ_DRIVER,
            bytes([len(DRIVER_PARAMETER_NAMES)]),
            "Read Driver Parameters",
        )

    def _read_motor_parameters(self) -> None:
        self._set_parameter_status("Reading motor parameters...", "info")
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

        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Driver Parameter Write",
            "Write the current Driver Parameters table to the drive?",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.No,
        )
        if reply != QtWidgets.QMessageBox.StandardButton.Yes:
            self._set_parameter_status("Driver parameter write cancelled.", "warn")
            return

        self._load_foc_controls_from_driver_table()
        self._set_parameter_status("Writing driver parameters...", "info")

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

        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Motor Parameter Write",
            "Write the current Motor Parameters table to the drive?",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.No,
        )
        if reply != QtWidgets.QMessageBox.StandardButton.Yes:
            self._set_parameter_status("Motor parameter write cancelled.", "warn")
            return

        chunks = build_parameter_write_chunks(values, chunk_size=20)
        self._set_parameter_status("Writing motor parameters...", "info")
        for index, payload in enumerate(chunks, start=1):
            self._enqueue_command(
                Command.CMD_WRITE_MOTOR,
                payload,
                f"Write Motor Parameters chunk {index}/{len(chunks)}",
            )

    def _apply_control_timing_mode(self) -> None:
        selected_mode = CONTROL_TIMING_MODE_16KHZ
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
        self.auto_poll_checkbox.setChecked(True)
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
            sent_command = self._last_sent.command if self._last_sent is not None else None
            echoed_command = frame.payload[0] if frame.payload else None
            if not (self._last_sent and self._last_sent.quiet):
                if echoed_command is None:
                    self._append_log("ACK received")
                else:
                    self._append_log(f"ACK received for command 0x{echoed_command:02X}")
            if sent_command == Command.CMD_WRITE_DRIVER:
                self._set_parameter_status("Driver parameter write acknowledged.", "ok")
            elif sent_command == Command.CMD_WRITE_MOTOR:
                self._set_parameter_status("Motor parameter write chunk acknowledged.", "ok")
            self._ack_timer.stop()
            self._awaiting_ack = False
            self._last_sent = None
            self._try_send_next()
            return

        if frame.code == ACK_ERROR:
            sent_command = self._last_sent.command if self._last_sent is not None else None
            self._append_log("Driver returned ACK_ERROR")
            if sent_command == Command.CMD_READ_DRIVER:
                self._set_parameter_status("Driver parameter read failed.", "error")
                QtWidgets.QMessageBox.warning(self, "Read Driver Parameters", "Driver parameter read failed.")
            elif sent_command == Command.CMD_READ_MOTOR:
                self._set_parameter_status("Motor parameter read failed.", "error")
                QtWidgets.QMessageBox.warning(self, "Read Motor Parameters", "Motor parameter read failed.")
            elif sent_command == Command.CMD_WRITE_DRIVER:
                self._set_parameter_status("Driver parameter write failed.", "error")
                QtWidgets.QMessageBox.warning(self, "Write Driver Parameters", "Driver parameter write failed.")
            elif sent_command == Command.CMD_WRITE_MOTOR:
                self._set_parameter_status("Motor parameter write failed.", "error")
                QtWidgets.QMessageBox.warning(self, "Write Motor Parameters", "Motor parameter write failed.")
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

        if subcommand == UpdateCode.CMD_FOC_DEBUG_TEXT:
            message = body.split(b"\x00", 1)[0].decode("utf-8", errors="replace").strip()
            if message:
                self._append_log(message)
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

        if subcommand == UpdateCode.CMD_AUTOTUNING_DATA_THINH:
            try:
                chunk = parse_autotune_payload(body)
            except ValueError as exc:
                self._append_log(f"Auto-tune parse error: {exc}")
                return
            self._handle_autotune_chunk(chunk)
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
            self._load_foc_controls_from_driver_table()
            self._append_log(f"Driver parameter chunk received ({len(entries)} items)")
            self._set_parameter_status(
                f"Driver parameters loaded successfully ({len(entries)} items).",
                "ok",
            )
            self.statusBar().showMessage("Driver parameters loaded", 3000)
            return

        if subcommand == UpdateCode.CMD_READ_MOTOR_DATA:
            entries = parse_parameter_chunk(body)
            self._update_parameter_table(self.motor_table, entries)
            self._append_log(f"Motor parameter chunk received ({len(entries)} items)")
            self._set_parameter_status(
                f"Motor parameters loaded successfully ({len(entries)} items).",
                "ok",
            )
            self.statusBar().showMessage("Motor parameters loaded", 3000)
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

    def _update_current_tuning_scope_views(self) -> None:
        capture = self._current_tuning_capture
        expected = capture.total_samples or max(
            (len(series) for series in capture.series_data),
            default=0,
        )
        status_text = f"Id tuning: {capture.received_samples}/{expected} samples"

        def update_subset(
            view: ScopeCaptureView,
            title: str,
            indices: list[int],
        ) -> None:
            series_defs: list[dict[str, str]] = []
            series_data: list[list[float]] = []
            for index in indices:
                if index < len(capture.series_defs) and index < len(capture.series_data):
                    series_defs.append(capture.series_defs[index])
                    series_data.append(capture.series_data[index])

            if series_defs and series_data:
                view.set_capture(
                    title,
                    series_defs,
                    series_data,
                    capture.sample_period_s,
                    status_text,
                )
            else:
                view.clear()

        update_subset(
            self.ctuning_current_scope_view,
            "Id Square-Wave Current Response",
            [0, 1],
        )
        update_subset(
            self.ctuning_voltage_scope_view,
            "Id Square-Wave Voltage Response",
            [2, 3],
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
        self._update_current_tuning_scope_views()

    def _handle_trace_chunk(self, chunk: TraceChunk) -> None:
        capture_kind = self._active_trace_target
        if capture_kind == "current_tuning":
            capture = self._current_tuning_capture
            view = None
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
        if capture_kind == "current_tuning":
            self._update_current_tuning_scope_views()
        else:
            self._update_scope_view(view, capture, prefix)

    def _append_snapshot_to_trend_buffer(self, snapshot) -> None:
        self._trend_buffer.append_sample(
            time.monotonic(),
            {
                "phase_u": snapshot.phase_u,
                "phase_v": snapshot.phase_v,
                "phase_w": snapshot.phase_w,
                "id_ref": snapshot.id_ref,
                "id_current": snapshot.id_current,
                "iq_ref": snapshot.iq_ref,
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
                "id_ref": (
                    float(last_monitor.id_ref) if last_monitor is not None else 0.0
                ),
                "id_current": (
                    float(last_monitor.id_current) if last_monitor is not None else 0.0
                ),
                "iq_ref": (
                    float(last_monitor.iq_ref) if last_monitor is not None else 0.0
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
        if run_mode == RUN_MODE_AUTOTUNE:
            return "Motor Auto-Tune"
        if run_mode == RUN_MODE_ALIGNMENT_ONLY:
            return "Encoder Alignment / Hold"
        if run_mode == RUN_MODE_OPEN_LOOP_VF:
            return "Open Loop V/F"
        if run_mode == RUN_MODE_FOC:
            return "FOC"
        return f"Unknown ({run_mode})"

    def _format_debug_terminal_text(self, snapshot) -> str:
        timestamp = QtCore.QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")
        theta_deg = float(snapshot.theta) * 180.0 / 3.141592653589793
        mechanical_deg = (
            float(snapshot.debug_mechanical_angle_rad) * 180.0 / 3.141592653589793
        )
        electrical_deg = (
            float(snapshot.debug_electrical_angle_rad) * 180.0 / 3.141592653589793
        )

        lines = [
            f"Snapshot: {timestamp}",
            "",
            "[Drive]",
            f"Port: {self._current_port or '-'}",
            f"Run Mode: {self._run_mode_text(snapshot.run_mode)}",
            f"Servo: {'ON' if snapshot.enable_run else 'OFF'}",
            f"Fault: {format_fault_text(snapshot.fault_occurred)}",
            f"Timing: {self._timing_mode_text(snapshot.control_timing_mode)} | Control {snapshot.control_loop_frequency_hz:.1f} Hz | Speed {snapshot.speed_loop_frequency_hz:.1f} Hz",
            f"ADC Offset Ia / Ib: {self._adc_offset_text(snapshot.adc_offset_ia)} / {self._adc_offset_text(snapshot.adc_offset_ib)}",
            f"Calibration Status: {self._calibration_status_text(snapshot.calibration_status)}",
            "",
            "[Encoder Alignment]",
            f"Policy: {self._alignment_policy_text(snapshot.debug_alignment_policy)}",
            f"Status: {self._alignment_status_text(snapshot.debug_alignment_status)}",
            f"Active Offset: {snapshot.debug_encoder_offset_counts:d} counts",
            f"Captured Offset: {snapshot.debug_alignment_captured_offset_counts:d} counts",
            f"Flash Save: {self._alignment_save_state_text(snapshot.debug_alignment_policy, snapshot.debug_alignment_status, snapshot.debug_alignment_needs_flash_save)}",
            "",
            "[Motor Auto-Tune]",
            f"State: {self._autotune_state_text(snapshot.autotune_state)}",
            f"Error: {self._autotune_error_text(snapshot.autotune_error)}",
            f"Progress: {snapshot.autotune_progress_percent:d} %",
            f"Rs / Ls: {snapshot.autotune_measured_rs:.6f} ohm / {snapshot.autotune_measured_ls * 1e6:.2f} uH",
            f"Ke / Flux: {snapshot.autotune_measured_ke:.6f} V/(rad/s) / {snapshot.autotune_measured_flux:.6f} Wb",
            f"Pole Pairs: {snapshot.autotune_measured_pole_pairs:.2f}",
            f"Current PI: Kp={snapshot.autotune_current_kp:.4f}, Ki={snapshot.autotune_current_ki:.4f}",
            f"Speed PI: Kp={snapshot.autotune_speed_kp:.4f}, Ki={snapshot.autotune_speed_ki:.4f}",
            f"Position P: Kp={snapshot.autotune_position_kp:.4f}",
            "",
            "[Motion]",
            f"Cmd Speed: {snapshot.cmd_speed:.3f}",
            f"Act Speed: {snapshot.act_speed:.3f}",
            f"Speed Error: {snapshot.speed_error:.3f}",
            f"Act Position: {snapshot.act_position:.3f}",
            "",
            "[Currents]",
            f"Id Ref / Id: {snapshot.id_ref:.3f} A / {snapshot.id_current:.3f} A",
            f"Iq Ref / Iq: {snapshot.iq_ref:.3f} A / {snapshot.iq_current:.3f} A",
            f"Phase Currents: Iu={snapshot.phase_u:.3f} A, Iv={snapshot.phase_v:.3f} A, Iw={snapshot.phase_w:.3f} A",
            "",
            "[Voltages]",
            f"Vdc: {snapshot.vdc:.3f} V",
            f"Vd / Vq: {snapshot.vd:.3f} V / {snapshot.vq:.3f} V",
            f"Phase Voltages: Vu={snapshot.v_phase_u:.3f} V, Vv={snapshot.v_phase_v:.3f} V, Vw={snapshot.v_phase_w:.3f} V",
            "",
            "[Angles]",
            f"Theta: {snapshot.theta:.6f} rad ({theta_deg:.2f} deg)",
            f"Mechanical: {snapshot.debug_mechanical_angle_rad:.6f} rad ({mechanical_deg:.2f} deg)",
            f"Electrical: {snapshot.debug_electrical_angle_rad:.6f} rad ({electrical_deg:.2f} deg)",
        ]

        if (
            abs(float(snapshot.vf_frequency)) > 1e-6
            or abs(float(snapshot.vf_voltage)) > 1e-6
            or abs(float(snapshot.debug_open_loop_electrical_hz_cmd)) > 1e-6
            or snapshot.run_mode == RUN_MODE_OPEN_LOOP_VF
        ):
            lines.extend(
                [
                    "",
                    "[Open Loop]",
                    f"Cmd Electrical Hz: {snapshot.debug_open_loop_electrical_hz_cmd:.6f}",
                    f"Sync RPM Command: {snapshot.debug_open_loop_sync_rpm_cmd:.6f}",
                    f"Observed Electrical Hz: {snapshot.debug_observed_electrical_hz:.6f}",
                    f"Raw Speed RPM: {snapshot.debug_speed_raw_rpm:.6f}",
                    f"Expected DeltaPos: {snapshot.debug_expected_delta_pos_sync:.6f}",
                    f"Measured DeltaPos: {snapshot.debug_delta_pos}",
                ]
            )

        if (
            snapshot.debug_isr_delta_cycles != 0
            or abs(float(snapshot.debug_isr_frequency_hz)) > 1e-6
            or snapshot.debug_isr_measure_only_mode != 0
        ):
            lines.extend(
                [
                    "",
                    "[ISR Debug]",
                    f"Isr Delta Cycles: {snapshot.debug_isr_delta_cycles}",
                    f"Isr Period: {snapshot.debug_isr_period_us:.6f} us",
                    f"Isr Frequency: {snapshot.debug_isr_frequency_hz:.6f} Hz",
                    f"Measure Only Mode: {snapshot.debug_isr_measure_only_mode}",
                    f"Measure Edge Frequency: {snapshot.debug_isr_measure_edge_frequency_hz:.6f} Hz",
                ]
            )

        return "\n".join(lines)

    def _refresh_debug_terminal(self, snapshot) -> None:
        if snapshot is None:
            self.debug_terminal_edit.setPlainText(
                "Waiting for monitor data...\n\n"
                "Connect the driver, press Refresh Snapshot, then copy the text here."
            )
            return
        self.debug_terminal_edit.setPlainText(self._format_debug_terminal_text(snapshot))

    def _set_monitor_value(self, key: str, text: str, tooltip: str | None = None) -> None:
        label = self.monitor_labels.get(key)
        if label is None:
            return
        label.setText(text)
        if tooltip is not None:
            label.setToolTip(tooltip)

    def _clear_monitor_display(self) -> None:
        for label in self.monitor_labels.values():
            label.setText("-")
            label.setToolTip("")
            label.setStyleSheet("")
        self._set_monitor_value("enable_run", "OFF")
        self._set_monitor_value("run_mode", "Idle")
        self._set_monitor_value("control_timing_mode", self._timing_mode_text(self._active_timing_mode))
        self._set_monitor_value("effective_loop_hz", f"{self._active_control_loop_hz:.0f} Hz")
        self._set_monitor_value("calibration_status", "Idle")
        self._set_monitor_value("adc_offset_ia", "0x7FFF (default)")
        self._set_monitor_value("adc_offset_ib", "0x7FFF (default)")
        self._set_monitor_value("alignment_status", "Idle")
        self._set_monitor_value("alignment_offset", "0 counts")
        self._set_monitor_value("alignment_save", "No pending save")
        fault_label = self.monitor_labels.get("fault_occurred")
        if fault_label is not None:
            fault_label.setText("OK")
            fault_label.setStyleSheet("color: #6aa84f; font-weight: 600;")
        self._active_fault_code = 0
        self._active_alarm_summary_text = "No active alarms"
        self._active_alarm_detail_text = "No active alarms"
        self._active_alarm_source_text = "-"
        self._active_alarm_severity_text = "OK"
        self._set_alarm_banner_state(
            0,
            "No active alarms",
            "Drive protection is idle.",
            "OK",
        )
        self._set_alarm_panel_state(0, "No active alarms", "No active alarms", "OK", "-")
        self._refresh_alignment_panel(None)
        self._refresh_foc_control_panel(None)

    def _fault_severity(self, fault_code: int) -> str:
        if int(fault_code) == 0:
            return "OK"
        critical_mask = 0x0001 | 0x0002 | 0x0004 | 0x0008 | 0x0040 | 0x0080
        if int(fault_code) & critical_mask:
            return "CRITICAL"
        return "WARNING"

    def _alarm_palette(self, severity: str) -> tuple[str, str]:
        if severity == "CRITICAL":
            return ("#fff1f0", "#c62828")
        if severity == "WARNING":
            return ("#fff7e6", "#b26a00")
        return ("#eef8ef", "#2e7d32")

    def _set_alarm_banner_state(
        self,
        fault_code: int,
        summary_text: str,
        detail_text: str,
        severity: str,
    ) -> None:
        background, foreground = self._alarm_palette(severity)
        self.alarm_banner_status_label.setText(summary_text)
        self.alarm_banner_detail_label.setText(detail_text)
        self.alarm_banner_reset_button.setEnabled(int(fault_code) != 0)
        if hasattr(self, "ack_fault_button"):
            self.ack_fault_button.setEnabled(int(fault_code) != 0)
        self.alarm_banner_title_label.setText(
            "Alarm Active" if int(fault_code) != 0 else "Alarm Status"
        )
        self.alarm_banner_status_label.setStyleSheet(
            f"color: {foreground}; font-weight: 700;"
        )
        self.alarm_banner_detail_label.setStyleSheet(f"color: {foreground};")
        self.alarm_banner_title_label.setStyleSheet(f"color: {foreground}; font-weight: 700;")
        self.alarm_banner_status_label.parentWidget().setStyleSheet(
            f"QFrame {{ background-color: {background}; border: 1px solid {foreground}; border-radius: 6px; }}"
        )

    def _set_alarm_panel_state(
        self,
        fault_code: int,
        summary_text: str,
        detail_text: str,
        severity: str,
        source_text: str,
    ) -> None:
        background, foreground = self._alarm_palette(severity)
        self.alarm_active_status_value_label.setText(detail_text)
        self.alarm_active_status_value_label.setToolTip(summary_text)
        self.alarm_active_severity_value_label.setText(severity)
        self.alarm_active_source_value_label.setText(source_text)
        self.alarm_active_code_value_label.setText(f"0x{int(fault_code) & 0xFFFF:04X}")
        style = f"color: {foreground}; font-weight: 700; background-color: {background}; padding: 2px 6px; border-radius: 4px;"
        self.alarm_active_status_value_label.setStyleSheet(style)
        self.alarm_active_severity_value_label.setStyleSheet(style)
        self.alarm_reset_button.setEnabled(int(fault_code) != 0)

    def _append_alarm_history_entry(
        self,
        fault_code: int,
        source_text: str,
        summary_text: str,
        severity: str,
    ) -> None:
        entry = AlarmHistoryEntry(
            timestamp_text=QtCore.QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss"),
            severity_text=severity,
            source_text=source_text,
            fault_code=int(fault_code),
            summary_text=summary_text,
        )
        self._alarm_history.insert(0, entry)
        self.alarm_history_table.insertRow(0)

        values = [
            entry.timestamp_text,
            entry.severity_text,
            entry.source_text,
            f"0x{entry.fault_code & 0xFFFF:04X}",
            entry.summary_text,
        ]
        background, foreground = self._alarm_palette(severity)
        bg_color = QtGui.QColor(background)
        fg_color = QtGui.QColor(foreground)
        for column, value in enumerate(values):
            item = QtWidgets.QTableWidgetItem(value)
            item.setBackground(bg_color)
            item.setForeground(fg_color)
            if column == 1:
                item.setFont(QtGui.QFont(item.font().family(), item.font().pointSize(), QtGui.QFont.Weight.Bold))
            self.alarm_history_table.setItem(0, column, item)

        while self.alarm_history_table.rowCount() > 200:
            self.alarm_history_table.removeRow(self.alarm_history_table.rowCount() - 1)
        if len(self._alarm_history) > 200:
            self._alarm_history = self._alarm_history[:200]

    def _handle_fault_transition(
        self,
        fault_code: int,
        source_text: str,
        detail_text: str | None = None,
    ) -> None:
        fault_code = int(fault_code)
        summary_text = self._fault_summary_text(fault_code)
        incoming_detail_text = detail_text or format_fault_text(fault_code)
        severity = self._fault_severity(fault_code)

        if fault_code == 0:
            self._active_alarm_summary_text = "No active alarms"
            self._active_alarm_detail_text = "No active alarms"
            self._active_alarm_source_text = "-"
            self._active_alarm_severity_text = "OK"
        elif (fault_code != self._active_fault_code) or (detail_text is not None):
            self._active_alarm_summary_text = summary_text
            self._active_alarm_detail_text = incoming_detail_text
            self._active_alarm_source_text = source_text
            self._active_alarm_severity_text = severity

        self._set_alarm_banner_state(
            fault_code,
            self._active_alarm_summary_text,
            self._active_alarm_detail_text,
            self._active_alarm_severity_text,
        )
        self._set_alarm_panel_state(
            fault_code,
            self._active_alarm_summary_text,
            self._active_alarm_detail_text,
            self._active_alarm_severity_text,
            self._active_alarm_source_text,
        )

        if fault_code != self._active_fault_code:
            if fault_code == 0 and self._active_fault_code != 0:
                self._append_alarm_history_entry(
                    0,
                    source_text,
                    "Alarm cleared",
                    "OK",
                )
                self.statusBar().showMessage("Alarm cleared", 3000)
            elif fault_code != 0:
                self._append_alarm_history_entry(
                    fault_code,
                    source_text,
                    incoming_detail_text,
                    severity,
                )
                if hasattr(self, "quick_command_tabs") and hasattr(self, "alarm_tab"):
                    self.quick_command_tabs.setCurrentWidget(self.alarm_tab)
                self.statusBar().showMessage(f"ALARM: {incoming_detail_text}", 5000)
            self._active_fault_code = fault_code

    def _fault_summary_text(self, fault_code: int) -> str:
        labels = decode_fault_flags(int(fault_code))
        return ", ".join(labels)

    def _fault_detail_text(
        self,
        fault_code: int,
        *,
        phase_u: float | None = None,
        phase_v: float | None = None,
        phase_w: float | None = None,
    ) -> str:
        detail = format_fault_text(fault_code)
        if (int(fault_code) & 0x0001) != 0 and None not in (phase_u, phase_v, phase_w):
            detail += (
                f" | Ia={float(phase_u):.3f} A, "
                f"Ib={float(phase_v):.3f} A, "
                f"Ic={float(phase_w):.3f} A"
            )
        return detail

    def _set_fault_label(self, fault_code: int) -> None:
        label = self.monitor_labels.get("fault_occurred")
        if label is None:
            return
        summary = self._fault_summary_text(fault_code)
        label.setText(summary)
        label.setToolTip(format_fault_text(fault_code))
        if fault_code == 0:
            label.setStyleSheet("color: #6aa84f; font-weight: 600;")
        else:
            label.setStyleSheet("color: #d9534f; font-weight: 600;")

    def _update_monitor(self, snapshot) -> None:
        self._set_monitor_value("enable_run", "ON" if snapshot.enable_run else "OFF")
        self._set_monitor_value("run_mode", self._run_mode_text(snapshot.run_mode))
        self._set_monitor_value("control_timing_mode", self._timing_mode_text(snapshot.control_timing_mode))
        self._set_monitor_value("effective_loop_hz", f"{snapshot.control_loop_frequency_hz:.0f} Hz")
        self._set_fault_label(snapshot.fault_occurred)
        self._handle_fault_transition(snapshot.fault_occurred, "Monitor")
        self._refresh_alignment_panel(snapshot)
        self._refresh_foc_control_panel(snapshot)
        self._refresh_autotune_panel(snapshot)
        self._set_monitor_value("vdc", f"{snapshot.vdc:.3f} V")
        self._set_monitor_value("temperature", f"{snapshot.temperature:.3f} C")
        self._set_monitor_value("calibration_status", self._calibration_status_text(snapshot.calibration_status))
        self._set_monitor_value(
            "adc_offset_ia",
            self._adc_offset_text(snapshot.adc_offset_ia),
            self._adc_offset_tooltip(snapshot.adc_offset_ia),
        )
        self._set_monitor_value(
            "adc_offset_ib",
            self._adc_offset_text(snapshot.adc_offset_ib),
            self._adc_offset_tooltip(snapshot.adc_offset_ib),
        )
        self._set_monitor_value("cmd_speed", f"{snapshot.cmd_speed:.3f}")
        self._set_monitor_value("act_speed", f"{snapshot.act_speed:.3f}")
        self._set_monitor_value("speed_error", f"{snapshot.speed_error:.3f}")
        self._set_monitor_value("act_position", f"{snapshot.act_position:.3f}")
        self._set_monitor_value("position_error", f"{snapshot.position_error:.3f}")
        self._set_monitor_value("id_ref", f"{snapshot.id_ref:.3f} A")
        self._set_monitor_value("iq_ref", f"{snapshot.iq_ref:.3f} A")
        self._set_monitor_value("phase_u", f"{snapshot.phase_u:.3f} A")
        self._set_monitor_value("phase_v", f"{snapshot.phase_v:.3f} A")
        self._set_monitor_value("phase_w", f"{snapshot.phase_w:.3f} A")
        self._set_monitor_value("id_current", f"{snapshot.id_current:.3f} A")
        self._set_monitor_value("iq_current", f"{snapshot.iq_current:.3f} A")
        self._set_monitor_value("vd", f"{snapshot.vd:.3f} V")
        self._set_monitor_value("vq", f"{snapshot.vq:.3f} V")
        self._set_monitor_value("vf_frequency", f"{snapshot.vf_frequency:.3f} Hz")
        self._set_monitor_value("vf_voltage", f"{snapshot.vf_voltage:.3f} V")

    def _update_error_snapshot(self, snapshot) -> None:
        self._append_error_snapshot_to_trend_buffer(snapshot)
        self._set_monitor_value("enable_run", "OFF")
        self._set_monitor_value("run_mode", "FAULT")
        self._set_monitor_value("control_timing_mode", self._timing_mode_text(self._active_timing_mode))
        self._set_monitor_value("effective_loop_hz", f"{self._active_control_loop_hz:.0f} Hz")
        self._set_monitor_value("vdc", f"{snapshot.vdc:.3f} V")
        self._set_monitor_value("temperature", f"{snapshot.temperature:.3f} C")
        if snapshot.fault_code & 0x0100:
            self._set_monitor_value("calibration_status", self._calibration_status_text(CURRENT_CALIB_STATUS_TIMEOUT))
        elif snapshot.fault_code & 0x0200:
            self._set_monitor_value(
                "calibration_status",
                self._calibration_status_text(CURRENT_CALIB_STATUS_OFFSET_INVALID),
            )
        self._set_monitor_value("cmd_speed", f"{snapshot.cmd_speed:.3f}")
        self._set_monitor_value("act_speed", f"{snapshot.act_speed:.3f}")
        self._set_monitor_value("speed_error", f"{snapshot.speed_error:.3f}")
        self._set_monitor_value("act_position", f"{snapshot.act_position:.3f}")
        self._set_monitor_value("position_error", f"{snapshot.position_error:.3f}")
        self._set_monitor_value("iq_ref", f"{snapshot.iq_ref:.3f} A")
        self._set_fault_label(snapshot.fault_code)
        self._handle_fault_transition(
            snapshot.fault_code,
            "Error Packet",
            self._fault_detail_text(
                snapshot.fault_code,
                phase_u=snapshot.phase_u,
                phase_v=snapshot.phase_v,
                phase_w=snapshot.phase_w,
            ),
        )
        self._refresh_foc_control_panel(snapshot)
        self._set_monitor_value("phase_u", f"{snapshot.phase_u:.3f} A")
        self._set_monitor_value("phase_v", f"{snapshot.phase_v:.3f} A")
        self._set_monitor_value("phase_w", f"{snapshot.phase_w:.3f} A")
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
