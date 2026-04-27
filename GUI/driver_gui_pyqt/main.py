"""Basic desktop GUI for ASD04 USB communication."""

from __future__ import annotations

import base64
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, field
import html
import math
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
    POSITION_TRACKING_MODE_MULTI_TURN,
    POSITION_TRACKING_MODE_SINGLE_TURN,
    RUN_MODE_ALIGNMENT_ONLY,
    RUN_MODE_AUTOTUNE,
    RUN_MODE_FOC,
    RUN_MODE_OPEN_LOOP_VF,
    RUN_MODE_UERROR_CHARACTERIZATION,
    SPEED_CONTROL_MODE,
    Command,
    DRIVER_PARAMETER_NAMES,
    MOTOR_PARAMETER_NAMES,
    ParsedFrame,
    TraceChunk,
    TRACE_TOTAL_SAMPLES,
    UERROR_LUT_POINT_COUNT_MAX,
    UerrorSweepChunk,
    UpdateCode,
    FrameStreamParser,
    build_command_frame,
    build_parameter_write_chunks,
    build_uerror_lut_payload,
    decode_fault_flags,
    format_fault_text,
    parse_autotune_payload,
    parse_current_tuning_payload,
    parse_error_payload,
    parse_monitor_payload,
    parse_parameter_chunk,
    parse_trace_payload,
    parse_uerror_payload,
)
from transport import PortDescriptor, SerialWorker, list_serial_ports


KNOWN_DEVICE_VID = 1100
KNOWN_DEVICE_PID = 22336
TREND_BUFFER_CAPACITY = 4000
TREND_REFRESH_INTERVAL_MS = 20
TRACE_CAPTURE_REFRESH_INTERVAL_MS = 100
SPEED_TEST_TIMER_INTERVAL_MS = 20
SPEED_TEST_TRAPEZOID_UPDATE_INTERVAL_MS = 20
SPEED_TEST_PROFILE_STEP = "step_response"
SPEED_TEST_PROFILE_REVERSE = "reversing"
SPEED_TEST_PROFILE_LOW_SPEED = "low_speed"
SPEED_TEST_PROFILE_TRAPEZOID = "trapezoidal"
SPEED_TEST_STEP_RAMP_BYPASS_SENTINEL_MS = -1.0
POSITION_TEST_TIMER_INTERVAL_MS = 20
POSITION_TEST_PROFILE_SHORT = "short_distance"
POSITION_TEST_PROFILE_LONG = "long_distance"
POSITION_TEST_PROFILE_BACKLASH = "back_and_forth"
POSITION_TEST_SETTLE_SPEED_TOLERANCE_RPM = 3.0
POSITION_TARGET_COUNT_GUI_LIMIT = 1.0e15
DEFAULT_CTUNING_CURRENT_KP = 5.0
DEFAULT_CTUNING_CURRENT_KI = 10.0

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
    "cmd_position_deg": {"label": "Setting Position", "unit": "deg", "color": "#bcbd22"},
    "act_position_deg": {"label": "Mechanical Angle", "unit": "deg", "color": "#17becf"},
    "position_error_deg": {"label": "Tracking Error", "unit": "deg", "color": "#7f7f7f"},
    "validation_error_deg": {"label": "Validation Error", "unit": "deg", "color": "#ff9896"},
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
    "cmd_position_deg": None,
    "act_position_deg": None,
    "position_error_deg": None,
    "validation_error_deg": None,
}

DEFAULT_DRIVER_PARAMETER_VALUES: dict[int, float] = {
    1: float(SPEED_CONTROL_MODE),
    2: 0.05,
    3: 0.0,
    4: 50.0,
    5: 0.02,
    6: 5.0,
    7: 0.50,
    10: 250.0,
    11: 250.0,
    12: DEFAULT_MOTOR_RATED_SPEED_RPM,
    16: float(POSITION_TRACKING_MODE_SINGLE_TURN),
}

DEFAULT_MOTOR_PARAMETER_VALUES: dict[int, float] = {
    0: DEFAULT_MOTOR_RATED_CURRENT_RMS,
    1: DEFAULT_MOTOR_PEAK_CURRENT_RMS,
    8: float(1 << 20),
    9: DEFAULT_MOTOR_MAXIMUM_POWER,
    10: DEFAULT_MOTOR_MAXIMUM_VOLTAGE,
    11: DEFAULT_MOTOR_POLE_PAIRS,
    13: DEFAULT_MOTOR_RATED_SPEED_RPM,
    19: 1.0,
}

PARAMETER_NAME_LABELS = {
    "MOTOR_RESISTANCE": "MOTOR_RESISTANCE [mOhm]",
    "MOTOR_INDUCTANCE": "MOTOR_INDUCTANCE [uH]",
    "MOTOR_BACK_EMF_CONSTANT": "MOTOR_BACK_EMF_CONSTANT [mV/(rad/s)]",
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
DRIVER_PARAM_POSITION_FF_GAIN = 3
DRIVER_PARAM_POSITION_FF_FILTER = 4
DRIVER_PARAM_SPEED_P_GAIN = 5
DRIVER_PARAM_SPEED_I_GAIN = 6
DRIVER_PARAM_POSITION_I_GAIN = 7
DRIVER_PARAM_ACCEL_TIME_MS = 10
DRIVER_PARAM_DECEL_TIME_MS = 11
DRIVER_PARAM_MAXIMUM_SPEED = 12
DRIVER_PARAM_POSITION_TRACKING_MODE = 16
MOTOR_PARAM_MAXIMUM_SPEED = MOTOR_PARAMETER_NAMES.index("MOTOR_MAXIMUM_SPEED")
MOTOR_PARAM_CURRENT_P_GAIN = MOTOR_PARAMETER_NAMES.index("MOTOR_CURRENT_P_GAIN")
MOTOR_PARAM_CURRENT_I_GAIN = MOTOR_PARAMETER_NAMES.index("MOTOR_CURRENT_I_GAIN")
MOTOR_PARAM_ENCODER_RESOLUTION = MOTOR_PARAMETER_NAMES.index("MOTOR_ENCODER_RESOLUTION")

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


@dataclass(slots=True)
class ChartReportCapture:
    title: str
    image: QtGui.QImage
    series_endpoints: dict[str, QtCore.QPointF]
    series_labels: dict[str, str]
    series_points: dict[str, list[QtCore.QPointF]] = field(default_factory=dict)
    dpi: int = 300


@dataclass(slots=True)
class SpeedTestSession:
    profile_key: str
    sequence: list[dict[str, object]]
    preparing_write: bool = True
    step_index: int = 0
    step_started_at: float = 0.0
    steady_since_at: float | None = None
    last_target_send_at: float = 0.0
    last_target_rpm: float | None = None
    completion_text: str = "Completed"


@dataclass(slots=True)
class PositionTestSession:
    profile_key: str
    sequence: list[dict[str, object]]
    preparing_write: bool = True
    step_index: int = 0
    step_started_at: float = 0.0
    steady_since_at: float | None = None
    completion_text: str = "Completed"
    validation_error_deg: float = 0.0
    endpoint_baselines: dict[str, float] = field(default_factory=dict)
    long_start_counts: float | None = None
    long_last_logged_turn: int = 0


@dataclass(slots=True)
class UerrorRawSample:
    target_current_a: float
    measured_id_a: float
    phase_u_a: float
    phase_v_a: float
    phase_w_a: float
    phase_voltage_u_v: float
    phase_voltage_v_v: float
    phase_voltage_w_v: float
    bus_voltage_v: float
    temperature_c: float


def _chart_scale_from_font_size(font_pt: int) -> float:
    return max(1.0, float(font_pt) / 12.0)


def _series_role_key(series_key: str | None, label: str, index: int) -> str:
    token = (series_key or label).lower()
    if "ref" in token or "cmd" in token or "target" in token:
        return "reference"
    if "iq" in token or "act_speed" in token or "feedback" in token:
        return "feedback"
    if "id" in token or "error" in token or "secondary" in token:
        return "secondary"
    return ("reference", "feedback", "secondary")[index % 3]


def _series_pen_spec(
    series_key: str | None,
    label: str,
    index: int,
    *,
    report_mode: bool,
    fallback_color: str,
) -> tuple[QtGui.QPen, str | None]:
    if not report_mode:
        return QtGui.QPen(QtGui.QColor(fallback_color), 1.8), None

    role = _series_role_key(series_key, label, index)
    if role == "reference":
        pen = QtGui.QPen(QtGui.QColor("#000000"), 2.5)
        pen.setStyle(QtCore.Qt.PenStyle.SolidLine)
        return pen, "square"
    if role == "feedback":
        pen = QtGui.QPen(QtGui.QColor("#000000"), 2.5)
        pen.setStyle(QtCore.Qt.PenStyle.DashLine)
        return pen, "circle"

    pen = QtGui.QPen(QtGui.QColor("#000000"), 2.5)
    pen.setStyle(QtCore.Qt.PenStyle.DotLine)
    return pen, "diamond"


def _draw_series_marker(
    painter: QtGui.QPainter,
    center: QtCore.QPointF,
    marker_shape: str | None,
    size_px: float,
    color: QtGui.QColor,
) -> None:
    if marker_shape is None:
        return

    painter.save()
    painter.setPen(QtGui.QPen(color, 1.0))
    painter.setBrush(QtGui.QBrush(color))
    half = size_px * 0.5
    if marker_shape == "square":
        painter.drawRect(
            QtCore.QRectF(center.x() - half, center.y() - half, size_px, size_px)
        )
    elif marker_shape == "diamond":
        painter.drawPolygon(
            QtGui.QPolygonF(
                [
                    QtCore.QPointF(center.x(), center.y() - half),
                    QtCore.QPointF(center.x() + half, center.y()),
                    QtCore.QPointF(center.x(), center.y() + half),
                    QtCore.QPointF(center.x() - half, center.y()),
                ]
            )
        )
    else:
        painter.drawEllipse(center, half, half)
    painter.restore()


def _report_marker_indices(count: int, step: int = 25) -> list[int]:
    if count <= 0:
        return []
    indices = list(range(step - 1, count, step))
    if not indices or indices[-1] != count - 1:
        indices.append(count - 1)
    return sorted(set(index for index in indices if 0 <= index < count))


def _report_marker_ratios(
    plot_width_px: float,
    *,
    min_spacing_px: float = 72.0,
    edge_padding_ratio: float = 0.04,
) -> list[float]:
    usable_width = max(1.0, float(plot_width_px))
    marker_count = max(4, int(usable_width / max(32.0, min_spacing_px)))
    start_ratio = max(0.0, min(0.2, edge_padding_ratio))
    end_ratio = 1.0 - start_ratio
    if marker_count <= 1 or end_ratio <= start_ratio:
        return [0.5]
    step = (end_ratio - start_ratio) / max(1, marker_count - 1)
    return [start_ratio + (step * index) for index in range(marker_count)]


def _nearest_sorted_index(values: list[float], target: float) -> int:
    if not values:
        return 0
    insert_at = bisect_left(values, target)
    if insert_at <= 0:
        return 0
    if insert_at >= len(values):
        return len(values) - 1
    prev_value = values[insert_at - 1]
    next_value = values[insert_at]
    if abs(target - prev_value) <= abs(next_value - target):
        return insert_at - 1
    return insert_at


def _common_unit(units: list[str]) -> str:
    normalized = [unit.strip() for unit in units if unit and unit.strip()]
    unique = sorted(set(normalized))
    return unique[0] if len(unique) == 1 else ""


def _format_axis_value(value: float, *, unit: str = "", decimals: int = 2) -> str:
    number = f"{value:.{decimals}f}"
    return f"{number} {unit}" if unit else number


def _format_elapsed_time_label(seconds: float) -> str:
    if seconds < 1.0:
        return f"{seconds * 1000.0:.0f} ms"
    if seconds < 10.0:
        return f"{seconds:.1f} s"
    return f"{seconds:.0f} s"


def _draw_report_legend_sample(
    painter: QtGui.QPainter,
    x: float,
    y: float,
    width: float,
    label: str,
    series_key: str | None,
    index: int,
    text_color: QtGui.QColor,
) -> float:
    pen, marker_shape = _series_pen_spec(
        series_key,
        label,
        index,
        report_mode=True,
        fallback_color="#000000",
    )
    sample_y = y + 5.0
    painter.save()
    painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
    painter.setPen(pen)
    painter.drawLine(
        QtCore.QPointF(x, sample_y),
        QtCore.QPointF(x + width, sample_y),
    )
    _draw_series_marker(
        painter,
        QtCore.QPointF(x + width * 0.55, sample_y),
        marker_shape,
        7.0,
        QtGui.QColor("#000000"),
    )
    painter.setPen(text_color)
    painter.restore()
    return x + width


def _build_drag_preview_pixmap(
    text: str,
    *,
    glow_color: str = "#00BFFF",
) -> QtGui.QPixmap:
    font = QtGui.QFont("Times New Roman", 12)
    font.setBold(True)
    metrics = QtGui.QFontMetrics(font)
    width = max(96, metrics.horizontalAdvance(text) + 36)
    height = max(32, metrics.height() + 14)
    pixmap = QtGui.QPixmap(width, height)
    pixmap.fill(QtCore.Qt.GlobalColor.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
    glow = QtGui.QColor(glow_color)
    glow.setAlpha(110)
    painter.setPen(QtCore.Qt.PenStyle.NoPen)
    painter.setBrush(glow)
    painter.drawRoundedRect(QtCore.QRectF(2, 2, width - 4, height - 4), 10.0, 10.0)
    painter.setPen(QtGui.QPen(QtGui.QColor("#00BFFF"), 2.0))
    painter.setBrush(QtGui.QColor("#ffffff"))
    painter.drawRoundedRect(QtCore.QRectF(4, 4, width - 8, height - 8), 9.0, 9.0)
    painter.setFont(font)
    painter.setPen(QtGui.QColor("#000000"))
    painter.drawText(
        QtCore.QRectF(12, 0, width - 24, height),
        QtCore.Qt.AlignmentFlag.AlignCenter,
        text,
    )
    painter.end()
    return pixmap


def _qimage_to_png_bytes(image: QtGui.QImage) -> bytes:
    byte_array = QtCore.QByteArray()
    buffer = QtCore.QBuffer(byte_array)
    buffer.open(QtCore.QIODevice.OpenModeFlag.WriteOnly)
    image.save(buffer, "PNG")
    buffer.close()
    return bytes(byte_array)


class ReportTextItem(QtWidgets.QGraphicsTextItem):
    def __init__(
        self,
        text: str,
        *,
        bold: bool = False,
        tag_style: bool = False,
        owner_annotation: "ArrowAnnotationItem | None" = None,
        parent: QtWidgets.QGraphicsItem | None = None,
    ) -> None:
        super().__init__(text, parent)
        self._owner_annotation = owner_annotation
        self._placeholder = text or "Label"
        self._tag_style = bool(tag_style)
        self._border_width = 1.8 if self._tag_style else 1.2
        self._programmatic_move = False
        self.document().setDocumentMargin(6.0 if self._tag_style else 4.0)
        font = QtGui.QFont("Times New Roman", 30)
        font.setBold(bold or self._tag_style)
        self.setFont(font)
        self.setDefaultTextColor(QtGui.QColor("#000000"))
        self.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.NoTextInteraction)
        self.setFlags(
            QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsMovable
            | QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable
            | QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsFocusable
            | QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )
        self.setZValue(25.0)

    def annotation_owner(self):
        return self._owner_annotation or self

    def outline_width(self) -> float:
        return self._border_width

    def set_outline_width(self, value: float) -> None:
        self._border_width = max(0.5, float(value))
        self.update()

    def set_font_point_size(self, value: int) -> None:
        font = QtGui.QFont(self.font())
        font.setPointSize(int(max(8, min(32, value))))
        self.setFont(font)
        self.update()

    def set_canvas_pos(
        self,
        pos: QtCore.QPointF,
        *,
        by_owner: bool = False,
    ) -> None:
        self._programmatic_move = by_owner
        self.setPos(pos)
        self._programmatic_move = False

    def start_editing(self, *, select_all: bool = True) -> None:
        self.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextEditorInteraction
        )
        self.setFocus(QtCore.Qt.FocusReason.MouseFocusReason)
        cursor = self.textCursor()
        if select_all:
            cursor.select(QtGui.QTextCursor.SelectionType.Document)
            self.setTextCursor(cursor)

    def boundingRect(self) -> QtCore.QRectF:  # noqa: N802
        return super().boundingRect().adjusted(-2.0, -2.0, 2.0, 2.0)

    def paint(
        self,
        painter: QtGui.QPainter,
        option: QtWidgets.QStyleOptionGraphicsItem,
        widget: QtWidgets.QWidget | None = None,
    ) -> None:
        _ = widget
        rect = super().boundingRect().adjusted(-1.5, -1.5, 1.5, 1.5)
        painter.save()
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        fill = QtGui.QColor("#f5f5f5" if self._tag_style else "#ffffff")
        fill.setAlpha(238 if self._tag_style else 224)
        border = QtGui.QPen(QtGui.QColor("#000000"), self._border_width)
        if option.state & QtWidgets.QStyle.StateFlag.State_Selected:
            border.setWidthF(self._border_width + 0.8)
            border.setStyle(QtCore.Qt.PenStyle.DashLine)
        painter.setPen(border)
        painter.setBrush(QtGui.QBrush(fill))
        painter.drawRoundedRect(rect, 8.0, 8.0)
        painter.restore()
        super().paint(painter, option, widget)

    def focusOutEvent(self, event: QtGui.QFocusEvent) -> None:  # noqa: N802
        text = self.toPlainText().strip()
        if not text:
            self.setPlainText(self._placeholder)
        else:
            self.setPlainText(text)
            self._placeholder = text
        self.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.NoTextInteraction)
        super().focusOutEvent(event)

    def mouseDoubleClickEvent(  # noqa: N802
        self, event: QtWidgets.QGraphicsSceneMouseEvent
    ) -> None:
        self.start_editing(select_all=False)
        event.accept()

    def itemChange(
        self,
        change: QtWidgets.QGraphicsItem.GraphicsItemChange,
        value,
    ):
        if (
            change
            == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged
            and self._owner_annotation is not None
            and not self._programmatic_move
        ):
            self._owner_annotation.notify_label_moved_by_user(self)
        if change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged:
            self.update()
        return super().itemChange(change, value)

    def svg_elements(self, origin: QtCore.QPointF) -> list[str]:
        scene_rect = self.sceneBoundingRect().translated(-origin)
        font = self.font()
        margin = self.document().documentMargin()
        text_pos = self.scenePos() - origin + QtCore.QPointF(
            margin,
            font.pointSizeF() + margin * 0.6,
        )
        return [
            (
                f"<rect x=\"{scene_rect.x():.2f}\" y=\"{scene_rect.y():.2f}\" "
                f"width=\"{scene_rect.width():.2f}\" height=\"{scene_rect.height():.2f}\" "
                f"rx=\"8\" ry=\"8\" fill=\"#ffffff\" fill-opacity=\"0.94\" "
                f"stroke=\"#000000\" stroke-width=\"{self._border_width:.2f}\" />"
            ),
            (
                f"<text x=\"{text_pos.x():.2f}\" y=\"{text_pos.y():.2f}\" "
                f"font-family=\"{html.escape(font.family())}\" "
                f"font-size=\"{font.pointSizeF():.2f}\" "
                f"font-weight=\"{'700' if font.bold() else '400'}\" "
                "fill=\"#000000\">"
                f"{html.escape(self.toPlainText())}</text>"
            ),
        ]


class ArrowHandleItem(QtWidgets.QGraphicsEllipseItem):
    def __init__(
        self,
        owner: "ArrowAnnotationItem",
        initial_pos: QtCore.QPointF,
        parent: QtWidgets.QGraphicsItem | None = None,
    ) -> None:
        super().__init__(-5.0, -5.0, 10.0, 10.0, parent)
        self._owner = owner
        self.setBrush(QtGui.QBrush(QtGui.QColor("#ffffff")))
        self.setPen(QtGui.QPen(QtGui.QColor("#000000"), 1.0))
        self.setPos(initial_pos)
        self.setFlags(
            QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsMovable
            | QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable
            | QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )
        self.setZValue(30.0)
        self.hide()

    def annotation_owner(self) -> "ArrowAnnotationItem":
        return self._owner

    def itemChange(
        self,
        change: QtWidgets.QGraphicsItem.GraphicsItemChange,
        value,
    ):
        if (
            change
            == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionChange
            and self._owner is not None
        ):
            return self._owner.constrain_handle_position(self, value)
        if (
            change
            == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged
            and self._owner is not None
        ):
            self._owner.handle_moved(self)
        return super().itemChange(change, value)


class ArrowAnnotationItem(QtWidgets.QGraphicsObject):
    def __init__(
        self,
        start_point: QtCore.QPointF,
        end_point: QtCore.QPointF,
        parent: QtWidgets.QGraphicsItem | None = None,
    ) -> None:
        super().__init__(parent)
        self._padding = 22.0
        self._line_thickness = 2.4
        self._label_detached = False
        self.setPos(start_point)
        self.start_handle = ArrowHandleItem(self, QtCore.QPointF(0.0, 0.0), self)
        self.end_handle = ArrowHandleItem(self, end_point - start_point, self)
        self.label_item = ReportTextItem(
            "Callout",
            owner_annotation=self,
            parent=self,
        )
        self.label_item.set_canvas_pos(self._suggested_label_pos(), by_owner=True)
        self.setFlags(
            QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsMovable
            | QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable
            | QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsFocusable
        )
        self.setZValue(18.0)

    def _set_handles_visible(self, visible: bool) -> None:
        self.start_handle.setVisible(visible)
        self.end_handle.setVisible(visible)

    def _suggested_label_pos(self) -> QtCore.QPointF:
        end_point = self.end_handle.pos()
        return end_point + QtCore.QPointF(12.0, -28.0)

    def constrain_handle_position(
        self,
        handle: ArrowHandleItem,
        value,
    ) -> QtCore.QPointF:
        _ = handle
        if isinstance(value, QtCore.QPointF):
            return QtCore.QPointF(value)
        return QtCore.QPointF()

    def notify_label_moved_by_user(self, item: ReportTextItem) -> None:
        if item is self.label_item:
            self._label_detached = True

    def line_thickness(self) -> float:
        return self._line_thickness

    def set_line_thickness(self, value: float) -> None:
        self.prepareGeometryChange()
        self._line_thickness = max(0.8, float(value))
        self.update()

    def text_item(self) -> ReportTextItem:
        return self.label_item

    def boundingRect(self) -> QtCore.QRectF:  # noqa: N802
        line_rect = QtCore.QRectF(
            self.start_handle.pos(), self.end_handle.pos()
        ).normalized()
        label_rect = self.label_item.mapRectToParent(self.label_item.boundingRect())
        return line_rect.united(label_rect).adjusted(
            -self._padding,
            -self._padding,
            self._padding,
            self._padding,
        )

    def shape(self) -> QtGui.QPainterPath:  # noqa: N802
        path = QtGui.QPainterPath()
        path.moveTo(self.start_handle.pos())
        path.lineTo(self.end_handle.pos())
        stroker = QtGui.QPainterPathStroker()
        stroker.setWidth(max(10.0, self._line_thickness + 8.0))
        result = stroker.createStroke(path)
        result.addRect(self.label_item.mapRectToParent(self.label_item.boundingRect()))
        return result

    def itemChange(
        self,
        change: QtWidgets.QGraphicsItem.GraphicsItemChange,
        value,
    ):
        if change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged:
            self._set_handles_visible(bool(value))
            self.update()
        return super().itemChange(change, value)

    def handle_moved(self, handle: ArrowHandleItem) -> None:
        _ = handle
        self.prepareGeometryChange()
        if not self._label_detached:
            self.label_item.set_canvas_pos(self._suggested_label_pos(), by_owner=True)
        self.update()

    def paint(
        self,
        painter: QtGui.QPainter,
        option: QtWidgets.QStyleOptionGraphicsItem,
        widget: QtWidgets.QWidget | None = None,
    ) -> None:
        _ = widget
        start_point = self.start_handle.pos()
        end_point = self.end_handle.pos()
        painter.save()
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        pen = QtGui.QPen(QtGui.QColor("#000000"), self._line_thickness)
        if option.state & QtWidgets.QStyle.StateFlag.State_Selected:
            pen.setStyle(QtCore.Qt.PenStyle.DashLine)
        painter.setPen(pen)
        painter.drawLine(start_point, end_point)

        angle = math.atan2(
            start_point.y() - end_point.y(),
            start_point.x() - end_point.x(),
        )
        arrow_size = 12.0 + self._line_thickness
        left = QtCore.QPointF(
            end_point.x() + math.cos(angle + math.pi / 6.0) * arrow_size,
            end_point.y() + math.sin(angle + math.pi / 6.0) * arrow_size,
        )
        right = QtCore.QPointF(
            end_point.x() + math.cos(angle - math.pi / 6.0) * arrow_size,
            end_point.y() + math.sin(angle - math.pi / 6.0) * arrow_size,
        )
        painter.setBrush(QtGui.QBrush(QtGui.QColor("#000000")))
        painter.drawPolygon(QtGui.QPolygonF([end_point, left, right]))
        painter.restore()

    def mouseDoubleClickEvent(  # noqa: N802
        self, event: QtWidgets.QGraphicsSceneMouseEvent
    ) -> None:
        self.label_item.start_editing(select_all=False)
        event.accept()

    def svg_geometry(self) -> tuple[QtCore.QPointF, QtCore.QPointF]:
        return (
            self.mapToScene(self.start_handle.pos()),
            self.mapToScene(self.end_handle.pos()),
        )


class CropOverlayItem(QtWidgets.QGraphicsObject):
    def __init__(
        self,
        image_rect: QtCore.QRectF,
        on_change,
        parent: QtWidgets.QGraphicsItem | None = None,
    ) -> None:
        super().__init__(parent)
        self._image_rect = QtCore.QRectF(image_rect)
        self._crop_rect = QtCore.QRectF(image_rect)
        self._on_change = on_change
        self._active = False
        self._handle_size = 12.0
        self._min_size = 120.0
        self._drag_mode: str | None = None
        self._press_pos = QtCore.QPointF()
        self._press_crop = QtCore.QRectF()
        self.setAcceptedMouseButtons(QtCore.Qt.MouseButton.NoButton)
        self.setAcceptHoverEvents(True)
        self.setZValue(12.0)

    def boundingRect(self) -> QtCore.QRectF:  # noqa: N802
        return QtCore.QRectF(self._image_rect)

    def crop_rect(self) -> QtCore.QRectF:
        return QtCore.QRectF(self._crop_rect)

    def set_active(self, active: bool) -> None:
        self._active = bool(active)
        self.setAcceptedMouseButtons(
            QtCore.Qt.MouseButton.LeftButton
            if self._active
            else QtCore.Qt.MouseButton.NoButton
        )
        self.update()

    def set_crop_rect(self, rect: QtCore.QRectF) -> None:
        crop = QtCore.QRectF(rect).normalized()
        crop = crop.intersected(self._image_rect)
        if crop.width() < self._min_size:
            crop.setWidth(min(self._min_size, self._image_rect.width()))
        if crop.height() < self._min_size:
            crop.setHeight(min(self._min_size, self._image_rect.height()))
        if crop.right() > self._image_rect.right():
            crop.moveRight(self._image_rect.right())
        if crop.bottom() > self._image_rect.bottom():
            crop.moveBottom(self._image_rect.bottom())
        if crop.left() < self._image_rect.left():
            crop.moveLeft(self._image_rect.left())
        if crop.top() < self._image_rect.top():
            crop.moveTop(self._image_rect.top())
        self.prepareGeometryChange()
        self._crop_rect = crop
        self.update()
        if callable(self._on_change):
            self._on_change()

    def _handle_rects(self) -> dict[str, QtCore.QRectF]:
        half = self._handle_size * 0.5
        rect = self._crop_rect
        points = {
            "top_left": rect.topLeft(),
            "top": QtCore.QPointF(rect.center().x(), rect.top()),
            "top_right": rect.topRight(),
            "right": QtCore.QPointF(rect.right(), rect.center().y()),
            "bottom_right": rect.bottomRight(),
            "bottom": QtCore.QPointF(rect.center().x(), rect.bottom()),
            "bottom_left": rect.bottomLeft(),
            "left": QtCore.QPointF(rect.left(), rect.center().y()),
        }
        return {
            key: QtCore.QRectF(
                point.x() - half,
                point.y() - half,
                self._handle_size,
                self._handle_size,
            )
            for key, point in points.items()
        }

    def _hit_test(self, pos: QtCore.QPointF) -> str | None:
        for key, rect in self._handle_rects().items():
            if rect.contains(pos):
                return key
        if self._crop_rect.contains(pos):
            return "move"
        return None

    def _cursor_for_mode(self, mode: str | None) -> QtCore.Qt.CursorShape:
        mapping = {
            "top_left": QtCore.Qt.CursorShape.SizeFDiagCursor,
            "bottom_right": QtCore.Qt.CursorShape.SizeFDiagCursor,
            "top_right": QtCore.Qt.CursorShape.SizeBDiagCursor,
            "bottom_left": QtCore.Qt.CursorShape.SizeBDiagCursor,
            "left": QtCore.Qt.CursorShape.SizeHorCursor,
            "right": QtCore.Qt.CursorShape.SizeHorCursor,
            "top": QtCore.Qt.CursorShape.SizeVerCursor,
            "bottom": QtCore.Qt.CursorShape.SizeVerCursor,
            "move": QtCore.Qt.CursorShape.SizeAllCursor,
        }
        return mapping.get(mode, QtCore.Qt.CursorShape.ArrowCursor)

    def _drag_rect(self, pos: QtCore.QPointF) -> QtCore.QRectF:
        delta = pos - self._press_pos
        source = QtCore.QRectF(self._press_crop)
        if self._drag_mode == "move":
            candidate = source.translated(delta)
            if candidate.left() < self._image_rect.left():
                candidate.translate(self._image_rect.left() - candidate.left(), 0.0)
            if candidate.right() > self._image_rect.right():
                candidate.translate(self._image_rect.right() - candidate.right(), 0.0)
            if candidate.top() < self._image_rect.top():
                candidate.translate(0.0, self._image_rect.top() - candidate.top())
            if candidate.bottom() > self._image_rect.bottom():
                candidate.translate(0.0, self._image_rect.bottom() - candidate.bottom())
            return candidate

        left = source.left()
        right = source.right()
        top = source.top()
        bottom = source.bottom()

        if self._drag_mode and "left" in self._drag_mode:
            left = min(
                max(self._image_rect.left(), source.left() + delta.x()),
                right - self._min_size,
            )
        if self._drag_mode and "right" in self._drag_mode:
            right = max(
                min(self._image_rect.right(), source.right() + delta.x()),
                left + self._min_size,
            )
        if self._drag_mode and "top" in self._drag_mode:
            top = min(
                max(self._image_rect.top(), source.top() + delta.y()),
                bottom - self._min_size,
            )
        if self._drag_mode and "bottom" in self._drag_mode:
            bottom = max(
                min(self._image_rect.bottom(), source.bottom() + delta.y()),
                top + self._min_size,
            )
        return QtCore.QRectF(
            QtCore.QPointF(left, top),
            QtCore.QPointF(right, bottom),
        ).normalized()

    def paint(
        self,
        painter: QtGui.QPainter,
        option: QtWidgets.QStyleOptionGraphicsItem,
        widget: QtWidgets.QWidget | None = None,
    ) -> None:
        _ = option
        _ = widget
        painter.save()
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        if self._active:
            outer = QtGui.QPainterPath()
            outer.addRect(self._image_rect)
            inner = QtGui.QPainterPath()
            inner.addRect(self._crop_rect)
            painter.fillPath(
                outer.subtracted(inner),
                QtGui.QColor(120, 120, 120, 110),
            )
            border_pen = QtGui.QPen(QtGui.QColor("#000000"), 1.8)
            border_pen.setStyle(QtCore.Qt.PenStyle.DashLine)
            painter.setPen(border_pen)
            painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            painter.drawRect(self._crop_rect)
            for rect in self._handle_rects().values():
                painter.setPen(QtGui.QPen(QtGui.QColor("#000000"), 1.2))
                painter.setBrush(QtGui.QBrush(QtGui.QColor("#ffffff")))
                painter.drawRect(rect)
        else:
            border_pen = QtGui.QPen(QtGui.QColor("#555555"), 1.0)
            border_pen.setStyle(QtCore.Qt.PenStyle.DotLine)
            painter.setPen(border_pen)
            painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            painter.drawRect(self._crop_rect)
        painter.restore()

    def hoverMoveEvent(self, event: QtWidgets.QGraphicsSceneHoverEvent) -> None:  # noqa: N802
        mode = self._hit_test(event.pos()) if self._active else None
        self.setCursor(self._cursor_for_mode(mode))
        super().hoverMoveEvent(event)

    def mousePressEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:  # noqa: N802
        if not self._active:
            event.ignore()
            return
        self._drag_mode = self._hit_test(event.pos())
        if self._drag_mode is None:
            event.ignore()
            return
        self._press_pos = event.pos()
        self._press_crop = QtCore.QRectF(self._crop_rect)
        event.accept()

    def mouseMoveEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:  # noqa: N802
        if not self._active or self._drag_mode is None:
            event.ignore()
            return
        self.set_crop_rect(self._drag_rect(event.pos()))
        event.accept()

    def mouseReleaseEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:  # noqa: N802
        self._drag_mode = None
        self.setCursor(QtCore.Qt.CursorShape.ArrowCursor)
        super().mouseReleaseEvent(event)


class SignalTagBadge(QtWidgets.QLabel):
    MIME_TYPE = "application/x-report-signal-tag"

    def __init__(self, series_key: str, label: str, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(label, parent)
        self.series_key = series_key
        self.label_text = label
        self._drag_start = QtCore.QPoint()
        self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.setMinimumHeight(30)
        self.setStyleSheet(
            "QLabel {"
            "background: #ffffff; color: #111111; border: 1px solid #6c757d; border-radius: 8px;"
            "padding: 4px 8px; font-weight: 700;}"
        )

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        self._drag_start = event.position().toPoint()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if not (event.buttons() & QtCore.Qt.MouseButton.LeftButton):
            return
        if (
            event.position().toPoint() - self._drag_start
        ).manhattanLength() < QtWidgets.QApplication.startDragDistance():
            return

        drag = QtGui.QDrag(self)
        mime = QtCore.QMimeData()
        mime.setData(
            self.MIME_TYPE,
            f"{self.series_key}\n{self.label_text}".encode("utf-8"),
        )
        drag.setMimeData(mime)
        drag.setPixmap(_build_drag_preview_pixmap(self.label_text))
        drag.exec(QtCore.Qt.DropAction.CopyAction)


class ReportCanvasView(QtWidgets.QGraphicsView):
    def __init__(self, dialog: "ReportEditorDialog") -> None:
        super().__init__(dialog._scene, dialog)
        self._dialog = dialog
        self._tool_mode = "select"
        self._draft_arrow: ArrowAnnotationItem | None = None
        self.setAcceptDrops(True)
        self.setMouseTracking(True)
        self.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        self.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
        self.setDragMode(QtWidgets.QGraphicsView.DragMode.RubberBandDrag)
        self.setViewportUpdateMode(
            QtWidgets.QGraphicsView.ViewportUpdateMode.FullViewportUpdate
        )
        self.setBackgroundBrush(QtGui.QColor("#1f1f1f"))

    def set_tool_mode(self, mode: str) -> None:
        self._tool_mode = mode
        if mode == "select":
            self.setDragMode(QtWidgets.QGraphicsView.DragMode.RubberBandDrag)
            self.viewport().setCursor(QtCore.Qt.CursorShape.ArrowCursor)
        else:
            self.setDragMode(QtWidgets.QGraphicsView.DragMode.NoDrag)
            self.viewport().setCursor(QtCore.Qt.CursorShape.CrossCursor)

    def dragEnterEvent(self, event: QtGui.QDragEnterEvent) -> None:  # noqa: N802
        if event.mimeData().hasFormat(SignalTagBadge.MIME_TYPE):
            event.acceptProposedAction()
            return
        super().dragEnterEvent(event)

    def dragMoveEvent(self, event: QtGui.QDragMoveEvent) -> None:  # noqa: N802
        if event.mimeData().hasFormat(SignalTagBadge.MIME_TYPE):
            event.acceptProposedAction()
            return
        super().dragMoveEvent(event)

    def dropEvent(self, event: QtGui.QDropEvent) -> None:  # noqa: N802
        if not event.mimeData().hasFormat(SignalTagBadge.MIME_TYPE):
            super().dropEvent(event)
            return
        payload = bytes(event.mimeData().data(SignalTagBadge.MIME_TYPE)).decode(
            "utf-8", errors="ignore"
        )
        scene_pos = self.mapToScene(event.position().toPoint())
        self._dialog.add_signal_tag_from_payload(payload, scene_pos)
        event.acceptProposedAction()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if (
            self._tool_mode == "arrow"
            and event.button() == QtCore.Qt.MouseButton.LeftButton
        ):
            scene_pos = self._dialog.clamp_scene_point(self.mapToScene(event.pos()))
            self._draft_arrow = self._dialog.create_arrow(scene_pos, scene_pos)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if self._draft_arrow is not None:
            scene_pos = self._dialog.clamp_scene_point(self.mapToScene(event.pos()))
            local_end = self._draft_arrow.mapFromScene(scene_pos)
            self._draft_arrow.end_handle.setPos(local_end)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if self._draft_arrow is not None:
            arrow = self._draft_arrow
            self._draft_arrow = None
            scene_pos = self._dialog.clamp_scene_point(self.mapToScene(event.pos()))
            snapped_pos = self._dialog.snap_scene_point(scene_pos)
            arrow.end_handle.setPos(arrow.mapFromScene(snapped_pos))
            length = QtCore.QLineF(
                arrow.start_handle.pos(), arrow.end_handle.pos()
            ).length()
            if length < 12.0:
                self.scene().removeItem(arrow)
            else:
                self._dialog.finalize_arrow(arrow)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if event.button() != QtCore.Qt.MouseButton.LeftButton:
            super().mouseDoubleClickEvent(event)
            return
        if self._tool_mode == "crop":
            super().mouseDoubleClickEvent(event)
            return

        item = self.itemAt(event.pos())
        if isinstance(item, (ReportTextItem, ArrowAnnotationItem, ArrowHandleItem)):
            super().mouseDoubleClickEvent(event)
            return

        scene_pos = self._dialog.clamp_scene_point(self.mapToScene(event.pos()))
        self._dialog.add_text_note_at(scene_pos)
        event.accept()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._dialog.layout_overlays()


class FloatingAnnotationMenu(QtWidgets.QFrame):
    def __init__(self, view: ReportCanvasView) -> None:
        super().__init__(view.viewport())
        self._view = view
        self._target: ArrowAnnotationItem | ReportTextItem | None = None
        self._updating = False
        self.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.setStyleSheet(
            "QFrame { background: rgba(255,255,255,252); border: 2px solid #7d8b99; border-radius: 10px; }"
            "QLabel { color: #111111; font-weight: 700; }"
            "QPushButton { background: #ffffff; color: #111111; border: 1px solid #b8c2cc;"
            " border-radius: 8px; padding: 5px 10px; font-weight: 700; }"
            "QPushButton:hover { background: #eef4fb; border-color: #7aa7d9; }"
            "QSpinBox, QDoubleSpinBox { background: #ffffff; color: #111111;"
            " border: 1px solid #97a6b4; border-radius: 8px; padding: 4px 8px; font-weight: 600; }"
        )
        shadow = QtWidgets.QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(22.0)
        shadow.setOffset(0.0, 5.0)
        shadow.setColor(QtGui.QColor(0, 0, 0, 60))
        self.setGraphicsEffect(shadow)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)

        self.delete_button = QtWidgets.QPushButton("Delete")
        self.delete_button.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        self.font_spin = QtWidgets.QSpinBox()
        self.font_spin.setRange(8, 32)
        self.font_spin.setSuffix(" pt")
        self.font_spin.setMinimumWidth(96)
        self.thickness_spin = QtWidgets.QDoubleSpinBox()
        self.thickness_spin.setRange(0.5, 8.0)
        self.thickness_spin.setDecimals(1)
        self.thickness_spin.setSingleStep(0.5)
        self.thickness_spin.setSuffix(" px")
        self.thickness_spin.setMinimumWidth(104)
        if self.font_spin.lineEdit() is not None:
            self.font_spin.lineEdit().setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        if self.thickness_spin.lineEdit() is not None:
            self.thickness_spin.lineEdit().setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(self.delete_button)
        layout.addWidget(QtWidgets.QLabel("Font"))
        layout.addWidget(self.font_spin)
        layout.addWidget(QtWidgets.QLabel("Line"))
        layout.addWidget(self.thickness_spin)

        self.delete_button.clicked.connect(self._delete_target)
        self.font_spin.valueChanged.connect(self._apply_font_size)
        self.thickness_spin.valueChanged.connect(self._apply_thickness)
        self.hide()

    def _target_text_item(self) -> ReportTextItem | None:
        if isinstance(self._target, ArrowAnnotationItem):
            return self._target.text_item()
        if isinstance(self._target, ReportTextItem):
            return self._target
        return None

    def set_target(
        self, target: ArrowAnnotationItem | ReportTextItem | None
    ) -> None:
        self._target = target
        if target is None:
            self.hide()
            return

        self._updating = True
        text_item = self._target_text_item()
        if text_item is not None:
            self.font_spin.setValue(text_item.font().pointSize())
            self.thickness_spin.setValue(text_item.outline_width())
        if isinstance(target, ArrowAnnotationItem):
            self.thickness_spin.setValue(target.line_thickness())
        self._updating = False
        self._reposition()
        self.show()
        self.raise_()

    def _reposition(self) -> None:
        if self._target is None:
            return
        rect = self._target.sceneBoundingRect()
        top_right = self._view.mapFromScene(rect.topRight())
        x = max(8, min(top_right.x() + 12, self._view.viewport().width() - self.width() - 8))
        y = max(8, min(top_right.y() - 8, self._view.viewport().height() - self.height() - 8))
        self.move(int(x), int(y))

    def _delete_target(self) -> None:
        if self._target is None:
            return
        scene = self._view.scene()
        if scene is None:
            return
        if isinstance(self._target, ArrowAnnotationItem):
            scene.removeItem(self._target)
        else:
            owner = self._target.annotation_owner()
            if isinstance(owner, ArrowAnnotationItem):
                scene.removeItem(owner)
            else:
                scene.removeItem(self._target)
        self.set_target(None)

    def _apply_font_size(self, value: int) -> None:
        if self._updating:
            return
        text_item = self._target_text_item()
        if text_item is not None:
            text_item.set_font_point_size(value)
            self._reposition()

    def _apply_thickness(self, value: float) -> None:
        if self._updating or self._target is None:
            return
        if isinstance(self._target, ArrowAnnotationItem):
            self._target.set_line_thickness(value)
        else:
            self._target.set_outline_width(value)


class ReportEditorDialog(QtWidgets.QDialog):
    def __init__(
        self,
        capture: ChartReportCapture,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(f"Report Editor - {capture.title}")
        self.resize(1440, 920)
        self.setModal(True)

        self._capture = capture
        self._base_image = capture.image
        self._base_endpoints = dict(capture.series_endpoints)
        self._series_labels = dict(capture.series_labels)
        self._base_rect = QtCore.QRectF(
            0.0,
            0.0,
            float(self._base_image.width()),
            float(self._base_image.height()),
        )

        self._scene = QtWidgets.QGraphicsScene(self)
        self._view = ReportCanvasView(self)
        self._base_pixmap_item = QtWidgets.QGraphicsPixmapItem(
            QtGui.QPixmap.fromImage(self._base_image)
        )
        self._base_pixmap_item.setZValue(0.0)
        self._scene.addItem(self._base_pixmap_item)

        self._crop_overlay = CropOverlayItem(self._base_rect, self._handle_crop_changed)
        self._scene.addItem(self._crop_overlay)

        self._caption_backdrop = QtWidgets.QGraphicsRectItem()
        self._caption_backdrop.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 255, 242)))
        self._caption_backdrop.setPen(QtGui.QPen(QtGui.QColor("#c4ccd4"), 1.0))
        self._caption_backdrop.setZValue(9.0)
        self._scene.addItem(self._caption_backdrop)

        self._caption_item = QtWidgets.QGraphicsTextItem()
        caption_font = QtGui.QFont("Segoe UI", 34)
        caption_font.setBold(True)
        self._caption_item.setFont(caption_font)
        self._caption_item.setDefaultTextColor(QtGui.QColor("#101418"))
        self._caption_item.setZValue(10.0)
        self._scene.addItem(self._caption_item)

        self._floating_menu = FloatingAnnotationMenu(self._view)
        self._series_points = {
            key: [QtCore.QPointF(point) for point in points]
            for key, points in capture.series_points.items()
        }

        self.select_tool_button = QtWidgets.QToolButton()
        self.select_tool_button.setText("Select")
        self.select_tool_button.setCheckable(True)
        self.crop_tool_button = QtWidgets.QToolButton()
        self.crop_tool_button.setText("Crop")
        self.crop_tool_button.setCheckable(True)
        self.arrow_tool_button = QtWidgets.QToolButton()
        self.arrow_tool_button.setText("Arrow")
        self.arrow_tool_button.setCheckable(True)
        self.add_label_button = QtWidgets.QToolButton()
        self.add_label_button.setText("Add Label")
        self.fit_view_button = QtWidgets.QToolButton()
        self.fit_view_button.setText("Fit View")
        self.copy_clipboard_button = QtWidgets.QPushButton("Save to Clipboard")
        self.save_png_button = QtWidgets.QPushButton("Save PNG")
        self.save_svg_button = QtWidgets.QPushButton("Save SVG")

        overlay_style = (
            "QFrame { background: rgba(255, 255, 255, 255);"
            " border: 2px solid #97a6b4; border-radius: 12px; }"
            "QLabel { color: #101418; font-weight: 700; }"
            "QToolButton, QPushButton {"
            " background: #ffffff; color: #111111; border: 1px solid #b4c0cb;"
            " border-radius: 8px; padding: 6px 12px; font-weight: 700; }"
            "QToolButton:hover, QPushButton:hover { background: #eef5ff; border-color: #4f8dd6; }"
            "QToolButton:checked { background: #0d6efd; color: #ffffff; border-color: #0b5ed7; }"
            "QLineEdit { background: #ffffff; color: #111111; border: 1px solid #97a6b4;"
            " border-radius: 8px; padding: 6px 10px; selection-background-color: #b6d4fe; font-weight: 600; }"
        )
        self._tool_palette = QtWidgets.QFrame(self._view.viewport())
        self._tool_palette.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self._tool_palette.setStyleSheet(overlay_style)
        toolbar_layout = QtWidgets.QHBoxLayout(self._tool_palette)
        toolbar_layout.setContentsMargins(12, 10, 12, 10)
        toolbar_layout.setSpacing(8)
        toolbar_layout.addWidget(self.select_tool_button)
        toolbar_layout.addWidget(self.crop_tool_button)
        toolbar_layout.addWidget(self.arrow_tool_button)
        toolbar_layout.addWidget(self.add_label_button)
        toolbar_layout.addWidget(self.fit_view_button)

        self._export_palette = QtWidgets.QFrame(self._view.viewport())
        self._export_palette.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self._export_palette.setStyleSheet(overlay_style)
        export_layout = QtWidgets.QHBoxLayout(self._export_palette)
        export_layout.setContentsMargins(12, 10, 12, 10)
        export_layout.setSpacing(8)
        export_layout.addWidget(self.copy_clipboard_button)
        export_layout.addWidget(self.save_png_button)
        export_layout.addWidget(self.save_svg_button)

        self._tag_palette = QtWidgets.QFrame(self._view.viewport())
        self._tag_palette.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self._tag_palette.setStyleSheet(overlay_style)
        palette_layout = QtWidgets.QVBoxLayout(self._tag_palette)
        palette_layout.setContentsMargins(12, 12, 12, 12)
        palette_layout.setSpacing(8)
        palette_title = QtWidgets.QLabel("Signal Tags")
        palette_title.setStyleSheet("font-weight: 800; color: #111111;")
        palette_help = QtWidgets.QLabel("Drag a tag straight onto the chart line.")
        palette_help.setWordWrap(True)
        palette_help.setStyleSheet("color: #374151; font-size: 11px; font-weight: 600;")
        palette_layout.addWidget(palette_title)
        palette_layout.addWidget(palette_help)
        for series_key, label in self._ordered_signal_tags():
            palette_layout.addWidget(SignalTagBadge(series_key, label, self._tag_palette))
        palette_layout.addStretch(1)

        self._caption_overlay = QtWidgets.QFrame(self._view.viewport())
        self._caption_overlay.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self._caption_overlay.setStyleSheet(overlay_style)
        caption_layout = QtWidgets.QHBoxLayout(self._caption_overlay)
        caption_layout.setContentsMargins(12, 10, 12, 10)
        caption_layout.setSpacing(8)
        self.figure_prefix_edit = QtWidgets.QLineEdit("Figure 1.")
        self.caption_edit = QtWidgets.QLineEdit(capture.title)
        caption_layout.addWidget(QtWidgets.QLabel("Figure"))
        caption_layout.addWidget(self.figure_prefix_edit, 0)
        caption_layout.addWidget(QtWidgets.QLabel("Caption"))
        caption_layout.addWidget(self.caption_edit, 1)

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(self._view, 1)

        for widget in (
            self._tool_palette,
            self._export_palette,
            self._tag_palette,
            self._caption_overlay,
        ):
            shadow = QtWidgets.QGraphicsDropShadowEffect(widget)
            shadow.setBlurRadius(24.0)
            shadow.setOffset(0.0, 6.0)
            shadow.setColor(QtGui.QColor(0, 0, 0, 58))
            widget.setGraphicsEffect(shadow)

        self.select_tool_button.clicked.connect(lambda: self._set_tool_mode("select"))
        self.crop_tool_button.clicked.connect(lambda: self._set_tool_mode("crop"))
        self.arrow_tool_button.clicked.connect(lambda: self._set_tool_mode("arrow"))
        self.add_label_button.clicked.connect(self._add_annotation_label)
        self.fit_view_button.clicked.connect(self._fit_to_artboard)
        self.copy_clipboard_button.clicked.connect(self._copy_to_clipboard)
        self.save_png_button.clicked.connect(self._save_png)
        self.save_svg_button.clicked.connect(self._save_svg)
        self.figure_prefix_edit.textChanged.connect(self._update_caption_item)
        self.caption_edit.textChanged.connect(self._update_caption_item)
        self._scene.selectionChanged.connect(self._update_context_menu)

        self._set_tool_mode("select")
        self._tool_palette.show()
        self._export_palette.show()
        self._tag_palette.show()
        self._caption_overlay.show()
        QtCore.QTimer.singleShot(0, self._finish_initial_layout)

    def _ordered_signal_tags(self) -> list[tuple[str, str]]:
        entries = list(self._series_labels.items())

        def rank(item: tuple[str, str]) -> tuple[int, str]:
            key, label = item
            token = f"{key} {label}".lower()
            if "iq" in token:
                return (0, label)
            if "id" in token:
                return (1, label)
            if "speed" in token:
                return (2, label)
            return (3, label)

        entries.sort(key=rank)
        return entries

    def _finish_initial_layout(self) -> None:
        self.layout_overlays()
        self._update_caption_item()
        self._fit_to_artboard()

    def image_scene_rect(self) -> QtCore.QRectF:
        return QtCore.QRectF(self._base_rect)

    def clamp_scene_point(self, point: QtCore.QPointF) -> QtCore.QPointF:
        return QtCore.QPointF(
            min(max(point.x(), self._base_rect.left()), self._base_rect.right()),
            min(max(point.y(), self._base_rect.top()), self._base_rect.bottom()),
        )

    def _set_tool_mode(self, mode: str) -> None:
        self.select_tool_button.setChecked(mode == "select")
        self.crop_tool_button.setChecked(mode == "crop")
        self.arrow_tool_button.setChecked(mode == "arrow")
        self._view.set_tool_mode(mode)
        self._crop_overlay.set_active(mode == "crop")
        if mode == "crop":
            self._scene.clearSelection()
        self._update_context_menu()

    def _handle_crop_changed(self) -> None:
        self._update_caption_item()
        self._scene.setSceneRect(self._scene.itemsBoundingRect().adjusted(-30, -30, 30, 30))

    def _update_caption_item(self) -> None:
        figure_prefix = self.figure_prefix_edit.text().strip() or "Figure 1."
        caption_text = self.caption_edit.text().strip() or self._capture.title
        self._caption_item.setPlainText(f"{figure_prefix} {caption_text}")
        crop_rect = self._crop_overlay.crop_rect()
        caption_rect = self._caption_item.boundingRect()
        caption_pos = QtCore.QPointF(
            max(0.0, crop_rect.center().x() - caption_rect.width() * 0.5),
            crop_rect.bottom() + 22.0,
        )
        self._caption_item.setPos(caption_pos)
        scene_caption_rect = self._caption_item.mapToScene(caption_rect).boundingRect()
        backdrop_rect = scene_caption_rect.adjusted(-16.0, -8.0, 16.0, 10.0)
        self._caption_backdrop.setRect(backdrop_rect)
        self._scene.setSceneRect(self._scene.itemsBoundingRect().adjusted(-30, -30, 30, 30))

    def _fit_view_reserved_rect(self) -> QtCore.QRectF:
        viewport = self._view.viewport()
        if viewport is None:
            return QtCore.QRectF()
        rect = QtCore.QRectF(viewport.rect())
        margin = 16.0
        top_reserved = 0.0
        left_reserved = 0.0
        bottom_reserved = 0.0

        if self._tool_palette.isVisible():
            top_reserved = max(
                top_reserved,
                float(self._tool_palette.y() + self._tool_palette.height() + margin),
            )
        if self._export_palette.isVisible():
            top_reserved = max(
                top_reserved,
                float(self._export_palette.y() + self._export_palette.height() + margin),
            )
        if self._tag_palette.isVisible():
            left_reserved = max(
                left_reserved,
                float(self._tag_palette.x() + self._tag_palette.width() + margin),
            )
            top_reserved = max(
                top_reserved,
                float(self._tag_palette.y() + min(28, self._tag_palette.height() // 4)),
            )
        if self._caption_overlay.isVisible():
            bottom_reserved = max(
                bottom_reserved,
                float(viewport.height() - self._caption_overlay.y() + margin),
            )

        return rect.adjusted(left_reserved, top_reserved, -margin, -bottom_reserved)

    def _fit_to_artboard(self) -> None:
        target_rect = self._crop_overlay.crop_rect().united(
            self._caption_backdrop.mapToScene(self._caption_backdrop.rect()).boundingRect()
        )
        target_rect = target_rect.adjusted(-30.0, -30.0, 30.0, 30.0)
        available_rect = self._fit_view_reserved_rect()
        if available_rect.width() <= 40.0 or available_rect.height() <= 40.0:
            self._view.fitInView(
                target_rect,
                QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            )
            return

        scale_x = available_rect.width() / max(1.0, target_rect.width())
        scale_y = available_rect.height() / max(1.0, target_rect.height())
        scale = max(0.01, min(scale_x, scale_y))

        transform = QtGui.QTransform()
        transform.scale(scale, scale)
        self._view.setTransform(transform)

        viewport_rect = QtCore.QRectF(self._view.viewport().rect())
        delta_view = available_rect.center() - viewport_rect.center()
        delta_scene = QtCore.QPointF(delta_view.x() / scale, delta_view.y() / scale)
        self._view.centerOn(target_rect.center() - delta_scene)

    def _default_drop_point(self) -> QtCore.QPointF:
        crop_rect = self._crop_overlay.crop_rect()
        return crop_rect.center() + QtCore.QPointF(-30.0, -20.0)

    def _add_annotation_label(self) -> None:
        self.add_text_note_at(self._default_drop_point())

    def add_text_note_at(self, scene_pos: QtCore.QPointF) -> None:
        item = ReportTextItem("Note")
        item.set_canvas_pos(self.clamp_scene_point(scene_pos))
        self._scene.addItem(item)
        self._scene.clearSelection()
        item.setSelected(True)
        item.start_editing()
        self._set_tool_mode("select")
        self._update_context_menu()

    def _nearest_series_point(
        self,
        scene_pos: QtCore.QPointF,
        *,
        series_key: str | None = None,
        radius: float = 96.0,
    ) -> QtCore.QPointF | None:
        candidate_keys = [series_key] if series_key is not None else list(self._series_points.keys())
        best_point: QtCore.QPointF | None = None
        best_distance = radius
        for key in candidate_keys:
            if key is None:
                continue
            for point in self._series_points.get(key, []):
                if not self._crop_overlay.crop_rect().contains(point):
                    continue
                distance = QtCore.QLineF(scene_pos, point).length()
                if distance < best_distance:
                    best_distance = distance
                    best_point = QtCore.QPointF(point)
        return best_point

    def snap_scene_point(self, scene_pos: QtCore.QPointF) -> QtCore.QPointF:
        nearest = self._nearest_series_point(scene_pos, radius=72.0)
        if nearest is not None:
            return nearest
        return self.clamp_scene_point(scene_pos)

    def _snap_signal_tag_position(
        self,
        series_key: str,
        scene_pos: QtCore.QPointF,
    ) -> QtCore.QPointF:
        nearest = self._nearest_series_point(
            scene_pos,
            series_key=series_key,
            radius=120.0,
        )
        if nearest is not None:
            return nearest + QtCore.QPointF(14.0, -18.0)
        endpoint = self._base_endpoints.get(series_key)
        if endpoint is None:
            return scene_pos
        if self._crop_overlay.crop_rect().contains(endpoint):
            if QtCore.QLineF(scene_pos, endpoint).length() <= 90.0:
                return endpoint + QtCore.QPointF(14.0, -18.0)
        return scene_pos

    def add_signal_tag_from_payload(
        self, payload: str, scene_pos: QtCore.QPointF
    ) -> None:
        parts = payload.splitlines()
        if not parts:
            return
        series_key = parts[0].strip()
        label = parts[1].strip() if len(parts) > 1 else self._series_labels.get(series_key, series_key)
        item = ReportTextItem(label, bold=True, tag_style=True)
        item.set_canvas_pos(self._snap_signal_tag_position(series_key, scene_pos))
        self._scene.addItem(item)
        self._scene.clearSelection()
        item.setSelected(True)
        self._set_tool_mode("select")
        self._update_context_menu()

    def create_arrow(
        self,
        start_point: QtCore.QPointF,
        end_point: QtCore.QPointF,
    ) -> ArrowAnnotationItem:
        arrow = ArrowAnnotationItem(start_point, end_point)
        self._scene.addItem(arrow)
        return arrow

    def finalize_arrow(self, arrow: ArrowAnnotationItem) -> None:
        self._scene.clearSelection()
        arrow.setSelected(True)
        arrow.text_item().start_editing()
        self._set_tool_mode("select")
        self._update_context_menu()

    def _selected_annotation_target(self):
        if self._crop_overlay.crop_rect().isNull():
            return None
        if self.crop_tool_button.isChecked():
            return None
        for item in self._scene.selectedItems():
            if isinstance(item, ArrowAnnotationItem):
                return item
            if isinstance(item, ReportTextItem):
                owner = item.annotation_owner()
                return owner if isinstance(owner, ArrowAnnotationItem) else item
            if isinstance(item, ArrowHandleItem):
                return item.annotation_owner()
        return None

    def _update_context_menu(self) -> None:
        target = self._selected_annotation_target()
        self._floating_menu.set_target(target)

    def layout_overlays(self) -> None:
        viewport = self._view.viewport()
        if viewport is None:
            return
        margin = 14

        for widget in (
            self._tool_palette,
            self._export_palette,
            self._tag_palette,
            self._caption_overlay,
        ):
            widget.adjustSize()

        self._tool_palette.move(margin, margin)
        self._export_palette.move(
            max(margin, viewport.width() - self._export_palette.width() - margin),
            margin,
        )
        self._tag_palette.move(
            margin,
            max(
                self._tool_palette.y() + self._tool_palette.height() + margin,
                margin + 92,
            ),
        )
        max_overlay_width = max(260, viewport.width() - (2 * margin))
        caption_width = min(max_overlay_width, max(420, viewport.width() - (2 * margin) - 120))
        self._caption_overlay.resize(
            caption_width,
            self._caption_overlay.sizeHint().height(),
        )
        self._caption_overlay.move(
            max(margin, (viewport.width() - self._caption_overlay.width()) // 2),
            max(
                margin,
                viewport.height() - self._caption_overlay.height() - margin,
            ),
        )

        self._floating_menu.raise_()
        self._tool_palette.raise_()
        self._export_palette.raise_()
        self._tag_palette.raise_()
        self._caption_overlay.raise_()

    def _export_scene_rect(self) -> QtCore.QRectF:
        crop_rect = self._crop_overlay.crop_rect()
        caption_rect = self._caption_backdrop.mapToScene(
            self._caption_backdrop.rect()
        ).boundingRect()
        return crop_rect.united(caption_rect).adjusted(-12.0, -12.0, 12.0, 12.0)

    def _prepare_export(self) -> tuple[list[QtWidgets.QGraphicsItem], bool]:
        selected = list(self._scene.selectedItems())
        for item in selected:
            item.setSelected(False)
        overlay_visible = self._crop_overlay.isVisible()
        self._crop_overlay.setVisible(False)
        self._floating_menu.hide()
        return selected, overlay_visible

    def _restore_export(
        self,
        selected: list[QtWidgets.QGraphicsItem],
        overlay_visible: bool,
    ) -> None:
        self._crop_overlay.setVisible(overlay_visible)
        for item in selected:
            item.setSelected(True)
        self._update_context_menu()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(event)
        self.layout_overlays()

    def _render_scene_to_image(self) -> QtGui.QImage:
        export_rect = self._export_scene_rect()
        selected, overlay_visible = self._prepare_export()
        try:
            image = QtGui.QImage(
                max(1, int(math.ceil(export_rect.width()))),
                max(1, int(math.ceil(export_rect.height()))),
                QtGui.QImage.Format.Format_ARGB32,
            )
            dpi = max(96, int(self._capture.dpi))
            image.setDotsPerMeterX(int(dpi / 25.4 * 1000.0))
            image.setDotsPerMeterY(int(dpi / 25.4 * 1000.0))
            image.fill(QtGui.QColor("#ffffff"))
            painter = QtGui.QPainter(image)
            painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
            self._scene.render(
                painter,
                QtCore.QRectF(image.rect()),
                export_rect,
            )
            painter.end()
            return image
        finally:
            self._restore_export(selected, overlay_visible)

    def _copy_to_clipboard(self) -> None:
        QtWidgets.QApplication.clipboard().setImage(self._render_scene_to_image())

    def _save_png(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Report PNG",
            "chart_report.png",
            "PNG Files (*.png)",
        )
        if not path:
            return
        self._render_scene_to_image().save(path, "PNG")

    def _save_svg(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Report SVG",
            "chart_report.svg",
            "SVG Files (*.svg)",
        )
        if not path:
            return

        export_rect = self._export_scene_rect()
        crop_rect = self._crop_overlay.crop_rect().toRect().intersected(self._base_image.rect())
        crop_image = self._base_image.copy(crop_rect)
        png_b64 = base64.b64encode(_qimage_to_png_bytes(crop_image)).decode("ascii")
        origin = export_rect.topLeft()
        width = max(1.0, export_rect.width())
        height = max(1.0, export_rect.height())

        svg_lines = [
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>",
            (
                f"<svg xmlns=\"http://www.w3.org/2000/svg\" "
                f"xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
                f"width=\"{width:.0f}\" height=\"{height:.0f}\" "
                f"viewBox=\"0 0 {width:.2f} {height:.2f}\">"
            ),
            "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>",
            (
                f"<image x=\"{crop_rect.x() - origin.x():.2f}\" y=\"{crop_rect.y() - origin.y():.2f}\" "
                f"width=\"{crop_image.width()}\" height=\"{crop_image.height()}\" "
                f"xlink:href=\"data:image/png;base64,{png_b64}\" />"
            ),
        ]

        selected, overlay_visible = self._prepare_export()
        try:
            for item in self._scene.items():
                if isinstance(item, ArrowAnnotationItem):
                    if not item.sceneBoundingRect().intersects(export_rect):
                        continue
                    start_point, end_point = item.svg_geometry()
                    start_point -= origin
                    end_point -= origin
                    angle = math.atan2(
                        start_point.y() - end_point.y(),
                        start_point.x() - end_point.x(),
                    )
                    arrow_size = 12.0 + item.line_thickness()
                    left = QtCore.QPointF(
                        end_point.x() + math.cos(angle + math.pi / 6.0) * arrow_size,
                        end_point.y() + math.sin(angle + math.pi / 6.0) * arrow_size,
                    )
                    right = QtCore.QPointF(
                        end_point.x() + math.cos(angle - math.pi / 6.0) * arrow_size,
                        end_point.y() + math.sin(angle - math.pi / 6.0) * arrow_size,
                    )
                    svg_lines.append(
                        (
                            f"<line x1=\"{start_point.x():.2f}\" y1=\"{start_point.y():.2f}\" "
                            f"x2=\"{end_point.x():.2f}\" y2=\"{end_point.y():.2f}\" "
                            f"stroke=\"#000000\" stroke-width=\"{item.line_thickness():.2f}\" />"
                        )
                    )
                    svg_lines.append(
                        (
                            "<polygon "
                            f"points=\"{end_point.x():.2f},{end_point.y():.2f} "
                            f"{left.x():.2f},{left.y():.2f} "
                            f"{right.x():.2f},{right.y():.2f}\" "
                            "fill=\"#000000\" />"
                        )
                    )

            for item in self._scene.items():
                if not isinstance(item, ReportTextItem):
                    continue
                if not item.sceneBoundingRect().intersects(export_rect):
                    continue
                svg_lines.extend(item.svg_elements(origin))

            caption_text = html.escape(self._caption_item.toPlainText())
            caption_font = self._caption_item.font()
            caption_pos = self._caption_item.scenePos() - origin
            svg_lines.append(
                (
                    f"<text x=\"{caption_pos.x():.2f}\" y=\"{caption_pos.y() + caption_font.pointSizeF():.2f}\" "
                    f"font-family=\"{html.escape(caption_font.family())}\" "
                    f"font-size=\"{caption_font.pointSizeF():.2f}\" font-weight=\"700\" fill=\"#000000\">"
                    f"{caption_text}</text>"
                )
            )
        finally:
            self._restore_export(selected, overlay_visible)

        svg_lines.append("</svg>")
        with open(path, "w", encoding="utf-8") as svg_file:
            svg_file.write("\n".join(svg_lines))


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
        self._report_mode = False
        self._report_font_point_size = 14
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

    def set_report_mode(self, enabled: bool) -> None:
        self._report_mode = bool(enabled)
        self.update()

    def set_report_font_point_size(self, value: int) -> None:
        self._report_font_point_size = int(max(12, min(24, value)))
        self.update()

    def prepare_for_report(self, dpi: int = 300) -> ChartReportCapture:
        if not self._render_times:
            self.refresh_from_buffer()
        previous_report_mode = self._report_mode
        self._report_mode = True
        scale = float(dpi) / 96.0
        image = QtGui.QImage(
            int(self.width() * scale),
            int(self.height() * scale),
            QtGui.QImage.Format.Format_ARGB32,
        )
        image.setDotsPerMeterX(int(dpi / 25.4 * 1000.0))
        image.setDotsPerMeterY(int(dpi / 25.4 * 1000.0))
        image.fill(QtGui.QColor("#ffffff"))

        previous_hover = self._hover_index
        self._hover_index = None
        painter = QtGui.QPainter(image)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.scale(scale, scale)
        try:
            self.render(painter)
        finally:
            self._report_mode = previous_report_mode
        painter.end()
        self._hover_index = previous_hover

        return ChartReportCapture(
            title=self._title,
            image=image,
            series_endpoints=self._series_endpoint_map(scale_factor=scale),
            series_labels={key: TREND_SERIES_META[key]["label"] for key in self._series_keys},
            series_points=self._series_point_map(scale_factor=scale),
            dpi=dpi,
        )

    def build_report_capture(self, dpi: int = 300) -> ChartReportCapture:
        return self.prepare_for_report(dpi=dpi)

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

    def _render_scale(self) -> float:
        return _chart_scale_from_font_size(self._report_font_point_size) if self._report_mode else 1.0

    def _title_font(self) -> QtGui.QFont:
        font = QtGui.QFont(self.font())
        if self._report_mode:
            font.setFamily("Times New Roman")
            font.setPointSize(max(14, self._report_font_point_size + 2))
            font.setBold(True)
        return font

    def _legend_font(self) -> QtGui.QFont:
        font = QtGui.QFont(self.font())
        if self._report_mode:
            font.setFamily("Times New Roman")
            font.setPointSize(self._report_font_point_size)
        return font

    def _tick_font(self) -> QtGui.QFont:
        font = QtGui.QFont(self.font())
        if self._report_mode:
            font.setFamily("Times New Roman")
            font.setPointSize(max(12, self._report_font_point_size - 1))
        return font

    def _chart_colors(self) -> dict[str, QtGui.QColor]:
        if self._report_mode:
            return {
                "background": QtGui.QColor("#FFFFFF"),
                "text": QtGui.QColor("#000000"),
                "border": QtGui.QColor("#000000"),
                "grid": QtGui.QColor("#B8B8B8"),
                "cursor": QtGui.QColor("#666666"),
            }
        return {
            "background": self.palette().base().color(),
            "text": self.palette().text().color(),
            "border": self.palette().mid().color(),
            "grid": self.palette().midlight().color(),
            "cursor": QtGui.QColor("#888888"),
        }

    def _plot_geometry(
        self,
        rect: QtCore.QRectF,
    ) -> tuple[QtCore.QRectF, int, int, int]:
        scale = self._render_scale()
        title_height = QtGui.QFontMetrics(self._title_font()).height() + int(4 * scale)
        legend_height = max(QtGui.QFontMetrics(self._legend_font()).height(), int(10 * scale)) + int(8 * scale)
        bottom_space = QtGui.QFontMetrics(self._tick_font()).height() + int(30 * scale)
        right_axis_space = int(42 * scale)
        legend_y = rect.top() + title_height + int(6 * scale)
        plot_rect = QtCore.QRectF(
            rect.left(),
            legend_y + legend_height + int(8 * scale),
            rect.width() - right_axis_space,
            rect.height() - title_height - legend_height - bottom_space - int(16 * scale),
        )
        return plot_rect, title_height, legend_height, bottom_space

    def _legend_step_width(self, key: str) -> int:
        metrics = QtGui.QFontMetrics(self._legend_font())
        scale = self._render_scale()
        return max(int(metrics.horizontalAdvance(TREND_SERIES_META[key]["label"]) + (42 * scale)), 96)

    def _y_axis_unit(self) -> str:
        return _common_unit([TREND_SERIES_META[key]["unit"] for key in self._series_keys])

    def _series_endpoint_map(self, scale_factor: float = 1.0) -> dict[str, QtCore.QPointF]:
        if not self._render_times:
            return {}
        rect = QtCore.QRectF(self.rect().adjusted(8, 8, -8, -8))
        plot_rect, _, _, _ = self._plot_geometry(rect)
        if plot_rect.height() <= 0:
            return {}
        y_min, y_max = self._frozen_range
        y_span = max(y_max - y_min, 0.1)
        endpoint_map: dict[str, QtCore.QPointF] = {}
        start_time = self._render_anchor_s - self._time_window_s
        for key in self._series_keys:
            series = self._render_values.get(key, [])
            if not series:
                continue
            index = len(series) - 1
            time_ratio = (
                (self._render_times[index] - start_time) / self._time_window_s
                if self._time_window_s > 0
                else 1.0
            )
            time_ratio = min(max(time_ratio, 0.0), 1.0)
            x = plot_rect.left() + plot_rect.width() * time_ratio
            y = plot_rect.bottom() - (((series[index] - y_min) / y_span) * plot_rect.height())
            endpoint_map[key] = QtCore.QPointF(x * scale_factor, y * scale_factor)
        return endpoint_map

    def _series_point_map(
        self,
        *,
        scale_factor: float = 1.0,
        point_limit: int = 220,
    ) -> dict[str, list[QtCore.QPointF]]:
        if not self._render_times:
            return {}
        rect = QtCore.QRectF(self.rect().adjusted(8, 8, -8, -8))
        plot_rect, _, _, _ = self._plot_geometry(rect)
        if plot_rect.height() <= 0:
            return {}
        y_min, y_max = self._frozen_range
        y_span = max(y_max - y_min, 0.1)
        start_time = self._render_anchor_s - self._time_window_s
        point_map: dict[str, list[QtCore.QPointF]] = {}
        for key in self._series_keys:
            series = self._render_values.get(key, [])
            if not series:
                continue
            step = max(1, len(series) // point_limit)
            indices = list(range(0, len(series), step))
            if indices[-1] != len(series) - 1:
                indices.append(len(series) - 1)
            points: list[QtCore.QPointF] = []
            for index in indices:
                time_ratio = (
                    (self._render_times[index] - start_time) / self._time_window_s
                    if self._time_window_s > 0
                    else 1.0
                )
                time_ratio = min(max(time_ratio, 0.0), 1.0)
                x = plot_rect.left() + (plot_rect.width() * time_ratio)
                y = plot_rect.bottom() - (((series[index] - y_min) / y_span) * plot_rect.height())
                points.append(QtCore.QPointF(x * scale_factor, y * scale_factor))
            point_map[key] = points
        return point_map

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        super().paintEvent(event)

        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        rect = self.rect().adjusted(8, 8, -8, -8)
        colors = self._chart_colors()
        painter.fillRect(rect, colors["background"])

        painter.setFont(self._title_font())
        plot_rect, title_height, legend_height, _ = self._plot_geometry(QtCore.QRectF(rect))
        title_rect = QtCore.QRect(rect.left(), rect.top(), rect.width(), title_height)
        painter.setPen(colors["text"])
        painter.drawText(
            title_rect,
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
            self._title,
        )

        legend_y = rect.top() + title_height + int(6 * self._render_scale())
        legend_x = rect.left()
        painter.setFont(self._legend_font())
        for series_index, key in enumerate(self._series_keys):
            legend_scale = self._render_scale()
            if self._report_mode:
                _draw_report_legend_sample(
                    painter,
                    legend_x,
                    legend_y + (legend_height * 0.5) - (5.0 * legend_scale),
                    28.0 * legend_scale,
                    TREND_SERIES_META[key]["label"],
                    key,
                    series_index,
                    colors["text"],
                )
            else:
                legend_color = QtGui.QColor(TREND_SERIES_META[key]["color"])
                painter.fillRect(
                    QtCore.QRect(
                        legend_x,
                        legend_y,
                        int(10 * legend_scale),
                        int(10 * legend_scale),
                    ),
                    legend_color,
                )
            painter.drawText(
                QtCore.QRect(
                    legend_x + int((42 if self._report_mode else 14) * legend_scale),
                    legend_y - int(3 * legend_scale),
                    self._legend_step_width(key),
                    legend_height,
                ),
                QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
                TREND_SERIES_META[key]["label"],
            )
            legend_x += self._legend_step_width(key) + int((22 if self._report_mode else 18) * legend_scale)

        if plot_rect.height() <= 0:
            return
        self._plot_rect = plot_rect

        border_pen = QtGui.QPen(colors["border"], 1.2 if self._report_mode else 1.0)
        painter.setPen(border_pen)
        painter.drawRect(plot_rect)

        if not self._render_times:
            painter.setFont(self._legend_font())
            painter.drawText(
                plot_rect.toRect(),
                QtCore.Qt.AlignmentFlag.AlignCenter,
                "Waiting for monitor data...",
            )
            return

        y_min, y_max = self._frozen_range
        y_span = max(y_max - y_min, 0.1)

        grid_pen = QtGui.QPen(colors["grid"], 1.0)
        grid_pen.setStyle(QtCore.Qt.PenStyle.DotLine if self._report_mode else QtCore.Qt.PenStyle.DashLine)
        painter.setPen(grid_pen)
        for grid_index in range(1, 5):
            y = plot_rect.top() + (plot_rect.height() * grid_index / 5.0)
            painter.drawLine(
                QtCore.QPointF(plot_rect.left(), y),
                QtCore.QPointF(plot_rect.right(), y),
            )

        painter.setFont(self._tick_font())
        value_text_rect = QtCore.QRectF(
            plot_rect.right() - (78 * self._render_scale()),
            plot_rect.top() + (6 * self._render_scale()),
            74 * self._render_scale(),
            18 * self._render_scale(),
        )
        painter.setPen(colors["text"])
        y_unit = self._y_axis_unit()
        painter.drawText(
            value_text_rect,
            QtCore.Qt.AlignmentFlag.AlignRight,
            _format_axis_value(y_max, unit=y_unit, decimals=2),
        )
        value_text_rect.moveTop(plot_rect.bottom() - (28 * self._render_scale()))
        painter.drawText(
            value_text_rect,
            QtCore.Qt.AlignmentFlag.AlignRight,
            _format_axis_value(y_min, unit=y_unit, decimals=2),
        )

        time_labels = [0.0, self._time_window_s * 0.5, self._time_window_s]
        for time_index, elapsed_s in enumerate(time_labels):
            ratio = elapsed_s / self._time_window_s if self._time_window_s > 0 else 0.0
            x = plot_rect.left() + (plot_rect.width() * ratio)
            painter.drawLine(
                QtCore.QPointF(x, plot_rect.bottom()),
                QtCore.QPointF(x, plot_rect.bottom() + 4),
            )
            label = _format_elapsed_time_label(elapsed_s)
            label_rect = QtCore.QRectF(
                x - (34 * self._render_scale()),
                plot_rect.bottom() + (10 * self._render_scale()),
                68 * self._render_scale(),
                18 * self._render_scale(),
            )
            if time_index == 0:
                label_rect.moveLeft(plot_rect.left())
            elif time_index == len(time_labels) - 1:
                label_rect.moveRight(plot_rect.right() - (4 * self._render_scale()))
            painter.drawText(
                label_rect,
                QtCore.Qt.AlignmentFlag.AlignCenter,
                label,
            )

        for series_index, key in enumerate(self._series_keys):
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

            series_pen, marker_shape = _series_pen_spec(
                key,
                TREND_SERIES_META[key]["label"],
                series_index,
                report_mode=self._report_mode,
                fallback_color=TREND_SERIES_META[key]["color"],
            )
            painter.setPen(series_pen)
            painter.drawPath(path)
            if self._report_mode:
                marker_color = QtGui.QColor("#000000")
                window_start = self._render_anchor_s - self._time_window_s
                for time_ratio in _report_marker_ratios(plot_rect.width(), min_spacing_px=64.0):
                    target_time = window_start + (time_ratio * self._time_window_s)
                    marker_index = _nearest_sorted_index(self._render_times, target_time)
                    x = plot_rect.left() + (plot_rect.width() * time_ratio)
                    y = plot_rect.bottom() - (((series[marker_index] - y_min) / y_span) * plot_rect.height())
                    _draw_series_marker(
                        painter,
                        QtCore.QPointF(x, y),
                        marker_shape,
                        7.0 * self._render_scale(),
                        marker_color,
                    )

        if self._hover_index is not None and 0 <= self._hover_index < len(self._render_times):
            hover_time = self._render_times[self._hover_index]
            ratio = (
                (hover_time - (self._render_anchor_s - self._time_window_s)) / self._time_window_s
            )
            x = plot_rect.left() + (plot_rect.width() * ratio)
            cursor_pen = QtGui.QPen(colors["cursor"])
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
        on_export_requested=None,
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
        self.report_mode_checkbox = QtWidgets.QCheckBox("Report Mode")
        self.report_font_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.report_font_slider.setRange(12, 24)
        self.report_font_slider.setValue(14)
        self.report_font_slider.setFixedWidth(110)
        self.report_font_label = QtWidgets.QLabel("14 pt")
        self.export_report_button = QtWidgets.QPushButton("Capture & Edit")

        toolbar.addWidget(self.pause_button)
        toolbar.addWidget(self.auto_scale_checkbox)
        toolbar.addWidget(QtWidgets.QLabel("Window"))
        toolbar.addWidget(self.time_window_combo)
        toolbar.addWidget(self.report_mode_checkbox)
        toolbar.addWidget(QtWidgets.QLabel("Font"))
        toolbar.addWidget(self.report_font_slider)
        toolbar.addWidget(self.report_font_label)
        toolbar.addWidget(self.export_report_button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.plot_view = ScadaTrendView(title, trend_buffer, series_keys, self)
        layout.addWidget(self.plot_view)

        self.pause_button.toggled.connect(self._handle_pause_toggled)
        self.auto_scale_checkbox.toggled.connect(self.plot_view.set_auto_scale)
        self.time_window_combo.currentTextChanged.connect(self._handle_window_changed)
        self.report_mode_checkbox.toggled.connect(self.plot_view.set_report_mode)
        self.report_font_slider.valueChanged.connect(self._handle_report_font_changed)
        if on_export_requested is not None:
            self.export_report_button.clicked.connect(lambda: on_export_requested(self.plot_view))

    def _handle_pause_toggled(self, checked: bool) -> None:
        self.pause_button.setText("Resume" if checked else "Pause")
        self.plot_view.set_paused(checked)

    def _handle_window_changed(self, text: str) -> None:
        value = float(text.split()[0])
        self.plot_view.set_time_window_s(value)

    def _handle_report_font_changed(self, value: int) -> None:
        self.report_font_label.setText(f"{int(value)} pt")
        self.plot_view.set_report_font_point_size(int(value))

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
        self._report_mode = False
        self._report_font_point_size = 14
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

    def set_report_mode(self, enabled: bool) -> None:
        self._report_mode = bool(enabled)
        self.update()

    def set_report_font_point_size(self, value: int) -> None:
        self._report_font_point_size = int(max(12, min(24, value)))
        self.update()

    def prepare_for_report(self, dpi: int = 300) -> ChartReportCapture:
        previous_report_mode = self._report_mode
        self._report_mode = True
        scale = float(dpi) / 96.0
        image = QtGui.QImage(
            int(self.width() * scale),
            int(self.height() * scale),
            QtGui.QImage.Format.Format_ARGB32,
        )
        image.setDotsPerMeterX(int(dpi / 25.4 * 1000.0))
        image.setDotsPerMeterY(int(dpi / 25.4 * 1000.0))
        image.fill(QtGui.QColor("#ffffff"))

        previous_hover = self._hover_index
        self._hover_index = None
        painter = QtGui.QPainter(image)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.scale(scale, scale)
        try:
            self.render(painter)
        finally:
            self._report_mode = previous_report_mode
        painter.end()
        self._hover_index = previous_hover

        return ChartReportCapture(
            title=self._title,
            image=image,
            series_endpoints=self._series_endpoint_map(scale_factor=scale),
            series_labels={
                series_def["label"]: series_def["label"] for series_def in self._series_defs
            },
            series_points=self._series_point_map(scale_factor=scale),
            dpi=dpi,
        )

    def build_report_capture(self, dpi: int = 300) -> ChartReportCapture:
        return self.prepare_for_report(dpi=dpi)

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

    def _render_scale(self) -> float:
        return _chart_scale_from_font_size(self._report_font_point_size) if self._report_mode else 1.0

    def _title_font(self) -> QtGui.QFont:
        font = QtGui.QFont(self.font())
        if self._report_mode:
            font.setFamily("Times New Roman")
            font.setPointSize(max(14, self._report_font_point_size + 2))
            font.setBold(True)
        return font

    def _legend_font(self) -> QtGui.QFont:
        font = QtGui.QFont(self.font())
        if self._report_mode:
            font.setFamily("Times New Roman")
            font.setPointSize(self._report_font_point_size)
        return font

    def _tick_font(self) -> QtGui.QFont:
        font = QtGui.QFont(self.font())
        if self._report_mode:
            font.setFamily("Times New Roman")
            font.setPointSize(max(12, self._report_font_point_size - 1))
        return font

    def _chart_colors(self) -> dict[str, QtGui.QColor]:
        if self._report_mode:
            return {
                "background": QtGui.QColor("#FFFFFF"),
                "text": QtGui.QColor("#000000"),
                "border": QtGui.QColor("#000000"),
                "grid": QtGui.QColor("#B8B8B8"),
                "cursor": QtGui.QColor("#666666"),
            }
        return {
            "background": self.palette().base().color(),
            "text": self.palette().text().color(),
            "border": self.palette().mid().color(),
            "grid": self.palette().midlight().color(),
            "cursor": QtGui.QColor("#888888"),
        }

    def _plot_geometry(self, rect: QtCore.QRectF) -> tuple[QtCore.QRectF, int, int]:
        scale = self._render_scale()
        title_height = QtGui.QFontMetrics(self._title_font()).height() + int(4 * scale)
        legend_height = max(QtGui.QFontMetrics(self._legend_font()).height(), int(10 * scale)) + int(8 * scale)
        bottom_space = QtGui.QFontMetrics(self._tick_font()).height() + int(38 * scale)
        legend_y = rect.top() + title_height + int(8 * scale)
        left_axis_space = int(8 * scale)
        right_axis_space = int(62 * scale)
        plot_rect = QtCore.QRectF(
            rect.left() + left_axis_space,
            legend_y + legend_height + int(8 * scale),
            max(80.0, rect.width() - left_axis_space - right_axis_space),
            rect.height() - title_height - legend_height - bottom_space - int(18 * scale),
        )
        return plot_rect, title_height, legend_height

    def _series_endpoint_map(self, scale_factor: float = 1.0) -> dict[str, QtCore.QPointF]:
        if not self._series_data or not self._series_data[0]:
            return {}
        rect = QtCore.QRectF(self.rect().adjusted(8, 8, -8, -8))
        plot_rect, _, _ = self._plot_geometry(rect)
        if plot_rect.height() <= 0:
            return {}
        y_min, y_max = self._frozen_range
        y_span = max(y_max - y_min, 0.1)
        sample_count = len(self._series_data[0])
        endpoint_map: dict[str, QtCore.QPointF] = {}
        for series_def, series in zip(self._series_defs, self._series_data):
            if not series:
                continue
            ratio = (len(series) - 1) / max(sample_count - 1, 1)
            x = plot_rect.left() + plot_rect.width() * ratio
            y = plot_rect.bottom() - (((series[-1] - y_min) / y_span) * plot_rect.height())
            endpoint_map[series_def["label"]] = QtCore.QPointF(x * scale_factor, y * scale_factor)
        return endpoint_map

    def _y_axis_unit(self) -> str:
        return _common_unit([series_def.get("unit", "") for series_def in self._series_defs])

    def _series_point_map(
        self,
        *,
        scale_factor: float = 1.0,
        point_limit: int = 220,
    ) -> dict[str, list[QtCore.QPointF]]:
        if not self._series_data or not self._series_data[0]:
            return {}
        rect = QtCore.QRectF(self.rect().adjusted(8, 8, -8, -8))
        plot_rect, _, _ = self._plot_geometry(rect)
        if plot_rect.height() <= 0:
            return {}
        y_min, y_max = self._frozen_range
        y_span = max(y_max - y_min, 0.1)
        sample_count = len(self._series_data[0])
        point_map: dict[str, list[QtCore.QPointF]] = {}
        for series_def, series in zip(self._series_defs, self._series_data):
            if not series:
                continue
            step = max(1, len(series) // point_limit)
            indices = list(range(0, len(series), step))
            if indices[-1] != len(series) - 1:
                indices.append(len(series) - 1)
            points: list[QtCore.QPointF] = []
            for index in indices:
                ratio = index / max(sample_count - 1, 1)
                x = plot_rect.left() + plot_rect.width() * ratio
                y = plot_rect.bottom() - (((series[index] - y_min) / y_span) * plot_rect.height())
                points.append(QtCore.QPointF(x * scale_factor, y * scale_factor))
            point_map[series_def["label"]] = points
        return point_map

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        rect = self.rect().adjusted(8, 8, -8, -8)
        colors = self._chart_colors()
        painter.fillRect(rect, colors["background"])
        painter.setPen(colors["text"])
        painter.setFont(self._title_font())
        painter.drawText(
            QtCore.QRect(rect.left(), rect.top(), rect.width(), QtGui.QFontMetrics(self._title_font()).height() + int(4 * self._render_scale())),
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
            self._title,
        )

        plot_rect, title_height, legend_height = self._plot_geometry(QtCore.QRectF(rect))
        legend_y = rect.top() + title_height + int(8 * self._render_scale())
        legend_x = rect.left()
        painter.setFont(self._legend_font())
        for series_index, series_def in enumerate(self._series_defs):
            legend_scale = self._render_scale()
            if self._report_mode:
                _draw_report_legend_sample(
                    painter,
                    legend_x,
                    legend_y + (legend_height * 0.5) - (5.0 * legend_scale),
                    28.0 * legend_scale,
                    series_def["label"],
                    None,
                    series_index,
                    colors["text"],
                )
            else:
                painter.fillRect(
                    QtCore.QRect(
                        legend_x,
                        legend_y,
                        int(10 * legend_scale),
                        int(10 * legend_scale),
                    ),
                    QtGui.QColor(series_def["color"]),
                )
            text_width = QtGui.QFontMetrics(self._legend_font()).horizontalAdvance(series_def["label"])
            painter.drawText(
                QtCore.QRect(
                    legend_x + int((42 if self._report_mode else 14) * legend_scale),
                    legend_y - int(3 * legend_scale),
                    max(text_width + int(50 * self._render_scale()), 120),
                    legend_height,
                ),
                QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
                series_def["label"],
            )
            legend_x += max(text_width + int((70 if self._report_mode else 42) * legend_scale), 124)

        self._plot_rect = plot_rect
        painter.setPen(QtGui.QPen(colors["border"], 1.2 if self._report_mode else 1.0))
        painter.drawRect(plot_rect)

        if not self._series_data or not self._series_data[0]:
            painter.setFont(self._legend_font())
            painter.drawText(plot_rect.toRect(), QtCore.Qt.AlignmentFlag.AlignCenter, self._status_text)
            return

        y_min, y_max = self._frozen_range
        y_span = max(y_max - y_min, 0.1)
        sample_count = len(self._series_data[0])
        total_ms = (sample_count - 1) * self._sample_period_s * 1000.0

        grid_pen = QtGui.QPen(colors["grid"], 1.0)
        grid_pen.setStyle(QtCore.Qt.PenStyle.DotLine if self._report_mode else QtCore.Qt.PenStyle.DashLine)
        painter.setPen(grid_pen)
        for grid_index in range(1, 5):
            y = plot_rect.top() + (plot_rect.height() * grid_index / 5.0)
            painter.drawLine(QtCore.QPointF(plot_rect.left(), y), QtCore.QPointF(plot_rect.right(), y))

        painter.setPen(colors["text"])
        painter.setFont(self._tick_font())
        scale = self._render_scale()
        y_unit = self._y_axis_unit()
        painter.drawText(
            QtCore.QRectF(
                plot_rect.right() - (88 * scale),
                plot_rect.top() + (6 * scale),
                84 * scale,
                18 * scale,
            ),
            QtCore.Qt.AlignmentFlag.AlignRight,
            _format_axis_value(y_max, unit=y_unit, decimals=3),
        )
        painter.drawText(
            QtCore.QRectF(
                plot_rect.right() - (88 * scale),
                plot_rect.bottom() - (34 * scale),
                84 * scale,
                18 * scale,
            ),
            QtCore.Qt.AlignmentFlag.AlignRight,
            _format_axis_value(y_min, unit=y_unit, decimals=3),
        )

        for marker_ratio in (0.0, 0.5, 1.0):
            x = plot_rect.left() + plot_rect.width() * marker_ratio
            marker_ms = total_ms * marker_ratio
            painter.drawLine(QtCore.QPointF(x, plot_rect.bottom()), QtCore.QPointF(x, plot_rect.bottom() + 4))
            label_rect = QtCore.QRectF(
                x - (38 * scale),
                plot_rect.bottom() + (14 * scale),
                76 * scale,
                16 * scale,
            )
            if marker_ratio <= 0.0:
                label_rect.moveLeft(plot_rect.left() + (2 * scale))
            elif marker_ratio >= 1.0:
                label_rect.moveRight(plot_rect.right() - (14 * scale))
            painter.drawText(
                label_rect,
                QtCore.Qt.AlignmentFlag.AlignCenter,
                f"{marker_ms:.1f} ms",
            )

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
            series_pen, marker_shape = _series_pen_spec(
                None,
                self._series_defs[series_index]["label"],
                series_index,
                report_mode=self._report_mode,
                fallback_color=self._series_defs[series_index]["color"],
            )
            painter.setPen(series_pen)
            painter.drawPath(path)
            if self._report_mode:
                for marker_index in _report_marker_indices(len(series), 16):
                    ratio = marker_index / max(sample_count - 1, 1)
                    x = plot_rect.left() + plot_rect.width() * ratio
                    y = plot_rect.bottom() - (((series[marker_index] - y_min) / y_span) * plot_rect.height())
                    _draw_series_marker(
                        painter,
                        QtCore.QPointF(x, y),
                        marker_shape,
                        7.0 * self._render_scale(),
                        QtGui.QColor("#000000"),
                    )

        if self._hover_index is not None and 0 <= self._hover_index < sample_count:
            ratio = self._hover_index / max(sample_count - 1, 1)
            x = plot_rect.left() + plot_rect.width() * ratio
            cursor_pen = QtGui.QPen(colors["cursor"])
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


class XyPlotView(QtWidgets.QWidget):
    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._title = "Plot"
        self._x_label = "X"
        self._x_unit = ""
        self._y_label = "Y"
        self._y_unit = ""
        self._series_defs: list[dict[str, str]] = []
        self._series_data: list[list[tuple[float, float]]] = []
        self._status_text = "Waiting for data..."
        self.setMinimumHeight(250)

    def clear(self, status_text: str = "Waiting for data...") -> None:
        self._series_defs = []
        self._series_data = []
        self._status_text = status_text
        self.update()

    def set_plot(
        self,
        *,
        title: str,
        x_label: str,
        x_unit: str,
        y_label: str,
        y_unit: str,
        series_defs: list[dict[str, str]],
        series_data: list[list[tuple[float, float]]],
        status_text: str,
    ) -> None:
        self._title = title
        self._x_label = x_label
        self._x_unit = x_unit
        self._y_label = y_label
        self._y_unit = y_unit
        self._series_defs = list(series_defs)
        self._series_data = [[(float(x), float(y)) for x, y in series] for series in series_data]
        self._status_text = status_text
        self.update()

    def _chart_colors(self) -> dict[str, QtGui.QColor]:
        return {
            "background": self.palette().base().color(),
            "text": self.palette().text().color(),
            "border": self.palette().mid().color(),
            "grid": self.palette().midlight().color(),
        }

    def _plot_geometry(self, rect: QtCore.QRectF) -> tuple[QtCore.QRectF, int, int]:
        title_height = QtGui.QFontMetrics(self.font()).height() + 6
        legend_height = QtGui.QFontMetrics(self.font()).height() + 14
        bottom_space = QtGui.QFontMetrics(self.font()).height() + 42
        plot_rect = QtCore.QRectF(
            rect.left() + 8.0,
            rect.top() + title_height + legend_height + 14.0,
            max(80.0, rect.width() - 80.0),
            max(80.0, rect.height() - title_height - legend_height - bottom_space),
        )
        return plot_rect, title_height, legend_height

    def _ranges(self) -> tuple[float, float, float, float]:
        xs = [point[0] for series in self._series_data for point in series]
        ys = [point[1] for series in self._series_data for point in series]
        if not xs or not ys:
            return (-1.0, 1.0, -1.0, 1.0)
        x_min = min(xs)
        x_max = max(xs)
        y_min = min(ys)
        y_max = max(ys)
        if abs(x_max - x_min) < 1.0e-6:
            x_min -= 1.0
            x_max += 1.0
        if abs(y_max - y_min) < 1.0e-6:
            y_min -= 1.0
            y_max += 1.0
        x_margin = (x_max - x_min) * 0.08
        y_margin = (y_max - y_min) * 0.12
        return (
            x_min - x_margin,
            x_max + x_margin,
            y_min - y_margin,
            y_max + y_margin,
        )

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        rect = QtCore.QRectF(self.rect().adjusted(8, 8, -8, -8))
        colors = self._chart_colors()
        painter.fillRect(rect, colors["background"])
        painter.setPen(colors["text"])
        painter.drawText(
            QtCore.QRectF(rect.left(), rect.top(), rect.width(), 24.0),
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
            self._title,
        )

        plot_rect, title_height, legend_height = self._plot_geometry(rect)
        legend_x = rect.left()
        legend_y = rect.top() + title_height + 2.0
        for series_def in self._series_defs:
            if not series_def.get("show_legend", True):
                continue
            label = series_def.get("label", "Series")
            color = QtGui.QColor(series_def.get("color", "#4dabf7"))
            alpha = int(series_def.get("alpha", 255))
            color.setAlpha(max(0, min(255, alpha)))
            painter.fillRect(QtCore.QRectF(legend_x, legend_y + 6.0, 10.0, 10.0), color)
            painter.drawText(
                QtCore.QRectF(legend_x + 16.0, legend_y, 170.0, legend_height),
                QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
                label,
            )
            legend_x += 170.0

        painter.setPen(QtGui.QPen(colors["border"], 1.0))
        painter.drawRect(plot_rect)

        if not self._series_data or not any(self._series_data):
            painter.drawText(
                plot_rect,
                QtCore.Qt.AlignmentFlag.AlignCenter,
                self._status_text,
            )
            return

        x_min, x_max, y_min, y_max = self._ranges()
        x_span = max(1.0e-6, x_max - x_min)
        y_span = max(1.0e-6, y_max - y_min)

        grid_pen = QtGui.QPen(colors["grid"], 1.0)
        grid_pen.setStyle(QtCore.Qt.PenStyle.DashLine)
        painter.setPen(grid_pen)
        for grid_index in range(1, 5):
            y = plot_rect.top() + (plot_rect.height() * grid_index / 5.0)
            painter.drawLine(QtCore.QPointF(plot_rect.left(), y), QtCore.QPointF(plot_rect.right(), y))

        painter.setPen(colors["text"])
        painter.drawText(
            QtCore.QRectF(plot_rect.right() - 92.0, plot_rect.top() + 6.0, 88.0, 18.0),
            QtCore.Qt.AlignmentFlag.AlignRight,
            _format_axis_value(y_max, unit=self._y_unit, decimals=3),
        )
        painter.drawText(
            QtCore.QRectF(plot_rect.right() - 92.0, plot_rect.bottom() - 30.0, 88.0, 18.0),
            QtCore.Qt.AlignmentFlag.AlignRight,
            _format_axis_value(y_min, unit=self._y_unit, decimals=3),
        )
        painter.drawText(
            QtCore.QRectF(plot_rect.left(), plot_rect.bottom() + 12.0, 110.0, 18.0),
            QtCore.Qt.AlignmentFlag.AlignLeft,
            _format_axis_value(x_min, unit=self._x_unit, decimals=3),
        )
        painter.drawText(
            QtCore.QRectF(plot_rect.right() - 110.0, plot_rect.bottom() + 12.0, 110.0, 18.0),
            QtCore.Qt.AlignmentFlag.AlignRight,
            _format_axis_value(x_max, unit=self._x_unit, decimals=3),
        )
        painter.drawText(
            QtCore.QRectF(plot_rect.left(), plot_rect.bottom() + 28.0, plot_rect.width(), 18.0),
            QtCore.Qt.AlignmentFlag.AlignCenter,
            f"{self._x_label} / {self._y_label}",
        )

        for series_index, series in enumerate(self._series_data):
            if not series:
                continue
            series_def = self._series_defs[series_index] if series_index < len(self._series_defs) else {}
            color = QtGui.QColor(series_def.get("color", "#4dabf7"))
            alpha = int(series_def.get("alpha", 255))
            color.setAlpha(max(0, min(255, alpha)))
            mode = str(series_def.get("mode", "line"))
            line_width = float(series_def.get("line_width", 1.8))
            marker_radius = float(series_def.get("marker_radius", 2.0))
            marker_every = int(series_def.get("marker_every", 1))
            marker_every = max(1, marker_every)
            draw_line = mode in ("line", "line_scatter")
            draw_scatter = mode in ("scatter", "line_scatter")

            points: list[QtCore.QPointF] = []
            for x_value, y_value in series:
                x = plot_rect.left() + ((x_value - x_min) / x_span) * plot_rect.width()
                y = plot_rect.bottom() - ((y_value - y_min) / y_span) * plot_rect.height()
                points.append(QtCore.QPointF(x, y))

            if draw_line and points:
                pen = QtGui.QPen(color, line_width)
                painter.setPen(pen)
                path = QtGui.QPainterPath()
                path.moveTo(points[0])
                for point in points[1:]:
                    path.lineTo(point)
                painter.drawPath(path)

            if draw_scatter and points:
                painter.setPen(QtGui.QPen(color, max(1.0, marker_radius * 0.4)))
                painter.setBrush(QtGui.QBrush(color))
                for point_index, point in enumerate(points):
                    if (point_index % marker_every) != 0:
                        continue
                    painter.drawEllipse(point, marker_radius, marker_radius)


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
        self._autotune_continue_signature: tuple[int, int] | None = None
        self._trace_capture = _empty_scope_state("Firmware Trace Scope")
        self._uerror_samples: list[UerrorRawSample] = []
        self._uerror_status_text = "Waiting for survey..."
        self._uerror_total_samples = 0
        self._uerror_state = 0
        self._uerror_lut_preview_norm: list[float] = []
        self._uerror_lut_preview_current_max_a = 0.0
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
        self._speed_test_session: SpeedTestSession | None = None
        self._position_test_session: PositionTestSession | None = None

        self._ack_timer = QtCore.QTimer(self)
        self._ack_timer.setSingleShot(True)
        self._ack_timer.timeout.connect(self._on_ack_timeout)

        self._monitor_timer = QtCore.QTimer(self)
        self._monitor_timer.setInterval(250)
        self._monitor_timer.timeout.connect(self._queue_monitor_poll)

        self._trend_refresh_timer = QtCore.QTimer(self)
        self._trend_refresh_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self._trend_refresh_timer.setInterval(TREND_REFRESH_INTERVAL_MS)
        self._trend_refresh_timer.timeout.connect(self._refresh_scada_ui)

        self._speed_test_timer = QtCore.QTimer(self)
        self._speed_test_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self._speed_test_timer.setInterval(SPEED_TEST_TIMER_INTERVAL_MS)
        self._speed_test_timer.timeout.connect(self._on_speed_test_timer)

        self._position_test_timer = QtCore.QTimer(self)
        self._position_test_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self._position_test_timer.setInterval(POSITION_TEST_TIMER_INTERVAL_MS)
        self._position_test_timer.timeout.connect(self._on_position_test_timer)

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
        self._refresh_runtime_toggle_buttons()

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
            ("setting_position", "Setting Position"),
            ("tracking_error", "Tracking Error"),
            ("mech_angle_single", "Mech Angle (ST)"),
            ("mech_angle_multi", "Mech Angle (MT)"),
            ("total_distance", "Total Distance"),
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

        layout.addWidget(dashboard_widget, 1)
        layout.addWidget(trend_note)
        return box

    def _build_monitor_tabs(self) -> QtWidgets.QTabWidget:
        tabs = QtWidgets.QTabWidget()
        self.driver_monitor_tab = self._build_monitor_group()
        self.parameter_panel = self._build_parameter_panel()
        tabs.addTab(self.driver_monitor_tab, "Driver Monitor")
        tabs.addTab(self.parameter_panel, "Parameters")
        self.monitor_tabs = tabs
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
            on_export_requested=self._open_report_editor_for_chart,
        )
        self.dq_current_panel = ScadaTrendPanel(
            "D/Q Currents",
            self._trend_buffer,
            ["id_ref", "id_current", "iq_ref", "iq_current"],
            on_export_requested=self._open_report_editor_for_chart,
        )
        self.vdq_voltage_panel = ScadaTrendPanel(
            "D/Q Voltages",
            self._trend_buffer,
            ["vd", "vq"],
            on_export_requested=self._open_report_editor_for_chart,
        )
        self.speed_panel = ScadaTrendPanel(
            "Speed Trend",
            self._trend_buffer,
            ["act_speed", "cmd_speed", "speed_error"],
            on_export_requested=self._open_report_editor_for_chart,
        )
        self.position_panel = ScadaTrendPanel(
            "Position Trend",
            self._trend_buffer,
            ["act_position_deg", "cmd_position_deg", "position_error_deg", "validation_error_deg"],
            on_export_requested=self._open_report_editor_for_chart,
        )

        tabs.addTab(self.phase_current_panel, "Phase Currents")
        tabs.addTab(self.dq_current_panel, "D/Q Currents")
        tabs.addTab(self.vdq_voltage_panel, "Vd/Vq")
        tabs.addTab(self.speed_panel, "Speed")
        tabs.addTab(self.position_panel, "Position")

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
            self.position_panel,
        ]
        return dialog

    def _build_tuning_window(self) -> QtWidgets.QDialog:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Commissioning / Scope")
        dialog.resize(1280, 920)
        self._configure_auxiliary_window(dialog)

        layout = QtWidgets.QVBoxLayout(dialog)
        tabs = QtWidgets.QTabWidget(dialog)
        tabs.addTab(self._wrap_tuning_tab_scroll_area(self._build_id_tuning_tab()), "Id Tuning")
        tabs.addTab(self._wrap_tuning_tab_scroll_area(self._build_autotune_tab()), "Motor Auto-Tune")
        tabs.addTab(self._wrap_tuning_tab_scroll_area(self._build_trace_scope_tab()), "Trace Scope")
        tabs.addTab(self._wrap_tuning_tab_scroll_area(self._build_uerror_tab()), "Uerror Characterization")
        layout.addWidget(tabs)
        return dialog

    def _wrap_tuning_tab_scroll_area(
        self,
        content: QtWidgets.QWidget,
    ) -> QtWidgets.QScrollArea:
        layout = content.layout()
        if layout is not None:
            layout.setSizeConstraint(
                QtWidgets.QLayout.SizeConstraint.SetMinimumSize
            )

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        scroll.setVerticalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded
        )
        scroll.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop)
        scroll.setWidget(content)
        return scroll

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
        self.ctuning_kp_spin.setValue(DEFAULT_CTUNING_CURRENT_KP)

        self.ctuning_ki_spin = QtWidgets.QDoubleSpinBox()
        self.ctuning_ki_spin.setRange(0.0, 10000.0)
        self.ctuning_ki_spin.setDecimals(5)
        self.ctuning_ki_spin.setSingleStep(0.001)
        self.ctuning_ki_spin.setValue(DEFAULT_CTUNING_CURRENT_KI)

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

        scope_toolbar.addWidget(self.ctuning_auto_scale_checkbox)
        scope_toolbar.addWidget(self.ctuning_clear_button)
        self._append_report_controls(
            scope_toolbar,
            [self.ctuning_current_scope_view, self.ctuning_voltage_scope_view],
            export_targets=[
                ("Export Id Trace", self.ctuning_current_scope_view),
                ("Export Vd/Vq Trace", self.ctuning_voltage_scope_view),
            ],
        )
        scope_toolbar.addWidget(scope_hint, 1)

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

        self.autotune_ls_frequency_spin = QtWidgets.QDoubleSpinBox()
        self.autotune_ls_frequency_spin.setRange(10.0, 1000.0)
        self.autotune_ls_frequency_spin.setDecimals(3)
        self.autotune_ls_frequency_spin.setValue(200.0)
        self.autotune_ls_frequency_spin.setSuffix(" Hz")

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
        self.autotune_kt_value_label = QtWidgets.QLabel("-")
        self.autotune_current_gain_value_label = QtWidgets.QLabel("-")
        self.autotune_speed_gain_value_label = QtWidgets.QLabel("-")
        self.autotune_position_gain_value_label = QtWidgets.QLabel("-")

        autotune_hint = QtWidgets.QLabel(
            "Workflow: keep the rotor mechanically locked for Rs and the Ls sine-injection stage, then let the motor run unloaded during the open-loop flux stage. Before starting the flux stage, try the same Flux Voltage and Flux Frequency in open-loop V/F mode first. The motor should spin smoothly without obvious jerks or abnormal vibration; otherwise adjust those values before auto-tune so the estimated flux/Ke stays trustworthy. The module now uses the fixed 16 kHz runtime profile reported by firmware."
        )
        autotune_hint.setWordWrap(True)

        config_layout.addWidget(QtWidgets.QLabel("Rs Low Current"), 0, 0)
        config_layout.addWidget(self.autotune_rs_low_spin, 0, 1)
        config_layout.addWidget(QtWidgets.QLabel("Rs High Current"), 0, 2)
        config_layout.addWidget(self.autotune_rs_high_spin, 0, 3)
        config_layout.addWidget(QtWidgets.QLabel("Ls Sine Voltage"), 1, 0)
        config_layout.addWidget(self.autotune_ls_voltage_spin, 1, 1)
        config_layout.addWidget(QtWidgets.QLabel("Ls Frequency"), 1, 2)
        config_layout.addWidget(self.autotune_ls_frequency_spin, 1, 3)
        config_layout.addWidget(QtWidgets.QLabel("Flux Frequency"), 2, 0)
        config_layout.addWidget(self.autotune_flux_frequency_spin, 2, 1)
        config_layout.addWidget(QtWidgets.QLabel("Flux Voltage"), 2, 2)
        config_layout.addWidget(self.autotune_flux_voltage_spin, 2, 3)
        config_layout.addWidget(QtWidgets.QLabel("Current Bandwidth"), 3, 0)
        config_layout.addWidget(self.autotune_current_bw_spin, 3, 1)
        config_layout.addWidget(QtWidgets.QLabel("Speed Bandwidth"), 3, 2)
        config_layout.addWidget(self.autotune_speed_bw_spin, 3, 3)
        config_layout.addWidget(QtWidgets.QLabel("Position Bandwidth"), 4, 0)
        config_layout.addWidget(self.autotune_position_bw_spin, 4, 1)
        config_layout.addWidget(self.autotune_start_button, 5, 0)
        config_layout.addWidget(self.autotune_stop_button, 5, 1)
        config_layout.addWidget(self.autotune_apply_button, 5, 2, 1, 2)
        config_layout.addWidget(QtWidgets.QLabel("Progress"), 6, 0)
        config_layout.addWidget(self.autotune_progress_bar, 6, 1, 1, 3)
        config_layout.addWidget(autotune_hint, 7, 0, 1, 4)

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
        results_layout.addWidget(QtWidgets.QLabel("Position PI"), 5, 2)
        results_layout.addWidget(self.autotune_position_gain_value_label, 5, 3)
        results_layout.addWidget(QtWidgets.QLabel("Kt"), 6, 0)
        results_layout.addWidget(self.autotune_kt_value_label, 6, 1)

        scope_toolbar = QtWidgets.QHBoxLayout()
        self.autotune_auto_scale_checkbox = QtWidgets.QCheckBox("Auto-scale Y")
        self.autotune_auto_scale_checkbox.setChecked(True)
        self.autotune_clear_button = QtWidgets.QPushButton("Clear Capture")
        scope_note = QtWidgets.QLabel(
            "Rs chart shows Id/Vd during the two-current steady-state test. Ls chart shows the direct d-axis voltage step response used to estimate di/dt with the active loop timing."
        )
        scope_note.setWordWrap(True)

        self.autotune_scope_view = ScopeCaptureView()
        self._refresh_autotune_panel(None)

        scope_toolbar.addWidget(self.autotune_auto_scale_checkbox)
        scope_toolbar.addWidget(self.autotune_clear_button)
        self._append_report_controls(
            scope_toolbar,
            [self.autotune_scope_view],
            export_targets=[("Capture & Edit", self.autotune_scope_view)],
        )
        scope_toolbar.addWidget(scope_note, 1)

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
        self.trace_scope_view = ScopeCaptureView()

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
        report_toolbar = QtWidgets.QHBoxLayout()
        self._append_report_controls(
            report_toolbar,
            [self.trace_scope_view],
            export_targets=[("Capture & Edit", self.trace_scope_view)],
        )
        layout.addWidget(control_group)
        layout.addLayout(report_toolbar)
        layout.addWidget(self.trace_scope_view, 1)

        self._apply_trace_preset(self.trace_preset_combo.currentText())
        return widget

    def _build_uerror_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        config_group = QtWidgets.QGroupBox("Driver Uerror Characterization")
        config_layout = QtWidgets.QGridLayout(config_group)

        self.uerror_rs_actual_spin = QtWidgets.QDoubleSpinBox()
        self.uerror_rs_actual_spin.setRange(0.001, 100.0)
        self.uerror_rs_actual_spin.setDecimals(5)
        self.uerror_rs_actual_spin.setValue(3.40000)
        self.uerror_rs_actual_spin.setSuffix(" ohm")

        self.uerror_sweep_current_spin = QtWidgets.QDoubleSpinBox()
        self.uerror_sweep_current_spin.setRange(0.05, 20.0)
        self.uerror_sweep_current_spin.setDecimals(3)
        self.uerror_sweep_current_spin.setValue(0.60)
        self.uerror_sweep_current_spin.setSuffix(" A")

        self.uerror_fine_zone_spin = QtWidgets.QDoubleSpinBox()
        self.uerror_fine_zone_spin.setRange(0.01, 10.0)
        self.uerror_fine_zone_spin.setDecimals(3)
        self.uerror_fine_zone_spin.setValue(0.15)
        self.uerror_fine_zone_spin.setSuffix(" A")

        self.uerror_fine_step_spin = QtWidgets.QDoubleSpinBox()
        self.uerror_fine_step_spin.setRange(0.005, 5.0)
        self.uerror_fine_step_spin.setDecimals(3)
        self.uerror_fine_step_spin.setValue(0.02)
        self.uerror_fine_step_spin.setSuffix(" A")

        self.uerror_coarse_step_spin = QtWidgets.QDoubleSpinBox()
        self.uerror_coarse_step_spin.setRange(0.01, 5.0)
        self.uerror_coarse_step_spin.setDecimals(3)
        self.uerror_coarse_step_spin.setValue(0.05)
        self.uerror_coarse_step_spin.setSuffix(" A")

        self.uerror_settle_ms_spin = QtWidgets.QDoubleSpinBox()
        self.uerror_settle_ms_spin.setRange(1.0, 2000.0)
        self.uerror_settle_ms_spin.setDecimals(1)
        self.uerror_settle_ms_spin.setValue(40.0)
        self.uerror_settle_ms_spin.setSuffix(" ms")

        self.uerror_average_samples_spin = QtWidgets.QSpinBox()
        self.uerror_average_samples_spin.setRange(8, 512)
        self.uerror_average_samples_spin.setValue(96)

        self.uerror_start_button = QtWidgets.QPushButton("Start Sweep")
        self.uerror_stop_button = QtWidgets.QPushButton("Stop")
        self.uerror_clear_button = QtWidgets.QPushButton("Clear")
        self.uerror_apply_runtime_button = QtWidgets.QPushButton("Apply Runtime LUT")
        self.uerror_save_flash_button = QtWidgets.QPushButton("Save LUT to Flash")
        self.uerror_progress_bar = QtWidgets.QProgressBar()
        self.uerror_progress_bar.setRange(0, 100)
        self.uerror_progress_bar.setValue(0)
        self.uerror_status_label = QtWidgets.QLabel("Idle")
        self.uerror_status_label.setWordWrap(True)

        self.uerror_enforce_odd_checkbox = QtWidgets.QCheckBox("Force odd symmetry")
        self.uerror_enforce_odd_checkbox.setChecked(True)
        self.uerror_smooth_checkbox = QtWidgets.QCheckBox("Smooth LUT")
        self.uerror_smooth_checkbox.setChecked(True)

        config_hint = QtWidgets.QLabel(
            "<b>How To Read These Settings</b>"
            "<ul>"
            "<li><b>Rs Actual</b>: the real stator resistance that GUI uses for "
            "<code>Uerror = Vcmd - Rs * I</code>. Enter the measured value you trust.</li>"
            "<li><b>Sweep Current Max</b>: the largest positive and negative current target used to span the driver characteristic.</li>"
            "<li><b>Fine Zone</b>: the low-current region around 0 A that is sampled more densely to capture dead-time and zero-crossing nonlinearity.</li>"
            "<li><b>Fine Step</b>: current step inside the Fine Zone.</li>"
            "<li><b>Coarse Step</b>: current step outside the Fine Zone, where the curve is usually smoother.</li>"
            "<li><b>Settle Time</b>: wait time at each sweep point before averaging starts.</li>"
            "<li><b>Average Samples</b>: number of samples averaged at each point to suppress noise.</li>"
            "<li><b>Force odd symmetry</b>: enforces <code>f(-I) = -f(I)</code> on the preview LUT.</li>"
            "<li><b>Smooth LUT</b>: applies a light smoothing pass before runtime apply or flash save.</li>"
            "</ul>"
            "<b>Recommended Flow</b>"
            "<ul>"
            "<li>Lock the rotor mechanically and make sure encoder alignment is already valid.</li>"
            "<li>Run the sweep, inspect the three raw plots, then review the normalized LUT preview.</li>"
            "<li>Apply the LUT in RAM first. Save to flash only after the low-speed behavior looks better.</li>"
            "</ul>"
        )
        config_hint.setTextFormat(QtCore.Qt.TextFormat.RichText)
        config_hint.setWordWrap(True)

        config_layout.addWidget(QtWidgets.QLabel("Rs Actual"), 0, 0)
        config_layout.addWidget(self.uerror_rs_actual_spin, 0, 1)
        config_layout.addWidget(QtWidgets.QLabel("Sweep Current Max"), 0, 2)
        config_layout.addWidget(self.uerror_sweep_current_spin, 0, 3)
        config_layout.addWidget(QtWidgets.QLabel("Fine Zone"), 1, 0)
        config_layout.addWidget(self.uerror_fine_zone_spin, 1, 1)
        config_layout.addWidget(QtWidgets.QLabel("Fine Step"), 1, 2)
        config_layout.addWidget(self.uerror_fine_step_spin, 1, 3)
        config_layout.addWidget(QtWidgets.QLabel("Coarse Step"), 2, 0)
        config_layout.addWidget(self.uerror_coarse_step_spin, 2, 1)
        config_layout.addWidget(QtWidgets.QLabel("Settle Time"), 2, 2)
        config_layout.addWidget(self.uerror_settle_ms_spin, 2, 3)
        config_layout.addWidget(QtWidgets.QLabel("Average Samples"), 3, 0)
        config_layout.addWidget(self.uerror_average_samples_spin, 3, 1)
        config_layout.addWidget(self.uerror_enforce_odd_checkbox, 3, 2)
        config_layout.addWidget(self.uerror_smooth_checkbox, 3, 3)
        config_layout.addWidget(self.uerror_start_button, 4, 0)
        config_layout.addWidget(self.uerror_stop_button, 4, 1)
        config_layout.addWidget(self.uerror_clear_button, 4, 2)
        config_layout.addWidget(self.uerror_apply_runtime_button, 4, 3)
        config_layout.addWidget(QtWidgets.QLabel("Progress"), 5, 0)
        config_layout.addWidget(self.uerror_progress_bar, 5, 1, 1, 3)
        config_layout.addWidget(QtWidgets.QLabel("Status"), 6, 0)
        config_layout.addWidget(self.uerror_status_label, 6, 1, 1, 3)
        config_layout.addWidget(self.uerror_save_flash_button, 7, 0, 1, 4)
        config_layout.addWidget(config_hint, 8, 0, 1, 4)

        self.uerror_voltage_plot = XyPlotView()
        self.uerror_uerror_plot = XyPlotView()
        self.uerror_lut_plot = XyPlotView()
        self._refresh_uerror_plots()

        layout.addWidget(config_group)
        layout.addWidget(self.uerror_voltage_plot, 1)
        layout.addWidget(self.uerror_uerror_plot, 1)
        layout.addWidget(self.uerror_lut_plot, 1)
        return widget

    def _build_quick_command_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Controls")
        layout = QtWidgets.QVBoxLayout(box)

        self.servo_on_button = QtWidgets.QPushButton("Servo ON")
        self.servo_off_button = QtWidgets.QPushButton("Servo OFF")
        self.ack_fault_button = QtWidgets.QPushButton("Reset Alarm")
        self.ack_fault_button.setEnabled(False)
        self.save_flash_button = QtWidgets.QPushButton("Save Params to Flash")

        self.quick_command_tabs = QtWidgets.QTabWidget()

        drive_tab = QtWidgets.QWidget()
        drive_layout = QtWidgets.QVBoxLayout(drive_tab)
        button_grid = QtWidgets.QGridLayout()
        button_grid.addWidget(self.servo_on_button, 0, 0)
        button_grid.addWidget(self.servo_off_button, 0, 1)
        button_grid.addWidget(self.ack_fault_button, 0, 2)
        button_grid.addWidget(self.save_flash_button, 0, 3)
        drive_layout.addLayout(button_grid)
        drive_hint = QtWidgets.QLabel(
            "Servo ON only performs the safe arm sequence with no motion command. Watch Driver Monitor for current calibration and encoder alignment status, then use the FOC Control tab to start closed-loop motion when the drive is ready."
        )
        drive_hint.setWordWrap(True)
        drive_layout.addWidget(drive_hint)
        self.alarm_tab = self._build_alarm_tab()
        drive_layout.addWidget(self.alarm_tab, 1)

        test_tab = self._build_test_mode_tab()
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
        self.vf_toggle_button = QtWidgets.QPushButton("Start V/F")
        vf_note = QtWidgets.QLabel(
            "Default motor profile: 4 pole pairs, 3000 rpm, 200 Hz electrical, 91 V, 1.6 A, 200 W. Open-loop electrical frequency now accepts both positive and negative commands for direction/sign checks, while voltage remains firmware-clamped for safe testing. Trend Charts are low-rate monitor views; use Trace Scope with the 'Phase Currents' preset when you need waveform-level Iu/Iv/Iw detail."
        )
        vf_note.setWordWrap(True)
        vf_layout.addWidget(QtWidgets.QLabel("Electrical Frequency"), 0, 0)
        vf_layout.addWidget(self.vf_frequency_spin, 0, 1)
        vf_layout.addWidget(QtWidgets.QLabel("Voltage Reference"), 1, 0)
        vf_layout.addWidget(self.vf_voltage_spin, 1, 1)
        vf_layout.addWidget(self.vf_toggle_button, 2, 0, 1, 2)
        vf_layout.addWidget(vf_note, 3, 0, 1, 2)
        vf_layout.setRowStretch(4, 1)

        self.quick_command_tabs.addTab(drive_tab, "Drive")
        self.quick_command_tabs.addTab(foc_tab, "FOC Control")
        self.quick_command_tabs.addTab(test_tab, "Test Mode")
        self.quick_command_tabs.addTab(vf_tab, "Open Loop V/F")
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

        self.foc_target_position_label = QtWidgets.QLabel("Target Counts")
        self.foc_target_position_spin = QtWidgets.QDoubleSpinBox()
        self.foc_target_position_spin.setRange(
            -POSITION_TARGET_COUNT_GUI_LIMIT,
            POSITION_TARGET_COUNT_GUI_LIMIT,
        )
        self.foc_target_position_spin.setDecimals(1)
        self.foc_target_position_spin.setSingleStep(1000.0)
        self.foc_target_position_spin.setSuffix(" cnt")
        self.foc_target_position_spin.setReadOnly(True)
        self.foc_target_position_spin.setButtonSymbols(
            QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons
        )

        self.foc_position_tracking_label = QtWidgets.QLabel("Tracking")
        self.foc_position_tracking_combo = QtWidgets.QComboBox()
        self.foc_position_tracking_combo.addItem(
            "Single Turn",
            POSITION_TRACKING_MODE_SINGLE_TURN,
        )
        self.foc_position_tracking_combo.addItem(
            "Multi Turn",
            POSITION_TRACKING_MODE_MULTI_TURN,
        )

        self.foc_target_angle_label = QtWidgets.QLabel("Go to Angle")
        self.foc_target_angle_spin = QtWidgets.QDoubleSpinBox()
        self.foc_target_angle_spin.setRange(0.0, 360.0)
        self.foc_target_angle_spin.setDecimals(2)
        self.foc_target_angle_spin.setSingleStep(1.0)
        self.foc_target_angle_spin.setSuffix(" deg")

        self.foc_jog_label = QtWidgets.QLabel("Relative Move")
        self.foc_jog_minus_90_button = QtWidgets.QPushButton("<<")
        self.foc_jog_minus_10_button = QtWidgets.QPushButton("<")
        self.foc_jog_plus_10_button = QtWidgets.QPushButton(">")
        self.foc_jog_plus_90_button = QtWidgets.QPushButton(">>")
        self.foc_jog_minus_90_button.setToolTip("Move target by -90 deg")
        self.foc_jog_minus_10_button.setToolTip("Move target by -10 deg")
        self.foc_jog_plus_10_button.setToolTip("Move target by +10 deg")
        self.foc_jog_plus_90_button.setToolTip("Move target by +90 deg")

        self.foc_angle_slider_label = QtWidgets.QLabel("Angle Slider")
        self.foc_angle_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.foc_angle_slider.setRange(0, 3600)
        self.foc_angle_slider.setSingleStep(10)
        self.foc_angle_slider.setPageStep(100)
        self.foc_angle_slider_value_label = QtWidgets.QLabel("0.0 deg")
        self.foc_target_counts_hint_label = QtWidgets.QLabel("0.0 cnt")
        self.foc_target_counts_hint_label.setStyleSheet("color: #9aa0a6;")

        self.foc_position_kp_label = QtWidgets.QLabel("Position Kp")
        self.foc_position_kp_spin = QtWidgets.QDoubleSpinBox()
        self.foc_position_kp_spin.setRange(0.0, 1000.0)
        self.foc_position_kp_spin.setDecimals(5)
        self.foc_position_kp_spin.setSingleStep(0.01)
        self.foc_position_kp_spin.setValue(0.05)

        self.foc_position_ki_label = QtWidgets.QLabel("Position Ki")
        self.foc_position_ki_spin = QtWidgets.QDoubleSpinBox()
        self.foc_position_ki_spin.setRange(0.0, 10000.0)
        self.foc_position_ki_spin.setDecimals(5)
        self.foc_position_ki_spin.setSingleStep(0.01)
        self.foc_position_ki_spin.setValue(0.50)

        self.foc_position_vff_gain_label = QtWidgets.QLabel("Pos VFF Gain")
        self.foc_position_vff_gain_spin = QtWidgets.QDoubleSpinBox()
        self.foc_position_vff_gain_spin.setRange(0.0, 1000.0)
        self.foc_position_vff_gain_spin.setDecimals(5)
        self.foc_position_vff_gain_spin.setSingleStep(0.01)
        self.foc_position_vff_gain_spin.setValue(0.0)

        self.foc_position_vff_filter_label = QtWidgets.QLabel("Pos VFF Filter")
        self.foc_position_vff_filter_spin = QtWidgets.QDoubleSpinBox()
        self.foc_position_vff_filter_spin.setRange(0.1, 5000.0)
        self.foc_position_vff_filter_spin.setDecimals(2)
        self.foc_position_vff_filter_spin.setSingleStep(5.0)
        self.foc_position_vff_filter_spin.setSuffix(" Hz")
        self.foc_position_vff_filter_spin.setValue(50.0)

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
        self.foc_direction_test_status_label = QtWidgets.QLabel("Idle")
        self.foc_live_summary_label = QtWidgets.QLabel(
            "Speed Mode is ready. Enter target speed and gains, then press Start FOC."
        )
        self.foc_live_summary_label.setWordWrap(True)

        self.start_foc_rotating_theta_test_button = QtWidgets.QPushButton("Run Rotating Theta Current Test")
        self.start_foc_rotating_theta_voltage_test_button = QtWidgets.QPushButton("Run Rotating Theta Voltage Test")
        self.start_foc_current_feedback_map_test_button = QtWidgets.QPushButton("Run Current Feedback Map Test")
        self.start_foc_button = QtWidgets.QPushButton("Start FOC")
        self.stop_foc_button = QtWidgets.QPushButton("Stop FOC")

        jog_button_layout = QtWidgets.QHBoxLayout()
        jog_button_layout.setContentsMargins(0, 0, 0, 0)
        jog_button_layout.setSpacing(6)
        jog_button_layout.addWidget(self.foc_jog_minus_90_button)
        jog_button_layout.addWidget(self.foc_jog_minus_10_button)
        jog_button_layout.addWidget(self.foc_jog_plus_10_button)
        jog_button_layout.addWidget(self.foc_jog_plus_90_button)
        self.foc_jog_button_widget = QtWidgets.QWidget()
        self.foc_jog_button_widget.setLayout(jog_button_layout)

        angle_entry_layout = QtWidgets.QHBoxLayout()
        angle_entry_layout.setContentsMargins(0, 0, 0, 0)
        angle_entry_layout.setSpacing(6)
        angle_entry_layout.addWidget(self.foc_target_angle_spin, 1)
        self.foc_target_angle_widget = QtWidgets.QWidget()
        self.foc_target_angle_widget.setLayout(angle_entry_layout)

        angle_slider_layout = QtWidgets.QHBoxLayout()
        angle_slider_layout.setContentsMargins(0, 0, 0, 0)
        angle_slider_layout.setSpacing(8)
        angle_slider_layout.addWidget(self.foc_angle_slider, 1)
        angle_slider_layout.addWidget(self.foc_angle_slider_value_label)
        self.foc_angle_slider_widget = QtWidgets.QWidget()
        self.foc_angle_slider_widget.setLayout(angle_slider_layout)

        summary_layout.addWidget(QtWidgets.QLabel("Mode"), 0, 0)
        summary_layout.addWidget(self.foc_mode_combo, 0, 1)
        summary_layout.addWidget(self.foc_target_speed_label, 0, 2)
        summary_layout.addWidget(self.foc_target_speed_spin, 0, 3)
        summary_layout.addWidget(self.foc_position_tracking_label, 1, 0)
        summary_layout.addWidget(self.foc_position_tracking_combo, 1, 1)
        summary_layout.addWidget(self.foc_position_kp_label, 1, 2)
        summary_layout.addWidget(self.foc_position_kp_spin, 1, 3)
        summary_layout.addWidget(self.foc_target_angle_label, 2, 0)
        summary_layout.addWidget(self.foc_target_angle_widget, 2, 1)
        summary_layout.addWidget(self.foc_position_ki_label, 2, 2)
        summary_layout.addWidget(self.foc_position_ki_spin, 2, 3)
        summary_layout.addWidget(self.foc_jog_label, 3, 0)
        summary_layout.addWidget(self.foc_jog_button_widget, 3, 1)
        summary_layout.addWidget(self.foc_position_vff_gain_label, 3, 2)
        summary_layout.addWidget(self.foc_position_vff_gain_spin, 3, 3)
        summary_layout.addWidget(self.foc_angle_slider_label, 4, 0)
        summary_layout.addWidget(self.foc_angle_slider_widget, 4, 1, 1, 3)
        summary_layout.addWidget(self.foc_target_position_label, 5, 0)
        summary_layout.addWidget(self.foc_target_position_spin, 5, 1)
        summary_layout.addWidget(self.foc_position_vff_filter_label, 5, 2)
        summary_layout.addWidget(self.foc_position_vff_filter_spin, 5, 3)
        summary_layout.addWidget(QtWidgets.QLabel("Speed Kp"), 6, 0)
        summary_layout.addWidget(self.foc_speed_kp_spin, 6, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Speed Ki"), 6, 2)
        summary_layout.addWidget(self.foc_speed_ki_spin, 6, 3)
        summary_layout.addWidget(self.foc_debug_angle_label, 7, 0)
        summary_layout.addWidget(self.foc_debug_angle_value_label, 7, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Acceleration"), 8, 0)
        summary_layout.addWidget(self.foc_accel_spin, 8, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Deceleration"), 8, 2)
        summary_layout.addWidget(self.foc_decel_spin, 8, 3)
        summary_layout.addWidget(QtWidgets.QLabel("Status"), 9, 0)
        summary_layout.addWidget(self.foc_status_value_label, 9, 1)
        summary_layout.addWidget(QtWidgets.QLabel("Diag"), 9, 2)
        summary_layout.addWidget(self.foc_direction_test_status_label, 9, 3)
        summary_layout.addWidget(self.foc_live_summary_label, 10, 0, 1, 4)
        summary_layout.addWidget(self.start_foc_rotating_theta_test_button, 11, 0, 1, 2)
        summary_layout.addWidget(self.start_foc_rotating_theta_voltage_test_button, 11, 2, 1, 2)
        summary_layout.addWidget(self.start_foc_current_feedback_map_test_button, 12, 0, 1, 4)
        summary_layout.addWidget(self.start_foc_button, 13, 0, 1, 2)
        summary_layout.addWidget(self.stop_foc_button, 13, 2, 1, 2)

        self.foc_jog_minus_90_button.clicked.connect(lambda: self._apply_relative_angle_delta(-90.0))
        self.foc_jog_minus_10_button.clicked.connect(lambda: self._apply_relative_angle_delta(-10.0))
        self.foc_jog_plus_10_button.clicked.connect(lambda: self._apply_relative_angle_delta(10.0))
        self.foc_jog_plus_90_button.clicked.connect(lambda: self._apply_relative_angle_delta(90.0))
        self.foc_target_angle_spin.editingFinished.connect(
            self._sync_position_target_from_angle_input
        )
        self.foc_angle_slider.valueChanged.connect(self._handle_position_angle_slider_changed)
        self.foc_angle_slider.sliderReleased.connect(self._handle_position_angle_slider_released)
        self.foc_position_tracking_combo.currentIndexChanged.connect(self._handle_position_tracking_mode_changed)

        note_group = QtWidgets.QGroupBox("How It Works")
        note_layout = QtWidgets.QVBoxLayout(note_group)
        self.foc_mode_description_label = QtWidgets.QLabel()
        self.foc_mode_description_label.setWordWrap(True)
        self.foc_mode_description_label.setTextFormat(QtCore.Qt.TextFormat.RichText)
        self.foc_servo_note_label = QtWidgets.QLabel(
            "<b>FOC Runtime Notes</b>"
            "<ul style='margin-top:6px; margin-bottom:0px; margin-left:18px;'>"
            "<li><b>Servo ON</b> only arms the drive: current-sensor offset calibration, zero-current references, and PWM enable.</li>"
            "<li>Watch <b>Driver Monitor</b> until encoder alignment is done before starting motion.</li>"
            "<li><b>Start FOC</b> sends the selected mode, target, gains, and ramp limits to firmware.</li>"
            "<li>Runtime FOC uses <b>raw theta</b> with no extra electrical frame offset.</li>"
            "<li>Current-loop decoupling assumes <b>Ld = Lq = estimated MOTOR_INDUCTANCE</b>, matching the present motor setup.</li>"
            "</ul>"
        )
        self.foc_servo_note_label.setWordWrap(True)
        self.foc_servo_note_label.setTextFormat(QtCore.Qt.TextFormat.RichText)
        note_layout.addWidget(self.foc_mode_description_label)
        note_layout.addWidget(self.foc_servo_note_label)

        control_tabs = QtWidgets.QTabWidget()
        control_page = QtWidgets.QWidget()
        control_page_layout = QtWidgets.QVBoxLayout(control_page)
        control_page_layout.setContentsMargins(0, 0, 0, 0)
        control_page_layout.addWidget(summary_group)
        control_page_layout.addStretch(1)

        help_page = QtWidgets.QWidget()
        help_page_layout = QtWidgets.QVBoxLayout(help_page)
        help_page_layout.setContentsMargins(0, 0, 0, 0)
        help_page_layout.addWidget(note_group)
        help_page_layout.addStretch(1)

        control_tabs.addTab(control_page, "Control")
        control_tabs.addTab(help_page, "Help")

        layout.addWidget(control_tabs)
        layout.addStretch(1)
        self._update_foc_mode_ui()
        return widget

    def _build_test_mode_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        layout.addWidget(self._build_speed_test_group())
        layout.addWidget(self._build_position_test_group())
        note = QtWidgets.QLabel(
            "Use this tab for automated command sequences only. Configure gains, ramps, and core FOC limits on the FOC Control tab first, then return here to run repeatable speed or position validation profiles."
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch(1)
        return widget

    def _build_speed_test_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Speed Test Mode")
        self.speed_test_group = box
        layout = QtWidgets.QGridLayout(box)

        self.speed_test_profile_combo = QtWidgets.QComboBox()
        self.speed_test_profile_combo.addItem("Step Response", SPEED_TEST_PROFILE_STEP)
        self.speed_test_profile_combo.addItem("Reversing Test", SPEED_TEST_PROFILE_REVERSE)
        self.speed_test_profile_combo.addItem("Low-speed Stability", SPEED_TEST_PROFILE_LOW_SPEED)
        self.speed_test_profile_combo.addItem("Trapezoidal Profile", SPEED_TEST_PROFILE_TRAPEZOID)

        self.speed_test_stack = QtWidgets.QStackedWidget()

        step_page = QtWidgets.QWidget()
        step_layout = QtWidgets.QGridLayout(step_page)
        self.speed_test_step_target_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_step_target_spin.setRange(-2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM, 2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM)
        self.speed_test_step_target_spin.setDecimals(1)
        self.speed_test_step_target_spin.setSingleStep(50.0)
        self.speed_test_step_target_spin.setSuffix(" rpm")
        self.speed_test_step_target_spin.setValue(1000.0)
        self.speed_test_step_pre_delay_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_step_pre_delay_spin.setRange(0.0, 10000.0)
        self.speed_test_step_pre_delay_spin.setDecimals(0)
        self.speed_test_step_pre_delay_spin.setSingleStep(100.0)
        self.speed_test_step_pre_delay_spin.setSuffix(" ms")
        self.speed_test_step_pre_delay_spin.setValue(500.0)
        self.speed_test_step_run_time_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_step_run_time_spin.setRange(0.1, 120.0)
        self.speed_test_step_run_time_spin.setDecimals(2)
        self.speed_test_step_run_time_spin.setSingleStep(0.5)
        self.speed_test_step_run_time_spin.setSuffix(" s")
        self.speed_test_step_run_time_spin.setValue(3.0)
        step_layout.addWidget(QtWidgets.QLabel("Target"), 0, 0)
        step_layout.addWidget(self.speed_test_step_target_spin, 0, 1)
        step_layout.addWidget(QtWidgets.QLabel("Pre-step Wait"), 0, 2)
        step_layout.addWidget(self.speed_test_step_pre_delay_spin, 0, 3)
        step_layout.addWidget(QtWidgets.QLabel("Run Time"), 1, 0)
        step_layout.addWidget(self.speed_test_step_run_time_spin, 1, 1)
        step_layout.setColumnStretch(4, 1)
        self.speed_test_stack.addWidget(step_page)

        reverse_page = QtWidgets.QWidget()
        reverse_layout = QtWidgets.QGridLayout(reverse_page)
        self.speed_test_reverse_target_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_reverse_target_spin.setRange(1.0, 2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM)
        self.speed_test_reverse_target_spin.setDecimals(1)
        self.speed_test_reverse_target_spin.setSingleStep(50.0)
        self.speed_test_reverse_target_spin.setSuffix(" rpm")
        self.speed_test_reverse_target_spin.setValue(1000.0)
        self.speed_test_reverse_tolerance_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_reverse_tolerance_spin.setRange(0.5, 500.0)
        self.speed_test_reverse_tolerance_spin.setDecimals(1)
        self.speed_test_reverse_tolerance_spin.setSingleStep(1.0)
        self.speed_test_reverse_tolerance_spin.setSuffix(" rpm")
        self.speed_test_reverse_tolerance_spin.setValue(25.0)
        self.speed_test_reverse_hold_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_reverse_hold_spin.setRange(50.0, 10000.0)
        self.speed_test_reverse_hold_spin.setDecimals(0)
        self.speed_test_reverse_hold_spin.setSingleStep(100.0)
        self.speed_test_reverse_hold_spin.setSuffix(" ms")
        self.speed_test_reverse_hold_spin.setValue(800.0)
        self.speed_test_reverse_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_reverse_timeout_spin.setRange(0.5, 120.0)
        self.speed_test_reverse_timeout_spin.setDecimals(2)
        self.speed_test_reverse_timeout_spin.setSingleStep(0.5)
        self.speed_test_reverse_timeout_spin.setSuffix(" s")
        self.speed_test_reverse_timeout_spin.setValue(5.0)
        reverse_layout.addWidget(QtWidgets.QLabel("Magnitude"), 0, 0)
        reverse_layout.addWidget(self.speed_test_reverse_target_spin, 0, 1)
        reverse_layout.addWidget(QtWidgets.QLabel("Steady Tol"), 0, 2)
        reverse_layout.addWidget(self.speed_test_reverse_tolerance_spin, 0, 3)
        reverse_layout.addWidget(QtWidgets.QLabel("Hold"), 1, 0)
        reverse_layout.addWidget(self.speed_test_reverse_hold_spin, 1, 1)
        reverse_layout.addWidget(QtWidgets.QLabel("Timeout"), 1, 2)
        reverse_layout.addWidget(self.speed_test_reverse_timeout_spin, 1, 3)
        reverse_layout.setColumnStretch(4, 1)
        self.speed_test_stack.addWidget(reverse_page)

        low_speed_page = QtWidgets.QWidget()
        low_speed_layout = QtWidgets.QGridLayout(low_speed_page)
        self.speed_test_low_target_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_low_target_spin.setRange(-200.0, 200.0)
        self.speed_test_low_target_spin.setDecimals(1)
        self.speed_test_low_target_spin.setSingleStep(1.0)
        self.speed_test_low_target_spin.setSuffix(" rpm")
        self.speed_test_low_target_spin.setValue(15.0)
        self.speed_test_low_duration_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_low_duration_spin.setRange(1.0, 300.0)
        self.speed_test_low_duration_spin.setDecimals(1)
        self.speed_test_low_duration_spin.setSingleStep(1.0)
        self.speed_test_low_duration_spin.setSuffix(" s")
        self.speed_test_low_duration_spin.setValue(10.0)
        low_speed_layout.addWidget(QtWidgets.QLabel("Target"), 0, 0)
        low_speed_layout.addWidget(self.speed_test_low_target_spin, 0, 1)
        low_speed_layout.addWidget(QtWidgets.QLabel("Run Time"), 0, 2)
        low_speed_layout.addWidget(self.speed_test_low_duration_spin, 0, 3)
        low_speed_layout.setColumnStretch(4, 1)
        self.speed_test_stack.addWidget(low_speed_page)

        trapezoid_page = QtWidgets.QWidget()
        trapezoid_layout = QtWidgets.QGridLayout(trapezoid_page)
        self.speed_test_trapezoid_target_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_trapezoid_target_spin.setRange(-2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM, 2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM)
        self.speed_test_trapezoid_target_spin.setDecimals(1)
        self.speed_test_trapezoid_target_spin.setSingleStep(50.0)
        self.speed_test_trapezoid_target_spin.setSuffix(" rpm")
        self.speed_test_trapezoid_target_spin.setValue(1000.0)
        self.speed_test_trapezoid_lower_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_trapezoid_lower_spin.setRange(-2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM, 2.0 * DEFAULT_MOTOR_RATED_SPEED_RPM)
        self.speed_test_trapezoid_lower_spin.setDecimals(1)
        self.speed_test_trapezoid_lower_spin.setSingleStep(50.0)
        self.speed_test_trapezoid_lower_spin.setSuffix(" rpm")
        self.speed_test_trapezoid_lower_spin.setValue(0.0)
        self.speed_test_trapezoid_accel_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_trapezoid_accel_spin.setRange(0.1, 60.0)
        self.speed_test_trapezoid_accel_spin.setDecimals(2)
        self.speed_test_trapezoid_accel_spin.setSingleStep(0.1)
        self.speed_test_trapezoid_accel_spin.setSuffix(" s")
        self.speed_test_trapezoid_accel_spin.setValue(1.0)
        self.speed_test_trapezoid_hold_spin = QtWidgets.QDoubleSpinBox()
        self.speed_test_trapezoid_hold_spin.setRange(0.1, 300.0)
        self.speed_test_trapezoid_hold_spin.setDecimals(2)
        self.speed_test_trapezoid_hold_spin.setSingleStep(0.5)
        self.speed_test_trapezoid_hold_spin.setSuffix(" s")
        self.speed_test_trapezoid_hold_spin.setValue(3.0)
        trapezoid_layout.addWidget(QtWidgets.QLabel("Upper Speed"), 0, 0)
        trapezoid_layout.addWidget(self.speed_test_trapezoid_target_spin, 0, 1)
        trapezoid_layout.addWidget(QtWidgets.QLabel("Accel / Decel"), 0, 2)
        trapezoid_layout.addWidget(self.speed_test_trapezoid_accel_spin, 0, 3)
        trapezoid_layout.addWidget(QtWidgets.QLabel("Lower Speed"), 1, 0)
        trapezoid_layout.addWidget(self.speed_test_trapezoid_lower_spin, 1, 1)
        trapezoid_layout.addWidget(QtWidgets.QLabel("Hold"), 1, 2)
        trapezoid_layout.addWidget(self.speed_test_trapezoid_hold_spin, 1, 3)
        trapezoid_layout.setColumnStretch(4, 1)
        self.speed_test_stack.addWidget(trapezoid_page)

        self.speed_test_status_label = QtWidgets.QLabel("Idle")
        self.speed_test_status_label.setWordWrap(True)
        self.speed_test_status_label.setStyleSheet("font-weight: 600;")
        self.speed_test_run_button = QtWidgets.QPushButton("Run Test")
        self.speed_test_note_label = QtWidgets.QLabel(
            "Edit the profile here, then confirm. The GUI will copy the active FOC speed gains and ramp settings from the FOC Control tab into Driver Parameters, write them first, and only then start the automated test sequence."
        )
        self.speed_test_note_label.setWordWrap(True)

        layout.addWidget(QtWidgets.QLabel("Profile"), 0, 0)
        layout.addWidget(self.speed_test_profile_combo, 0, 1)
        layout.addWidget(self.speed_test_run_button, 0, 2)
        layout.addWidget(self.speed_test_stack, 1, 0, 1, 3)
        layout.addWidget(QtWidgets.QLabel("Status"), 2, 0)
        layout.addWidget(self.speed_test_status_label, 2, 1, 1, 2)
        layout.addWidget(self.speed_test_note_label, 3, 0, 1, 3)
        layout.setColumnStretch(1, 1)

        return box

    def _build_position_test_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Position Validation Suite")
        self.position_test_group = box
        layout = QtWidgets.QGridLayout(box)

        self.position_test_profile_combo = QtWidgets.QComboBox()
        self.position_test_profile_combo.addItem("Short-distance (Jitter / Sensitivity)", POSITION_TEST_PROFILE_SHORT)
        self.position_test_profile_combo.addItem("Long-distance (Thermal / Tracking)", POSITION_TEST_PROFILE_LONG)
        self.position_test_profile_combo.addItem("Back-and-forth (Backlash / Repeatability)", POSITION_TEST_PROFILE_BACKLASH)

        self.position_test_stack = QtWidgets.QStackedWidget()

        short_page = QtWidgets.QWidget()
        short_layout = QtWidgets.QGridLayout(short_page)
        self.position_test_short_step_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_short_step_spin.setRange(0.1, 45.0)
        self.position_test_short_step_spin.setDecimals(2)
        self.position_test_short_step_spin.setSingleStep(0.5)
        self.position_test_short_step_spin.setSuffix(" deg")
        self.position_test_short_step_spin.setValue(5.0)
        self.position_test_short_tol_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_short_tol_spin.setRange(0.01, 10.0)
        self.position_test_short_tol_spin.setDecimals(3)
        self.position_test_short_tol_spin.setSingleStep(0.05)
        self.position_test_short_tol_spin.setSuffix(" deg")
        self.position_test_short_tol_spin.setValue(0.25)
        self.position_test_short_hold_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_short_hold_spin.setRange(50.0, 5000.0)
        self.position_test_short_hold_spin.setDecimals(0)
        self.position_test_short_hold_spin.setSingleStep(50.0)
        self.position_test_short_hold_spin.setSuffix(" ms")
        self.position_test_short_hold_spin.setValue(200.0)
        self.position_test_short_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_short_timeout_spin.setRange(0.5, 60.0)
        self.position_test_short_timeout_spin.setDecimals(2)
        self.position_test_short_timeout_spin.setSingleStep(0.5)
        self.position_test_short_timeout_spin.setSuffix(" s")
        self.position_test_short_timeout_spin.setValue(5.0)
        short_layout.addWidget(QtWidgets.QLabel("Step Size"), 0, 0)
        short_layout.addWidget(self.position_test_short_step_spin, 0, 1)
        short_layout.addWidget(QtWidgets.QLabel("Settle Tol"), 0, 2)
        short_layout.addWidget(self.position_test_short_tol_spin, 0, 3)
        short_layout.addWidget(QtWidgets.QLabel("Settle Hold"), 1, 0)
        short_layout.addWidget(self.position_test_short_hold_spin, 1, 1)
        short_layout.addWidget(QtWidgets.QLabel("Timeout"), 1, 2)
        short_layout.addWidget(self.position_test_short_timeout_spin, 1, 3)
        short_layout.setColumnStretch(4, 1)
        self.position_test_stack.addWidget(short_page)

        long_page = QtWidgets.QWidget()
        long_layout = QtWidgets.QGridLayout(long_page)
        self.position_test_long_travel_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_long_travel_spin.setRange(-36000.0, 36000.0)
        self.position_test_long_travel_spin.setDecimals(1)
        self.position_test_long_travel_spin.setSingleStep(360.0)
        self.position_test_long_travel_spin.setSuffix(" deg")
        self.position_test_long_travel_spin.setValue(3600.0)
        self.position_test_long_tol_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_long_tol_spin.setRange(0.05, 10.0)
        self.position_test_long_tol_spin.setDecimals(3)
        self.position_test_long_tol_spin.setSingleStep(0.05)
        self.position_test_long_tol_spin.setSuffix(" deg")
        self.position_test_long_tol_spin.setValue(0.50)
        self.position_test_long_hold_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_long_hold_spin.setRange(50.0, 5000.0)
        self.position_test_long_hold_spin.setDecimals(0)
        self.position_test_long_hold_spin.setSingleStep(50.0)
        self.position_test_long_hold_spin.setSuffix(" ms")
        self.position_test_long_hold_spin.setValue(300.0)
        self.position_test_long_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_long_timeout_spin.setRange(1.0, 300.0)
        self.position_test_long_timeout_spin.setDecimals(1)
        self.position_test_long_timeout_spin.setSingleStep(1.0)
        self.position_test_long_timeout_spin.setSuffix(" s")
        self.position_test_long_timeout_spin.setValue(60.0)
        long_layout.addWidget(QtWidgets.QLabel("Travel"), 0, 0)
        long_layout.addWidget(self.position_test_long_travel_spin, 0, 1)
        long_layout.addWidget(QtWidgets.QLabel("Settle Tol"), 0, 2)
        long_layout.addWidget(self.position_test_long_tol_spin, 0, 3)
        long_layout.addWidget(QtWidgets.QLabel("Settle Hold"), 1, 0)
        long_layout.addWidget(self.position_test_long_hold_spin, 1, 1)
        long_layout.addWidget(QtWidgets.QLabel("Timeout"), 1, 2)
        long_layout.addWidget(self.position_test_long_timeout_spin, 1, 3)
        long_layout.setColumnStretch(4, 1)
        self.position_test_stack.addWidget(long_page)

        backlash_page = QtWidgets.QWidget()
        backlash_layout = QtWidgets.QGridLayout(backlash_page)
        self.position_test_backlash_a_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_backlash_a_spin.setRange(0.0, 360.0)
        self.position_test_backlash_a_spin.setDecimals(2)
        self.position_test_backlash_a_spin.setSingleStep(5.0)
        self.position_test_backlash_a_spin.setSuffix(" deg")
        self.position_test_backlash_a_spin.setValue(0.0)
        self.position_test_backlash_b_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_backlash_b_spin.setRange(0.0, 360.0)
        self.position_test_backlash_b_spin.setDecimals(2)
        self.position_test_backlash_b_spin.setSingleStep(5.0)
        self.position_test_backlash_b_spin.setSuffix(" deg")
        self.position_test_backlash_b_spin.setValue(180.0)
        self.position_test_backlash_dwell_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_backlash_dwell_spin.setRange(50.0, 10000.0)
        self.position_test_backlash_dwell_spin.setDecimals(0)
        self.position_test_backlash_dwell_spin.setSingleStep(50.0)
        self.position_test_backlash_dwell_spin.setSuffix(" ms")
        self.position_test_backlash_dwell_spin.setValue(500.0)
        self.position_test_backlash_cycles_spin = QtWidgets.QSpinBox()
        self.position_test_backlash_cycles_spin.setRange(1, 200)
        self.position_test_backlash_cycles_spin.setValue(5)
        self.position_test_backlash_tol_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_backlash_tol_spin.setRange(0.05, 10.0)
        self.position_test_backlash_tol_spin.setDecimals(3)
        self.position_test_backlash_tol_spin.setSingleStep(0.05)
        self.position_test_backlash_tol_spin.setSuffix(" deg")
        self.position_test_backlash_tol_spin.setValue(0.30)
        self.position_test_backlash_hold_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_backlash_hold_spin.setRange(50.0, 5000.0)
        self.position_test_backlash_hold_spin.setDecimals(0)
        self.position_test_backlash_hold_spin.setSingleStep(50.0)
        self.position_test_backlash_hold_spin.setSuffix(" ms")
        self.position_test_backlash_hold_spin.setValue(250.0)
        self.position_test_backlash_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.position_test_backlash_timeout_spin.setRange(0.5, 60.0)
        self.position_test_backlash_timeout_spin.setDecimals(2)
        self.position_test_backlash_timeout_spin.setSingleStep(0.5)
        self.position_test_backlash_timeout_spin.setSuffix(" s")
        self.position_test_backlash_timeout_spin.setValue(5.0)
        backlash_layout.addWidget(QtWidgets.QLabel("Position A"), 0, 0)
        backlash_layout.addWidget(self.position_test_backlash_a_spin, 0, 1)
        backlash_layout.addWidget(QtWidgets.QLabel("Position B"), 0, 2)
        backlash_layout.addWidget(self.position_test_backlash_b_spin, 0, 3)
        backlash_layout.addWidget(QtWidgets.QLabel("Dwell"), 1, 0)
        backlash_layout.addWidget(self.position_test_backlash_dwell_spin, 1, 1)
        backlash_layout.addWidget(QtWidgets.QLabel("Cycles"), 1, 2)
        backlash_layout.addWidget(self.position_test_backlash_cycles_spin, 1, 3)
        backlash_layout.addWidget(QtWidgets.QLabel("Settle Tol"), 2, 0)
        backlash_layout.addWidget(self.position_test_backlash_tol_spin, 2, 1)
        backlash_layout.addWidget(QtWidgets.QLabel("Settle Hold"), 2, 2)
        backlash_layout.addWidget(self.position_test_backlash_hold_spin, 2, 3)
        backlash_layout.addWidget(QtWidgets.QLabel("Timeout"), 3, 0)
        backlash_layout.addWidget(self.position_test_backlash_timeout_spin, 3, 1)
        backlash_layout.setColumnStretch(4, 1)
        self.position_test_stack.addWidget(backlash_page)

        self.position_test_status_label = QtWidgets.QLabel("Idle")
        self.position_test_status_label.setWordWrap(True)
        self.position_test_status_label.setStyleSheet("font-weight: 600;")
        self.position_test_run_button = QtWidgets.QPushButton("Run Validation")
        self.position_test_note_label = QtWidgets.QLabel(
            "Edit the profile here, then confirm. The GUI will copy the current FOC position and speed gains, ramps, speed limit, and tracking mode from the FOC Control tab into Driver Parameters before it starts the automated validation sequence."
        )
        self.position_test_note_label.setWordWrap(True)

        layout.addWidget(QtWidgets.QLabel("Profile"), 0, 0)
        layout.addWidget(self.position_test_profile_combo, 0, 1)
        layout.addWidget(self.position_test_run_button, 0, 2)
        layout.addWidget(self.position_test_stack, 1, 0, 1, 3)
        layout.addWidget(QtWidgets.QLabel("Status"), 2, 0)
        layout.addWidget(self.position_test_status_label, 2, 1, 1, 2)
        layout.addWidget(self.position_test_note_label, 3, 0, 1, 3)
        layout.setColumnStretch(1, 1)

        return box

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
        self.read_all_params_button = QtWidgets.QPushButton("Read Driver + Motor Params")
        self.write_driver_button = QtWidgets.QPushButton("Write Driver Params")
        self.write_motor_button = QtWidgets.QPushButton("Write Motor Params")
        self.parameter_status_label = QtWidgets.QLabel("Parameter tools are ready.")
        toolbar.addWidget(self.read_all_params_button)
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
            name_item = QtWidgets.QTableWidgetItem(PARAMETER_NAME_LABELS.get(name, name))
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
        self.foc_mode_combo.currentIndexChanged.connect(self._update_foc_mode_ui)
        self.foc_target_speed_spin.valueChanged.connect(self._on_foc_target_speed_value_changed)
        self.speed_test_profile_combo.currentIndexChanged.connect(self._update_speed_test_profile_ui)
        self.speed_test_run_button.clicked.connect(self._toggle_speed_test_run)
        self.position_test_profile_combo.currentIndexChanged.connect(self._update_position_test_profile_ui)
        self.position_test_run_button.clicked.connect(self._toggle_position_test_run)
        self.start_foc_rotating_theta_test_button.clicked.connect(self._start_foc_rotating_theta_test)
        self.start_foc_rotating_theta_voltage_test_button.clicked.connect(self._start_foc_rotating_theta_voltage_test)
        self.start_foc_current_feedback_map_test_button.clicked.connect(self._start_foc_current_feedback_map_test)
        self.start_foc_button.clicked.connect(self._start_foc_control)
        self.stop_foc_button.clicked.connect(self._stop_foc_control)
        self.vf_toggle_button.clicked.connect(self._toggle_open_loop_vf)
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
        self.read_all_params_button.clicked.connect(self._read_all_parameters)
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
        self.uerror_start_button.clicked.connect(self._start_uerror_characterization)
        self.uerror_stop_button.clicked.connect(self._stop_uerror_characterization)
        self.uerror_clear_button.clicked.connect(self._clear_uerror_characterization)
        self.uerror_apply_runtime_button.clicked.connect(self._apply_uerror_runtime_lut)
        self.uerror_save_flash_button.clicked.connect(self._save_uerror_lut_to_flash)
        self.uerror_rs_actual_spin.valueChanged.connect(self._refresh_uerror_plots)
        self.uerror_enforce_odd_checkbox.toggled.connect(self._refresh_uerror_plots)
        self.uerror_smooth_checkbox.toggled.connect(self._refresh_uerror_plots)

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

    def _open_report_editor_for_chart(self, chart_widget) -> None:
        if not hasattr(chart_widget, "build_report_capture"):
            return
        capture = chart_widget.build_report_capture(dpi=300)
        dialog = ReportEditorDialog(capture, self)
        dialog.exec()

    def _append_report_controls(
        self,
        toolbar: QtWidgets.QHBoxLayout,
        chart_widgets: list[QtWidgets.QWidget],
        *,
        export_targets: list[tuple[str, QtWidgets.QWidget]] | None = None,
    ) -> None:
        if not chart_widgets:
            return

        report_toggle = QtWidgets.QCheckBox("Report Mode")
        report_font_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        report_font_slider.setRange(12, 24)
        report_font_slider.setValue(14)
        report_font_slider.setFixedWidth(110)
        report_font_label = QtWidgets.QLabel("14 pt")
        toolbar.addWidget(report_toggle)
        toolbar.addWidget(QtWidgets.QLabel("Font"))
        toolbar.addWidget(report_font_slider)
        toolbar.addWidget(report_font_label)

        def _apply_report_mode(enabled: bool) -> None:
            for widget in chart_widgets:
                if hasattr(widget, "set_report_mode"):
                    widget.set_report_mode(enabled)

        def _apply_report_font(value: int) -> None:
            report_font_label.setText(f"{int(value)} pt")
            for widget in chart_widgets:
                if hasattr(widget, "set_report_font_point_size"):
                    widget.set_report_font_point_size(int(value))

        report_toggle.toggled.connect(_apply_report_mode)
        report_font_slider.valueChanged.connect(_apply_report_font)

        targets = export_targets or [("Capture & Edit", chart_widgets[0])]
        for button_label, widget in targets:
            export_button = QtWidgets.QPushButton(button_label)
            export_button.clicked.connect(lambda _=False, target=widget: self._open_report_editor_for_chart(target))
            toolbar.addWidget(export_button)

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

    def _motor_max_speed_rpm(self) -> float:
        if not hasattr(self, "motor_table"):
            return DEFAULT_MOTOR_RATED_SPEED_RPM
        motor_max_speed = self._table_float_value(
            self.motor_table,
            MOTOR_PARAM_MAXIMUM_SPEED,
            DEFAULT_MOTOR_RATED_SPEED_RPM,
        )
        if motor_max_speed <= 0.0:
            motor_max_speed = DEFAULT_MOTOR_RATED_SPEED_RPM
        return float(motor_max_speed)

    def _driver_max_speed_rpm(self) -> float:
        if not hasattr(self, "driver_table"):
            return DEFAULT_MOTOR_RATED_SPEED_RPM
        driver_max_speed = self._table_float_value(
            self.driver_table,
            DRIVER_PARAM_MAXIMUM_SPEED,
            DEFAULT_MOTOR_RATED_SPEED_RPM,
        )
        if driver_max_speed <= 0.0:
            driver_max_speed = self._motor_max_speed_rpm()
        return float(driver_max_speed)

    def _speed_mode_limit_rpm(self) -> float:
        limit_rpm = max(
            abs(float(self._foc_speed_target_rpm)),
            self._driver_max_speed_rpm(),
            self._motor_max_speed_rpm(),
        )
        if limit_rpm <= 0.0:
            limit_rpm = DEFAULT_MOTOR_RATED_SPEED_RPM
        return float(limit_rpm)

    def _encoder_resolution_counts(self) -> float:
        if not hasattr(self, "motor_table"):
            return float(1 << 20)
        encoder_resolution = self._table_float_value(
            self.motor_table,
            MOTOR_PARAM_ENCODER_RESOLUTION,
            float(1 << 20),
        )
        if encoder_resolution <= 1.0:
            encoder_resolution = float(1 << 20)
        return float(encoder_resolution)

    def _normalize_single_turn_counts(self, counts: float) -> float:
        encoder_resolution = self._encoder_resolution_counts()
        if encoder_resolution <= 1.0:
            return float(counts)
        counts = math.fmod(float(counts), encoder_resolution)
        if counts < 0.0:
            counts += encoder_resolution
        return counts

    def _position_tracking_mode(self) -> int:
        if not hasattr(self, "foc_position_tracking_combo"):
            return POSITION_TRACKING_MODE_SINGLE_TURN
        tracking_mode = int(self.foc_position_tracking_combo.currentData())
        if tracking_mode not in (
            POSITION_TRACKING_MODE_SINGLE_TURN,
            POSITION_TRACKING_MODE_MULTI_TURN,
        ):
            tracking_mode = POSITION_TRACKING_MODE_SINGLE_TURN
        return tracking_mode

    def _counts_to_single_turn_degrees(self, counts: float) -> float:
        encoder_resolution = self._encoder_resolution_counts()
        if encoder_resolution <= 1.0:
            return 0.0
        return (self._normalize_single_turn_counts(counts) * 360.0) / encoder_resolution

    def _counts_to_accumulated_degrees(self, counts: float) -> float:
        encoder_resolution = self._encoder_resolution_counts()
        if encoder_resolution <= 1.0:
            return 0.0
        return (float(counts) * 360.0) / encoder_resolution

    def _counts_to_turns(self, counts: float) -> float:
        encoder_resolution = self._encoder_resolution_counts()
        if encoder_resolution <= 1.0:
            return 0.0
        return float(counts) / encoder_resolution

    def _counts_to_position_mode_degrees(
        self,
        counts: float,
        tracking_mode: int | None = None,
    ) -> float:
        mode = self._position_tracking_mode() if tracking_mode is None else int(tracking_mode)
        if mode == POSITION_TRACKING_MODE_MULTI_TURN:
            return self._counts_to_accumulated_degrees(counts)
        return self._counts_to_single_turn_degrees(counts)

    def _counts_to_degrees(self, counts: float) -> float:
        return self._counts_to_single_turn_degrees(counts)

    def _degrees_to_counts(
        self,
        degrees: float,
        tracking_mode: int | None = None,
    ) -> float:
        encoder_resolution = self._encoder_resolution_counts()
        if encoder_resolution <= 1.0:
            return 0.0
        mode = self._position_tracking_mode() if tracking_mode is None else int(tracking_mode)
        if mode == POSITION_TRACKING_MODE_MULTI_TURN:
            return (float(degrees) / 360.0) * encoder_resolution
        normalized_degrees = float(degrees) % 360.0
        return (normalized_degrees / 360.0) * encoder_resolution

    def _position_angle_range_deg(self, tracking_mode: int | None = None) -> tuple[float, float]:
        mode = self._position_tracking_mode() if tracking_mode is None else int(tracking_mode)
        if mode == POSITION_TRACKING_MODE_MULTI_TURN:
            encoder_resolution = self._encoder_resolution_counts()
            if encoder_resolution > 1.0:
                max_degrees = self._counts_to_accumulated_degrees(
                    POSITION_TARGET_COUNT_GUI_LIMIT
                )
                return (-max_degrees, max_degrees)
            return (-360000.0, 360000.0)
        return (0.0, 360.0)

    def _position_tracking_mode_text(self, tracking_mode: int | None = None) -> str:
        mode = self._position_tracking_mode() if tracking_mode is None else int(tracking_mode)
        if mode == POSITION_TRACKING_MODE_MULTI_TURN:
            return "Multi Turn"
        return "Single Turn"

    def _counts_to_motion_display_degrees(self, counts: float) -> float:
        if (
            self._current_foc_mode() == POSITION_CONTROL_MODE
            and self._position_tracking_mode() == POSITION_TRACKING_MODE_MULTI_TURN
        ):
            return self._counts_to_accumulated_degrees(counts)
        return self._counts_to_single_turn_degrees(counts)

    def _wrap_position_error_counts(self, error_counts: float) -> float:
        encoder_resolution = self._encoder_resolution_counts()
        if encoder_resolution <= 1.0:
            return float(error_counts)
        wrapped_error = math.fmod(float(error_counts), encoder_resolution)
        half_encoder_resolution = 0.5 * encoder_resolution
        if wrapped_error > half_encoder_resolution:
            wrapped_error -= encoder_resolution
        if wrapped_error < -half_encoder_resolution:
            wrapped_error += encoder_resolution
        return wrapped_error

    def _display_position_error_counts(
        self,
        target_counts: float,
        actual_counts: float,
        tracking_mode: int | None = None,
    ) -> float:
        mode = self._position_tracking_mode() if tracking_mode is None else int(tracking_mode)
        raw_error_counts = float(target_counts) - float(actual_counts)
        if mode == POSITION_TRACKING_MODE_MULTI_TURN:
            return raw_error_counts
        return self._wrap_position_error_counts(raw_error_counts)

    def _display_position_error_degrees(
        self,
        target_counts: float,
        actual_counts: float,
        tracking_mode: int | None = None,
    ) -> float:
        error_counts = self._display_position_error_counts(
            target_counts,
            actual_counts,
            tracking_mode,
        )
        return self._counts_to_accumulated_degrees(error_counts)

    def _current_position_validation_error_degrees(
        self,
        target_counts: float,
        actual_counts: float,
        tracking_mode: int | None = None,
    ) -> float:
        session = self._position_test_session
        if session is None:
            return 0.0
        if session.profile_key == POSITION_TEST_PROFILE_BACKLASH and not session.endpoint_baselines:
            return 0.0
        return float(session.validation_error_deg)

    def _set_foc_target_position_counts(self, counts: float) -> None:
        tracking_mode = self._position_tracking_mode()
        normalized_counts = (
            self._normalize_single_turn_counts(counts)
            if tracking_mode == POSITION_TRACKING_MODE_SINGLE_TURN
            else float(counts)
        )
        self.foc_target_position_spin.blockSignals(True)
        self.foc_target_position_spin.setValue(normalized_counts)
        self.foc_target_position_spin.blockSignals(False)
        self.foc_target_counts_hint_label.setText(f"{normalized_counts:.1f} cnt")
        target_degrees = self._counts_to_position_mode_degrees(normalized_counts, tracking_mode)
        self.foc_target_angle_spin.blockSignals(True)
        self.foc_target_angle_spin.setValue(target_degrees)
        self.foc_target_angle_spin.blockSignals(False)
        if tracking_mode == POSITION_TRACKING_MODE_SINGLE_TURN:
            slider_value = int(round(self._counts_to_single_turn_degrees(normalized_counts) * 10.0))
            self.foc_angle_slider.blockSignals(True)
            self.foc_angle_slider.setValue(slider_value)
            self.foc_angle_slider.blockSignals(False)
            self.foc_angle_slider_value_label.setText(f"{slider_value / 10.0:.1f} deg")
        else:
            self.foc_angle_slider_value_label.setText("Disabled in multi-turn")

    def _handle_position_tracking_mode_changed(self) -> None:
        current_counts = float(self.foc_target_position_spin.value())
        tracking_mode = self._position_tracking_mode()
        lower_deg, upper_deg = self._position_angle_range_deg(tracking_mode)
        self.foc_target_angle_spin.blockSignals(True)
        self.foc_target_angle_spin.setRange(lower_deg, upper_deg)
        self.foc_target_angle_spin.setSingleStep(
            10.0 if tracking_mode == POSITION_TRACKING_MODE_MULTI_TURN else 1.0
        )
        self.foc_target_angle_spin.blockSignals(False)
        self._set_foc_target_position_counts(current_counts)
        self._update_foc_mode_ui()

    def _sync_position_target_from_angle_input(self) -> None:
        self._set_foc_target_position_counts(
            self._degrees_to_counts(self.foc_target_angle_spin.value())
        )

    def _apply_relative_angle_delta(self, delta_degrees: float) -> None:
        base_counts = float(self.foc_target_position_spin.value())
        if self._latest_monitor_snapshot is not None:
            base_counts = float(getattr(self._latest_monitor_snapshot, "act_position", base_counts))
        current_target_degrees = self._counts_to_position_mode_degrees(base_counts)
        self._set_foc_target_position_counts(
            self._degrees_to_counts(current_target_degrees + float(delta_degrees))
        )

    def _handle_position_angle_slider_changed(self, slider_value: int) -> None:
        self.foc_angle_slider_value_label.setText(f"{slider_value / 10.0:.1f} deg")

    def _handle_position_angle_slider_released(self) -> None:
        if self._position_tracking_mode() != POSITION_TRACKING_MODE_SINGLE_TURN:
            return
        self._set_foc_target_position_counts(
            self._degrees_to_counts(self.foc_angle_slider.value() / 10.0)
        )

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
        self.foc_position_ki_spin.setValue(
            self._table_float_value(self.driver_table, DRIVER_PARAM_POSITION_I_GAIN, 0.50)
        )
        tracking_mode = int(
            round(
                self._table_float_value(
                    self.driver_table,
                    DRIVER_PARAM_POSITION_TRACKING_MODE,
                    float(POSITION_TRACKING_MODE_SINGLE_TURN),
                )
            )
        )
        if tracking_mode not in (
            POSITION_TRACKING_MODE_SINGLE_TURN,
            POSITION_TRACKING_MODE_MULTI_TURN,
        ):
            tracking_mode = POSITION_TRACKING_MODE_SINGLE_TURN
        combo_index = self.foc_position_tracking_combo.findData(tracking_mode)
        if combo_index >= 0:
            self.foc_position_tracking_combo.blockSignals(True)
            self.foc_position_tracking_combo.setCurrentIndex(combo_index)
            self.foc_position_tracking_combo.blockSignals(False)
        self.foc_position_vff_gain_spin.setValue(
            self._table_float_value(self.driver_table, DRIVER_PARAM_POSITION_FF_GAIN, 0.0)
        )
        self.foc_position_vff_filter_spin.setValue(
            self._table_float_value(self.driver_table, DRIVER_PARAM_POSITION_FF_FILTER, 50.0)
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
        self._set_foc_target_position_counts(self.foc_target_position_spin.value())
        self._update_foc_mode_ui()

    def _sync_foc_controls_to_driver_table(self) -> None:
        if not hasattr(self, "driver_table"):
            return
        mode = self._current_foc_mode()
        if mode == POSITION_CONTROL_MODE:
            maximum_speed_rpm = abs(float(self._foc_position_speed_limit_rpm))
        else:
            maximum_speed_rpm = self._speed_mode_limit_rpm()
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_CONTROL_MODE,
            float(mode),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_POSITION_P_GAIN,
            float(self.foc_position_kp_spin.value()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_POSITION_I_GAIN,
            float(self.foc_position_ki_spin.value()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_POSITION_TRACKING_MODE,
            float(self._position_tracking_mode()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_POSITION_FF_GAIN,
            float(self.foc_position_vff_gain_spin.value()),
        )
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_POSITION_FF_FILTER,
            float(self.foc_position_vff_filter_spin.value()),
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
        self._set_table_float_value(
            self.driver_table,
            DRIVER_PARAM_MAXIMUM_SPEED,
            maximum_speed_rpm,
        )

    def _update_speed_test_profile_ui(self) -> None:
        profile_key = self._speed_test_profile_key()
        page_index = {
            SPEED_TEST_PROFILE_STEP: 0,
            SPEED_TEST_PROFILE_REVERSE: 1,
            SPEED_TEST_PROFILE_LOW_SPEED: 2,
            SPEED_TEST_PROFILE_TRAPEZOID: 3,
        }.get(profile_key, 0)
        self.speed_test_stack.setCurrentIndex(page_index)

    def _speed_test_profile_key(self) -> str:
        return str(self.speed_test_profile_combo.currentData() or SPEED_TEST_PROFILE_STEP)

    def _communication_idle(self) -> bool:
        return bool(self._connected and (not self._awaiting_ack) and (not self._pending_frames))

    def _queue_driver_parameter_write_chunks(
        self,
        values: list[float],
        description_prefix: str = "Write Driver Parameters",
    ) -> None:
        chunks = build_parameter_write_chunks(values, chunk_size=16)
        total_chunks = len(chunks)
        for index, payload in enumerate(chunks, start=1):
            self._enqueue_command(
                Command.CMD_WRITE_DRIVER,
                payload,
                f"{description_prefix} chunk {index}/{total_chunks}",
            )

    def _build_speed_start_payload(
        self,
        target_rpm: float,
        *,
        bypass_speed_ramp: bool = False,
    ) -> tuple[bytes, float]:
        speed_limit_rpm = max(
            abs(float(target_rpm)),
            self._driver_max_speed_rpm(),
            self._motor_max_speed_rpm(),
        )
        if speed_limit_rpm <= 0.0:
            speed_limit_rpm = DEFAULT_MOTOR_RATED_SPEED_RPM
        accel_ms = float(self.foc_accel_spin.value())
        decel_ms = float(self.foc_decel_spin.value())
        if bypass_speed_ramp:
            accel_ms = SPEED_TEST_STEP_RAMP_BYPASS_SENTINEL_MS
            decel_ms = SPEED_TEST_STEP_RAMP_BYPASS_SENTINEL_MS
        payload = struct.pack(
            "<6fBB",
            float(target_rpm),
            float(self.foc_speed_kp_spin.value()),
            float(self.foc_speed_ki_spin.value()),
            accel_ms,
            decel_ms,
            float(speed_limit_rpm),
            ID_SQUARE_ANGLE_TEST_NONE,
            0,
        )
        return payload, float(speed_limit_rpm)

    def _apply_speed_target_to_ui(self, target_rpm: float) -> None:
        self.foc_target_speed_spin.blockSignals(True)
        self.foc_target_speed_spin.setValue(float(target_rpm))
        self.foc_target_speed_spin.blockSignals(False)
        self._on_foc_target_speed_value_changed(float(target_rpm))

    def _apply_speed_ramp_to_ui(self, accel_ms: float, decel_ms: float) -> None:
        self.foc_accel_spin.blockSignals(True)
        self.foc_accel_spin.setValue(float(accel_ms))
        self.foc_accel_spin.blockSignals(False)
        self.foc_decel_spin.blockSignals(True)
        self.foc_decel_spin.setValue(float(decel_ms))
        self.foc_decel_spin.blockSignals(False)

    def _enqueue_speed_start_command(
        self,
        target_rpm: float,
        description: str,
        quiet: bool = False,
        request_monitor: bool = False,
        bypass_speed_ramp: bool = False,
    ) -> None:
        self._apply_speed_target_to_ui(target_rpm)
        self._sync_foc_controls_to_driver_table()
        payload, _ = self._build_speed_start_payload(
            target_rpm,
            bypass_speed_ramp=bypass_speed_ramp,
        )
        self._enqueue_command(Command.CMD_START_SPEEDCONTROL, payload, description, quiet=quiet)
        if request_monitor:
            self._request_monitor_once()

    def _enqueue_speed_stop_command(
        self,
        description: str = "Stop FOC",
        quiet: bool = False,
        request_monitor: bool = False,
    ) -> None:
        self._enqueue_command(Command.CMD_STOP_SPEEDCONTROL, b"", description, quiet=quiet)
        if request_monitor:
            self._request_monitor_once()

    def _apply_position_tracking_mode_to_ui(self, tracking_mode: int) -> None:
        combo_index = self.foc_position_tracking_combo.findData(int(tracking_mode))
        if combo_index >= 0 and combo_index != self.foc_position_tracking_combo.currentIndex():
            self.foc_position_tracking_combo.setCurrentIndex(combo_index)

    def _apply_position_target_to_ui(self, target_counts: float, tracking_mode: int) -> None:
        self._apply_position_tracking_mode_to_ui(tracking_mode)
        self._set_foc_target_position_counts(float(target_counts))

    def _build_position_start_payload(
        self,
        target_counts: float,
        tracking_mode: int,
    ) -> bytes:
        return struct.pack(
            "<10fBBB",
            float(target_counts),
            float(abs(self.foc_target_speed_spin.value())),
            float(self.foc_position_kp_spin.value()),
            float(self.foc_position_ki_spin.value()),
            float(self.foc_position_vff_gain_spin.value()),
            float(self.foc_position_vff_filter_spin.value()),
            float(self.foc_speed_kp_spin.value()),
            float(self.foc_speed_ki_spin.value()),
            float(self.foc_accel_spin.value()),
            float(self.foc_decel_spin.value()),
            ID_SQUARE_ANGLE_TEST_NONE,
            0,
            int(tracking_mode),
        )

    def _enqueue_position_start_command(
        self,
        target_counts: float,
        tracking_mode: int,
        description: str,
        quiet: bool = False,
        request_monitor: bool = False,
    ) -> None:
        self._apply_position_target_to_ui(target_counts, tracking_mode)
        self._sync_foc_controls_to_driver_table()
        payload = self._build_position_start_payload(target_counts, tracking_mode)
        self._enqueue_command(
            Command.CMD_START_POSITIONCONTROL,
            payload,
            description,
            quiet=quiet,
        )
        if request_monitor:
            self._request_monitor_once()

    def _enqueue_position_stop_command(
        self,
        description: str = "Stop FOC",
        quiet: bool = False,
        request_monitor: bool = False,
    ) -> None:
        self._enqueue_command(Command.CMD_STOP_POSITIONCONTROL, b"", description, quiet=quiet)
        if request_monitor:
            self._request_monitor_once()

    def _speed_test_summary_lines(self, profile_key: str) -> list[str]:
        if profile_key == SPEED_TEST_PROFILE_REVERSE:
            return [
                f"Profile: Reversing Test",
                f"Magnitude: {self.speed_test_reverse_target_spin.value():.1f} rpm",
                f"Steady tolerance: {self.speed_test_reverse_tolerance_spin.value():.1f} rpm",
                f"Steady hold: {self.speed_test_reverse_hold_spin.value():.0f} ms",
                f"Timeout per direction: {self.speed_test_reverse_timeout_spin.value():.2f} s",
            ]
        if profile_key == SPEED_TEST_PROFILE_LOW_SPEED:
            return [
                f"Profile: Low-speed Stability",
                f"Target: {self.speed_test_low_target_spin.value():.1f} rpm",
                f"Run time: {self.speed_test_low_duration_spin.value():.1f} s",
            ]
        if profile_key == SPEED_TEST_PROFILE_TRAPEZOID:
            return [
                f"Profile: Trapezoidal Profile",
                f"Upper Speed: {self.speed_test_trapezoid_target_spin.value():.1f} rpm",
                f"Lower Speed: {self.speed_test_trapezoid_lower_spin.value():.1f} rpm",
                f"Accel / Decel: {self.speed_test_trapezoid_accel_spin.value():.2f} s",
                f"Hold: {self.speed_test_trapezoid_hold_spin.value():.2f} s",
                "Behavior: Repeats until Cancel",
            ]
        return [
            f"Profile: Step Response",
            f"Target: {self.speed_test_step_target_spin.value():.1f} rpm",
            f"Pre-step wait: {self.speed_test_step_pre_delay_spin.value():.0f} ms",
            f"Run time: {self.speed_test_step_run_time_spin.value():.2f} s",
            "Ramp: bypassed during the commanded step",
        ]

    def _update_position_test_profile_ui(self) -> None:
        if not hasattr(self, "position_test_profile_combo"):
            return
        self.position_test_stack.setCurrentIndex(self.position_test_profile_combo.currentIndex())

    def _position_test_profile_key(self) -> str:
        return str(self.position_test_profile_combo.currentData() or POSITION_TEST_PROFILE_SHORT)

    def _position_test_summary_lines(self, profile_key: str) -> list[str]:
        if profile_key == POSITION_TEST_PROFILE_LONG:
            return [
                "Profile: Long-distance (Thermal / Tracking)",
                f"Travel: {self.position_test_long_travel_spin.value():.1f} deg",
                f"Settle Tol: {self.position_test_long_tol_spin.value():.3f} deg",
                f"Settle Hold: {self.position_test_long_hold_spin.value():.0f} ms",
                f"Timeout: {self.position_test_long_timeout_spin.value():.1f} s",
                "Tracking: Multi Turn",
                "Logs: turn-by-turn accumulated error",
            ]
        if profile_key == POSITION_TEST_PROFILE_BACKLASH:
            return [
                "Profile: Back-and-forth (Backlash / Repeatability)",
                f"Position A: {self.position_test_backlash_a_spin.value():.2f} deg",
                f"Position B: {self.position_test_backlash_b_spin.value():.2f} deg",
                f"Dwell: {self.position_test_backlash_dwell_spin.value():.0f} ms",
                f"Cycles: {self.position_test_backlash_cycles_spin.value()}",
                f"Settle Tol: {self.position_test_backlash_tol_spin.value():.3f} deg",
                f"Settle Hold: {self.position_test_backlash_hold_spin.value():.0f} ms",
                f"Timeout: {self.position_test_backlash_timeout_spin.value():.2f} s",
                "Tracking: Single Turn",
            ]
        return [
            "Profile: Short-distance (Jitter / Sensitivity)",
            f"Step Size: {self.position_test_short_step_spin.value():.2f} deg",
            f"Settle Tol: {self.position_test_short_tol_spin.value():.3f} deg",
            f"Settle Hold: {self.position_test_short_hold_spin.value():.0f} ms",
            f"Timeout: {self.position_test_short_timeout_spin.value():.2f} s",
            "Tracking: Single Turn",
        ]

    def _build_position_test_sequence(self, profile_key: str, snapshot) -> list[dict[str, object]]:
        current_counts = float(getattr(snapshot, "act_position", 0.0)) if snapshot is not None else 0.0
        current_single_deg = self._counts_to_single_turn_degrees(current_counts)
        if profile_key == POSITION_TEST_PROFILE_LONG:
            travel_deg = float(self.position_test_long_travel_spin.value())
            tolerance_deg = float(self.position_test_long_tol_spin.value())
            hold_s = float(self.position_test_long_hold_spin.value()) / 1000.0
            timeout_s = float(self.position_test_long_timeout_spin.value())
            start_counts = float(current_counts)
            target_counts = start_counts + self._degrees_to_counts(
                travel_deg,
                POSITION_TRACKING_MODE_MULTI_TURN,
            )
            start_deg = self._counts_to_accumulated_degrees(start_counts)
            target_deg = self._counts_to_accumulated_degrees(target_counts)
            return [
                {
                    "type": "prepare_position",
                    "target_counts": start_counts,
                    "tracking_mode": POSITION_TRACKING_MODE_MULTI_TURN,
                    "status": f"Step 1/6: Priming at {start_deg:.2f} deg...",
                },
                {
                    "type": "start_foc_position",
                    "target_counts": start_counts,
                    "tracking_mode": POSITION_TRACKING_MODE_MULTI_TURN,
                    "status": "Step 2/6: Starting FOC in Position Mode...",
                },
                {"type": "wait", "duration_s": 0.10, "status": "Step 3/6: Waiting for the drive to arm..."},
                {
                    "type": "set_position",
                    "target_counts": target_counts,
                    "tracking_mode": POSITION_TRACKING_MODE_MULTI_TURN,
                    "status": f"Step 4/6: Commanding long-distance move to {target_deg:.1f} deg...",
                },
                {
                    "type": "wait_position_steady",
                    "target_counts": target_counts,
                    "tracking_mode": POSITION_TRACKING_MODE_MULTI_TURN,
                    "tolerance_deg": tolerance_deg,
                    "hold_s": hold_s,
                    "timeout_s": timeout_s,
                    "start_counts": start_counts,
                    "status": "Step 5/6: Tracking the long-distance move...",
                    "log_turns": True,
                },
                {"type": "stop_foc_position", "status": "Step 6/6: Stopping FOC..."},
            ]

        if profile_key == POSITION_TEST_PROFILE_BACKLASH:
            position_a_deg = float(self.position_test_backlash_a_spin.value())
            position_b_deg = float(self.position_test_backlash_b_spin.value())
            dwell_s = float(self.position_test_backlash_dwell_spin.value()) / 1000.0
            cycles = int(self.position_test_backlash_cycles_spin.value())
            tolerance_deg = float(self.position_test_backlash_tol_spin.value())
            hold_s = float(self.position_test_backlash_hold_spin.value()) / 1000.0
            timeout_s = float(self.position_test_backlash_timeout_spin.value())
            position_a_counts = self._degrees_to_counts(position_a_deg, POSITION_TRACKING_MODE_SINGLE_TURN)
            position_b_counts = self._degrees_to_counts(position_b_deg, POSITION_TRACKING_MODE_SINGLE_TURN)
            sequence: list[dict[str, object]] = [
                {
                    "type": "prepare_position",
                    "target_counts": position_a_counts,
                    "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                    "status": f"Step 1/? Priming endpoint A at {position_a_deg:.2f} deg...",
                },
                {
                    "type": "start_foc_position",
                    "target_counts": position_a_counts,
                    "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                    "status": "Step 2/? Starting FOC in Position Mode...",
                },
                {"type": "wait", "duration_s": 0.10, "status": "Step 3/? Waiting for the drive to arm..."},
                {
                    "type": "wait_position_steady",
                    "target_counts": position_a_counts,
                    "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                    "tolerance_deg": tolerance_deg,
                    "hold_s": hold_s,
                    "timeout_s": timeout_s,
                    "endpoint_key": "A",
                    "status": "Step 4/? Settling at endpoint A...",
                },
                {"type": "wait", "duration_s": dwell_s, "status": f"Dwelling at endpoint A for {dwell_s:.2f} s..."},
            ]
            for cycle_index in range(cycles):
                sequence.extend(
                    [
                        {
                            "type": "set_position",
                            "target_counts": position_b_counts,
                            "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                            "status": f"Cycle {cycle_index + 1}/{cycles}: moving to endpoint B ({position_b_deg:.2f} deg)...",
                        },
                        {
                            "type": "wait_position_steady",
                            "target_counts": position_b_counts,
                            "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                            "tolerance_deg": tolerance_deg,
                            "hold_s": hold_s,
                            "timeout_s": timeout_s,
                            "endpoint_key": "B",
                            "status": f"Cycle {cycle_index + 1}/{cycles}: settling at endpoint B...",
                        },
                        {"type": "wait", "duration_s": dwell_s, "status": f"Cycle {cycle_index + 1}/{cycles}: dwelling at endpoint B..."},
                        {
                            "type": "set_position",
                            "target_counts": position_a_counts,
                            "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                            "status": f"Cycle {cycle_index + 1}/{cycles}: returning to endpoint A ({position_a_deg:.2f} deg)...",
                        },
                        {
                            "type": "wait_position_steady",
                            "target_counts": position_a_counts,
                            "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                            "tolerance_deg": tolerance_deg,
                            "hold_s": hold_s,
                            "timeout_s": timeout_s,
                            "endpoint_key": "A",
                            "status": f"Cycle {cycle_index + 1}/{cycles}: settling at endpoint A...",
                        },
                        {"type": "wait", "duration_s": dwell_s, "status": f"Cycle {cycle_index + 1}/{cycles}: dwelling at endpoint A..."},
                    ]
                )
            sequence.append({"type": "stop_foc_position", "status": "Final Step: Stopping FOC..."})
            return sequence

        step_deg = float(self.position_test_short_step_spin.value())
        tolerance_deg = float(self.position_test_short_tol_spin.value())
        hold_s = float(self.position_test_short_hold_spin.value()) / 1000.0
        timeout_s = float(self.position_test_short_timeout_spin.value())
        base_counts = self._degrees_to_counts(
            current_single_deg,
            POSITION_TRACKING_MODE_SINGLE_TURN,
        )
        target_counts = self._degrees_to_counts(
            current_single_deg + step_deg,
            POSITION_TRACKING_MODE_SINGLE_TURN,
        )
        target_deg = self._counts_to_single_turn_degrees(target_counts)
        return [
            {
                "type": "prepare_position",
                "target_counts": base_counts,
                "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                "status": f"Step 1/6: Priming at the current angle {current_single_deg:.2f} deg...",
            },
            {
                "type": "start_foc_position",
                "target_counts": base_counts,
                "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                "status": "Step 2/6: Starting FOC in Position Mode...",
            },
            {"type": "wait", "duration_s": 0.10, "status": "Step 3/6: Waiting for the drive to arm..."},
            {
                "type": "set_position",
                "target_counts": target_counts,
                "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                "status": f"Step 4/6: Applying the micro-step to {target_deg:.2f} deg...",
            },
            {
                "type": "wait_position_steady",
                "target_counts": target_counts,
                "tracking_mode": POSITION_TRACKING_MODE_SINGLE_TURN,
                "tolerance_deg": tolerance_deg,
                "hold_s": hold_s,
                "timeout_s": timeout_s,
                "endpoint_key": "SHORT",
                "status": "Step 5/6: Monitoring the micro-step settling...",
            },
            {"type": "stop_foc_position", "status": "Step 6/6: Stopping FOC..."},
        ]

    def _set_position_test_running_ui(self, active: bool) -> None:
        self.position_test_profile_combo.setEnabled(not active)
        self.position_test_stack.setEnabled(not active)
        self.foc_mode_combo.setEnabled(not active)
        self.start_foc_button.setEnabled(not active)
        self.stop_foc_button.setEnabled(not active)
        self.start_foc_rotating_theta_test_button.setEnabled(not active)
        self.start_foc_rotating_theta_voltage_test_button.setEnabled(not active)
        self.start_foc_current_feedback_map_test_button.setEnabled(not active)
        if hasattr(self, "speed_test_group"):
            self.speed_test_group.setEnabled(not active)
        self.position_test_run_button.setText("Cancel" if active else "Run Validation")

    def _toggle_position_test_run(self) -> None:
        if self._position_test_session is not None:
            self._cancel_position_test("Cancelled by user.", request_stop=True)
            return
        self._start_position_test()

    def _start_position_test(self) -> None:
        if self._speed_test_session is not None:
            QtWidgets.QMessageBox.warning(
                self,
                "Position Validation Suite",
                "Cancel the active speed test first, then launch the position validation sequence.",
            )
            return
        if not self._connected:
            QtWidgets.QMessageBox.warning(
                self,
                "Position Validation Suite",
                "Connect to the drive before starting a position validation sequence.",
            )
            return

        snapshot = self._latest_monitor_snapshot
        if snapshot is not None:
            alignment_policy = int(getattr(snapshot, "debug_alignment_policy", ENCODER_ALIGNMENT_POLICY_POWER_ON))
            alignment_status = int(getattr(snapshot, "debug_alignment_status", ENCODER_ALIGNMENT_STATUS_IDLE))
            if (
                alignment_policy == ENCODER_ALIGNMENT_POLICY_POWER_ON
                and alignment_status != ENCODER_ALIGNMENT_STATUS_DONE
            ):
                QtWidgets.QMessageBox.warning(
                    self,
                    "Position Validation Suite",
                    "Toggle Servo ON and wait until Driver Monitor shows Align Status = Done before running a position validation sequence.",
                )
                return
        active_run_modes = {
            RUN_MODE_FOC,
            RUN_MODE_OPEN_LOOP_VF,
            RUN_MODE_ALIGNMENT_ONLY,
            RUN_MODE_AUTOTUNE,
        }
        if snapshot is not None and (
            int(getattr(snapshot, "run_mode", -1)) in active_run_modes
            and (
                bool(getattr(snapshot, "enable_run", False))
                or abs(float(getattr(snapshot, "act_speed", 0.0))) > 10.0
            )
        ):
            QtWidgets.QMessageBox.warning(
                self,
                "Position Validation Suite",
                "Stop the current run first, then launch the automated position validation from a clean idle state.",
            )
            return

        if self._current_foc_mode() != POSITION_CONTROL_MODE:
            combo_index = self.foc_mode_combo.findData(POSITION_CONTROL_MODE)
            if combo_index >= 0:
                self.foc_mode_combo.setCurrentIndex(combo_index)

        profile_key = self._position_test_profile_key()
        preferred_tracking_mode = (
            POSITION_TRACKING_MODE_MULTI_TURN
            if profile_key == POSITION_TEST_PROFILE_LONG
            else POSITION_TRACKING_MODE_SINGLE_TURN
        )
        self._apply_position_tracking_mode_to_ui(preferred_tracking_mode)

        try:
            self._sync_foc_controls_to_driver_table()
            values = self._table_values(self.driver_table)
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid Driver Parameters", str(exc))
            return

        profile_lines = self._position_test_summary_lines(profile_key)
        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Position Validation",
            "\n".join(
                [
                    *profile_lines,
                    "",
                    "Current FOC position and speed gains, ramps, speed limit, and tracking mode from the FOC Control tab will be written to Driver Parameters before the test starts.",
                    "Continue?",
                ]
            ),
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.Yes,
        )
        if reply != QtWidgets.QMessageBox.StandardButton.Yes:
            self.position_test_status_label.setText("Idle")
            return

        self.auto_poll_checkbox.setChecked(True)
        sequence = self._build_position_test_sequence(profile_key, snapshot)
        self._queue_driver_parameter_write_chunks(values, "Position Validation: Write Driver Parameters")
        self._position_test_session = PositionTestSession(profile_key=profile_key, sequence=sequence)
        self._set_position_test_running_ui(True)
        self.position_test_status_label.setText("Preparing: writing driver parameters to the drive...")
        self._request_monitor_once()
        self._position_test_timer.start()

    def _finish_position_test(self, text: str) -> None:
        self._position_test_timer.stop()
        self._position_test_session = None
        self._set_position_test_running_ui(False)
        self.position_test_status_label.setText(text)
        self._request_monitor_once()

    def _cancel_position_test(self, text: str, request_stop: bool = True) -> None:
        self._position_test_timer.stop()
        self._position_test_session = None
        self._pending_frames.clear()
        self._set_position_test_running_ui(False)
        self.position_test_status_label.setText(text)
        if request_stop and self._connected:
            self._enqueue_position_stop_command(
                description="Position Validation: Emergency Stop",
                request_monitor=True,
            )

    def _advance_position_test_step(self) -> None:
        session = self._position_test_session
        if session is None:
            return
        session.step_index += 1
        session.step_started_at = 0.0
        session.steady_since_at = None
        if session.step_index >= len(session.sequence):
            self._finish_position_test(session.completion_text)
            return
        session.sequence[session.step_index].pop("_queued", None)

    def _record_position_test_endpoint(
        self,
        session: PositionTestSession,
        step: dict[str, object],
        actual_counts: float,
        error_deg: float,
        tracking_mode: int,
    ) -> None:
        target_counts = float(step.get("target_counts", actual_counts))
        target_deg = self._counts_to_position_mode_degrees(target_counts, tracking_mode)
        actual_deg = self._counts_to_position_mode_degrees(actual_counts, tracking_mode)
        endpoint_key = str(step.get("endpoint_key", "") or "")
        if endpoint_key:
            baseline_counts = session.endpoint_baselines.get(endpoint_key)
            if baseline_counts is None:
                session.endpoint_baselines[endpoint_key] = actual_counts
                repeatability_deg = 0.0
            else:
                repeatability_deg = self._counts_to_accumulated_degrees(actual_counts - baseline_counts)
            session.validation_error_deg = repeatability_deg
            self._append_log(
                f"[PosTest] Endpoint {endpoint_key}: target={target_deg:.3f} deg, actual={actual_deg:.3f} deg, "
                f"settling error={error_deg:.4f} deg, repeatability={repeatability_deg:.4f} deg"
            )
            return
        session.validation_error_deg = error_deg
        self._append_log(
            f"[PosTest] Settled: target={target_deg:.3f} deg, actual={actual_deg:.3f} deg, error={error_deg:.4f} deg"
        )

    def _log_long_distance_progress(
        self,
        session: PositionTestSession,
        step: dict[str, object],
        actual_counts: float,
    ) -> None:
        start_counts = float(step.get("start_counts", actual_counts))
        target_counts = float(step.get("target_counts", actual_counts))
        encoder_resolution = self._encoder_resolution_counts()
        if encoder_resolution <= 1.0:
            return
        total_travel_counts = target_counts - start_counts
        total_turns = int(abs(total_travel_counts) // encoder_resolution)
        if total_turns <= 0:
            return
        direction = 1.0 if total_travel_counts >= 0.0 else -1.0
        turns_completed = int(abs((actual_counts - start_counts) / encoder_resolution))
        while session.long_last_logged_turn < min(turns_completed, total_turns):
            turn_index = session.long_last_logged_turn + 1
            boundary_counts = start_counts + direction * turn_index * encoder_resolution
            boundary_error_deg = self._counts_to_accumulated_degrees(actual_counts - boundary_counts)
            self._append_log(
                f"[PosTest] Long-distance turn {turn_index}/{total_turns}: boundary error {boundary_error_deg:.4f} deg"
            )
            session.long_last_logged_turn = turn_index

    def _on_position_test_timer(self) -> None:
        session = self._position_test_session
        if session is None:
            return

        now = time.monotonic()
        if session.preparing_write:
            if self._communication_idle():
                session.preparing_write = False
                session.step_index = 0
                session.step_started_at = 0.0
                session.steady_since_at = None
            return

        if session.step_index >= len(session.sequence):
            self._finish_position_test(session.completion_text)
            return

        step = session.sequence[session.step_index]
        if session.step_started_at <= 0.0:
            session.step_started_at = now
            session.steady_since_at = None
            if step.get("status"):
                self.position_test_status_label.setText(str(step["status"]))

        step_type = str(step.get("type", ""))
        if step_type == "prepare_position":
            self._apply_position_target_to_ui(
                float(step.get("target_counts", 0.0)),
                int(step.get("tracking_mode", POSITION_TRACKING_MODE_SINGLE_TURN)),
            )
            self._advance_position_test_step()
            return

        if step_type == "start_foc_position":
            if not step.get("_queued") and self._communication_idle():
                self._enqueue_position_start_command(
                    float(step.get("target_counts", 0.0)),
                    int(step.get("tracking_mode", POSITION_TRACKING_MODE_SINGLE_TURN)),
                    "Position Validation: Start FOC Position Mode",
                    request_monitor=True,
                )
                step["_queued"] = True
                return
            if step.get("_queued") and self._communication_idle() and (now - session.step_started_at) >= 0.10:
                self._advance_position_test_step()
            return

        if step_type == "set_position":
            if not step.get("_queued") and self._communication_idle():
                target_counts = float(step.get("target_counts", 0.0))
                tracking_mode = int(step.get("tracking_mode", POSITION_TRACKING_MODE_SINGLE_TURN))
                target_deg = self._counts_to_position_mode_degrees(target_counts, tracking_mode)
                self._enqueue_position_start_command(
                    target_counts,
                    tracking_mode,
                    f"Position Validation: Set position to {target_deg:.2f} deg",
                    request_monitor=True,
                )
                step["_queued"] = True
                return
            if step.get("_queued") and self._communication_idle() and (now - session.step_started_at) >= 0.10:
                self._advance_position_test_step()
            return

        if step_type == "wait":
            if (now - session.step_started_at) >= float(step.get("duration_s", 0.0)):
                self._advance_position_test_step()
            return

        if step_type == "wait_position_steady":
            snapshot = self._latest_monitor_snapshot
            timeout_s = float(step.get("timeout_s", 0.0))
            if snapshot is not None:
                target_counts = float(step.get("target_counts", 0.0))
                tracking_mode = int(step.get("tracking_mode", POSITION_TRACKING_MODE_SINGLE_TURN))
                actual_counts = float(getattr(snapshot, "act_position", 0.0))
                error_deg = self._display_position_error_degrees(
                    target_counts,
                    actual_counts,
                    tracking_mode,
                )
                if session.profile_key != POSITION_TEST_PROFILE_BACKLASH:
                    session.validation_error_deg = error_deg
                if bool(step.get("log_turns", False)):
                    self._log_long_distance_progress(session, step, actual_counts)

                act_speed = abs(float(getattr(snapshot, "act_speed", 0.0)))
                if (
                    abs(error_deg) <= float(step.get("tolerance_deg", 0.0))
                    and act_speed <= POSITION_TEST_SETTLE_SPEED_TOLERANCE_RPM
                ):
                    if session.steady_since_at is None:
                        session.steady_since_at = now
                    elif (now - session.steady_since_at) >= float(step.get("hold_s", 0.0)):
                        self._record_position_test_endpoint(
                            session,
                            step,
                            actual_counts,
                            error_deg,
                            tracking_mode,
                        )
                        self._advance_position_test_step()
                        return
                else:
                    session.steady_since_at = None

            if (now - session.step_started_at) >= timeout_s:
                self._cancel_position_test(
                    "Timed out while waiting for the commanded position to settle.",
                    request_stop=True,
                )
            return

        if step_type == "stop_foc_position":
            if not step.get("_queued") and self._communication_idle():
                self._enqueue_position_stop_command(
                    description="Position Validation: Stop FOC",
                    request_monitor=True,
                )
                step["_queued"] = True
                return
            if step.get("_queued") and self._communication_idle() and (now - session.step_started_at) >= 0.10:
                self._advance_position_test_step()
            return

    def _build_speed_test_sequence(self, profile_key: str) -> list[dict[str, object]]:
        if profile_key == SPEED_TEST_PROFILE_REVERSE:
            target = abs(float(self.speed_test_reverse_target_spin.value()))
            tolerance = float(self.speed_test_reverse_tolerance_spin.value())
            hold_s = float(self.speed_test_reverse_hold_spin.value()) / 1000.0
            timeout_s = float(self.speed_test_reverse_timeout_spin.value())
            return [
                {"type": "prepare_target", "target_rpm": 0.0, "status": "Step 1/8: Priming the target to 0 rpm..."},
                {"type": "start_foc", "target_rpm": 0.0, "status": "Step 2/8: Starting FOC in Speed Mode..."},
                {"type": "wait", "duration_s": 0.10, "status": "Step 3/8: Waiting for the drive to arm..."},
                {"type": "set_speed", "target_rpm": target, "status": f"Step 4/8: Commanding +{target:.1f} rpm..."},
                {
                    "type": "wait_steady",
                    "target_rpm": target,
                    "tolerance_rpm": tolerance,
                    "hold_s": hold_s,
                    "timeout_s": timeout_s,
                    "status": f"Step 5/8: Waiting for +{target:.1f} rpm steady state...",
                },
                {"type": "set_speed", "target_rpm": -target, "status": f"Step 6/8: Reversing to -{target:.1f} rpm..."},
                {
                    "type": "wait_steady",
                    "target_rpm": -target,
                    "tolerance_rpm": tolerance,
                    "hold_s": hold_s,
                    "timeout_s": timeout_s,
                    "status": f"Step 7/8: Waiting for -{target:.1f} rpm steady state...",
                },
                {"type": "stop_foc", "status": "Step 8/8: Stopping FOC..."},
            ]
        if profile_key == SPEED_TEST_PROFILE_LOW_SPEED:
            target = float(self.speed_test_low_target_spin.value())
            duration_s = float(self.speed_test_low_duration_spin.value())
            return [
                {"type": "prepare_target", "target_rpm": 0.0, "status": "Step 1/6: Priming the target to 0 rpm..."},
                {"type": "start_foc", "target_rpm": 0.0, "status": "Step 2/6: Starting FOC in Speed Mode..."},
                {"type": "wait", "duration_s": 0.10, "status": "Step 3/6: Waiting for the drive to arm..."},
                {"type": "set_speed", "target_rpm": target, "status": f"Step 4/6: Commanding {target:.1f} rpm..."},
                {"type": "wait", "duration_s": duration_s, "status": f"Step 5/6: Monitoring ripple at {target:.1f} rpm..."},
                {"type": "stop_foc", "status": "Step 6/6: Stopping FOC..."},
            ]
        if profile_key == SPEED_TEST_PROFILE_TRAPEZOID:
            upper_target = float(self.speed_test_trapezoid_target_spin.value())
            lower_target = float(self.speed_test_trapezoid_lower_spin.value())
            accel_s = float(self.speed_test_trapezoid_accel_spin.value())
            hold_s = float(self.speed_test_trapezoid_hold_spin.value())
            return [
                {
                    "type": "prepare_target",
                    "target_rpm": lower_target,
                    "status": f"Step 1/8: Priming the target to {lower_target:.1f} rpm...",
                },
                {
                    "type": "start_foc",
                    "target_rpm": lower_target,
                    "status": f"Step 2/8: Starting FOC at the lower bound {lower_target:.1f} rpm...",
                },
                {"type": "wait", "duration_s": 0.10, "status": "Step 3/8: Waiting for the drive to arm..."},
                {
                    "type": "set_speed",
                    "target_rpm": upper_target,
                    "status": f"Step 4/8: Ramping linearly up to {upper_target:.1f} rpm...",
                },
                {"type": "wait", "duration_s": accel_s + hold_s, "status": f"Step 5/8: Holding the top plateau for {hold_s:.2f} s..."},
                {
                    "type": "set_speed",
                    "target_rpm": lower_target,
                    "status": f"Step 6/8: Ramping linearly back to {lower_target:.1f} rpm...",
                },
                {"type": "wait", "duration_s": accel_s, "status": "Step 7/8: Finishing the down-ramp..."},
                {"type": "repeat", "to_index": 3, "status": "Step 8/8: Restarting the trapezoid cycle..."},
            ]
        target = float(self.speed_test_step_target_spin.value())
        pre_delay_s = float(self.speed_test_step_pre_delay_spin.value()) / 1000.0
        run_time_s = float(self.speed_test_step_run_time_spin.value())
        return [
            {"type": "prepare_target", "target_rpm": 0.0, "status": "Step 1/6: Priming the target to 0 rpm..."},
            {"type": "start_foc", "target_rpm": 0.0, "status": "Step 2/6: Starting FOC in Speed Mode..."},
            {"type": "wait", "duration_s": pre_delay_s, "status": "Step 3/6: Waiting before the step command..."},
            {"type": "set_speed", "target_rpm": target, "status": f"Step 4/6: Stepping to {target:.1f} rpm..."},
            {"type": "wait", "duration_s": run_time_s, "status": f"Step 5/6: Holding {target:.1f} rpm for capture..."},
            {"type": "stop_foc", "status": "Step 6/6: Stopping FOC..."},
        ]

    def _set_speed_test_running_ui(self, active: bool) -> None:
        self.speed_test_profile_combo.setEnabled(not active)
        self.speed_test_stack.setEnabled(not active)
        self.foc_mode_combo.setEnabled(not active)
        self.start_foc_button.setEnabled(not active)
        self.stop_foc_button.setEnabled(not active)
        self.start_foc_rotating_theta_test_button.setEnabled(not active)
        self.start_foc_rotating_theta_voltage_test_button.setEnabled(not active)
        self.start_foc_current_feedback_map_test_button.setEnabled(not active)
        if hasattr(self, "position_test_group"):
            self.position_test_group.setEnabled(not active)
        self.speed_test_run_button.setText("Cancel" if active else "Run Test")

    def _toggle_speed_test_run(self) -> None:
        if self._speed_test_session is not None:
            self._cancel_speed_test("Cancelled by user.", request_stop=True)
            return
        self._start_speed_test()

    def _start_speed_test(self) -> None:
        if self._position_test_session is not None:
            QtWidgets.QMessageBox.warning(
                self,
                "Speed Test Mode",
                "Cancel the active position validation first, then launch the automated speed test.",
            )
            return
        if not self._connected:
            QtWidgets.QMessageBox.warning(
                self,
                "Speed Test Mode",
                "Connect to the drive before starting an automated speed test.",
            )
            return

        snapshot = self._latest_monitor_snapshot
        if snapshot is not None:
            alignment_policy = int(getattr(snapshot, "debug_alignment_policy", ENCODER_ALIGNMENT_POLICY_POWER_ON))
            alignment_status = int(getattr(snapshot, "debug_alignment_status", ENCODER_ALIGNMENT_STATUS_IDLE))
            if (
                alignment_policy == ENCODER_ALIGNMENT_POLICY_POWER_ON
                and alignment_status != ENCODER_ALIGNMENT_STATUS_DONE
            ):
                QtWidgets.QMessageBox.warning(
                    self,
                    "Speed Test Mode",
                    "Toggle Servo ON and wait until Driver Monitor shows Align Status = Done before running an automated speed test.",
                )
                return
        active_run_modes = {
            RUN_MODE_FOC,
            RUN_MODE_OPEN_LOOP_VF,
            RUN_MODE_ALIGNMENT_ONLY,
            RUN_MODE_AUTOTUNE,
        }
        if snapshot is not None and (
            int(getattr(snapshot, "run_mode", -1)) in active_run_modes
            and (
                bool(getattr(snapshot, "enable_run", False))
                or abs(float(getattr(snapshot, "act_speed", 0.0))) > 10.0
            )
        ):
            QtWidgets.QMessageBox.warning(
                self,
                "Speed Test Mode",
                "Stop the current run first, then launch the automated speed test from a clean idle state.",
            )
            return

        if self._current_foc_mode() != SPEED_CONTROL_MODE:
            combo_index = self.foc_mode_combo.findData(SPEED_CONTROL_MODE)
            if combo_index >= 0:
                self.foc_mode_combo.setCurrentIndex(combo_index)

        try:
            self._sync_foc_controls_to_driver_table()
            values = self._table_values(self.driver_table)
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid Driver Parameters", str(exc))
            return

        profile_key = self._speed_test_profile_key()
        if profile_key == SPEED_TEST_PROFILE_TRAPEZOID:
            accel_ms = float(self.speed_test_trapezoid_accel_spin.value()) * 1000.0
            self._apply_speed_ramp_to_ui(accel_ms, accel_ms)
        profile_lines = self._speed_test_summary_lines(profile_key)
        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Automated Speed Test",
            "\n".join(
                [
                    *profile_lines,
                    "",
                    "Current FOC speed gains, ramps, and max speed from this panel will be written to Driver Parameters before the test starts.",
                    "Continue?",
                ]
            ),
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.Yes,
        )
        if reply != QtWidgets.QMessageBox.StandardButton.Yes:
            self.speed_test_status_label.setText("Idle")
            return

        self.auto_poll_checkbox.setChecked(True)
        sequence = self._build_speed_test_sequence(profile_key)
        self._queue_driver_parameter_write_chunks(values, "Automated Speed Test: Write Driver Parameters")
        self._speed_test_session = SpeedTestSession(profile_key=profile_key, sequence=sequence)
        self._set_speed_test_running_ui(True)
        self.speed_test_status_label.setText("Preparing: writing driver parameters to the drive...")
        self._request_monitor_once()
        self._speed_test_timer.start()

    def _finish_speed_test(self, text: str) -> None:
        self._speed_test_timer.stop()
        self._speed_test_session = None
        self._set_speed_test_running_ui(False)
        self.speed_test_status_label.setText(text)
        self._request_monitor_once()

    def _cancel_speed_test(self, text: str, request_stop: bool = True) -> None:
        self._speed_test_timer.stop()
        self._speed_test_session = None
        self._pending_frames.clear()
        self._set_speed_test_running_ui(False)
        self.speed_test_status_label.setText(text)
        if request_stop and self._connected:
            self._enqueue_speed_stop_command(
                description="Automated Speed Test: Emergency Stop",
                request_monitor=True,
            )

    def _advance_speed_test_step(self) -> None:
        session = self._speed_test_session
        if session is None:
            return
        session.step_index += 1
        session.step_started_at = 0.0
        session.steady_since_at = None
        if session.step_index >= len(session.sequence):
            self._finish_speed_test(session.completion_text)
            return
        session.sequence[session.step_index].pop("_queued", None)

    def _on_speed_test_timer(self) -> None:
        session = self._speed_test_session
        if session is None:
            return

        now = time.monotonic()
        if session.preparing_write:
            if self._communication_idle():
                session.preparing_write = False
                session.step_index = 0
                session.step_started_at = 0.0
                session.steady_since_at = None
                session.last_target_send_at = 0.0
                session.last_target_rpm = None
            return

        if session.step_index >= len(session.sequence):
            self._finish_speed_test(session.completion_text)
            return

        step = session.sequence[session.step_index]
        if session.step_started_at <= 0.0:
            session.step_started_at = now
            session.steady_since_at = None
            if step.get("status"):
                self.speed_test_status_label.setText(str(step["status"]))

        step_type = str(step.get("type", ""))
        if step_type == "prepare_target":
            self._apply_speed_target_to_ui(float(step.get("target_rpm", 0.0)))
            self._advance_speed_test_step()
            return

        if step_type == "start_foc":
            if not step.get("_queued") and self._communication_idle():
                bypass_speed_ramp = session.profile_key == SPEED_TEST_PROFILE_STEP
                self._enqueue_speed_start_command(
                    float(step.get("target_rpm", self.foc_target_speed_spin.value())),
                    "Automated Speed Test: Start FOC Speed Mode",
                    request_monitor=True,
                    bypass_speed_ramp=bypass_speed_ramp,
                )
                step["_queued"] = True
                return
            if step.get("_queued") and self._communication_idle() and (now - session.step_started_at) >= 0.10:
                self._advance_speed_test_step()
            return

        if step_type == "set_speed":
            if not step.get("_queued") and self._communication_idle():
                target_rpm = float(step.get("target_rpm", 0.0))
                bypass_speed_ramp = session.profile_key == SPEED_TEST_PROFILE_STEP
                self._enqueue_speed_start_command(
                    target_rpm,
                    f"Automated Speed Test: Set speed to {target_rpm:.1f} rpm",
                    request_monitor=True,
                    bypass_speed_ramp=bypass_speed_ramp,
                )
                step["_queued"] = True
                session.last_target_rpm = target_rpm
                session.last_target_send_at = now
                return
            if step.get("_queued") and self._communication_idle() and (now - session.step_started_at) >= 0.10:
                self._advance_speed_test_step()
            return

        if step_type == "wait":
            if (now - session.step_started_at) >= float(step.get("duration_s", 0.0)):
                self._advance_speed_test_step()
            return

        if step_type == "repeat":
            session.step_index = max(0, int(step.get("to_index", 0)))
            session.step_started_at = 0.0
            session.steady_since_at = None
            if session.step_index < len(session.sequence):
                session.sequence[session.step_index].pop("_queued", None)
            return

        if step_type == "wait_steady":
            snapshot = self._latest_monitor_snapshot
            target_rpm = float(step.get("target_rpm", 0.0))
            tolerance_rpm = float(step.get("tolerance_rpm", 0.0))
            hold_s = float(step.get("hold_s", 0.0))
            timeout_s = float(step.get("timeout_s", 0.0))
            if snapshot is not None:
                act_speed = float(getattr(snapshot, "act_speed", 0.0))
                if abs(act_speed - target_rpm) <= tolerance_rpm:
                    if session.steady_since_at is None:
                        session.steady_since_at = now
                    elif (now - session.steady_since_at) >= hold_s:
                        self._advance_speed_test_step()
                        return
                else:
                    session.steady_since_at = None
            if (now - session.step_started_at) >= timeout_s:
                self._cancel_speed_test(
                    f"Timed out while waiting for {target_rpm:.1f} rpm steady state.",
                    request_stop=True,
                )
            return

    def _update_foc_mode_ui(self) -> None:
        position_mode = self._current_foc_mode() == POSITION_CONTROL_MODE
        tracking_mode = self._position_tracking_mode()
        lower_deg, upper_deg = self._position_angle_range_deg(tracking_mode)
        self.foc_target_angle_spin.blockSignals(True)
        self.foc_target_angle_spin.setRange(lower_deg, upper_deg)
        self.foc_target_angle_spin.blockSignals(False)
        target_value = (
            abs(self._foc_position_speed_limit_rpm) if position_mode else self._foc_speed_target_rpm
        )
        self.foc_target_speed_spin.blockSignals(True)
        self.foc_target_speed_spin.setValue(target_value)
        self.foc_target_speed_spin.blockSignals(False)
        self._last_foc_mode_ui = POSITION_CONTROL_MODE if position_mode else SPEED_CONTROL_MODE
        self.foc_target_speed_label.setText("Speed Limit" if position_mode else "Target Speed")
        self.foc_target_position_label.setEnabled(position_mode)
        self.foc_target_position_spin.setEnabled(False)
        self.foc_position_tracking_label.setEnabled(position_mode)
        self.foc_position_tracking_combo.setEnabled(position_mode)
        self.foc_target_angle_label.setEnabled(position_mode)
        self.foc_target_angle_spin.setEnabled(position_mode)
        self.foc_jog_label.setEnabled(position_mode)
        self.foc_jog_minus_90_button.setEnabled(position_mode)
        self.foc_jog_minus_10_button.setEnabled(position_mode)
        self.foc_jog_plus_10_button.setEnabled(position_mode)
        self.foc_jog_plus_90_button.setEnabled(position_mode)
        self.foc_angle_slider_label.setEnabled(position_mode)
        self.foc_angle_slider.setEnabled(
            position_mode and (tracking_mode == POSITION_TRACKING_MODE_SINGLE_TURN)
        )
        self.foc_angle_slider_value_label.setEnabled(position_mode)
        self.foc_target_counts_hint_label.setEnabled(position_mode)
        self.foc_position_kp_label.setEnabled(position_mode)
        self.foc_position_kp_spin.setEnabled(position_mode)
        self.foc_position_ki_label.setEnabled(position_mode)
        self.foc_position_ki_spin.setEnabled(position_mode)
        self.foc_position_vff_gain_label.setEnabled(position_mode)
        self.foc_position_vff_gain_spin.setEnabled(position_mode)
        self.foc_position_vff_filter_label.setEnabled(position_mode)
        self.foc_position_vff_filter_spin.setEnabled(position_mode)
        if position_mode:
            if tracking_mode == POSITION_TRACKING_MODE_MULTI_TURN:
                self.foc_angle_slider_value_label.setText("Disabled in multi-turn")
                self.foc_mode_description_label.setText(
                    "<b>Position Mode / Multi Turn</b>"
                    "<ul style='margin-top:6px; margin-bottom:0px; margin-left:18px;'>"
                    "<li>Enter an <b>accumulated target angle</b> in degrees.</li>"
                    "<li>Firmware compares it against the accumulated encoder position with <b>no shortest-path wrapping</b>.</li>"
                    "<li>Use this mode for <b>long-distance moves across many turns</b>.</li>"
                    "<li>The angle slider is disabled because it only makes sense for one-turn moves.</li>"
                    "</ul>"
                )
            else:
                self.foc_angle_slider_value_label.setText(
                    f"{self.foc_angle_slider.value() / 10.0:.1f} deg"
                )
                self.foc_mode_description_label.setText(
                    "<b>Position Mode / Single Turn</b>"
                    "<ul style='margin-top:6px; margin-bottom:0px; margin-left:18px;'>"
                    "<li>Enter the target in degrees for a <b>one-turn move</b>.</li>"
                    "<li>Use the relative jog buttons for quick manual moves.</li>"
                    "<li>Release the angle slider to send a clean target update.</li>"
                    "<li>Firmware uses <b>shortest-path wrapping</b> so the motor takes the nearest route within one mechanical turn.</li>"
                    "</ul>"
                )
            if not self._connected:
                self.foc_live_summary_label.setText(
                    "Position Mode is ready. Choose Single Turn or Multi Turn, enter the target angle, use relative jog if needed, set the speed limit and PI/VFF gains, then press Start FOC."
                )
        else:
            self.foc_mode_description_label.setText(
                "<b>Speed Mode</b>"
                "<ul style='margin-top:6px; margin-bottom:0px; margin-left:18px;'>"
                "<li>The <b>position loop is bypassed</b>.</li>"
                "<li><b>Target Speed</b> goes directly into the speed loop.</li>"
                "<li>The speed loop generates <b>Iq</b> for the inner current loop.</li>"
                "</ul>"
            )
            if not self._connected:
                self.foc_live_summary_label.setText(
                    "Speed Mode is ready. Enter target speed and gains, then press Start FOC."
                )
        if hasattr(self, "speed_test_group"):
            self.speed_test_group.setEnabled(self._position_test_session is None)
            if (
                self._speed_test_session is None
                and self.speed_test_status_label.text().startswith(
                    "Switch FOC Control to Speed Mode"
                )
            ):
                self.speed_test_status_label.setText("Idle")
        self._update_speed_test_profile_ui()
        self._update_position_test_profile_ui()
        self._refresh_foc_control_panel(self._latest_monitor_snapshot)

    def _on_foc_target_speed_value_changed(self, value: float) -> None:
        if self._current_foc_mode() == POSITION_CONTROL_MODE:
            self._foc_position_speed_limit_rpm = abs(float(value))
        else:
            self._foc_speed_target_rpm = float(value)

    def _refresh_foc_control_panel(self, snapshot) -> None:
        if not hasattr(self, "foc_status_value_label"):
            return
        self._refresh_runtime_toggle_buttons(snapshot)
        preview_mode = self._current_foc_mode()
        mode_text = "Position Mode" if preview_mode == POSITION_CONTROL_MODE else "Speed Mode"
        debug_suffix = " Electrical frame: none (raw theta)."
        if snapshot is None:
            self.foc_status_value_label.setText("Idle")
            self.foc_direction_test_status_label.setText("Idle")
            self.foc_live_summary_label.setText(
                f"{mode_text} is ready. Servo ON runs the arm sequence. Watch Driver Monitor for Cal Status and Align Status, then press Start FOC when the drive is ready.{debug_suffix}"
            )
            return
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
                tracking_mode = self._position_tracking_mode()
                target_deg = self._counts_to_position_mode_degrees(snapshot.cmd_position, tracking_mode)
                act_deg = self._counts_to_position_mode_degrees(snapshot.act_position, tracking_mode)
                error_deg = self._display_position_error_degrees(
                    snapshot.cmd_position,
                    snapshot.act_position,
                    tracking_mode,
                )
                self.foc_status_value_label.setText("Fault / stopped")
                self.foc_live_summary_label.setText(
                    f"Last position target {target_deg:.2f} deg / {snapshot.cmd_position:.1f} cnt | "
                    f"Act {act_deg:.2f} deg / {snapshot.act_position:.1f} cnt | "
                    f"Error {error_deg:.3f} deg | "
                    f"{self._position_tracking_mode_text(tracking_mode)}"
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
            tracking_mode = self._position_tracking_mode()
            target_deg = self._counts_to_position_mode_degrees(snapshot.cmd_position, tracking_mode)
            act_deg = self._counts_to_position_mode_degrees(snapshot.act_position, tracking_mode)
            error_deg = self._display_position_error_degrees(
                snapshot.cmd_position,
                snapshot.act_position,
                tracking_mode,
            )
            self.foc_status_value_label.setText("FOC Position Running")
            self.foc_live_summary_label.setText(
                f"Target {target_deg:.2f} deg / {snapshot.cmd_position:.1f} cnt | "
                f"Act {act_deg:.2f} deg / {snapshot.act_position:.1f} cnt | "
                f"Error {error_deg:.3f} deg | "
                f"Cmd Speed {snapshot.cmd_speed:.1f} rpm | "
                f"{self._position_tracking_mode_text(tracking_mode)}"
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
        if self._speed_test_session is not None:
            self._cancel_speed_test("Cancelled by Servo OFF.", request_stop=False)
        if self._position_test_session is not None:
            self._cancel_position_test("Cancelled by Servo OFF.", request_stop=False)
        self._enqueue_command(Command.CMD_SERVO_OFF, b"", "Servo OFF")
        self._request_monitor_once()

    def _start_foc_rotating_theta_test(self) -> None:
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
                    "Rotating Theta Test Blocked",
                    "Toggle Servo ON and wait until Driver Monitor shows Align Status = Done, then run the rotating-theta current-vector test.",
                )
                return

        self.auto_poll_checkbox.setChecked(True)
        self.foc_status_value_label.setText("Rotating-theta test starting...")
        self.foc_direction_test_status_label.setText("Rot theta")
        self.foc_live_summary_label.setText(
            "Running a synthetic rotating-theta FOC test: the firmware ramps an internal electrical angle, applies the fixed frame compensation, and injects a small +Iq with Id = 0. If the motor spins smoothly here, Park/Clarke/PWM/current loop are likely fine and the remaining bug is in the encoder theta runtime path."
        )
        self._enqueue_command(
            Command.CMD_START_FOC_ROTATING_THETA_TEST,
            b"",
            "Run FOC Rotating Theta Current Test",
        )
        self._request_monitor_once()

    def _start_foc_rotating_theta_voltage_test(self) -> None:
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
                    "Rotating Theta Voltage Test Blocked",
                    "Toggle Servo ON and wait until Driver Monitor shows Align Status = Done, then run the rotating-theta voltage-vector test.",
                )
                return

        self.auto_poll_checkbox.setChecked(True)
        self.foc_status_value_label.setText("Rotating-theta voltage test starting...")
        self.foc_direction_test_status_label.setText("Rot theta V")
        self.foc_live_summary_label.setText(
            "Running a synthetic rotating-theta voltage-vector test: the firmware ramps an internal electrical angle, applies the fixed frame compensation, and injects a small direct +Vq with Vd = 0. If this spins smoothly but the current-loop version still steps, the remaining bug sits in current feedback / dq measurement rather than inverse Park-Clarke-PWM."
        )
        self._enqueue_command(
            Command.CMD_START_FOC_ROTATING_THETA_VOLTAGE_TEST,
            b"",
            "Run FOC Rotating Theta Voltage Test",
        )
        self._request_monitor_once()

    def _start_foc_current_feedback_map_test(self) -> None:
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
                    "Current Feedback Map Test Blocked",
                    "Toggle Servo ON and wait until Driver Monitor shows Align Status = Done, then run the mapping sweep.",
                )
                return

        self.auto_poll_checkbox.setChecked(True)
        self.foc_status_value_label.setText("Current feedback map test starting...")
        self.foc_direction_test_status_label.setText("Map sweep")
        self.foc_live_summary_label.setText(
            "Running a controlled 4-case current-feedback mapping sweep on the rotating-theta current test. The firmware will try normal, invert, swap, and swap+invert mappings and log Id/Iq summary metrics for each case so we can pick the cleanest feedback convention without hand-editing code each round."
        )
        self._enqueue_command(
            Command.CMD_START_FOC_CURRENT_FEEDBACK_MAP_TEST,
            b"",
            "Run FOC Current Feedback Map Test",
        )
        self._request_monitor_once()

    def _foc_is_running(self, snapshot=None) -> bool:
        active_snapshot = self._latest_monitor_snapshot if snapshot is None else snapshot
        return bool(
            active_snapshot is not None
            and int(getattr(active_snapshot, "run_mode", -1)) == RUN_MODE_FOC
        )

    def _open_loop_vf_is_running(self, snapshot=None) -> bool:
        active_snapshot = self._latest_monitor_snapshot if snapshot is None else snapshot
        return bool(
            active_snapshot is not None
            and int(getattr(active_snapshot, "run_mode", -1)) == RUN_MODE_OPEN_LOOP_VF
        )

    def _refresh_runtime_toggle_buttons(self, snapshot=None) -> None:
        active_snapshot = self._latest_monitor_snapshot if snapshot is None else snapshot
        if hasattr(self, "start_foc_button") and hasattr(self, "stop_foc_button"):
            manual_allowed = (
                self._speed_test_session is None and self._position_test_session is None
            )
            self.start_foc_button.setEnabled(manual_allowed)
            self.stop_foc_button.setEnabled(manual_allowed)
        if hasattr(self, "vf_toggle_button"):
            self.vf_toggle_button.setText(
                "Stop V/F"
                if self._open_loop_vf_is_running(active_snapshot)
                else "Start V/F"
            )

    def _toggle_open_loop_vf(self) -> None:
        if self._open_loop_vf_is_running():
            self._stop_open_loop_vf()
            return
        self._start_open_loop_vf()


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
            tracking_mode = self._position_tracking_mode()
            position_target = self._degrees_to_counts(
                float(self.foc_target_angle_spin.value()),
                tracking_mode,
            )
            self._set_foc_target_position_counts(position_target)
            position_target_degrees = self._counts_to_position_mode_degrees(
                position_target,
                tracking_mode,
            )
            current_position = (
                float(getattr(last_monitor, "act_position", position_target))
                if last_monitor is not None
                else position_target
            )
            current_position_degrees = self._counts_to_position_mode_degrees(
                current_position,
                tracking_mode,
            )
            payload = struct.pack(
                "<10fBBB",
                position_target,
                float(self._foc_position_speed_limit_rpm),
                float(self.foc_position_kp_spin.value()),
                float(self.foc_position_ki_spin.value()),
                float(self.foc_position_vff_gain_spin.value()),
                float(self.foc_position_vff_filter_spin.value()),
                float(self.foc_speed_kp_spin.value()),
                float(self.foc_speed_ki_spin.value()),
                float(self.foc_accel_spin.value()),
                float(self.foc_decel_spin.value()),
                ID_SQUARE_ANGLE_TEST_NONE,
                0,
                tracking_mode,
            )
            description = (
                f"Start FOC Position Mode "
                f"(target={position_target_degrees:.2f} deg / {position_target:.1f} cnt, "
                f"limit={self._foc_position_speed_limit_rpm:.1f} rpm, "
                f"pi=({self.foc_position_kp_spin.value():.3f}, {self.foc_position_ki_spin.value():.3f}), "
                f"vff=({self.foc_position_vff_gain_spin.value():.3f}, {self.foc_position_vff_filter_spin.value():.1f} Hz), "
                f"tracking={self._position_tracking_mode_text(tracking_mode).lower()}, "
                "frame=none)"
            )
            command = Command.CMD_START_POSITIONCONTROL
            if abs(position_target - current_position) < 1.0:
                self.foc_live_summary_label.setText(
                    "Position Mode start sent, but Target Position already matches the current position within 1 count. Change the target angle if you expect motion."
                )
            else:
                self.foc_live_summary_label.setText(
                    f"Starting Position Mode: target {position_target_degrees:.2f} deg / {position_target:.1f} cnt | "
                    f"current {current_position_degrees:.2f} deg / {current_position:.1f} cnt | "
                    f"speed limit {self._foc_position_speed_limit_rpm:.1f} rpm | "
                    f"{self._position_tracking_mode_text(tracking_mode)}"
                )
        else:
            speed_limit_rpm = self._speed_mode_limit_rpm()
            payload = struct.pack(
                "<6fBB",
                float(self._foc_speed_target_rpm),
                float(self.foc_speed_kp_spin.value()),
                float(self.foc_speed_ki_spin.value()),
                float(self.foc_accel_spin.value()),
                float(self.foc_decel_spin.value()),
                speed_limit_rpm,
                ID_SQUARE_ANGLE_TEST_NONE,
                0,
            )
            description = (
                f"Start FOC Speed Mode "
                f"(target={self._foc_speed_target_rpm:.1f} rpm, "
                f"max={speed_limit_rpm:.1f} rpm, "
                "frame=none)"
            )
            command = Command.CMD_START_SPEEDCONTROL
            self.foc_live_summary_label.setText(
                f"Starting Speed Mode: target {self._foc_speed_target_rpm:.1f} rpm | "
                f"max speed {speed_limit_rpm:.1f} rpm"
            )

        self.foc_status_value_label.setText("Starting...")
        self._enqueue_command(command, payload, description)
        self._request_monitor_once()

    def _stop_foc_control(self) -> None:
        if self._speed_test_session is not None:
            self._cancel_speed_test("Cancelled by manual Stop FOC.", request_stop=False)
        if self._position_test_session is not None:
            self._cancel_position_test("Cancelled by manual Stop FOC.", request_stop=False)
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

        payload = bytearray()
        payload.append(MOTOR_PARAM_CURRENT_P_GAIN & 0xFF)
        payload.extend(struct.pack("<f", current_kp))
        payload.append(MOTOR_PARAM_CURRENT_I_GAIN & 0xFF)
        payload.extend(struct.pack("<f", current_ki))
        self._enqueue_command(
            Command.CMD_WRITE_MOTOR,
            bytes(payload),
            "Apply Current PI To FOC (safe partial write)",
        )

        self.ctuning_status_label.setText(
            f"Applied current PI only (safe partial write): scale={apply_scale:.2f}x, Kp={current_kp:.4f}, Ki={current_ki:.4f}"
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
            "<9f",
            float(self.autotune_rs_low_spin.value()),
            float(self.autotune_rs_high_spin.value()),
            float(self.autotune_ls_voltage_spin.value()),
            float(self.autotune_ls_frequency_spin.value()),
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
            return "Ls Sine Injection Response"
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
                {"label": "Vd Sine", "unit": "V", "color": "#d62728"},
                {"label": "Measure Window", "unit": "", "color": "#7f7f7f"},
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
            self.autotune_kt_value_label.setText("-")
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
        autotune_kt = (
            1.5
            * float(snapshot.autotune_measured_pole_pairs)
            * float(snapshot.autotune_measured_flux)
        )
        self.autotune_kt_value_label.setText(f"{autotune_kt:.6f} Nm/A")
        self.autotune_current_gain_value_label.setText(
            f"Kp={snapshot.autotune_current_kp:.4f}, Ki={snapshot.autotune_current_ki:.4f}"
        )
        self.autotune_speed_gain_value_label.setText(
            f"Kp={snapshot.autotune_speed_kp:.4f}, Ki={snapshot.autotune_speed_ki:.4f}"
        )
        self.autotune_position_gain_value_label.setText(
            f"Kp={snapshot.autotune_position_kp:.4f}, Ki={snapshot.autotune_position_ki:.4f}"
        )
        self.autotune_apply_button.setEnabled(
            int(snapshot.autotune_state) == MOTOR_AUTOTUNE_STATE_DONE
        )
        self._maybe_continue_autotune_stage(snapshot)

    def _clear_autotune_capture(self) -> None:
        self._autotune_capture = _empty_scope_state("Motor Auto-Tune Charts")
        self._autotune_continue_signature = None
        self.autotune_scope_view.clear()
        self.autotune_chart_state_label.setText("Waiting for capture")
        if self._active_trace_target == "autotune":
            self._active_trace_target = None

    def _autotune_capture_stage(self) -> int:
        if self._autotune_capture.title == self._autotune_chart_title(MOTOR_AUTOTUNE_CHART_RS):
            return MOTOR_AUTOTUNE_CHART_RS
        if self._autotune_capture.title == self._autotune_chart_title(MOTOR_AUTOTUNE_CHART_LS):
            return MOTOR_AUTOTUNE_CHART_LS
        return MOTOR_AUTOTUNE_CHART_NONE

    def _autotune_expected_chart_stage(self, autotune_state: int) -> int:
        if int(autotune_state) == MOTOR_AUTOTUNE_STATE_RS:
            return MOTOR_AUTOTUNE_CHART_RS
        if int(autotune_state) == MOTOR_AUTOTUNE_STATE_LS:
            return MOTOR_AUTOTUNE_CHART_LS
        return MOTOR_AUTOTUNE_CHART_NONE

    def _maybe_continue_autotune_stage(self, snapshot=None) -> None:
        active_snapshot = self._latest_monitor_snapshot if snapshot is None else snapshot
        if active_snapshot is None:
            return
        if self._autotune_capture.total_samples <= 0:
            return
        if self._autotune_capture.received_samples < self._autotune_capture.total_samples:
            return
        if int(active_snapshot.autotune_data_ready) == 0:
            return

        capture_stage = self._autotune_capture_stage()
        expected_stage = self._autotune_expected_chart_stage(active_snapshot.autotune_state)
        if capture_stage == MOTOR_AUTOTUNE_CHART_NONE or capture_stage != expected_stage:
            return

        signature = (capture_stage, int(self._autotune_capture.total_samples))
        if self._autotune_continue_signature == signature:
            return

        self._autotune_continue_signature = signature
        self.autotune_chart_state_label.setText("Chart complete, continuing auto-tune...")
        self._enqueue_command(
            Command.CMD_CONTINUE_AUTO_TUNING_STATE,
            b"",
            "Continue Auto-Tune Stage",
            quiet=True,
        )

    def _start_motor_autotune(self) -> None:
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None and last_monitor.enable_run:
            QtWidgets.QMessageBox.warning(
                self,
                "Auto-Tune Safety",
                "Stop the drive before starting auto-tune. Rs/Ls assume a locked rotor, and the flux stage should only run after the electrical-zero setup is ready.",
            )
            return

        self.auto_poll_checkbox.setChecked(True)
        self._clear_autotune_capture()
        self.autotune_chart_state_label.setText("Starting auto-tune...")
        self._enqueue_command(
            Command.CMD_START_AUTOTUNING_T,
            self._build_autotune_payload(),
            "Start Motor Auto-Tune",
        )
        self._request_monitor_once()

    def _stop_motor_autotune(self) -> None:
        self._autotune_continue_signature = None
        self._enqueue_command(
            Command.CMD_STOP_AUTOTUNING_T,
            b"",
            "Stop Motor Auto-Tune",
        )
        self.autotune_chart_state_label.setText("Stopped")

    def _apply_motor_autotune_estimates(self) -> None:
        synced_from_snapshot = self._sync_parameter_tables_from_autotune_snapshot()
        if synced_from_snapshot:
            self._set_parameter_status(
                "Prepared auto-tune gains in GUI. Applying them to firmware...",
                "info",
            )
        self._enqueue_command(
            Command.CMD_UPDATE_TUNING_GAIN,
            b"",
            "Apply Auto-Tune Estimates",
        )

    def _sync_parameter_tables_from_autotune_snapshot(self) -> bool:
        snapshot = self._latest_monitor_snapshot
        if snapshot is None:
            return False

        if hasattr(self, "motor_table"):
            self._set_table_float_value(
                self.motor_table,
                MOTOR_PARAMETER_NAMES.index("MOTOR_RESISTANCE"),
                float(snapshot.autotune_measured_rs) * 1000.0,
            )
            self._set_table_float_value(
                self.motor_table,
                MOTOR_PARAMETER_NAMES.index("MOTOR_INDUCTANCE"),
                float(snapshot.autotune_measured_ls) * 1.0e6,
            )
            self._set_table_float_value(
                self.motor_table,
                MOTOR_PARAMETER_NAMES.index("MOTOR_BACK_EMF_CONSTANT"),
                float(snapshot.autotune_measured_ke) * 1000.0,
            )
            self._set_table_float_value(
                self.motor_table,
                MOTOR_PARAMETER_NAMES.index("MOTOR_NUMBER_POLE_PAIRS"),
                float(snapshot.autotune_measured_pole_pairs),
            )
            self._set_table_float_value(
                self.motor_table,
                MOTOR_PARAM_CURRENT_P_GAIN,
                float(snapshot.autotune_current_kp),
            )
            self._set_table_float_value(
                self.motor_table,
                MOTOR_PARAM_CURRENT_I_GAIN,
                float(snapshot.autotune_current_ki),
            )

        if hasattr(self, "driver_table"):
            self._set_table_float_value(
                self.driver_table,
                DRIVER_PARAM_SPEED_P_GAIN,
                float(snapshot.autotune_speed_kp),
            )
            self._set_table_float_value(
                self.driver_table,
                DRIVER_PARAM_SPEED_I_GAIN,
                float(snapshot.autotune_speed_ki),
            )
            self._set_table_float_value(
                self.driver_table,
                DRIVER_PARAM_POSITION_P_GAIN,
                float(snapshot.autotune_position_kp),
            )
            self._set_table_float_value(
                self.driver_table,
                DRIVER_PARAM_POSITION_I_GAIN,
                float(snapshot.autotune_position_ki),
            )
            self._load_foc_controls_from_driver_table()

        return True

    def _queue_autotune_parameter_resync(self) -> None:
        self._set_parameter_status(
            "Auto-tune estimates applied. Syncing driver and motor parameters...",
            "info",
        )
        self._enqueue_command(
            Command.CMD_READ_DRIVER,
            bytes([len(DRIVER_PARAMETER_NAMES)]),
            "Read Driver Parameters After Auto-Tune Apply",
        )
        self._enqueue_command(
            Command.CMD_READ_MOTOR,
            bytes([len(MOTOR_PARAMETER_NAMES)]),
            "Read Motor Parameters After Auto-Tune Apply",
        )
        self._enqueue_command(
            Command.CMD_UPDATE_MONITOR,
            b"",
            "Monitor Poll After Auto-Tune Apply",
            quiet=True,
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
        self._maybe_continue_autotune_stage()

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

    def _uerror_state_text(self, state: int) -> str:
        return {
            0: "Idle",
            1: "Settling",
            2: "Averaging",
            3: "Done",
            4: "Aborted",
            5: "Error",
        }.get(int(state), f"State {state}")

    def _set_uerror_status(self, text: str, progress_percent: int | None = None) -> None:
        self._uerror_status_text = text
        if hasattr(self, "uerror_status_label"):
            self.uerror_status_label.setText(text)
        if (progress_percent is not None) and hasattr(self, "uerror_progress_bar"):
            self.uerror_progress_bar.setValue(max(0, min(int(progress_percent), 100)))

    def _clear_uerror_characterization(self) -> None:
        self._uerror_samples = []
        self._uerror_total_samples = 0
        self._uerror_state = 0
        self._uerror_lut_preview_norm = []
        self._uerror_lut_preview_current_max_a = 0.0
        self._set_uerror_status("Waiting for survey...", 0)
        self._refresh_uerror_plots()

    def _start_uerror_characterization(self) -> None:
        payload = struct.pack(
            "<7f",
            float(self.uerror_rs_actual_spin.value()),
            float(self.uerror_sweep_current_spin.value()),
            float(self.uerror_fine_zone_spin.value()),
            float(self.uerror_fine_step_spin.value()),
            float(self.uerror_coarse_step_spin.value()),
            float(self.uerror_settle_ms_spin.value()),
            float(self.uerror_average_samples_spin.value()),
        )
        self._clear_uerror_characterization()
        self._set_uerror_status("Starting survey...", 0)
        self._enqueue_command(
            Command.CMD_START_UERROR_CHARACTERIZATION,
            payload,
            "Start Uerror Characterization",
        )

    def _stop_uerror_characterization(self) -> None:
        self._enqueue_command(
            Command.CMD_STOP_UERROR_CHARACTERIZATION,
            b"",
            "Stop Uerror Characterization",
        )
        self._set_uerror_status("Stopping survey...", self.uerror_progress_bar.value())

    def _build_uerror_plot_series(
        self,
    ) -> tuple[
        list[list[tuple[float, float]]],
        list[list[tuple[float, float]]],
        list[list[tuple[float, float]]],
        list[list[tuple[float, float]]],
        list[tuple[float, float]],
    ]:
        voltage_raw_series = [[], [], []]
        voltage_trend_series = [[], [], []]
        uerror_raw_series = [[], [], []]
        uerror_trend_series = [[], [], []]
        merged_norm: list[tuple[float, float]] = []
        rs_actual = float(self.uerror_rs_actual_spin.value()) if hasattr(self, "uerror_rs_actual_spin") else 0.0

        def smoothed_series(
            series: list[tuple[float, float]],
            *,
            window_radius: int = 2,
        ) -> list[tuple[float, float]]:
            if len(series) <= 2:
                return list(series)
            smoothed: list[tuple[float, float]] = []
            for index, (x_value, _) in enumerate(series):
                start = max(0, index - window_radius)
                end = min(len(series), index + window_radius + 1)
                y_avg = sum(point[1] for point in series[start:end]) / float(end - start)
                smoothed.append((x_value, y_avg))
            return smoothed

        for sample in self._uerror_samples:
            if (
                abs(sample.bus_voltage_v) < 1.0e-9
                and abs(sample.target_current_a) < 1.0e-9
                and abs(sample.phase_u_a) < 1.0e-9
                and abs(sample.phase_v_a) < 1.0e-9
                and abs(sample.phase_w_a) < 1.0e-9
            ):
                continue
            phase_points = [
                (sample.phase_u_a, sample.phase_voltage_u_v),
                (sample.phase_v_a, sample.phase_voltage_v_v),
                (sample.phase_w_a, sample.phase_voltage_w_v),
            ]
            for series_index, (phase_current, phase_voltage) in enumerate(phase_points):
                voltage_raw_series[series_index].append((phase_current, phase_voltage))
                uerror_value = phase_voltage - (rs_actual * phase_current)
                uerror_raw_series[series_index].append((phase_current, uerror_value))
                if sample.bus_voltage_v > 1.0e-6:
                    merged_norm.append((phase_current, uerror_value / sample.bus_voltage_v))
        for series in voltage_raw_series:
            series.sort(key=lambda item: item[0])
        for series in uerror_raw_series:
            series.sort(key=lambda item: item[0])
        for raw_series, trend_series in zip(voltage_raw_series, voltage_trend_series):
            trend_series.extend(smoothed_series(raw_series))
        for raw_series, trend_series in zip(uerror_raw_series, uerror_trend_series):
            trend_series.extend(smoothed_series(raw_series))
        merged_norm.sort(key=lambda item: item[0])
        return (
            voltage_raw_series,
            voltage_trend_series,
            uerror_raw_series,
            uerror_trend_series,
            merged_norm,
        )

    def _refresh_uerror_plots(self) -> None:
        if not hasattr(self, "uerror_voltage_plot"):
            return

        if not self._uerror_samples:
            self.uerror_voltage_plot.clear("Waiting for survey data...")
            self.uerror_uerror_plot.clear("Waiting for survey data...")
            self.uerror_lut_plot.clear("Waiting for survey data...")
            return

        (
            voltage_raw_series,
            voltage_trend_series,
            uerror_raw_series,
            uerror_trend_series,
            merged_norm,
        ) = self._build_uerror_plot_series()
        phase_colors = ["#1f77b4", "#2ca02c", "#d62728"]
        phase_labels = ["Phase U", "Phase V", "Phase W"]
        voltage_series_defs: list[dict[str, object]] = []
        voltage_series_data: list[list[tuple[float, float]]] = []
        uerror_series_defs: list[dict[str, object]] = []
        uerror_series_data: list[list[tuple[float, float]]] = []
        for color, label, raw_series, trend_series in zip(
            phase_colors,
            phase_labels,
            voltage_raw_series,
            voltage_trend_series,
        ):
            voltage_series_defs.extend(
                [
                    {
                        "label": f"{label} Raw",
                        "color": color,
                        "mode": "scatter",
                        "marker_radius": 2.3,
                        "alpha": 95,
                        "show_legend": False,
                    },
                    {
                        "label": label,
                        "color": color,
                        "mode": "line",
                        "line_width": 2.1,
                    },
                ]
            )
            voltage_series_data.extend([raw_series, trend_series])
        for color, label, raw_series, trend_series in zip(
            phase_colors,
            phase_labels,
            uerror_raw_series,
            uerror_trend_series,
        ):
            uerror_series_defs.extend(
                [
                    {
                        "label": f"{label} Raw",
                        "color": color,
                        "mode": "scatter",
                        "marker_radius": 2.3,
                        "alpha": 95,
                        "show_legend": False,
                    },
                    {
                        "label": label,
                        "color": color,
                        "mode": "line",
                        "line_width": 2.1,
                    },
                ]
            )
            uerror_series_data.extend([raw_series, trend_series])
        sample_count_text = (
            f"{len(merged_norm) // 3 if merged_norm else 0} sweep points captured. "
            "Points are mapped phase-current samples; solid lines show a local trend."
        )
        self.uerror_voltage_plot.set_plot(
            title="Phase Command Voltage vs Current",
            x_label="Mapped Phase Current",
            x_unit="A",
            y_label="Phase Voltage",
            y_unit="V",
            series_defs=voltage_series_defs,
            series_data=voltage_series_data,
            status_text=sample_count_text,
        )
        self.uerror_uerror_plot.set_plot(
            title="Raw Uerror vs Current",
            x_label="Mapped Phase Current",
            x_unit="A",
            y_label="Uerror",
            y_unit="V",
            series_defs=uerror_series_defs,
            series_data=uerror_series_data,
            status_text=sample_count_text,
        )

        self._uerror_lut_preview_norm = []
        self._uerror_lut_preview_current_max_a = 0.0
        if not merged_norm:
            self.uerror_lut_plot.clear("No valid Vbus samples available for LUT preview.")
            return

        currents = [point[0] for point in merged_norm]
        values = [point[1] for point in merged_norm]
        current_max = max(abs(min(currents)), abs(max(currents)))
        if current_max < 1.0e-6:
            self.uerror_lut_plot.clear("Measured phase current range is too small for LUT preview.")
            return

        grid_count = UERROR_LUT_POINT_COUNT_MAX
        grid = [
            (-current_max + ((2.0 * current_max * index) / max(grid_count - 1, 1)))
            for index in range(grid_count)
        ]
        lut_values: list[float] = []
        for current_target in grid:
            insert_at = bisect_left(currents, current_target)
            if insert_at <= 0:
                lut_values.append(values[0])
                continue
            if insert_at >= len(currents):
                lut_values.append(values[-1])
                continue
            x0 = currents[insert_at - 1]
            x1 = currents[insert_at]
            y0 = values[insert_at - 1]
            y1 = values[insert_at]
            if abs(x1 - x0) < 1.0e-9:
                lut_values.append(0.5 * (y0 + y1))
                continue
            ratio = (current_target - x0) / (x1 - x0)
            lut_values.append(y0 + ((y1 - y0) * ratio))

        if self.uerror_enforce_odd_checkbox.isChecked():
            midpoint = grid_count // 2
            for index in range(midpoint):
                odd_pos = 0.5 * (lut_values[-1 - index] - lut_values[index])
                lut_values[index] = -odd_pos
                lut_values[-1 - index] = odd_pos
            if grid_count % 2 == 1:
                lut_values[midpoint] = 0.0

        if self.uerror_smooth_checkbox.isChecked() and len(lut_values) >= 5:
            smoothed = list(lut_values)
            for index in range(1, len(lut_values) - 1):
                smoothed[index] = (
                    (lut_values[index - 1] * 0.25)
                    + (lut_values[index] * 0.50)
                    + (lut_values[index + 1] * 0.25)
                )
            lut_values = smoothed
            if self.uerror_enforce_odd_checkbox.isChecked():
                midpoint = grid_count // 2
                for index in range(midpoint):
                    odd_pos = 0.5 * (lut_values[-1 - index] - lut_values[index])
                    lut_values[index] = -odd_pos
                    lut_values[-1 - index] = odd_pos
                if grid_count % 2 == 1:
                    lut_values[midpoint] = 0.0

        self._uerror_lut_preview_norm = [float(value) for value in lut_values]
        self._uerror_lut_preview_current_max_a = float(current_max)
        raw_norm_series = merged_norm
        lut_preview_series = list(zip(grid, self._uerror_lut_preview_norm))
        self.uerror_lut_plot.set_plot(
            title="Normalized LUT Preview",
            x_label="Phase Current",
            x_unit="A",
            y_label="Uerror / Vbus",
            y_unit="pu",
            series_defs=[
                {
                    "label": "Merged Raw",
                    "color": "#7f7f7f",
                    "mode": "scatter",
                    "marker_radius": 2.0,
                    "alpha": 120,
                },
                {
                    "label": "Preview LUT",
                    "color": "#ff7f0e",
                    "mode": "line_scatter",
                    "line_width": 2.1,
                    "marker_radius": 2.4,
                    "marker_every": 4,
                },
            ],
            series_data=[raw_norm_series, lut_preview_series],
            status_text=f"{grid_count}-point shared LUT preview",
        )

    def _apply_uerror_runtime_lut(self) -> None:
        if (not self._uerror_lut_preview_norm) or (self._uerror_lut_preview_current_max_a <= 0.0):
            QtWidgets.QMessageBox.warning(
                self,
                "Apply Uerror LUT",
                "No valid LUT preview is available yet. Run a sweep first.",
            )
            return
        payload = build_uerror_lut_payload(
            self._uerror_lut_preview_current_max_a,
            self._uerror_lut_preview_norm,
        )
        self._enqueue_command(
            Command.CMD_APPLY_UERROR_LUT,
            payload,
            "Apply Uerror LUT",
        )
        self._set_uerror_status("Applying runtime LUT...", self.uerror_progress_bar.value())

    def _save_uerror_lut_to_flash(self) -> None:
        if (not self._uerror_lut_preview_norm) or (self._uerror_lut_preview_current_max_a <= 0.0):
            QtWidgets.QMessageBox.warning(
                self,
                "Save Uerror LUT",
                "No valid LUT preview is available yet. Run a sweep first.",
            )
            return
        last_monitor = self._latest_monitor_snapshot
        if last_monitor is not None and (
            bool(getattr(last_monitor, "enable_run", False))
            or abs(float(getattr(last_monitor, "act_speed", 0.0))) > 1.0
        ):
            QtWidgets.QMessageBox.warning(
                self,
                "Save Uerror LUT",
                "Servo OFF first, then wait until the motor is fully stopped before saving the LUT to flash. Firmware blocks flash writes while the drive state machine is still RUN/START.",
            )
            return
        payload = build_uerror_lut_payload(
            self._uerror_lut_preview_current_max_a,
            self._uerror_lut_preview_norm,
        )
        self._enqueue_command(
            Command.CMD_APPLY_UERROR_LUT,
            payload,
            "Apply Uerror LUT Before Save",
            quiet=True,
        )
        self._enqueue_command(
            Command.CMD_SAVE_UERROR_LUT_FLASH,
            b"",
            "Save Uerror LUT to Flash",
        )
        self._set_uerror_status("Saving LUT to flash...", self.uerror_progress_bar.value())

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
            if self._speed_test_session is not None:
                self._cancel_speed_test("Cancelled because the connection was closed.", request_stop=False)
            if self._position_test_session is not None:
                self._cancel_position_test("Cancelled because the connection was closed.", request_stop=False)
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
            if self._last_sent.command == Command.CMD_CONTINUE_AUTO_TUNING_STATE:
                self._autotune_continue_signature = None
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

    def _read_all_parameters(self) -> None:
        self._set_parameter_status("Reading driver and motor parameters...", "info")
        self._enqueue_command(
            Command.CMD_READ_DRIVER,
            bytes([len(DRIVER_PARAMETER_NAMES)]),
            "Read Driver Parameters",
        )
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
        if hasattr(self, "vf_toggle_button"):
            self.vf_toggle_button.setText("Stop V/F")

    def _stop_open_loop_vf(self) -> None:
        self._enqueue_command(Command.CMD_STOP_OPEN_LOOP_VF, b"", "Stop Open Loop V/F")
        if hasattr(self, "vf_toggle_button"):
            self.vf_toggle_button.setText("Start V/F")

    def _handle_frame(self, frame: ParsedFrame) -> None:
        if frame.code == ACK_NOERROR:
            sent_command = self._last_sent.command if self._last_sent is not None else None
            echoed_command = frame.payload[0] if frame.payload else None
            should_resync_after_autotune_apply = (
                sent_command == Command.CMD_UPDATE_TUNING_GAIN
            )
            if not (self._last_sent and self._last_sent.quiet):
                if echoed_command is None:
                    self._append_log("ACK received")
                else:
                    self._append_log(f"ACK received for command 0x{echoed_command:02X}")
            if sent_command == Command.CMD_WRITE_DRIVER:
                self._set_parameter_status("Driver parameter write acknowledged.", "ok")
            elif sent_command == Command.CMD_WRITE_MOTOR:
                self._set_parameter_status("Motor parameter write chunk acknowledged.", "ok")
            elif sent_command == Command.CMD_START_UERROR_CHARACTERIZATION:
                self._set_uerror_status("Survey running...", 0)
            elif sent_command == Command.CMD_STOP_UERROR_CHARACTERIZATION:
                self._set_uerror_status("Survey stop acknowledged.", self.uerror_progress_bar.value())
            elif sent_command == Command.CMD_APPLY_UERROR_LUT:
                self._set_uerror_status("Runtime LUT applied.", self.uerror_progress_bar.value())
            elif sent_command == Command.CMD_SAVE_UERROR_LUT_FLASH:
                self._set_uerror_status("LUT saved to flash.", self.uerror_progress_bar.value())
            elif should_resync_after_autotune_apply:
                self._set_parameter_status(
                    "Auto-tune estimates acknowledged. Refreshing GUI and parameter tables...",
                    "info",
                )
            self._ack_timer.stop()
            self._awaiting_ack = False
            self._last_sent = None
            if should_resync_after_autotune_apply:
                synced_from_snapshot = self._sync_parameter_tables_from_autotune_snapshot()
                if synced_from_snapshot:
                    self.statusBar().showMessage(
                        "Auto-tune gains applied and pushed into FOC/Test controls. Reading back parameters...",
                        5000,
                    )
                else:
                    self.statusBar().showMessage(
                        "Auto-tune gains applied. Reading back parameters from firmware...",
                        5000,
                    )
                self._queue_autotune_parameter_resync()
            self._try_send_next()
            return

        if frame.code == ACK_ERROR:
            sent_command = self._last_sent.command if self._last_sent is not None else None
            self._append_log("Driver returned ACK_ERROR")
            if sent_command == Command.CMD_CONTINUE_AUTO_TUNING_STATE:
                self._autotune_continue_signature = None
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
            elif sent_command == Command.CMD_UPDATE_TUNING_GAIN:
                self._set_parameter_status("Auto-tune estimate apply failed.", "error")
                QtWidgets.QMessageBox.warning(
                    self,
                    "Apply Auto-Tune Estimates",
                    "Firmware did not accept the auto-tune parameters.",
                )
            elif sent_command == Command.CMD_START_UERROR_CHARACTERIZATION:
                self._set_uerror_status("Survey start rejected by firmware.", 0)
                QtWidgets.QMessageBox.warning(
                    self,
                    "Start Uerror Characterization",
                    "Firmware did not accept the Uerror survey request.",
                )
            elif sent_command == Command.CMD_APPLY_UERROR_LUT:
                self._set_uerror_status("Runtime LUT apply failed.", self.uerror_progress_bar.value())
                QtWidgets.QMessageBox.warning(
                    self,
                    "Apply Uerror LUT",
                    "Firmware did not accept the LUT payload.",
                )
            elif sent_command == Command.CMD_SAVE_UERROR_LUT_FLASH:
                self._set_uerror_status("Flash save failed.", self.uerror_progress_bar.value())
                QtWidgets.QMessageBox.warning(
                    self,
                    "Save Uerror LUT",
                    "Firmware could not save the LUT to flash.",
                )
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

        if subcommand == UpdateCode.CMD_UERROR_DATA:
            try:
                chunk = parse_uerror_payload(body)
            except ValueError as exc:
                self._append_log(f"Uerror parse error: {exc}")
                return
            self._handle_uerror_chunk(chunk)
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

    def _handle_uerror_chunk(self, chunk: UerrorSweepChunk) -> None:
        self._uerror_state = int(chunk.state)
        self._uerror_total_samples = max(0, int(chunk.total_samples))

        expected_size = max(self._uerror_total_samples, chunk.sample_start + chunk.sample_count)
        if expected_size > len(self._uerror_samples):
            self._uerror_samples.extend(
                UerrorRawSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                for _ in range(expected_size - len(self._uerror_samples))
            )

        for index in range(chunk.sample_count):
            sample_index = chunk.sample_start + index
            if sample_index >= len(self._uerror_samples):
                continue
            self._uerror_samples[sample_index] = UerrorRawSample(
                target_current_a=chunk.target_current[index],
                measured_id_a=chunk.measured_id[index],
                phase_u_a=chunk.phase_u[index],
                phase_v_a=chunk.phase_v[index],
                phase_w_a=chunk.phase_w[index],
                phase_voltage_u_v=chunk.v_phase_u[index],
                phase_voltage_v_v=chunk.v_phase_v[index],
                phase_voltage_w_v=chunk.v_phase_w[index],
                bus_voltage_v=chunk.vdc[index],
                temperature_c=chunk.temperature[index],
            )

        valid_count = sum(
            1
            for sample in self._uerror_samples
            if (
                abs(sample.bus_voltage_v) >= 1.0e-9
                or abs(sample.target_current_a) >= 1.0e-9
                or abs(sample.phase_u_a) >= 1.0e-9
                or abs(sample.phase_v_a) >= 1.0e-9
                or abs(sample.phase_w_a) >= 1.0e-9
            )
        )
        progress_percent = 0
        if self._uerror_total_samples > 0:
            progress_percent = int(round((valid_count / self._uerror_total_samples) * 100.0))
        self._set_uerror_status(
            f"{self._uerror_state_text(chunk.state)}: {valid_count}/{self._uerror_total_samples} points",
            progress_percent,
        )
        self._refresh_uerror_plots()

    def _append_snapshot_to_trend_buffer(self, snapshot) -> None:
        tracking_mode = self._position_tracking_mode()
        cmd_position_deg = self._counts_to_position_mode_degrees(snapshot.cmd_position, tracking_mode)
        act_position_deg = self._counts_to_position_mode_degrees(snapshot.act_position, tracking_mode)
        position_error_deg = self._display_position_error_degrees(
            snapshot.cmd_position,
            snapshot.act_position,
            tracking_mode,
        )
        validation_error_deg = self._current_position_validation_error_degrees(
            snapshot.cmd_position,
            snapshot.act_position,
            tracking_mode,
        )
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
                "cmd_position_deg": cmd_position_deg,
                "act_position_deg": act_position_deg,
                "position_error_deg": position_error_deg,
                "validation_error_deg": validation_error_deg,
            },
        )

    def _append_error_snapshot_to_trend_buffer(self, snapshot) -> None:
        last_monitor = self._latest_monitor_snapshot
        cmd_position_counts = float(last_monitor.cmd_position) if last_monitor is not None else 0.0
        act_position_counts = float(snapshot.act_position)
        tracking_mode = self._position_tracking_mode()
        cmd_position_deg = self._counts_to_position_mode_degrees(cmd_position_counts, tracking_mode)
        act_position_deg = self._counts_to_position_mode_degrees(act_position_counts, tracking_mode)
        position_error_deg = self._display_position_error_degrees(
            cmd_position_counts,
            act_position_counts,
            tracking_mode,
        )
        validation_error_deg = self._current_position_validation_error_degrees(
            cmd_position_counts,
            act_position_counts,
            tracking_mode,
        )
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
                "cmd_position_deg": cmd_position_deg,
                "act_position_deg": act_position_deg,
                "position_error_deg": position_error_deg,
                "validation_error_deg": validation_error_deg,
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
            f"Kt: {1.5 * float(snapshot.autotune_measured_pole_pairs) * float(snapshot.autotune_measured_flux):.6f} Nm/A",
            f"Current PI: Kp={snapshot.autotune_current_kp:.4f}, Ki={snapshot.autotune_current_ki:.4f}",
            f"Speed PI: Kp={snapshot.autotune_speed_kp:.4f}, Ki={snapshot.autotune_speed_ki:.4f}",
            f"Position PI: Kp={snapshot.autotune_position_kp:.4f}, Ki={snapshot.autotune_position_ki:.4f}",
            "",
            "[Motion]",
            f"Cmd Speed: {snapshot.cmd_speed:.3f}",
            f"Act Speed: {snapshot.act_speed:.3f}",
            f"Speed Error: {snapshot.speed_error:.3f}",
            f"Setting Position: {self._counts_to_position_mode_degrees(snapshot.cmd_position):.3f} deg",
            f"Tracking Error: {self._display_position_error_degrees(snapshot.cmd_position, snapshot.act_position):.3f} deg",
            f"Mechanical Angle (Single-turn): {self._counts_to_single_turn_degrees(snapshot.act_position):.3f} deg",
            f"Mechanical Angle (Multi-turn): {self._counts_to_accumulated_degrees(snapshot.act_position):.3f} deg",
            f"Total Distance: {self._counts_to_turns(snapshot.act_position):.3f} turns",
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
                if hasattr(self, "monitor_tabs") and hasattr(self, "driver_monitor_tab"):
                    self.monitor_tabs.setCurrentWidget(self.driver_monitor_tab)
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
        position_error_deg = self._display_position_error_degrees(
            snapshot.cmd_position,
            snapshot.act_position,
        )
        setting_position_deg = self._counts_to_position_mode_degrees(snapshot.cmd_position)
        mechanical_angle_single_deg = self._counts_to_single_turn_degrees(snapshot.act_position)
        mechanical_angle_multi_deg = self._counts_to_accumulated_degrees(snapshot.act_position)
        total_distance_turns = self._counts_to_turns(snapshot.act_position)
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
        self._set_monitor_value("setting_position", f"{setting_position_deg:.3f} deg")
        self._set_monitor_value("tracking_error", f"{position_error_deg:.3f} deg")
        self._set_monitor_value("mech_angle_single", f"{mechanical_angle_single_deg:.3f} deg")
        self._set_monitor_value("mech_angle_multi", f"{mechanical_angle_multi_deg:.3f} deg")
        self._set_monitor_value("total_distance", f"{total_distance_turns:.3f} turns")
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
        position_error_deg = self._display_position_error_degrees(
            getattr(self._latest_monitor_snapshot, "cmd_position", 0.0),
            snapshot.act_position,
        )
        cmd_position_counts = getattr(self._latest_monitor_snapshot, "cmd_position", 0.0)
        setting_position_deg = self._counts_to_position_mode_degrees(cmd_position_counts)
        mechanical_angle_single_deg = self._counts_to_single_turn_degrees(snapshot.act_position)
        mechanical_angle_multi_deg = self._counts_to_accumulated_degrees(snapshot.act_position)
        total_distance_turns = self._counts_to_turns(snapshot.act_position)
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
        self._set_monitor_value("setting_position", f"{setting_position_deg:.3f} deg")
        self._set_monitor_value("tracking_error", f"{position_error_deg:.3f} deg")
        self._set_monitor_value("mech_angle_single", f"{mechanical_angle_single_deg:.3f} deg")
        self._set_monitor_value("mech_angle_multi", f"{mechanical_angle_multi_deg:.3f} deg")
        self._set_monitor_value("total_distance", f"{total_distance_turns:.3f} turns")
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
