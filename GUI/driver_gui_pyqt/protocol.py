"""USB serial protocol helpers for the ASD04 driver."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

PROTOCOL_STX = 0x02
PROTOCOL_SYN = 0x16
PROTOCOL_ETX = 0x03
ACK_NOERROR = 0xF0
ACK_ERROR = 0xFF

HEADER_SIZE = 2
CRC_SIZE = 1
REG_SIZE = 1
CURRENT_LOOP_FREQUENCY_HZ = 16000.0
TRACE_SAMPLES_PER_CHUNK = 10
TRACE_CHANNELS_PER_SAMPLE = 4
CURRENT_TUNING_SAMPLES_PER_CHUNK = 20
CURRENT_TUNING_TOTAL_SAMPLES = 600
TRACE_TOTAL_SAMPLES = 1000
AUTOTUNE_SAMPLES_PER_CHUNK = 8

CONTROL_TIMING_MODE_16KHZ = 0
SPEED_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 2
RUN_MODE_FOC = 0
RUN_MODE_OPEN_LOOP_VF = 1
RUN_MODE_ALIGNMENT_ONLY = 2
RUN_MODE_AUTOTUNE = 3
ENCODER_ALIGNMENT_POLICY_POWER_ON = 0
ENCODER_ALIGNMENT_POLICY_MANUAL_SAVE = 1
ENCODER_ALIGNMENT_STATUS_IDLE = 0
ENCODER_ALIGNMENT_STATUS_REQUESTED = 1
ENCODER_ALIGNMENT_STATUS_RUNNING = 2
ENCODER_ALIGNMENT_STATUS_DONE = 3
ENCODER_ALIGNMENT_STATUS_FAULT = 4
ID_SQUARE_ANGLE_TEST_NONE = 0
ID_SQUARE_ANGLE_TEST_PLUS_90 = 1
ID_SQUARE_ANGLE_TEST_MINUS_90 = 2
ID_SQUARE_ANGLE_TEST_PLUS_180 = 3
ID_SQUARE_TUNING_MODE_SQUARE_WAVE = 0
ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD = 1
MOTOR_AUTOTUNE_STATE_IDLE = 0
MOTOR_AUTOTUNE_STATE_RS = 1
MOTOR_AUTOTUNE_STATE_LS = 2
MOTOR_AUTOTUNE_STATE_FLUX = 3
MOTOR_AUTOTUNE_STATE_DONE = 4
MOTOR_AUTOTUNE_STATE_ERROR = 5
MOTOR_AUTOTUNE_ERROR_NONE = 0
MOTOR_AUTOTUNE_ERROR_OVERCURRENT = 1
MOTOR_AUTOTUNE_ERROR_STALL = 2
MOTOR_AUTOTUNE_ERROR_INVALID_CONFIG = 3
MOTOR_AUTOTUNE_ERROR_SIGNAL = 4
MOTOR_AUTOTUNE_CHART_NONE = 0
MOTOR_AUTOTUNE_CHART_RS = 1
MOTOR_AUTOTUNE_CHART_LS = 2
FOC_DIRECTION_TEST_IDLE = 0
FOC_DIRECTION_TEST_RUNNING = 1
FOC_DIRECTION_TEST_DONE_OK = 2
FOC_DIRECTION_TEST_DONE_FLIPPED = 3
FOC_DIRECTION_TEST_INCONCLUSIVE = 4
FOC_DIRECTION_TEST_FAULT = 5
CURRENT_CALIB_STATUS_IDLE = 0
CURRENT_CALIB_STATUS_RUNNING = 1
CURRENT_CALIB_STATUS_DONE = 2
CURRENT_CALIB_STATUS_TIMEOUT = 3
CURRENT_CALIB_STATUS_OFFSET_INVALID = 4

DEFAULT_MOTOR_RATED_CURRENT_RMS = 1.6
DEFAULT_MOTOR_PEAK_CURRENT_RMS = 3.2
DEFAULT_MOTOR_MAXIMUM_POWER = 200.0
DEFAULT_MOTOR_MAXIMUM_VOLTAGE = 91.0
DEFAULT_MOTOR_RATED_SPEED_RPM = 3000.0
DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ = 200.0
DEFAULT_MOTOR_POLE_PAIRS = 4.0


class Command(IntEnum):
    CMD_SERVO_ON = 0x01
    CMD_SERVO_OFF = 0x02
    CMD_STOP_PPF = 0x03
    CMD_APPLY_SPEED_FILTER = 0x04
    CMD_APPLY_BODE_OPEN_CLOSE_LOOP = 0x05
    CMD_APPLY_CTUNING = 0x06
    CMD_APPLY_STUNING = 0x07
    CMD_START_CTUNING = 0x08
    CMD_STOP_CTUNING = 0x09
    CMD_START_AUTOCOMMUTATION = 0x0A
    CMD_APPLY_MAN_JOG = 0x0B
    CMD_START_TORQUECONTROL = 0x0C
    CMD_STOP_TORQUECONTROL = 0x0D
    CMD_START_SPEEDCONTROL = 0x0E
    CMD_STOP_SPEEDCONTROL = 0x0F
    CMD_START_POSITIONCONTROL = 0x10
    CMD_STOP_POSITIONCONTROL = 0x11
    CMD_SERVO_JOGPROGRAM = 0x12
    CMD_SVOFF_JOGPROGRAM = 0x13
    CMD_START_JOGPROGRAM = 0x14
    CMD_STOP_JOGPROGRAM = 0x15
    CMD_START_MJOG_INC = 0x16
    CMD_START_MJOG_DEC = 0x17
    CMD_STOP_MJOG = 0x18
    CMD_RECALC_PROFILECONSTANT = 0x19
    CMD_START_STUNING = 0x1A
    CMD_STOP_STUNING = 0x1B
    CMD_UPDATE_MONITOR = 0x1C
    CMD_APPLY_TRACE = 0x1D
    CMD_APPLY_MJOG = 0x1E
    CMD_APPLY_JOGPROGRAM = 0x1F
    CMD_START_PSTUNING = 0x20
    CMD_STOP_PSTUNING = 0x21
    CMD_EMIT_MOTORCHANGE = 0x22
    CMD_SAVE_EEPROM = 0x23
    CMD_START_TRACE = 0x24
    CMD_READ_DRIVER = 0x25
    CMD_READ_MOTOR = 0x26
    CMD_WRITE_DRIVER = 0x27
    CMD_WRITE_MOTOR = 0x28
    CMD_ACK_FAULT = 0x29
    CMD_STOP_TRACE = 0x2A
    CMD_WRITE_TO_FLASH = 0x2B
    CMD_APPLY_FILTER_NGUYEN = 0x2C
    CMD_PLOT_BODE_NGUYEN = 0x2D
    CMD_APPLY_ATUNING_THINH = 0x2E
    CMD_EXECUTE_ATUNING_THINH = 0x2F
    CMD_START_COMMUTATION_NEW = 0x50
    CMD_START_AUTOTUNING_T = 0x51
    CMD_STOP_AUTOTUNING_T = 0x52
    CMD_UPDATE_TUNING_GAIN = 0x53
    CMD_CONTINUE_AUTO_TUNING_STATE = 0x54
    CMD_START_OPEN_LOOP_VF = 0x55
    CMD_STOP_OPEN_LOOP_VF = 0x56
    CMD_APPLY_ID_SQUARE_TUNING = 0x57
    CMD_START_ID_SQUARE_TUNING = 0x58
    CMD_STOP_ID_SQUARE_TUNING = 0x59
    CMD_SET_CONTROL_TIMING_MODE = 0x5A
    CMD_START_ENCODER_ALIGNMENT = 0x5B
    CMD_START_FOC_DIRECTION_TEST = 0x5C
    CMD_START_FOC_ANGLE_FIT = 0x5D
    CMD_START_FOC_ROTATING_THETA_TEST = 0x5E
    CMD_START_FOC_ROTATING_THETA_VOLTAGE_TEST = 0x5F
    CMD_START_FOC_CURRENT_FEEDBACK_MAP_TEST = 0x60


class UpdateCode(IntEnum):
    CMD_CTUNNING_DATA = 0x30
    CMD_STUNNING_DATA = 0x31
    CMD_READ_DRIVER_DATA = 0x32
    CMD_READ_MOTOR_DATA = 0x33
    CMD_TRACE_DATA = 0x34
    CMD_MONITOR_DATA = 0x35
    CMD_FFT_DATA_NGUYEN = 0x36
    CMD_FFT_DATA_THINH = 0x37
    CMD_AUTOTUNING_DATA_THINH = 0x38
    CMD_FOC_DEBUG_TEXT = 0x39
    MTR_CODE_ERROR = 0xE1


FAULT_FLAG_LABELS = {
    0x0001: "Overcurrent",
    0x0002: "Overvoltage",
    0x0004: "Undervoltage",
    0x0008: "Overtemperature",
    0x0010: "Startup Fail",
    0x0020: "Speed Following Error",
    0x0040: "Software Error",
    0x0080: "Flash Read Error",
    0x0100: "Calibration Timeout",
    0x0200: "Current Offset Invalid",
}
_KNOWN_FAULT_MASK = 0
for _fault_bit in FAULT_FLAG_LABELS:
    _KNOWN_FAULT_MASK |= _fault_bit


DRIVER_PARAMETER_NAMES = [
    "DEVICE_ID",
    "CONTROL_MODE",
    "POSITION_P_GAIN",
    "POSITION_FF_GAIN",
    "POSITION_FF_FILTER",
    "SPEED_P_GAIN",
    "SPEED_I_GAIN",
    "SPEED_FF_GAIN",
    "SPEED_FF_FILTER",
    "SPEED_DETECTION_FILTER_FREQUENCY",
    "ACCELERATION_TIME",
    "DECELERATION_TIME",
    "MAXIMUM_SPEED",
    "SPEED_MOVING_THRESHOLD",
    "SPEED_UNIT",
    "TORQUE_FILTER_FREQUENCY",
]


MOTOR_PARAMETER_NAMES = [
    "MOTOR_RATED_CURRENT_RMS",
    "MOTOR_PEAK_CURRENT_RMS",
    "MOTOR_RESISTANCE",
    "MOTOR_INDUCTANCE",
    "MOTOR_BACK_EMF_CONSTANT",
    "MOTOR_ROTOR_INERTIA",
    "MOTOR_ABS_ENCODER_MODE",
    "MOTOR_ENCODER_ID",
    "MOTOR_ENCODER_RESOLUTION",
    "MOTOR_MAXIMUM_POWER",
    "MOTOR_MAXIMUM_VOLTAGE",
    "MOTOR_NUMBER_POLE_PAIRS",
    "MOTOR_RATED_TORQUE",
    "MOTOR_MAXIMUM_SPEED",
    "MOTOR_OVERLOAD_TORQUE",
    "MOTOR_OVERLOAD_TIME",
    "MOTOR_CURRENT_P_GAIN",
    "MOTOR_CURRENT_I_GAIN",
    "MOTOR_HALL_OFFSET",
    "MOTOR_CURRENT_CTRL_DIRECTION",
    "MOTOR_FORWARD_HALL_0",
    "MOTOR_FORWARD_HALL_1",
    "MOTOR_FORWARD_HALL_2",
    "MOTOR_FORWARD_HALL_3",
    "MOTOR_FORWARD_HALL_4",
    "MOTOR_FORWARD_HALL_5",
    "MOTOR_REVERSE_HALL_0",
    "MOTOR_REVERSE_HALL_1",
    "MOTOR_REVERSE_HALL_2",
    "MOTOR_REVERSE_HALL_3",
    "MOTOR_REVERSE_HALL_4",
    "MOTOR_REVERSE_HALL_5",
]


@dataclass(slots=True)
class ParsedFrame:
    code: int
    size: int
    payload: bytes
    raw: bytes

    @property
    def is_ack(self) -> bool:
        return self.code in (ACK_NOERROR, ACK_ERROR)

    @property
    def is_syn(self) -> bool:
        return self.code == PROTOCOL_SYN

    @property
    def subcommand(self) -> int | None:
        if not self.payload:
            return None
        return self.payload[0]


@dataclass(slots=True)
class MonitorSnapshot:
    enable_run: bool
    vdc: float
    temperature: float
    cmd_speed: float
    act_speed: float
    speed_error: float
    cmd_position: float
    act_position: float
    position_error: float
    iq_ref: float
    motor_power: int
    fault_occurred: int
    run_mode: int = 0
    id_ref: float = 0.0
    phase_u: float = 0.0
    phase_v: float = 0.0
    phase_w: float = 0.0
    id_current: float = 0.0
    iq_current: float = 0.0
    vd: float = 0.0
    vq: float = 0.0
    v_phase_u: float = 0.0
    v_phase_v: float = 0.0
    v_phase_w: float = 0.0
    theta: float = 0.0
    vf_frequency: float = 0.0
    vf_voltage: float = 0.0
    debug_open_loop_electrical_hz_cmd: float = 0.0
    debug_open_loop_sync_rpm_cmd: float = 0.0
    debug_observed_electrical_hz: float = 0.0
    debug_speed_raw_rpm: float = 0.0
    debug_expected_delta_pos_sync: float = 0.0
    debug_delta_pos_avg: float = 0.0
    debug_encoder_turns: float = 0.0
    debug_mechanical_angle_rad: float = 0.0
    debug_electrical_angle_rad: float = 0.0
    debug_speed_raw_rpm_avg: float = 0.0
    debug_observed_electrical_hz_avg: float = 0.0
    debug_delta_pos: int = 0
    debug_enc_single_turn: int = 0
    debug_isr_delta_cycles: int = 0
    debug_isr_period_us: float = 0.0
    debug_isr_frequency_hz: float = 0.0
    debug_isr_measure_only_mode: int = 0
    debug_isr_measure_edge_frequency_hz: float = 0.0
    control_timing_mode: int = CONTROL_TIMING_MODE_16KHZ
    control_loop_frequency_hz: float = CURRENT_LOOP_FREQUENCY_HZ
    speed_loop_frequency_hz: float = CURRENT_LOOP_FREQUENCY_HZ * 0.5
    debug_encoder_offset_counts: int = 0
    debug_alignment_captured_offset_counts: int = 0
    debug_alignment_policy: int = ENCODER_ALIGNMENT_POLICY_POWER_ON
    debug_alignment_status: int = ENCODER_ALIGNMENT_STATUS_IDLE
    debug_alignment_needs_flash_save: int = 0
    autotune_state: int = MOTOR_AUTOTUNE_STATE_IDLE
    autotune_error: int = MOTOR_AUTOTUNE_ERROR_NONE
    autotune_progress_percent: int = 0
    autotune_data_ready: int = 0
    autotune_measured_rs: float = 0.0
    autotune_measured_ls: float = 0.0
    autotune_measured_ke: float = 0.0
    autotune_measured_flux: float = 0.0
    autotune_measured_pole_pairs: float = 0.0
    autotune_current_kp: float = 0.0
    autotune_current_ki: float = 0.0
    autotune_speed_kp: float = 0.0
    autotune_speed_ki: float = 0.0
    autotune_position_kp: float = 0.0
    foc_direction_test_status: int = FOC_DIRECTION_TEST_IDLE
    foc_direction_test_open_loop_delta_pos: int = 0
    foc_direction_test_foc_delta_pos: int = 0
    adc_offset_ia: int = 0x7FFF
    adc_offset_ib: int = 0x7FFF
    calibration_status: int = CURRENT_CALIB_STATUS_IDLE


@dataclass(slots=True)
class ErrorSnapshot:
    fault_code: int
    vdc: float
    temperature: float
    cmd_speed: float
    act_speed: float
    speed_error: float
    cmd_position: float
    act_position: float
    position_error: float
    iq_ref: float
    motor_power: int
    phase_u: float
    phase_v: float
    phase_w: float


@dataclass(slots=True)
class TraceChunk:
    chunk_index: int
    channels: tuple[list[float], list[float], list[float], list[float]]


@dataclass(slots=True)
class CurrentTuningChunk:
    chunk_index: int
    reference: list[float]
    feedback: list[float]


@dataclass(slots=True)
class AutoTuneChunk:
    stage: int
    sample_start: int
    sample_count: int
    total_samples: int
    sample_period_s: float
    primary: list[float]
    secondary: list[float]
    tertiary: list[float]


def calc_crc(u_code: int, u_size: int, buffer: bytes) -> int:
    """CRC logic copied from both firmware and legacy Qt app."""

    total = u_size
    for item in buffer[:u_size]:
        total += item
    crc = total & 0xFF
    crc += (total >> 8) & 0xFF
    return crc & 0xFF


def build_command_frame(command: int, payload: bytes = b"") -> bytes:
    size = len(payload) + REG_SIZE
    frame = bytearray()
    frame.append(PROTOCOL_STX)
    frame.append(PROTOCOL_SYN)
    frame.append(size)
    frame.append(command & 0xFF)
    frame.extend(payload)
    frame.append(calc_crc(frame[1], frame[2], frame[3 : 3 + size]))
    frame.append(PROTOCOL_ETX)
    return bytes(frame)


class FrameStreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[ParsedFrame]:
        frames: list[ParsedFrame] = []
        if not data:
            return frames

        self._buffer.extend(data)
        while len(self._buffer) >= 5:
            if self._buffer[0] != PROTOCOL_STX:
                del self._buffer[0]
                continue

            size = self._buffer[2]
            frame_len = size + 5
            if len(self._buffer) < frame_len:
                break

            if self._buffer[frame_len - 1] != PROTOCOL_ETX:
                del self._buffer[0]
                continue

            raw = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]

            code = raw[1]
            payload = raw[3 : 3 + size]
            crc = raw[3 + size]
            if calc_crc(code, size, payload) != crc:
                continue
            frames.append(ParsedFrame(code=code, size=size, payload=payload, raw=raw))
        return frames


def parse_monitor_payload(payload: bytes) -> MonitorSnapshot:
    if len(payload) < 41:
        raise ValueError(f"Monitor payload too short: {len(payload)}")

    offset = 0
    enable_run = bool(payload[offset])
    offset += 1

    fields = struct.unpack_from("<9f", payload, offset)
    offset += 9 * 4
    motor_power = struct.unpack_from("<h", payload, offset)[0]
    offset += 2
    fault_occurred = struct.unpack_from("<H", payload, offset)[0]

    # Firmware scales Vdc by 0.1 before sending monitor packets.
    vdc = fields[0] * 10.0
    snapshot = MonitorSnapshot(
        enable_run=enable_run,
        vdc=vdc,
        temperature=fields[1],
        cmd_speed=fields[2],
        act_speed=fields[3],
        speed_error=fields[4],
        cmd_position=fields[5],
        act_position=fields[6],
        position_error=fields[7],
        iq_ref=fields[8],
        motor_power=motor_power,
        fault_occurred=fault_occurred,
    )
    offset += 2

    if len(payload) >= 98:
        snapshot.run_mode = payload[offset]
        offset += 1
        (
            snapshot.id_ref,
            snapshot.phase_u,
            snapshot.phase_v,
            snapshot.phase_w,
            snapshot.id_current,
            snapshot.iq_current,
            snapshot.vd,
            snapshot.vq,
            snapshot.v_phase_u,
            snapshot.v_phase_v,
            snapshot.v_phase_w,
            snapshot.theta,
            snapshot.vf_frequency,
            snapshot.vf_voltage,
        ) = struct.unpack_from("<14f", payload, offset)
        offset += 14 * 4

    if len(payload) >= offset + (11 * 4) + (2 * 4):
        (
            snapshot.debug_open_loop_electrical_hz_cmd,
            snapshot.debug_open_loop_sync_rpm_cmd,
            snapshot.debug_observed_electrical_hz,
            snapshot.debug_speed_raw_rpm,
            snapshot.debug_expected_delta_pos_sync,
            snapshot.debug_delta_pos_avg,
            snapshot.debug_encoder_turns,
            snapshot.debug_mechanical_angle_rad,
            snapshot.debug_electrical_angle_rad,
            snapshot.debug_speed_raw_rpm_avg,
            snapshot.debug_observed_electrical_hz_avg,
            snapshot.debug_delta_pos,
            snapshot.debug_enc_single_turn,
        ) = struct.unpack_from("<11f2i", payload, offset)
        offset += struct.calcsize("<11f2i")

    if len(payload) >= offset + struct.calcsize("<I2f"):
        (
            snapshot.debug_isr_delta_cycles,
            snapshot.debug_isr_period_us,
            snapshot.debug_isr_frequency_hz,
        ) = struct.unpack_from("<I2f", payload, offset)
        offset += struct.calcsize("<I2f")

    if len(payload) >= offset + struct.calcsize("<Bf"):
        (
            snapshot.debug_isr_measure_only_mode,
            snapshot.debug_isr_measure_edge_frequency_hz,
        ) = struct.unpack_from("<Bf", payload, offset)
        offset += struct.calcsize("<Bf")

    if len(payload) >= offset + struct.calcsize("<B2f"):
        (
            snapshot.control_timing_mode,
            snapshot.control_loop_frequency_hz,
            snapshot.speed_loop_frequency_hz,
        ) = struct.unpack_from("<B2f", payload, offset)
        offset += struct.calcsize("<B2f")

    if len(payload) >= offset + struct.calcsize("<2i3B"):
        (
            snapshot.debug_encoder_offset_counts,
            snapshot.debug_alignment_captured_offset_counts,
            snapshot.debug_alignment_policy,
            snapshot.debug_alignment_status,
            snapshot.debug_alignment_needs_flash_save,
        ) = struct.unpack_from("<2i3B", payload, offset)
        offset += struct.calcsize("<2i3B")

    if len(payload) >= offset + struct.calcsize("<4B10f"):
        (
            snapshot.autotune_state,
            snapshot.autotune_error,
            snapshot.autotune_progress_percent,
            snapshot.autotune_data_ready,
            snapshot.autotune_measured_rs,
            snapshot.autotune_measured_ls,
            snapshot.autotune_measured_ke,
            snapshot.autotune_measured_flux,
            snapshot.autotune_measured_pole_pairs,
            snapshot.autotune_current_kp,
            snapshot.autotune_current_ki,
            snapshot.autotune_speed_kp,
            snapshot.autotune_speed_ki,
            snapshot.autotune_position_kp,
        ) = struct.unpack_from("<4B10f", payload, offset)
        offset += struct.calcsize("<4B10f")

    if len(payload) >= offset + struct.calcsize("<B2i"):
        (
            snapshot.foc_direction_test_status,
            snapshot.foc_direction_test_open_loop_delta_pos,
            snapshot.foc_direction_test_foc_delta_pos,
        ) = struct.unpack_from("<B2i", payload, offset)
        offset += struct.calcsize("<B2i")

    if len(payload) >= offset + struct.calcsize("<2HB"):
        (
            snapshot.adc_offset_ia,
            snapshot.adc_offset_ib,
            snapshot.calibration_status,
        ) = struct.unpack_from("<2HB", payload, offset)

    return snapshot


def parse_error_payload(payload: bytes) -> ErrorSnapshot:
    if len(payload) < 52:
        raise ValueError(f"Error payload too short: {len(payload)}")

    offset = 0
    fault_code = struct.unpack_from("<H", payload, offset)[0]
    offset += 2
    (
        vdc,
        temperature,
        cmd_speed,
        act_speed,
        speed_error,
        cmd_position,
        act_position,
        position_error,
        iq_ref,
    ) = struct.unpack_from("<9f", payload, offset)
    offset += 9 * 4
    motor_power = struct.unpack_from("<h", payload, offset)[0]
    offset += 2
    phase_u, phase_v, phase_w = struct.unpack_from("<3f", payload, offset)
    return ErrorSnapshot(
        fault_code=fault_code,
        vdc=vdc,
        temperature=temperature,
        cmd_speed=cmd_speed,
        act_speed=act_speed,
        speed_error=speed_error,
        cmd_position=cmd_position,
        act_position=act_position,
        position_error=position_error,
        iq_ref=iq_ref,
        motor_power=motor_power,
        phase_u=phase_u,
        phase_v=phase_v,
        phase_w=phase_w,
    )


def parse_trace_payload(payload: bytes) -> TraceChunk:
    expected_size = 2 + TRACE_SAMPLES_PER_CHUNK * TRACE_CHANNELS_PER_SAMPLE * 4
    if len(payload) < expected_size:
        raise ValueError(f"Trace payload too short: {len(payload)}")

    chunk_index = struct.unpack_from("<H", payload, 0)[0]
    offset = 2
    channels = ([], [], [], [])
    for _ in range(TRACE_SAMPLES_PER_CHUNK):
        values = struct.unpack_from("<4f", payload, offset)
        offset += 16
        for channel_index, value in enumerate(values):
            channels[channel_index].append(value)

    return TraceChunk(
        chunk_index=chunk_index,
        channels=channels,
    )


def parse_current_tuning_payload(payload: bytes) -> CurrentTuningChunk:
    expected_size = 2 + 1 + CURRENT_TUNING_SAMPLES_PER_CHUNK * 2 * 4
    if len(payload) < expected_size:
        raise ValueError(f"Current tuning payload too short: {len(payload)}")

    chunk_size = struct.unpack_from("<H", payload, 0)[0]
    if chunk_size < CURRENT_TUNING_SAMPLES_PER_CHUNK * 2 * 4:
        raise ValueError(f"Unexpected current tuning chunk size: {chunk_size}")

    chunk_index = payload[2]
    offset = 3
    reference: list[float] = []
    feedback: list[float] = []
    for _ in range(CURRENT_TUNING_SAMPLES_PER_CHUNK):
        reference.append(struct.unpack_from("<f", payload, offset)[0])
        feedback.append(struct.unpack_from("<f", payload, offset + 4)[0])
        offset += 8

    return CurrentTuningChunk(
        chunk_index=chunk_index,
        reference=reference,
        feedback=feedback,
    )


def parse_autotune_payload(payload: bytes) -> AutoTuneChunk:
    expected_header = struct.calcsize("<BHBHf")
    expected_total = expected_header + (AUTOTUNE_SAMPLES_PER_CHUNK * 3 * 4)
    if len(payload) < expected_total:
        raise ValueError(f"Autotune payload too short: {len(payload)}")

    stage, sample_start, sample_count, total_samples, sample_period_s = struct.unpack_from(
        "<BHBHf", payload, 0
    )
    offset = expected_header
    primary: list[float] = []
    secondary: list[float] = []
    tertiary: list[float] = []
    for _ in range(AUTOTUNE_SAMPLES_PER_CHUNK):
        a, b, c = struct.unpack_from("<3f", payload, offset)
        offset += 12
        primary.append(a)
        secondary.append(b)
        tertiary.append(c)

    valid_count = max(0, min(int(sample_count), AUTOTUNE_SAMPLES_PER_CHUNK))
    return AutoTuneChunk(
        stage=int(stage),
        sample_start=int(sample_start),
        sample_count=valid_count,
        total_samples=int(total_samples),
        sample_period_s=float(sample_period_s),
        primary=primary[:valid_count],
        secondary=secondary[:valid_count],
        tertiary=tertiary[:valid_count],
    )


def parse_parameter_chunk(payload: bytes) -> dict[int, float]:
    clean = payload.rstrip(b"\r\n")
    result: dict[int, float] = {}
    offset = 0
    while offset + 5 <= len(clean):
        index = clean[offset]
        value = struct.unpack_from("<f", clean, offset + 1)[0]
        result[index] = value
        offset += 5
    return result


def build_parameter_write_chunks(values: list[float], chunk_size: int) -> list[bytes]:
    chunks: list[bytes] = []
    pairs = [(index, float(value)) for index, value in enumerate(values)]
    for start in range(0, len(pairs), chunk_size):
        chunk_pairs = pairs[start : start + chunk_size]
        if not chunk_pairs:
            continue
        while len(chunk_pairs) < chunk_size:
            chunk_pairs.append(chunk_pairs[-1])

        payload = bytearray()
        for index, value in chunk_pairs:
            payload.append(index & 0xFF)
            payload.extend(struct.pack("<f", value))
        chunks.append(bytes(payload))
    return chunks


def format_hex(data: bytes) -> str:
    return " ".join(f"{item:02X}" for item in data)


def decode_fault_flags(fault_code: int) -> list[str]:
    if fault_code == 0:
        return ["OK"]

    labels: list[str] = []
    for bitmask, label in FAULT_FLAG_LABELS.items():
        if fault_code & bitmask:
            labels.append(label)

    unknown_bits = fault_code & ~_KNOWN_FAULT_MASK
    if unknown_bits:
        labels.append(f"Unknown 0x{unknown_bits:04X}")
    return labels


def format_fault_text(fault_code: int) -> str:
    if fault_code == 0:
        return "OK"
    return f"{', '.join(decode_fault_flags(fault_code))} (0x{fault_code:04X})"
