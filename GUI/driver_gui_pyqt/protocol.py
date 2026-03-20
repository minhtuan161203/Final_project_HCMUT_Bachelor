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
    MTR_CODE_ERROR = 0xE1


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
