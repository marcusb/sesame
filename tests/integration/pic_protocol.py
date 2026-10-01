import enum
import struct
from dataclasses import dataclass
from typing import Any, Optional, Union

DCM_HEADER_BYTE = 0x55
MAX_DCM_MSG_SIZE = 20


class DcmMsgType(enum.IntEnum):
    CMD_0x04 = 0x04
    DOOR_CMD = 0x10
    ALERT_CMD = 0x11
    AUDIO_CMD = 0x12
    DOOR_STATUS_REQUEST = 0x15
    DOOR_STATUS_UPDATE = 0x16
    OPS_EVENT = 0x17
    DOOR_ACK = 0x90
    ALERT_ACK = 0x91
    AUDIO_ACK = 0x92
    SENSOR_VERSION = 0x95


class DoorOpenState(enum.IntEnum):
    CLOSED = 0
    OPEN = 1
    UNKNOWN = 2
    FTM = 3


class DoorDirection(enum.IntEnum):
    DOWN = 0
    UP = 1
    STOPPED = 2
    UNKNOWN = 3


class OpsEvent(enum.IntEnum):
    UNUSED = 0
    NEW_LIMITS_DETECTED = 1
    POSITION_ADJUST = 2
    LIGHT_ON = 3
    LIGHT_ALERT_DONE = 4
    AUDIO_ALERT_DONE = 5
    MOTOR_START = 8
    MOTOR_STOP = 9


def calc_chk_sum(data: bytes) -> int:
    """Calculates XOR checksum over bytes (excluding leading 0x55 header)."""
    res = 0
    for b in data:
        res ^= b
    return res


@dataclass
class DcmCmd0x04:
    zero: bytes = b"\x00" * 9

    def pack(self) -> bytes:
        return struct.pack("<9s", self.zero[:9].ljust(9, b"\x00"))

    @classmethod
    def unpack(cls, data: bytes) -> "DcmCmd0x04":
        (zero,) = struct.unpack("<9s", data[:9])
        return cls(zero=zero)


@dataclass
class DcmDoorCmd:
    val: int = 1  # 1 = open, 0 = close
    reserved: int = 0

    def pack(self) -> bytes:
        return struct.pack("<BB", self.val, self.reserved)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmDoorCmd":
        val, res = struct.unpack("<BB", data[:2])
        return cls(val=val, reserved=res)


@dataclass
class DcmAlertCmd:
    val: int = 1
    duration_s: int = 5
    reserved: int = 0

    def pack(self) -> bytes:
        return struct.pack("<BBB", self.val, self.duration_s, self.reserved)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmAlertCmd":
        val, dur, res = struct.unpack("<BBB", data[:3])
        return cls(val=val, duration_s=dur, reserved=res)


@dataclass
class DcmAudioCmd:
    val: int = 2
    duration_s: int = 5
    reserved: int = 0

    def pack(self) -> bytes:
        return struct.pack("<BBB", self.val, self.duration_s, self.reserved)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmAudioCmd":
        val, dur, res = struct.unpack("<BBB", data[:3])
        return cls(val=val, duration_s=dur, reserved=res)


@dataclass
class DcmDoorStatusReq:
    time: int = 0
    state: DoorOpenState = DoorOpenState.CLOSED
    direction: DoorDirection = DoorDirection.STOPPED
    pos: int = 32768
    up_limit: int = 32888
    down_limit: int = 32768

    def pack(self) -> bytes:
        return struct.pack(
            "<IBBHHH",
            self.time,
            int(self.state),
            int(self.direction),
            self.pos,
            self.up_limit,
            self.down_limit,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "DcmDoorStatusReq":
        t, st, d, pos, up, down = struct.unpack("<IBBHHH", data[:12])
        return cls(
            time=t,
            state=DoorOpenState(st),
            direction=DoorDirection(d),
            pos=pos,
            up_limit=up,
            down_limit=down,
        )


@dataclass
class DcmDoorStatusUpdate:
    time: int = 0
    state: DoorOpenState = DoorOpenState.CLOSED
    direction: DoorDirection = DoorDirection.STOPPED
    pos: int = 32768
    up_limit: int = 32888
    down_limit: int = 32768
    motor_current: int = 0

    def pack(self) -> bytes:
        return struct.pack(
            "<IBBHHHH",
            self.time,
            int(self.state),
            int(self.direction),
            self.pos,
            self.up_limit,
            self.down_limit,
            self.motor_current,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "DcmDoorStatusUpdate":
        t, st, d, pos, up, down, mc = struct.unpack("<IBBHHHH", data[:14])
        return cls(
            time=t,
            state=DoorOpenState(st),
            direction=DoorDirection(d),
            pos=pos,
            up_limit=up,
            down_limit=down,
            motor_current=mc,
        )


@dataclass
class DcmSensorVersion:
    reserved: int = 0
    state: DoorOpenState = DoorOpenState.CLOSED
    direction: DoorDirection = DoorDirection.STOPPED
    model_code: int = 4
    reserved2: bytes = b"\x00\x00"
    pos: int = 32768
    time: int = 0
    hw_caps: int = 8
    reserved3: int = 0
    sensor_restart_reason: int = 9
    major: int = 1
    minor: int = 0
    patch: int = 7
    suffix: bytes = b"P"

    def pack(self) -> bytes:
        return struct.pack(
            "<BBBB2sHIBBBBBBc",
            self.reserved,
            int(self.state),
            int(self.direction),
            self.model_code,
            self.reserved2[:2],
            self.pos,
            self.time,
            self.hw_caps,
            self.reserved3,
            self.sensor_restart_reason,
            self.major,
            self.minor,
            self.patch,
            self.suffix[:1],
        )

    @classmethod
    def unpack(cls, data: bytes) -> "DcmSensorVersion":
        (
            res,
            st,
            d,
            model,
            res2,
            pos,
            t,
            caps,
            res3,
            reason,
            maj,
            min_,
            pat,
            suf,
        ) = struct.unpack("<BBBB2sHIBBBBBBc", data[:19])
        return cls(
            reserved=res,
            state=DoorOpenState(st),
            direction=DoorDirection(d),
            model_code=model,
            reserved2=res2,
            pos=pos,
            time=t,
            hw_caps=caps,
            reserved3=res3,
            sensor_restart_reason=reason,
            major=maj,
            minor=min_,
            patch=pat,
            suffix=suf,
        )


@dataclass
class DcmOpsEvent:
    time: int = 0
    event: OpsEvent = OpsEvent.AUDIO_ALERT_DONE
    reserved: int = 0
    up_limit: int = 32888
    down_limit: int = 32768

    def pack(self) -> bytes:
        return struct.pack(
            "<IBBHH",
            self.time,
            int(self.event),
            self.reserved,
            self.up_limit & 0xFFFF,
            self.down_limit & 0xFFFF,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "DcmOpsEvent":
        t, ev, res, up, down = struct.unpack("<IBBHH", data[:10])
        return cls(
            time=t,
            event=OpsEvent(ev) if ev in [e.value for e in OpsEvent] else ev,
            reserved=res,
            up_limit=up,
            down_limit=down,
        )


@dataclass
class DcmDoorAck:
    val: int = 0
    reserved: int = 0

    def pack(self) -> bytes:
        return struct.pack("<BB", self.val, self.reserved)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmDoorAck":
        val, res = struct.unpack("<BB", data[:2])
        return cls(val=val, reserved=res)


@dataclass
class DcmAlertAck:
    val: int = 0

    def pack(self) -> bytes:
        return struct.pack("<B", self.val)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmAlertAck":
        (val,) = struct.unpack("<B", data[:1])
        return cls(val=val)


@dataclass
class DcmAudioAck:
    val: int = 0

    def pack(self) -> bytes:
        return struct.pack("<B", self.val)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmAudioAck":
        (val,) = struct.unpack("<B", data[:1])
        return cls(val=val)


@dataclass
class DcmRawPayload:
    data: bytes

    def pack(self) -> bytes:
        return self.data

    @classmethod
    def unpack(cls, data: bytes) -> "DcmRawPayload":
        return cls(data=data)


PayloadType = Union[
    DcmCmd0x04,
    DcmDoorCmd,
    DcmAlertCmd,
    DcmAudioCmd,
    DcmDoorStatusReq,
    DcmDoorStatusUpdate,
    DcmSensorVersion,
    DcmOpsEvent,
    DcmDoorAck,
    DcmAlertAck,
    DcmAudioAck,
    DcmRawPayload,
]

PAYLOAD_TYPE_MAP: dict[int, Any] = {
    DcmMsgType.CMD_0x04: DcmCmd0x04,
    DcmMsgType.DOOR_CMD: DcmDoorCmd,
    DcmMsgType.ALERT_CMD: DcmAlertCmd,
    DcmMsgType.AUDIO_CMD: DcmAudioCmd,
    DcmMsgType.DOOR_STATUS_REQUEST: DcmDoorStatusReq,
    DcmMsgType.DOOR_STATUS_UPDATE: DcmDoorStatusUpdate,
    DcmMsgType.OPS_EVENT: DcmOpsEvent,
    DcmMsgType.DOOR_ACK: DcmDoorAck,
    DcmMsgType.ALERT_ACK: DcmAlertAck,
    DcmMsgType.AUDIO_ACK: DcmAudioAck,
    DcmMsgType.SENSOR_VERSION: DcmSensorVersion,
}


@dataclass
class DcmFrame:
    seq: int
    msg_type: Union[DcmMsgType, int]
    payload: PayloadType
    header: int = DCM_HEADER_BYTE

    def to_bytes(self) -> bytes:
        payload_bytes = (
            self.payload.pack()
            if hasattr(self.payload, "pack")
            else bytes(self.payload)
        )
        length = len(payload_bytes)
        hdr = struct.pack("<BBBB", self.header, length, self.seq, int(self.msg_type))
        chksum = calc_chk_sum(hdr[1:] + payload_bytes)
        return hdr + payload_bytes + bytes([chksum])

    @classmethod
    def from_bytes(cls, data: bytes) -> "DcmFrame":
        if len(data) < 5:
            raise ValueError(f"Frame too short: {len(data)} bytes")
        header, length, seq, raw_type = struct.unpack("<BBBB", data[:4])
        if header != DCM_HEADER_BYTE:
            raise ValueError(f"Invalid header byte: {hex(header)}")
        if len(data) < length + 5:
            raise ValueError(
                f"Incomplete frame: expected {length + 5}, got {len(data)}"
            )
        payload_data = data[4 : 4 + length]
        expected_chk = data[4 + length]
        computed_chk = calc_chk_sum(data[1 : 4 + length])
        if computed_chk != expected_chk:
            raise ValueError(
                f"Checksum mismatch: computed {hex(computed_chk)} != expected {hex(expected_chk)}"
            )

        try:
            msg_type = DcmMsgType(raw_type)
        except ValueError:
            msg_type = raw_type

        payload_cls = PAYLOAD_TYPE_MAP.get(int(raw_type), DcmRawPayload)
        try:
            payload = payload_cls.unpack(payload_data)
        except Exception:
            payload = DcmRawPayload(data=payload_data)

        return cls(header=header, seq=seq, msg_type=msg_type, payload=payload)

    @classmethod
    def parse_stream(cls, buf: bytearray) -> Optional["DcmFrame"]:
        """
        Parses a single frame from the start of a bytearray buffer.
        Pops consumed bytes from buf. Returns None if no complete frame is available.
        """
        while len(buf) > 0 and buf[0] != DCM_HEADER_BYTE:
            buf.pop(0)

        if len(buf) < 5:
            return None

        length = buf[1]
        frame_len = length + 5
        if len(buf) < frame_len:
            return None

        frame_data = bytes(buf[:frame_len])
        computed_chk = calc_chk_sum(frame_data[1:-1])
        if computed_chk != frame_data[-1]:
            # Corrupted frame, skip sync byte and continue searching
            buf.pop(0)
            return cls.parse_stream(buf)

        del buf[:frame_len]
        return cls.from_bytes(frame_data)
