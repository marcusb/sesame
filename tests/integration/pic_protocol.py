import enum
import struct
from dataclasses import dataclass
from typing import Any, Optional, Union

DCM_HEADER_BYTE = 0x55
MAX_DCM_MSG_SIZE = 36


class DcmMsgType(enum.IntEnum):
    CMD_0x04 = 0x04
    DOOR_CMD = 0x10
    ALERT_CMD = 0x11
    AUDIO_CMD = 0x12
    LIGHT_CMD = 0x13
    STATUS_UPDATE = 0x14
    DOOR_STATUS_REQUEST = 0x15
    STATUS_UPDATE_EXT = 0x16
    DOOR_STATUS_UPDATE = 0x16  # backward-compatible alias
    OPS_EVENT = 0x17
    DOOR_ACK = 0x90
    ALERT_ACK = 0x91
    AUDIO_ACK = 0x92
    LIGHT_ACK = 0x93
    SENSOR_VERSION = 0x95


class DoorOpenState(enum.IntEnum):
    CLOSED = 0
    OPEN = 1
    UNKNOWN = 2
    FTM = 4


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


def calc_chk_sum_crc8(data: bytes) -> int:
    """Calculates PIC DCM CRC-8 checksum over bytes."""
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xB2
            else:
                crc = crc >> 1
    return crc ^ 0xFF


def calc_chk_sum_xor(data: bytes) -> int:
    """Calculates PIC DCM XOR checksum over bytes (excluding header)."""
    res = 0
    for b in data:
        res ^= b
    return res


def calc_chk_sum(data: bytes, use_crc8: bool = False) -> int:
    """Calculates PIC DCM checksum over frame data (either CRC-8 or XOR)."""
    if use_crc8:
        return calc_chk_sum_crc8(data)
    if data and data[0] == DCM_HEADER_BYTE:
        return calc_chk_sum_xor(data[1:])
    return calc_chk_sum_xor(data)


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
    reserved: int = 0
    val: int = 2
    reserved2: int = 0

    def pack(self) -> bytes:
        return struct.pack("<BBB", self.reserved, self.val, self.reserved2)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmAlertCmd":
        res1, val, res2 = struct.unpack("<BBB", data[:3])
        return cls(reserved=res1, val=val, reserved2=res2)


@dataclass
class DcmAudioCmd:
    reserved: int = 0
    val: int = 2
    reserved2: int = 0

    def pack(self) -> bytes:
        return struct.pack("<BBB", self.reserved, self.val, self.reserved2)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmAudioCmd":
        res1, val, res2 = struct.unpack("<BBB", data[:3])
        return cls(reserved=res1, val=val, reserved2=res2)


@dataclass
class DcmDoorStatusReq:
    time: int = 0
    reserved: int = 0
    state: DoorOpenState = DoorOpenState.CLOSED
    direction: DoorDirection = DoorDirection.STOPPED
    pos: int = 32768
    reserved1: int = 0
    reserved2: int = 0
    restore_pos: int = 0
    up_limit: int = 32888
    reserved3: int = 0
    down_limit: int = 32768

    @property
    def update_limits(self) -> int:
        return self.restore_pos

    def pack(self) -> bytes:
        return struct.pack(
            "<IHBBHHHHHHH",
            self.time,
            self.reserved,
            int(self.state),
            int(self.direction),
            self.pos,
            self.reserved1,
            self.reserved2,
            self.restore_pos,
            self.up_limit,
            self.reserved3,
            self.down_limit,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "DcmDoorStatusReq":
        t, res, st, d, pos, r1, r2, upd, up, r3, down = struct.unpack(
            "<IHBBHHHHHHH", data[:22]
        )
        return cls(
            time=t,
            reserved=res,
            state=DoorOpenState(st),
            direction=DoorDirection(d),
            pos=pos,
            reserved1=r1,
            reserved2=r2,
            restore_pos=upd,
            up_limit=up,
            reserved3=r3,
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
class DcmDoorStatusUpdateExt:
    time: int = 0
    reserved: int = 0
    state: DoorOpenState = DoorOpenState.CLOSED
    direction: DoorDirection = DoorDirection.STOPPED
    pos: int = 32768
    up_limit: int = 32888
    motor_current: int = 0
    down_limit: int = 32768
    extra: bytes = b"\x00" * 12

    def pack(self) -> bytes:
        return struct.pack(
            "<IHBBHHHH12s",
            self.time,
            self.reserved,
            int(self.state),
            int(self.direction),
            self.pos,
            self.up_limit,
            self.motor_current,
            self.down_limit,
            self.extra[:12].ljust(12, b"\x00"),
        )

    @classmethod
    def unpack(cls, data: bytes) -> "DcmDoorStatusUpdateExt":
        t, res, st, d, pos, up, mc, down, extra = struct.unpack(
            "<IHBBHHHH12s", data[:28]
        )
        return cls(
            time=t,
            reserved=res,
            state=DoorOpenState(st),
            direction=DoorDirection(d),
            pos=pos,
            up_limit=up,
            motor_current=mc,
            down_limit=down,
            extra=extra,
        )


@dataclass
class DcmSensorVersion:
    time: int = 0
    msg_info: int = 4
    status_flags: int = 0xB0
    reserved: int = 0
    hw_caps: int = 0x0D
    major: int = 1
    minor: int = 0
    patch: int = 7
    suffix: bytes = b"P"
    extra: bytes = b"\x00" * 4

    def pack(self) -> bytes:
        return struct.pack(
            "<IHHIBBBBc4s",
            self.time,
            self.msg_info,
            self.status_flags,
            self.reserved,
            self.hw_caps,
            self.major,
            self.minor,
            self.patch,
            self.suffix[:1],
            self.extra[:4],
        )

    @classmethod
    def unpack(cls, data: bytes) -> "DcmSensorVersion":
        (
            t,
            info,
            flags,
            res,
            caps,
            maj,
            min_,
            pat,
            suf,
            extra,
        ) = struct.unpack("<IHHIBBBBc4s", data[:21])
        return cls(
            time=t,
            msg_info=info,
            status_flags=flags,
            reserved=res,
            hw_caps=caps,
            major=maj,
            minor=min_,
            patch=pat,
            suffix=suf,
            extra=extra,
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
class DcmLightCmd:
    val: int  # 1 = on, 0 = off
    reserved: int = 0

    def pack(self) -> bytes:
        return struct.pack("<BB", self.val, self.reserved)

    @classmethod
    def unpack(cls, data: bytes) -> "DcmLightCmd":
        val, res = struct.unpack("<BB", data[:2])
        return cls(val=val, reserved=res)


@dataclass
class DcmLightAck:
    state: int  # 1 = on, 0 = off
    reserved: bytes = b"\x00\x00"

    def pack(self) -> bytes:
        return struct.pack("<B", self.state) + self.reserved

    @classmethod
    def unpack(cls, data: bytes) -> "DcmLightAck":
        state = data[0]
        res = data[1:3] if len(data) >= 3 else b"\x00\x00"
        return cls(state=state, reserved=res)


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
    DcmLightCmd,
    DcmDoorStatusReq,
    DcmDoorStatusUpdate,
    DcmDoorStatusUpdateExt,
    DcmSensorVersion,
    DcmOpsEvent,
    DcmDoorAck,
    DcmAlertAck,
    DcmAudioAck,
    DcmLightAck,
    DcmRawPayload,
]

PAYLOAD_TYPE_MAP: dict[int, Any] = {
    DcmMsgType.CMD_0x04: DcmCmd0x04,
    DcmMsgType.DOOR_CMD: DcmDoorCmd,
    DcmMsgType.ALERT_CMD: DcmAlertCmd,
    DcmMsgType.AUDIO_CMD: DcmAudioCmd,
    DcmMsgType.LIGHT_CMD: DcmLightCmd,
    DcmMsgType.DOOR_STATUS_REQUEST: DcmDoorStatusReq,
    DcmMsgType.STATUS_UPDATE: DcmDoorStatusUpdate,
    DcmMsgType.STATUS_UPDATE_EXT: DcmDoorStatusUpdateExt,
    DcmMsgType.DOOR_STATUS_UPDATE: DcmDoorStatusUpdateExt,
    DcmMsgType.OPS_EVENT: DcmOpsEvent,
    DcmMsgType.DOOR_ACK: DcmDoorAck,
    DcmMsgType.ALERT_ACK: DcmAlertAck,
    DcmMsgType.AUDIO_ACK: DcmAudioAck,
    DcmMsgType.LIGHT_ACK: DcmLightAck,
    DcmMsgType.SENSOR_VERSION: DcmSensorVersion,
}


@dataclass
class DcmFrame:
    seq: int
    msg_type: Union[DcmMsgType, int]
    payload: PayloadType
    header: int = DCM_HEADER_BYTE

    def to_bytes(self, use_crc8: bool = True) -> bytes:
        payload_bytes = (
            self.payload.pack()
            if hasattr(self.payload, "pack")
            else bytes(self.payload)
        )
        length = len(payload_bytes)
        hdr = struct.pack("<BBBB", self.header, length, self.seq, int(self.msg_type))
        chksum = calc_chk_sum_crc8((hdr + payload_bytes)[: length + 3])
        return hdr + payload_bytes + bytes([chksum])

    @classmethod
    def from_bytes(cls, data: bytes) -> "DcmFrame":
        if len(data) < 5:
            raise ValueError(f"Frame too short: {len(data)} bytes")
        header, length, seq, raw_type = struct.unpack("<BBBB", data[:4])
        if header != DCM_HEADER_BYTE:
            raise ValueError(f"Invalid header byte: {hex(header)}")

        # Check hardware PIC RX format (len includes type, frame length is length + 4)
        if len(data) == length + 4:
            payload_data = data[4 : 3 + length]
            expected_chk = data[3 + length]
            crc_chk = calc_chk_sum_crc8(data[: length + 3])
        elif len(data) >= length + 5:
            payload_data = data[4 : 4 + length]
            expected_chk = data[4 + length]
            crc_chk = calc_chk_sum_crc8(data[: length + 3])
        else:
            raise ValueError(
                f"Incomplete frame: expected at least {length + 4}, got {len(data)}"
            )

        if expected_chk != crc_chk:
            raise ValueError(
                f"Checksum mismatch: expected {hex(expected_chk)}, CRC8: {hex(crc_chk)}"
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

        # 1. Try hardware PIC format (len includes type byte, wire size = length + 4)
        if len(buf) >= length + 4:
            frame_data = bytes(buf[: length + 4])
            crc_chk = calc_chk_sum_crc8(frame_data[: length + 3])
            if frame_data[-1] == crc_chk:
                del buf[: length + 4]
                return cls.from_bytes(frame_data)

        # 2. Try legacy/sim format (wire size = length + 5)
        if len(buf) >= length + 5:
            frame_data = bytes(buf[: length + 5])
            crc_chk = calc_chk_sum_crc8(frame_data[: length + 3])
            if frame_data[-1] == crc_chk:
                del buf[: length + 5]
                return cls.from_bytes(frame_data)

        if len(buf) < length + 5:
            return None

        # Neither matched and buffer is long enough, skip sync byte
        buf.pop(0)
        return cls.parse_stream(buf)
