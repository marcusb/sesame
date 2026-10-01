import logging
import os
import select
import threading
import time
from typing import Callable, List, Optional

from pic_protocol import (
    DcmAlertAck,
    DcmAlertCmd,
    DcmAudioAck,
    DcmAudioCmd,
    DcmDoorAck,
    DcmDoorCmd,
    DcmDoorStatusUpdate,
    DcmFrame,
    DcmMsgType,
    DcmOpsEvent,
    DcmSensorVersion,
    DoorDirection,
    DoorOpenState,
    OpsEvent,
)

logger = logging.getLogger(__name__)


class PicSimulator:
    """
    Simulates the PIC16 door controller communicating over a virtual UART (PTY).
    Handles POST self-test, status polling, door commands, and sends status updates.
    """

    def __init__(
        self,
        master_fd: int,
        down_limit: int = 32768,
        up_limit: int = 32888,
        initial_pos: int = 32768,
        initial_state: DoorOpenState = DoorOpenState.CLOSED,
    ):
        self.master_fd = master_fd
        self.down_limit = down_limit
        self.up_limit = up_limit
        self.pos = initial_pos
        self.state = initial_state
        self.direction = DoorDirection.STOPPED
        self.motor_current = 0

        self.seq_counter = 0xA0
        self.running = True
        self.rx_buffer = bytearray()
        self.received_frames: List[DcmFrame] = []
        self._cond = threading.Condition()

        # Flags for automatic responses
        self.auto_ack_audio = True
        self.auto_ack_alert = True
        self.auto_ack_door = True
        self.auto_reply_status_req = True
        self.auto_simulate_motion = False

        self._read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._read_thread.start()

    def _next_seq(self) -> int:
        self.seq_counter = (self.seq_counter + 1) & 0xFF
        return self.seq_counter

    def send_frame(self, frame: DcmFrame):
        raw = frame.to_bytes()
        os.write(self.master_fd, raw)

    def send_msg(self, msg_type: DcmMsgType, payload, seq: Optional[int] = None):
        if seq is None:
            seq = self._next_seq()
        frame = DcmFrame(seq=seq, msg_type=msg_type, payload=payload)
        self.send_frame(frame)

    def send_status_update(
        self,
        state: Optional[DoorOpenState] = None,
        direction: Optional[DoorDirection] = None,
        pos: Optional[int] = None,
        motor_current: Optional[int] = None,
    ):
        if state is not None:
            self.state = state
        if direction is not None:
            self.direction = direction
        if pos is not None:
            self.pos = pos
        if motor_current is not None:
            self.motor_current = motor_current

        update = DcmDoorStatusUpdate(
            time=int(time.time()) & 0xFFFFFFFF,
            state=self.state,
            direction=self.direction,
            pos=self.pos,
            up_limit=self.up_limit,
            down_limit=self.down_limit,
            motor_current=self.motor_current,
        )
        self.send_msg(DcmMsgType.DOOR_STATUS_UPDATE, update)

    def send_sensor_version(self, seq: int):
        version = DcmSensorVersion(
            state=self.state,
            direction=self.direction,
            pos=self.pos,
            time=0,
        )
        self.send_msg(DcmMsgType.SENSOR_VERSION, version, seq=seq)

    def _reader_loop(self):
        while self.running:
            try:
                r, _, _ = select.select([self.master_fd], [], [], 0.05)
                if not r:
                    continue
                chunk = os.read(self.master_fd, 256)
                if not chunk:
                    break
                logger.debug("[SIMULATOR RX RAW] %s", chunk.hex())
                self.rx_buffer.extend(chunk)

                while True:
                    frame = DcmFrame.parse_stream(self.rx_buffer)
                    if frame is None:
                        break
                    logger.debug(
                        "[SIMULATOR PARSED] %s %s", frame.msg_type, frame.payload
                    )
                    self._handle_incoming_frame(frame)
            except (OSError, ValueError):
                if not self.running:
                    break
                logger.exception("Error in simulator reader loop")
                break
            except Exception:
                if not self.running:
                    break
                logger.exception("Unexpected error in simulator reader loop")
                break

    def _handle_incoming_frame(self, frame: DcmFrame):
        with self._cond:
            self.received_frames.append(frame)
            self._cond.notify_all()

        if frame.msg_type == DcmMsgType.AUDIO_CMD:
            if self.auto_ack_audio:
                # Send AUDIO_ACK with matching seq
                self.send_msg(DcmMsgType.AUDIO_ACK, DcmAudioAck(val=0), seq=frame.seq)
                if isinstance(frame.payload, DcmAudioCmd) and frame.payload.val == 5:
                    # Firmware POST test audio tone: send initial door status
                    time.sleep(0.02)
                    self.send_status_update()

        elif frame.msg_type == DcmMsgType.ALERT_CMD:
            if self.auto_ack_alert:
                self.send_msg(DcmMsgType.ALERT_ACK, DcmAlertAck(val=0), seq=frame.seq)
                # Emit alert done events
                time.sleep(0.01)
                self.send_msg(
                    DcmMsgType.OPS_EVENT,
                    DcmOpsEvent(
                        time=4,
                        event=OpsEvent.AUDIO_ALERT_DONE,
                        up_limit=self.up_limit,
                        down_limit=self.down_limit,
                    ),
                )
                self.send_msg(
                    DcmMsgType.OPS_EVENT,
                    DcmOpsEvent(
                        time=5,
                        event=OpsEvent.LIGHT_ALERT_DONE,
                        up_limit=self.up_limit,
                        down_limit=self.down_limit,
                    ),
                )

        elif frame.msg_type == DcmMsgType.DOOR_STATUS_REQUEST:
            if self.auto_reply_status_req:
                self.send_sensor_version(seq=frame.seq)

        elif frame.msg_type == DcmMsgType.DOOR_CMD:
            if self.auto_ack_door:
                self.send_msg(DcmMsgType.DOOR_ACK, DcmDoorAck(val=0), seq=frame.seq)
                if self.auto_simulate_motion:
                    cmd_val = (
                        frame.payload.val
                        if isinstance(frame.payload, DcmDoorCmd)
                        else 0
                    )
                    if cmd_val == 1:
                        threading.Thread(target=self.simulate_open, daemon=True).start()
                    elif cmd_val == 0:
                        threading.Thread(
                            target=self.simulate_close, daemon=True
                        ).start()

    def simulate_open(self, steps: int = 4, step_delay: float = 0.05):
        """Simulates opening motion up to up_limit."""
        span = self.up_limit - self.down_limit
        self.direction = DoorDirection.UP
        self.state = DoorOpenState.OPEN
        self.motor_current = 32000

        # Initial move packet
        self.send_status_update(
            state=DoorOpenState.OPEN,
            direction=DoorDirection.UP,
            pos=self.down_limit + 1,
            motor_current=0,
        )
        time.sleep(step_delay)

        # Progress through steps
        for step in range(1, steps):
            self.pos = self.down_limit + int(span * step / steps)
            self.send_status_update(
                state=DoorOpenState.OPEN,
                direction=DoorDirection.UP,
                pos=self.pos,
                motor_current=self.motor_current,
            )
            time.sleep(step_delay)

        # Reached open limit
        self.pos = self.up_limit
        self.direction = DoorDirection.STOPPED
        self.motor_current = 0
        self.send_status_update(
            state=DoorOpenState.OPEN,
            direction=DoorDirection.STOPPED,
            pos=self.pos,
            motor_current=0,
        )

    def simulate_close(self, steps: int = 4, step_delay: float = 0.05):
        """Simulates closing motion down to down_limit."""
        span = self.up_limit - self.down_limit
        self.direction = DoorDirection.DOWN
        self.state = DoorOpenState.OPEN
        self.motor_current = 26000

        # Initial move packet
        self.send_status_update(
            state=DoorOpenState.OPEN,
            direction=DoorDirection.DOWN,
            pos=self.up_limit - 1,
            motor_current=0,
        )
        time.sleep(step_delay)

        # Progress through steps
        for step in range(1, steps):
            self.pos = self.up_limit - int(span * step / steps)
            self.send_status_update(
                state=DoorOpenState.OPEN,
                direction=DoorDirection.DOWN,
                pos=self.pos,
                motor_current=self.motor_current,
            )
            time.sleep(step_delay)

        # Reached closed limit
        self.pos = self.down_limit
        self.state = DoorOpenState.CLOSED
        self.direction = DoorDirection.STOPPED
        self.motor_current = 0
        self.send_status_update(
            state=DoorOpenState.CLOSED,
            direction=DoorDirection.STOPPED,
            pos=self.pos,
            motor_current=0,
        )

    def trigger_external_open(self):
        """Simulates external button press causing the door to open."""
        self.simulate_open()

    def trigger_external_close(self):
        """Simulates external button press causing the door to close."""
        self.simulate_close()

    def wait_for_frame(
        self,
        msg_type: DcmMsgType,
        timeout: float = 10.0,
        predicate: Optional[Callable[[DcmFrame], bool]] = None,
    ) -> DcmFrame:
        deadline = time.time() + timeout
        with self._cond:
            while time.time() < deadline:
                for frame in self.received_frames:
                    if frame.msg_type == msg_type:
                        if predicate is None or predicate(frame):
                            return frame
                remaining = deadline - time.time()
                if remaining > 0:
                    self._cond.wait(remaining)

        for frame in self.received_frames:
            if frame.msg_type == msg_type:
                if predicate is None or predicate(frame):
                    return frame
        frames_repr = [(f.msg_type, f.payload) for f in self.received_frames]
        raise TimeoutError(
            f"Timed out waiting for frame type {msg_type.name} after {timeout}s. Frames: {frames_repr}"
        )

    def wait_for_door_cmd(self, val: int, timeout: float = 10.0) -> DcmFrame:
        return self.wait_for_frame(
            DcmMsgType.DOOR_CMD,
            timeout=timeout,
            predicate=lambda f: isinstance(f.payload, DcmDoorCmd)
            and f.payload.val == val,
        )

    def clear_frames(self):
        with self._cond:
            self.received_frames.clear()

    def has_door_cmd(self, val: Optional[int] = None) -> bool:
        with self._cond:
            for f in self.received_frames:
                if f.msg_type == DcmMsgType.DOOR_CMD:
                    if val is None or (
                        isinstance(f.payload, DcmDoorCmd) and f.payload.val == val
                    ):
                        return True
            return False

    def count_door_cmds(self, val: Optional[int] = None) -> int:
        with self._cond:
            count = 0
            for f in self.received_frames:
                if f.msg_type == DcmMsgType.DOOR_CMD:
                    if val is None or (
                        isinstance(f.payload, DcmDoorCmd) and f.payload.val == val
                    ):
                        count += 1
            return count

    def has_alert_cmds(self) -> bool:
        with self._cond:
            has_audio = any(
                f.msg_type == DcmMsgType.AUDIO_CMD
                and isinstance(f.payload, DcmAudioCmd)
                and f.payload.val == 2
                for f in self.received_frames
            )
            has_alert = any(
                f.msg_type == DcmMsgType.ALERT_CMD
                and isinstance(f.payload, DcmAlertCmd)
                and f.payload.val == 1
                for f in self.received_frames
            )
            return has_audio and has_alert

    def close(self):
        self.running = False
        try:
            os.close(self.master_fd)
        except Exception:
            pass
        if hasattr(self, "_read_thread") and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
