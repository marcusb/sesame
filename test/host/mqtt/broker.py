"""
Minimal MQTT 3.1.1 broker for host tests.

Implements: CONNECT/CONNACK, SUBSCRIBE/SUBACK, PUBLISH (QoS 0+1),
PINGREQ/PINGRESP, DISCONNECT, LWT, retained messages.
"""

import asyncio
import struct
import threading
from typing import Dict, Optional, Set, Tuple


# ---------------------------------------------------------------------------
# Wire helpers
# ---------------------------------------------------------------------------

def _encode_varlen(n: int) -> bytes:
    out = []
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            b |= 0x80
        out.append(b)
        if not n:
            break
    return bytes(out)


async def _read_packet(reader: asyncio.StreamReader) -> Tuple[int, bytes]:
    first = await reader.readexactly(1)
    ptype = first[0]
    multiplier, remaining = 1, 0
    while True:
        b = (await reader.readexactly(1))[0]
        remaining += (b & 0x7F) * multiplier
        multiplier *= 128
        if not (b & 0x80):
            break
    body = await reader.readexactly(remaining) if remaining else b""
    return ptype, body


def _read_str(data: bytes, offset: int) -> Tuple[str, int]:
    (length,) = struct.unpack_from(">H", data, offset)
    return data[offset + 2 : offset + 2 + length].decode(), offset + 2 + length


def _pack_str(s: str) -> bytes:
    enc = s.encode()
    return struct.pack(">H", len(enc)) + enc


def _topic_matches(topic: str, pattern: str) -> bool:
    if pattern == "#":
        return True
    tparts = topic.split("/")
    pparts = pattern.split("/")
    for i, p in enumerate(pparts):
        if p == "#":
            return True
        if i >= len(tparts) or (p != "+" and p != tparts[i]):
            return False
    return len(tparts) == len(pparts)


# ---------------------------------------------------------------------------
# Per-connection state
# ---------------------------------------------------------------------------

class _Conn:
    def __init__(self, reader, writer, broker: "_Broker"):
        self.reader = reader
        self.writer = writer
        self.broker = broker
        self.subscriptions: Set[str] = set()
        self.will_topic: Optional[str] = None
        self.will_payload: Optional[bytes] = None
        self.will_retain: bool = False

    async def run(self):
        try:
            await self._loop()
        except (asyncio.IncompleteReadError, ConnectionResetError, BrokenPipeError):
            pass
        finally:
            if self.will_topic is not None:
                await self.broker._route(self.will_topic, self.will_payload or b"", self.will_retain)
            self.broker._conns.discard(self)
            try:
                self.writer.close()
                await self.writer.wait_closed()
            except Exception:
                pass

    async def _loop(self):
        while True:
            try:
                ptype, body = await _read_packet(self.reader)
            except asyncio.IncompleteReadError:
                return
            cmd = ptype >> 4
            if cmd == 1:
                await self._connect(body)
            elif cmd == 3:
                await self._publish(ptype, body)
            elif cmd == 8:
                await self._subscribe(body)
            elif cmd == 10:
                pid = struct.unpack_from(">H", body)[0]
                self._write(b"\xb0\x02" + struct.pack(">H", pid))
            elif cmd == 12:
                self._write(b"\xd0\x00")
            elif cmd == 14:
                self.will_topic = None
                return

    def _write(self, data: bytes):
        self.writer.write(data)

    async def _flush(self):
        await self.writer.drain()

    async def _connect(self, body: bytes):
        # Variable header: 2+N proto name, 1 proto level, 1 flags, 2 keepalive
        _proto_name, offset = _read_str(body, 0)
        offset += 1  # proto level
        flags = body[offset]; offset += 1
        offset += 2  # keepalive

        _client_id, offset = _read_str(body, offset)

        will_flag = bool(flags & 0x04)
        if will_flag:
            will_topic, offset = _read_str(body, offset)
            (wlen,) = struct.unpack_from(">H", body, offset); offset += 2
            will_payload = body[offset : offset + wlen]; offset += wlen
            self.will_topic = will_topic
            self.will_payload = will_payload
            self.will_retain = bool(flags & 0x20)

        if flags & 0x80:  # username
            _u, offset = _read_str(body, offset)
        if flags & 0x40:  # password
            (plen,) = struct.unpack_from(">H", body, offset); offset += 2
            offset += plen

        self._write(b"\x20\x02\x00\x00")  # CONNACK, rc=0
        await self._flush()
        self.broker._conns.add(self)

    async def _publish(self, ptype: int, body: bytes):
        retain = bool(ptype & 0x01)
        qos = (ptype >> 1) & 0x03
        offset = 0
        topic, offset = _read_str(body, offset)
        packet_id = None
        if qos > 0:
            (packet_id,) = struct.unpack_from(">H", body, offset); offset += 2
        payload = body[offset:]

        if retain:
            if payload:
                self.broker._retained[topic] = payload
            else:
                self.broker._retained.pop(topic, None)

        await self.broker._route(topic, payload, retain)

        if qos == 1 and packet_id is not None:
            self._write(b"\x40\x02" + struct.pack(">H", packet_id))
            await self._flush()

    async def _subscribe(self, body: bytes):
        offset = 0
        (pid,) = struct.unpack_from(">H", body, offset); offset += 2
        new_filters = []
        while offset < len(body):
            filt, offset = _read_str(body, offset)
            _qos = body[offset]; offset += 1
            self.subscriptions.add(filt)
            new_filters.append(filt)

        rc = bytes([0] * len(new_filters))
        pkt = b"\x90" + _encode_varlen(2 + len(rc)) + struct.pack(">H", pid) + rc
        self._write(pkt)
        await self._flush()

        for topic, payload in list(self.broker._retained.items()):
            for filt in new_filters:
                if _topic_matches(topic, filt):
                    await self.send_publish(topic, payload, retain=True)
                    break

    async def send_publish(self, topic: str, payload: bytes, qos: int = 0, retain: bool = False):
        topic_enc = _pack_str(topic)
        flags = (qos << 1) | (1 if retain else 0)
        body = topic_enc + payload
        pkt = bytes([0x30 | flags]) + _encode_varlen(len(body)) + body
        self._write(pkt)
        try:
            await self._flush()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Broker
# ---------------------------------------------------------------------------

class _Broker:
    def __init__(self):
        self._conns: Set[_Conn] = set()
        self._retained: Dict[str, bytes] = {}
        self._server: Optional[asyncio.AbstractServer] = None

    async def start(self, host: str = "127.0.0.1") -> int:
        self._server = await asyncio.start_server(self._accept, host, 0)
        return self._server.sockets[0].getsockname()[1]

    async def stop(self):
        if self._server:
            self._server.close()
            await self._server.wait_closed()

    async def _accept(self, reader, writer):
        conn = _Conn(reader, writer, self)
        await conn.run()

    async def _route(self, topic: str, payload: bytes, retain: bool):
        for conn in list(self._conns):
            for filt in conn.subscriptions:
                if _topic_matches(topic, filt):
                    await conn.send_publish(topic, payload, retain=retain)
                    break

    async def inject(self, topic: str, payload: bytes, retain: bool = False):
        """Inject a publish from outside (thread-safe via run_coroutine_threadsafe)."""
        if retain:
            self._retained[topic] = payload
        await self._route(topic, payload, retain)


# ---------------------------------------------------------------------------
# Thread-safe wrapper used by pytest fixtures
# ---------------------------------------------------------------------------

class EmbeddedBroker:
    """
    MQTT 3.1.1 broker running in a background asyncio loop.
    Usage::

        broker = EmbeddedBroker()
        broker.start()        # returns bound port
        ...
        broker.stop()
    """

    def __init__(self):
        self._broker = _Broker()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._port: int = 0

    def start(self) -> int:
        ready = threading.Event()

        def _run():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

            async def _setup():
                self._port = await self._broker.start()
                ready.set()

            self._loop.run_until_complete(_setup())
            self._loop.run_forever()
            self._loop.run_until_complete(self._broker.stop())
            self._loop.close()

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        assert ready.wait(timeout=5), "embedded broker did not start"
        return self._port

    def stop(self):
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=5)

    @property
    def port(self) -> int:
        return self._port

    def inject(self, topic: str, payload: bytes, retain: bool = False):
        """Thread-safe publish into the broker (for use from test code)."""
        assert self._loop is not None
        fut = asyncio.run_coroutine_threadsafe(
            self._broker.inject(topic, payload, retain), self._loop
        )
        fut.result(timeout=5)
