import socket
import struct
import threading
import time
from typing import Callable, List, Optional, Tuple


def _decode_remaining_length(sock: socket.socket) -> int:
    mult = 1
    rem_len = 0
    while True:
        data = sock.recv(1)
        if not data:
            raise ConnectionResetError("Socket closed while reading remaining length")
        b = data[0]
        rem_len += (b & 0x7F) * mult
        if (b & 0x80) == 0:
            break
        mult *= 128
    return rem_len


def _encode_remaining_length(length: int) -> bytes:
    res = bytearray()
    val = length
    while True:
        encoded = val % 128
        val //= 128
        if val > 0:
            encoded |= 128
        res.append(encoded)
        if val == 0:
            break
    return bytes(res)


def _topic_matches(subscription: str, topic: str) -> bool:
    if subscription == "#" or subscription == topic:
        return True
    sub_parts = subscription.split("/")
    topic_parts = topic.split("/")
    for i, part in enumerate(sub_parts):
        if part == "#":
            return True
        if part == "+":
            if i >= len(topic_parts):
                return False
            continue
        if i >= len(topic_parts) or part != topic_parts[i]:
            return False
    return len(sub_parts) == len(topic_parts)


class EmbeddedMqttServer:
    """
    Lightweight in-process MQTT 3.1.1 broker for testing.
    Handles CONNECT, PUBLISH, SUBSCRIBE, PINGREQ, and forwards messages to subscribers.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 0):
        self.host = host
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, port))
        self.port = self.sock.getsockname()[1]
        self.sock.listen(10)

        self.running = True
        self.clients: dict[socket.socket, list[str]] = {}
        self.client_lock = threading.Lock()
        self.messages: List[Tuple[str, bytes]] = []
        self._message_callbacks: List[Callable[[str, bytes], None]] = []
        self._new_msg_cond = threading.Condition()

        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()

    def _accept_loop(self):
        while self.running:
            try:
                conn, _ = self.sock.accept()
                with self.client_lock:
                    self.clients[conn] = []
                threading.Thread(
                    target=self._handle_client, args=(conn,), daemon=True
                ).start()
            except Exception:
                break

    def _handle_client(self, conn: socket.socket):
        try:
            while self.running:
                b1 = conn.recv(1)
                if not b1:
                    break
                header_byte = b1[0]
                packet_type = header_byte >> 4
                rem_len = _decode_remaining_length(conn)

                payload = b""
                while len(payload) < rem_len:
                    chunk = conn.recv(rem_len - len(payload))
                    if not chunk:
                        break
                    payload += chunk

                if packet_type == 1:  # CONNECT
                    # Reply with CONNACK: Return code 0 (Accepted)
                    conn.sendall(bytes([0x20, 0x02, 0x00, 0x00]))

                elif packet_type == 8:  # SUBSCRIBE
                    pkt_id = payload[:2]
                    # Parse subscribed topics
                    offset = 2
                    subs = []
                    while offset < len(payload):
                        topic_len = struct.unpack(">H", payload[offset : offset + 2])[0]
                        offset += 2
                        sub_topic = payload[offset : offset + topic_len].decode("utf-8")
                        offset += topic_len
                        qos = payload[offset]
                        offset += 1
                        subs.append(sub_topic)
                    with self.client_lock:
                        if conn in self.clients:
                            self.clients[conn].extend(subs)

                    # Reply with SUBACK (granted QoS 0)
                    conn.sendall(bytes([0x90, 0x03, pkt_id[0], pkt_id[1], 0x00]))

                elif packet_type == 3:  # PUBLISH
                    flags = header_byte & 0x0F
                    qos = (flags >> 1) & 0x03
                    offset = 0
                    topic_len = struct.unpack(">H", payload[offset : offset + 2])[0]
                    offset += 2
                    topic = payload[offset : offset + topic_len].decode("utf-8")
                    offset += topic_len
                    pkt_id = None
                    if qos > 0:
                        pkt_id = struct.unpack(">H", payload[offset : offset + 2])[0]
                        offset += 2
                    msg_body = payload[offset:]

                    if qos == 1 and pkt_id is not None:
                        # Send PUBACK
                        conn.sendall(
                            bytes(
                                [
                                    0x40,
                                    0x02,
                                    (pkt_id >> 8) & 0xFF,
                                    pkt_id & 0xFF,
                                ]
                            )
                        )

                    with self._new_msg_cond:
                        self.messages.append((topic, msg_body))
                        for cb in self._message_callbacks:
                            try:
                                cb(topic, msg_body)
                            except Exception:
                                pass
                        self._new_msg_cond.notify_all()

                    # Broadcast packet to all matching subscribers
                    frame = (
                        bytes([header_byte])
                        + _encode_remaining_length(rem_len)
                        + payload
                    )
                    with self.client_lock:
                        for client_sock, subscriptions in list(self.clients.items()):
                            if any(_topic_matches(sub, topic) for sub in subscriptions):
                                try:
                                    client_sock.sendall(frame)
                                except Exception:
                                    pass

                elif packet_type == 12:  # PINGREQ
                    conn.sendall(bytes([0xD0, 0x00]))

                elif packet_type == 14:  # DISCONNECT
                    break

        except Exception:
            pass
        finally:
            with self.client_lock:
                if conn in self.clients:
                    del self.clients[conn]
            try:
                conn.close()
            except Exception:
                pass

    def add_message_callback(self, cb: Callable[[str, bytes], None]):
        self._message_callbacks.append(cb)

    def wait_for_message(
        self,
        topic: str,
        timeout: float = 10.0,
        predicate: Optional[Callable[[bytes], bool]] = None,
    ) -> bytes:
        deadline = time.time() + timeout
        with self._new_msg_cond:
            while time.time() < deadline:
                for t, body in self.messages:
                    if _topic_matches(topic, t):
                        if predicate is None or predicate(body):
                            return body
                remaining = deadline - time.time()
                if remaining > 0:
                    self._new_msg_cond.wait(remaining)

        # Check one last time before raising
        for t, body in self.messages:
            if _topic_matches(topic, t):
                if predicate is None or predicate(body):
                    return body
        raise TimeoutError(
            f"Timed out waiting for message on topic '{topic}' after {timeout}s"
        )

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        with self.client_lock:
            for c in list(self.clients.keys()):
                try:
                    c.close()
                except Exception:
                    pass
            self.clients.clear()
