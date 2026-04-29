"""
Tests for src/syslog.c running on host.

The SUT sends UDP syslog packets via the FreeRTOS stack; libslirp NATs
traffic destined for 10.0.2.2 to the host loopback, so we simply bind a
UDP listener on 127.0.0.1:<port> and tell the SUT to target 10.0.2.2:<port>.

We assert RFC 5424 framing:  "<PRI>1 - HOSTNAME TAG MSG_ID - MSG"
"""

import re
import socket
import time

import pytest


SYSLOG_FRAME_RE = re.compile(
    rb"^<(?P<pri>\d{1,3})>1 - (?P<host>\S+) (?P<app>\S+) (?P<proc>\S+) (?P<msgid>\d+) - (?P<msg>.*)$"
)


@pytest.fixture()
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(5.0)
    yield sock
    sock.close()


def _recv_one(sock) -> bytes:
    data, _ = sock.recvfrom(2048)
    return data


def test_info_syslog_rfc5424(syslog_harness, udp_listener):
    host, port = udp_listener.getsockname()
    # Guest addresses host via slirp gateway at 10.0.2.2.
    syslog_harness.send_cmd(f"cfg 10.0.2.2 {port}")
    time.sleep(0.3)  # let xTimerPendFunctionCall complete the bind
    syslog_harness.send_cmd("log info hello world")

    frame = _recv_one(udp_listener)
    m = SYSLOG_FRAME_RE.match(frame)
    assert m, f"bad frame: {frame!r}"
    pri = int(m.group("pri"))
    # LOG_FACILITY_LOCAL0 = 16, INFO severity = 6 → pri = 16*8 + 6 = 134
    assert pri == 134, pri
    assert m.group("host") == b"sesame-it"
    assert m.group("msg").endswith(b"hello world")


def test_error_has_severity_error(syslog_harness, udp_listener):
    host, port = udp_listener.getsockname()
    syslog_harness.send_cmd(f"cfg 10.0.2.2 {port}")
    time.sleep(0.3)
    syslog_harness.send_cmd("log error boom")

    frame = _recv_one(udp_listener)
    m = SYSLOG_FRAME_RE.match(frame)
    assert m
    # ERROR severity = 3 → pri = 16*8 + 3 = 131
    assert int(m.group("pri")) == 131
    assert m.group("msg").endswith(b"boom")
