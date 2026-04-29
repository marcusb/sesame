"""
Tests for src/ota_client.c running on host.

The SUT connects to an HTTP server inside libslirp (reachable at 10.0.2.2
from the guest) to download firmware. ota_stub.c accumulates an XOR checksum
over every byte written and emits "OTA checksum=0x... bytes=..." via the
inspector socket when ota_finish() is called.
"""

import os
import socket
import struct
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer

import pytest


FW_MAGIC_STR = 0x4D525648  # "MRVL"
FW_MAGIC_SIG = 0x2E9CF17B


def _make_firmware(size: int) -> bytes:
    hdr = struct.pack(
        "<IIIIII", FW_MAGIC_STR, FW_MAGIC_SIG, int(time.time()), 1, 0x1000, 0
    )
    remaining = size - len(hdr)
    return hdr + b"\xab" * remaining


def _ota_checksum(data: bytes) -> int:
    """XOR checksum matching ota_stub.c: ota_write_chunk accumulates
    running_checksum ^= buf[i] << ((i % 4) * 8)."""
    cs = 0
    for i, b in enumerate(data):
        cs ^= b << ((i % 4) * 8)
    return cs & 0xFFFFFFFF


class _RangeHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        path = self.server.firmware_path
        size = os.path.getsize(path)
        range_header = self.headers.get("Range")

        if not range_header:
            with open(path, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        try:
            range_spec = range_header.replace("bytes=", "")
            start_s, end_s = range_spec.split("-")
            start = int(start_s) if start_s else 0
            end = int(end_s) if end_s else size - 1
        except Exception:
            self.send_error(400)
            return

        start = min(start, size)
        end = min(end, size - 1)
        if start > end or start >= size:
            self.send_response(416)
            self.send_header("Content-Range", f"bytes */{size}")
            self.send_header("Content-Length", "0")
            self.end_headers()
            return

        with open(path, "rb") as f:
            f.seek(start)
            data = f.read(end - start + 1)

        self.send_response(206)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        self.wfile.write(data)


class FakeFirmwareServer:
    def __init__(self, firmware: bytes):
        self._tmp = tempfile.NamedTemporaryFile(delete=False)
        self._tmp.write(firmware)
        self._tmp.close()

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), _RangeHandler)
        self.server.firmware_path = self._tmp.name
        self.port = self.server.server_address[1]

        self._thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self._thread.start()

    def close(self):
        self.server.shutdown()
        os.unlink(self._tmp.name)


def _run_ota_test(ota_harness, size: int) -> None:
    firmware = _make_firmware(size)
    expected_checksum = _ota_checksum(firmware)

    server = FakeFirmwareServer(firmware)
    try:
        # 10.0.2.2 is the slirp host gateway, reachable from inside the guest stack
        ota_harness.send_cmd(f"upgrade http://10.0.2.2:{server.port}/fw.bin")
        evt = ota_harness.wait_event(timeout=60.0)
        assert evt.startswith("OTA "), f"unexpected inspector event: {evt!r}"

        # parse "OTA checksum=0x<hex> bytes=<dec>"
        fields = dict(part.split("=") for part in evt.split()[1:])
        assert int(fields["checksum"], 16) == expected_checksum
        assert int(fields["bytes"]) == len(firmware)
    finally:
        server.close()


def test_ota_downloads_and_verifies_checksum(ota_harness):
    # 2048 bytes = 2 full 1024-byte range requests; small enough to complete
    # quickly over libslirp's user-mode NIC.
    _run_ota_test(ota_harness, 2048)


def test_ota_partial_last_chunk(ota_harness):
    # 1500 bytes: first request returns a full 1024-byte chunk, second returns
    # the remaining 476 bytes — exercises the partial-last-chunk exit path
    # without a 416 range-not-satisfiable response.
    _run_ota_test(ota_harness, 1500)
