from __future__ import annotations

import socket
import time
from typing import Iterator

import pytest

from _harness import Harness, _run_harness


@pytest.fixture()
def httpd_harness() -> Iterator[Harness]:
    gen = _run_harness("HOST_TEST_BIN", "httpd")
    harness = next(gen)

    # READY fires when the network stack comes up; poll until httpd has opened
    # its listen socket and is ready to accept requests.
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection(("127.0.0.1", harness.port), timeout=0.5)
            s.sendall(b"GET / HTTP/1.0\r\n\r\n")
            s.recv(64)
            s.close()
            break
        except OSError:
            time.sleep(0.05)

    yield harness

    try:
        next(gen)
    except StopIteration:
        pass
