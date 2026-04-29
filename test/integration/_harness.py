"""
Shared test-harness helpers for Sesame integration tests in QEMU.

Each module's conftest.py imports _run_harness and Harness from here to build
its own fixture.
"""

from __future__ import annotations

import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import pytest

# SUT prints this when network is up.
# In QEMU integration mode, ports/inspector fields are placeholders (0).
READY_RE = re.compile(
    r"READY host=(?P<host>\S+) port=(?P<port>\d+) inspector=(?P<insp>\d+) guest_ip=(?P<guest_ip>\S+)"
)

_REPO_ROOT = Path(__file__).parent.parent.parent
_BUILD_DIR = _REPO_ROOT / "build"


def _find_build_artifact(rel: str) -> Path:
    p = _BUILD_DIR / "test" / "integration" / rel
    if p.exists():
        return p
    pytest.fail(
        f"build artifact {p} not found; run `ninja -C build integration_tests` first."
    )


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@dataclass
class Harness:
    proc: subprocess.Popen
    host: str
    port: int
    guest_ip: str
    inspector: "queue.Queue[str]"

    def wait_event(self, *, timeout: float = 10.0) -> str:
        try:
            return self.inspector.get(timeout=timeout)
        except queue.Empty:
            pytest.fail(f"timeout waiting for event (timeout={timeout}s)")

    def drain(self) -> list[str]:
        out = []
        while True:
            try:
                out.append(self.inspector.get_nowait())
            except queue.Empty:
                return out

    def send_cmd(self, line: str) -> None:
        assert self.proc.stdin is not None
        time.sleep(1.0)
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()


def _run_harness(binary_name: str, guest_port: int | None = None) -> Iterator[Harness]:
    binary = _find_build_artifact(f"{binary_name}/{binary_name}_it.axf")

    host_port = _free_port() if guest_port else 0

    qemu_cmd = [
        "qemu-system-arm",
        "-M",
        "mps2-an386",
        "-display",
        "none",
        "-semihosting-config",
        "enable=on,target=native,chardev=semihost_chr",
        "-chardev",
        "stdio,id=semihost_chr",
        "-kernel",
        str(binary),
        "-net",
        "nic,model=lan9118",
    ]

    net_user = "user"
    if guest_port:
        net_user += f",hostfwd=tcp:127.0.0.1:{host_port}-:{guest_port}"
    qemu_cmd.extend(["-net", net_user])

    proc = subprocess.Popen(
        qemu_cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    inspector_q: "queue.Queue[str]" = queue.Queue()
    ready_evt = threading.Event()
    guest_ip = ""

    def relay_stdout() -> None:
        nonlocal guest_ip
        assert proc.stdout is not None
        for line in proc.stdout:
            # sys.stdout.write(line)
            # sys.stdout.flush()

            # SUT events
            if "INSPECTOR:" in line:
                evt = line.split("INSPECTOR:")[1]
                inspector_q.put(evt.strip())
            elif line.startswith("SUT: ") or line.startswith("harness: "):
                sys.stdout.write("SUT: " + line)
                sys.stdout.flush()
            else:
                # Print QEMU internal errors/warnings
                if not line.startswith("READY "):
                    sys.stdout.write("QEMU: " + line)
                    sys.stdout.flush()

            m = READY_RE.search(line)
            if m:
                guest_ip = m.group("guest_ip")
                ready_evt.set()

    threading.Thread(target=relay_stdout, daemon=True).start()

    if not ready_evt.wait(timeout=30.0):
        proc.terminate()
        pytest.fail(f"QEMU binary {binary} did not print READY within 30s")

    if proc.poll() is not None:
        pytest.fail(f"QEMU binary {binary} exited early with rc={proc.returncode}")

    yield Harness(
        proc=proc,
        host="127.0.0.1",
        port=host_port,
        guest_ip=guest_ip,
        inspector=inspector_q,
    )

    # SYS_EXIT semihosting call will terminate QEMU if it reboots/exits.
    # Otherwise we terminate it here.
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
