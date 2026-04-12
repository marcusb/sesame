"""
Shared test-harness helpers for Sesame host tests.

Each module's conftest.py imports _run_harness and Harness from here to build
its own fixture, adding any protocol-specific readiness checks before yielding.
"""

from __future__ import annotations

import os
import queue
import re
import socket
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import pytest

READY_RE = re.compile(
    r"READY host=(?P<host>\S+) port=(?P<port>\d+) inspector=(?P<insp>\d+)"
)

_REPO_ROOT = Path(__file__).parent.parent.parent

_BUILD_DIRS = [
    _REPO_ROOT / "build" / "host-tests" / "test" / "host",
    _REPO_ROOT / "build-host" / "test" / "host",
]


def _find_build_artifact(rel: str) -> Path:
    for bd in _BUILD_DIRS:
        p = bd / rel
        if p.exists():
            return p
    paths = "\n  ".join(str(bd / rel) for bd in _BUILD_DIRS)
    pytest.fail(f"build artifact not found; run `ninja -C build host_tests` first.\nLooked in:\n  {paths}")


def _free_port(kind: int = socket.SOCK_STREAM) -> int:
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@dataclass
class Harness:
    proc: subprocess.Popen
    host: str
    port: int
    cmd_port: int
    inspector: "queue.Queue[str]"

    def wait_event(self, *, timeout: float = 5.0) -> str:
        return self.inspector.get(timeout=timeout)

    def drain(self) -> list[str]:
        out = []
        while True:
            try:
                out.append(self.inspector.get_nowait())
            except queue.Empty:
                return out

    def send_cmd(self, line: str) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(line.encode(), ("127.0.0.1", self.cmd_port))
        finally:
            sock.close()


def _run_harness(binary_env_var: str, binary_name: str | None = None) -> Iterator[Harness]:
    name = binary_name or binary_env_var
    binary_str = os.environ.get(binary_env_var)
    binary = Path(binary_str) if binary_str else _find_build_artifact(
        f"{name}/host_test_{name}"
    )

    env = os.environ.copy()

    if not env.get("LD_PRELOAD"):
        shim = _find_build_artifact("harness/libslirp_hostfwd.so")
        env["LD_PRELOAD"] = str(shim)

    host_port = _free_port(socket.SOCK_STREAM)
    insp_port = _free_port(socket.SOCK_DGRAM)
    cmd_port = _free_port(socket.SOCK_DGRAM)

    insp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    insp_sock.bind(("127.0.0.1", insp_port))
    insp_sock.settimeout(0.2)

    inspector_q: "queue.Queue[str]" = queue.Queue()

    def pump() -> None:
        while True:
            try:
                data, _ = insp_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            inspector_q.put(data.decode("utf-8", errors="replace"))

    threading.Thread(target=pump, daemon=True).start()

    env["HARNESS_HOST_HOST"] = "127.0.0.1"
    env["HARNESS_HOST_PORT"] = str(host_port)
    env["HARNESS_INSPECTOR_HOST"] = "127.0.0.1"
    env["HARNESS_INSPECTOR_PORT"] = str(insp_port)
    env["HARNESS_CMD_PORT"] = str(cmd_port)

    proc = subprocess.Popen(
        [str(binary)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env,
        text=True,
        bufsize=1,
    )

    ready_evt = threading.Event()

    def relay_stdout() -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            if READY_RE.search(line):
                ready_evt.set()

    threading.Thread(target=relay_stdout, daemon=True).start()

    if not ready_evt.wait(timeout=20.0):
        proc.terminate()
        pytest.fail(f"host binary {binary} did not print READY within 20s")
    if proc.poll() is not None:
        pytest.fail(f"host binary {binary} exited early with rc={proc.returncode}")

    yield Harness(
        proc=proc,
        host="127.0.0.1",
        port=host_port,
        cmd_port=cmd_port,
        inspector=inspector_q,
    )

    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
    insp_sock.close()
