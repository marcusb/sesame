from __future__ import annotations

import os
import shutil
from typing import Iterator

import pytest

from _harness import Harness, TapConfig, _run_harness


@pytest.fixture(scope="session")
def chip_tool_path() -> str:
    p = shutil.which("chip-tool")
    if p is None:
        pytest.skip("chip-tool not on PATH; install Matter SDK chip-tool")
    return p


@pytest.fixture(scope="session")
def qemu_tap() -> TapConfig:
    """
    Returns a pre-created tap interface for QEMU. The test operator must set
    SESAME_QEMU_TAP to the interface name and have configured it like:

        sudo ip tuntap add dev tap-sesame mode tap user $USER
        sudo ip addr add 10.20.30.1/24 dev tap-sesame
        sudo ip link set tap-sesame up

    The guest IP is the static address baked into main_matter.c (10.20.30.2).
    """
    name = os.environ.get("SESAME_QEMU_TAP")
    if not name:
        pytest.skip(
            "SESAME_QEMU_TAP not set. To enable matter integration tests:\n"
            "  sudo ip tuntap add dev tap-sesame mode tap user $USER\n"
            "  sudo ip addr add 10.20.30.1/24 dev tap-sesame\n"
            "  sudo ip link set tap-sesame up\n"
            "  export SESAME_QEMU_TAP=tap-sesame"
        )
    return TapConfig(name=name, host_ip="10.20.30.1", guest_ip="10.20.30.2")


@pytest.fixture()
def matter_harness(qemu_tap: TapConfig) -> Iterator[Harness]:
    gen = _run_harness("matter", net_mode="tap", tap=qemu_tap)
    harness = next(gen)
    yield harness
    try:
        next(gen)
    except StopIteration:
        pass
