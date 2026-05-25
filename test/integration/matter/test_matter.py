"""
End-to-end Matter integration tests.

The SUT (matter_it-qemu.axf) boots the real Matter task in QEMU on a tap
interface. These tests exercise:

  1. mDNS publication of the commissionable service (_matterc._udp).
  2. PASE+CASE commissioning via chip-tool.
  3. WindowCovering cluster commands round-tripping through ctrl_queue.
  4. Door-state attribute reports flowing back to the controller.

Prerequisites: see conftest.qemu_tap for tap setup, and chip-tool on PATH.
"""

from __future__ import annotations

import socket
import subprocess
import time
from pathlib import Path

import pytest
import zeroconf
from zeroconf import ServiceBrowser, ServiceListener, Zeroconf

from _harness import Harness


# Stable values baked into matter_task.c QEMU bootstrap (commit ec90347).
QEMU_DISCRIMINATOR = 3840
QEMU_PASSCODE = 20202021
MATTER_NODE_ID = 1
SESAME_DOOR_ENDPOINT = 2
MATTER_UDP_PORT = 5540
# Deterministic IPv6 link-local derived from the test MAC 00:11:22:33:44:55
# (RFC 4291 EUI-64 with U/L bit flipped).
QEMU_LINK_LOCAL = "fe80::211:22ff:fe33:4455"


def _qemu_link_local(tap_iface: str) -> str:
    return f"{QEMU_LINK_LOCAL}%{tap_iface}"


class _Collector(ServiceListener):
    def __init__(self) -> None:
        self.services: dict[str, object] = {}

    def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        info = zc.get_service_info(type_, name, timeout=5000)
        if info is not None:
            self.services[name] = info

    def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        self.add_service(zc, type_, name)

    def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        self.services.pop(name, None)


def _browse(
    service: str,
    timeout: float = 20.0,
    predicate=None,
    tap_iface: str | None = None,
) -> dict[str, object]:
    """Browse mDNS for `service`.

    If `predicate` is None, returns as soon as any service is seen (or after
    `timeout`). If `predicate` is given (called with a ServiceInfo), returns
    as soon as some collected service matches; otherwise keeps waiting until
    timeout. This lets callers select a specific instance on a LAN that may
    have several advertisers of the same service type.

    If `tap_iface` is given, binds zeroconf to that interface explicitly
    (zeroconf's default interface selection may skip TAP interfaces).
    """
    if tap_iface is not None:
        idx = socket.if_nametoindex(tap_iface)
        zc = Zeroconf(interfaces=[idx])
    else:
        zc = Zeroconf()
    listener = _Collector()
    ServiceBrowser(zc, service, listener)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if listener.services:
                if predicate is None:
                    return dict(listener.services)
                if any(predicate(i) for i in listener.services.values()):
                    return dict(listener.services)
            time.sleep(0.25)
        return dict(listener.services)
    finally:
        zc.close()


def _has_qemu_addr(info) -> bool:
    # QEMU advertises only IPv6 (INET_CONFIG_ENABLE_IPV4=0).
    # Check for the EUI-64 link-local derived from MAC 00:11:22:33:44:55.
    # FreeRTOS+TCP may skip the U/L bit flip, producing fe80::11:22ff:fe33:4455
    # instead of fe80::211:22ff:fe33:4455. Match either form.
    v6 = info.parsed_addresses(version=zeroconf.IPVersion.V6Only)
    for a in v6:
        low = a.lower()
        if "11:22ff:fe33:4455" in low or "211:22ff:fe33:4455" in low:
            return True
    return False


def _run_chip_tool(
    chip_tool: str, storage: Path, *args: str, timeout: float = 90.0
) -> subprocess.CompletedProcess[str]:
    cmd = [chip_tool, *args, "--storage-directory", str(storage)]
    return subprocess.run(
        cmd, capture_output=True, text=True, timeout=timeout, check=False
    )


def test_mdns_advertises_commissionable_service(
    matter_harness: Harness, qemu_tap
) -> None:
    services = _browse(
        "_matterc._udp.local.",
        timeout=30.0,
        predicate=_has_qemu_addr,
        tap_iface=qemu_tap.name,
    )
    assert services, (
        "no _matterc._udp service found via mDNS within 30s — "
        "matter_mdns publication is broken (host can reach guest?)"
    )

    # The host LAN may have other Matter commissioners (e.g. a real Sesame
    # device on the network) advertising _matterc._udp. Filter to the SUT by
    # IPv6 link-local — our QEMU guest uses the deterministic EUI-64
    # derived from MAC 00:11:22:33:44:55.
    info = next((i for i in services.values() if _has_qemu_addr(i)), None)

    # Re-resolve so we pick up AAAA/TXT records that may have arrived after
    # the SRV/TXT triggered the predicate, or were rate-limited by the SUT's
    # mDNS responder (RFC 6762 §6 limits multicast responses to 1/second).
    # Wait >1s before re-querying to let the rate limiter reset.
    if info is not None:
        time.sleep(1.5)
        tap_idx = socket.if_nametoindex(qemu_tap.name)
        zc = Zeroconf(interfaces=[tap_idx])
        try:
            from zeroconf import ServiceInfo
            refreshed = ServiceInfo(info.type, info.name)
            if refreshed.request(zc, timeout=5000):
                info = refreshed
        finally:
            zc.close()
    assert info is not None, (
        f"no _matterc._udp instance found among "
        f"{[(s, list(i.parsed_addresses())) for s, i in services.items()]}"
)
    txt = {
        k.decode(): (v.decode() if isinstance(v, bytes) else v)
        for k, v in info.properties.items()
        if k is not None
    }

    # Matter discriminator advertised in DNS-SD as "D" (per Matter Core spec
    # 4.3.1.4). We seeded 3840 in matter_task.c.
    assert txt.get("D") == str(QEMU_DISCRIMINATOR), txt
    # CM=1 means Basic Commissioning Mode advertised.
    assert txt.get("CM") in ("1", "2"), txt
    assert info.port == MATTER_UDP_PORT

    # QEMU runs IPv6-only (INET_CONFIG_ENABLE_IPV4=0), so only AAAA is advertised.
    v6 = info.parsed_addresses(version=zeroconf.IPVersion.V6Only)
    assert v6, f"no IPv6 address advertised for {info.server!r} — chip-tool needs IPv6"

    # Sanity-check the IPv6 address: deterministic EUI-64 from MAC 00:11:22:33:44:55.
    assert any(a.lower().startswith("fe80::") for a in v6), v6
    for a in v6:
        low = a.lower()
        if "11:22ff:fe33:4455" in low or "211:22ff:fe33:4455" in low:
            break
    else:
        assert False, f"expected EUI-64 from MAC 00:11:22:33:44:55 in {v6}"


def test_chip_tool_pairs_and_drives_door(
    matter_harness: Harness,
    chip_tool_path: str,
    tmp_path: Path,
    qemu_tap,
) -> None:
    storage = tmp_path / "chip_kvs"
    storage.mkdir()

    # Drain any boot-time CTRL events before pairing.
    matter_harness.drain()

    # Pair using the SUT's known IPv6 link-local. chip-tool's minmDNS resolver
    # doesn't browse tap-sesame, so onnetwork discovery times out — bypass it.
    pair = _run_chip_tool(
        chip_tool_path,
        storage,
        "pairing",
        "already-discovered",
        str(MATTER_NODE_ID),
        str(QEMU_PASSCODE),
        _qemu_link_local(qemu_tap.name),
        str(MATTER_UDP_PORT),
        timeout=120.0,
    )
    assert pair.returncode == 0, (
        f"chip-tool pairing failed (rc={pair.returncode})\n"
        f"stdout:\n{pair.stdout[-4000:]}\n"
        f"stderr:\n{pair.stderr[-2000:]}"
    )

    # Send up-or-open. WindowCovering cluster, command 0x00 → DOOR_CMD_OPEN.
    rc = _run_chip_tool(
        chip_tool_path,
        storage,
        "windowcovering",
        "up-or-open",
        str(MATTER_NODE_ID),
        str(SESAME_DOOR_ENDPOINT),
    )
    assert rc.returncode == 0, rc.stderr

    # Wait for the harness inspector to echo the ctrl_queue entry. cmd=1 is
    # DOOR_CMD_OPEN per controller.h:22.
    deadline = time.monotonic() + 5.0
    saw_open = False
    while time.monotonic() < deadline and not saw_open:
        for evt in matter_harness.drain():
            if "CTRL DOOR_CONTROL" in evt and "cmd=1" in evt:
                saw_open = True
                break
        if not saw_open:
            time.sleep(0.1)
    assert saw_open, "did not see CTRL DOOR_CONTROL cmd=1 (OPEN) on inspector"

    # down-or-close → DOOR_CMD_CLOSE (cmd=2)
    _run_chip_tool(
        chip_tool_path,
        storage,
        "windowcovering",
        "down-or-close",
        str(MATTER_NODE_ID),
        str(SESAME_DOOR_ENDPOINT),
    )
    deadline = time.monotonic() + 5.0
    saw_close = False
    while time.monotonic() < deadline and not saw_close:
        for evt in matter_harness.drain():
            if "CTRL DOOR_CONTROL" in evt and "cmd=2" in evt:
                saw_close = True
                break
        if not saw_close:
            time.sleep(0.1)
    assert saw_close, "did not see CTRL DOOR_CONTROL cmd=2 (CLOSE) on inspector"


def test_door_state_report_round_trip(
    matter_harness: Harness,
    chip_tool_path: str,
    tmp_path: Path,
    qemu_tap,
) -> None:
    storage = tmp_path / "chip_kvs"
    storage.mkdir()

    pair = _run_chip_tool(
        chip_tool_path,
        storage,
        "pairing",
        "already-discovered",
        str(MATTER_NODE_ID),
        str(QEMU_PASSCODE),
        _qemu_link_local(qemu_tap.name),
        str(MATTER_UDP_PORT),
        timeout=120.0,
    )
    assert pair.returncode == 0, pair.stderr

    # Synthesize a door-state update from the firmware side. State 1=OPEN,
    # direction 1=UP (opening), pos=50%. matter_report_door_state pushes this
    # into the Matter shadow which triggers an attribute report.
    matter_harness.send_cmd("door_state 1 1 50")
    time.sleep(1.0)

    # Read the lift percentage attribute. Encoded as percent100ths so 50% → 5000.
    read = _run_chip_tool(
        chip_tool_path,
        storage,
        "windowcovering",
        "read",
        "current-position-lift-percent100ths",
        str(MATTER_NODE_ID),
        str(SESAME_DOOR_ENDPOINT),
    )
    assert read.returncode == 0, read.stderr
    assert "5000" in read.stdout, (
        f"expected position 5000 in chip-tool read output, got:\n{read.stdout}"
    )
