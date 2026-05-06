"""
Matter integration smoke tests that don't require chip-tool or a tap interface.

These boot the real Matter task in QEMU (with default SLIRP networking) and
inspect the mDNS publication state via the harness command channel. They are
the first line of defense for the regression where _matterc._udp records
don't appear on real hardware (avahi-browse returns nothing).
"""

from __future__ import annotations

import time
from typing import Iterator

import pytest

from _harness import Harness, _run_harness


@pytest.fixture()
def matter_smoke_harness() -> Iterator[Harness]:
    """SLIRP networking. mDNS multicast won't traverse SLIRP, but the
    in-firmware record set is observable via the dump_mdns command."""
    gen = _run_harness("matter")
    harness = next(gen)
    yield harness
    try:
        next(gen)
    except StopIteration:
        pass


def _collect_mdns(harness: Harness, settle_s: float = 8.0) -> list[str]:
    """Wait for matter bootstrap to complete, then ask the SUT to dump its
    registered mDNS records and return the lines."""
    time.sleep(settle_s)
    harness.drain()
    harness.send_cmd("dump_mdns")

    records: list[str] = []
    deadline = time.monotonic() + 5.0
    count = -1
    while time.monotonic() < deadline:
        for evt in harness.drain():
            if evt.startswith("MDNS count="):
                count = int(evt.split("=", 1)[1])
                records = []
            elif evt.startswith("MDNS [") and count >= 0:
                records.append(evt)
                if len(records) >= count:
                    return records
        time.sleep(0.1)
    return records


def test_matterc_udp_service_record_registered(matter_smoke_harness: Harness) -> None:
    """The PTR/SRV/TXT triple for _matterc._udp must be present once
    commissioning starts."""
    records = _collect_mdns(matter_smoke_harness)
    assert records, "no mDNS records dumped — matter bootstrap likely failed"

    types_by_name: dict[str, set[str]] = {}
    for line in records:
        # Format: "MDNS [N] TYPE name [extras]"
        parts = line.split(maxsplit=3)
        if len(parts) < 4:
            continue
        rtype = parts[2]
        name = parts[3].split()[0]
        types_by_name.setdefault(name, set()).add(rtype)

    matterc_ptr = "_matterc._udp.local"
    assert matterc_ptr in types_by_name, (
        f"no PTR for {matterc_ptr} found. Dumped records:\n"
        + "\n".join(records)
    )
    assert "PTR" in types_by_name[matterc_ptr]

    # There should be exactly one instance under _matterc._udp; find its FQDN.
    instance_fqdns = [n for n in types_by_name if n.endswith("._matterc._udp.local")]
    assert instance_fqdns, (
        "no _matterc._udp instance fqdn registered. Records:\n"
        + "\n".join(records)
    )
    instance = instance_fqdns[0]
    assert "SRV" in types_by_name[instance], types_by_name[instance]
    assert "TXT" in types_by_name[instance], types_by_name[instance]


def test_commissionable_subtypes_registered(matter_smoke_harness: Harness) -> None:
    """Discriminator/short-discriminator/vendor/CM subtype PTRs must be
    advertised so DNS-SD subtype browses (used by chip-tool's discovery
    filters) work."""
    records = _collect_mdns(matter_smoke_harness)
    names = [line.split(maxsplit=3)[3].split()[0] for line in records]
    expected_subs = ["_L3840", "_S15", "_CM1"]
    for sub in expected_subs:
        prefix = f"{sub}._sub._matterc._udp.local"
        assert any(n.startswith(prefix) for n in names), (
            f"missing subtype {sub} in mDNS records. Names:\n"
            + "\n".join(names)
        )
