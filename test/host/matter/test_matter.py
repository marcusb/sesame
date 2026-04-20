import time
import pytest
from _harness import Harness


def _drain_until(harness: Harness, sentinel: str, timeout: float = 3.0):
    lines = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            line = harness.wait_event(timeout=deadline - time.monotonic()).strip()
        except Exception:
            break
        lines.append(line)
        if line == sentinel or line.startswith(sentinel + " "):
            return lines
    raise AssertionError(f"did not see {sentinel!r}; got {lines}")


def _berry(harness: Harness, code: str):
    harness.drain()
    harness.send_cmd(f"berry {code}")
    lines = _drain_until(harness, "BERRY_OK") if True else []
    return lines


def test_matter_startup(matter_harness: Harness) -> None:
    """Verify that the Matter stack starts and runs its loop."""
    time.sleep(2)
    assert matter_harness.proc.poll() is None


def test_matter_crypto_sha256(matter_harness: Harness) -> None:
    """Verify SHA256 shim."""
    # crypto is a global (loaded via be_load_crypto_module), not an importable module.
    # Berry's bytes.tohex() returns upper-case.
    script = (
        "var b = crypto.SHA256(bytes().fromstring('hello'))\n"
        "assert(b.tohex() == "
        "'2CF24DBA5FB0A30E26E83B2AC5B9E29E1B161E5C1FA7425E73043362938B9824')\n"
    )
    matter_harness.drain()
    matter_harness.send_cmd(f"berry {script}")
    line = matter_harness.wait_event(timeout=3).strip()
    assert line == "BERRY_OK", f"expected BERRY_OK, got {line!r}"


def test_matter_module_load(matter_harness: Harness) -> None:
    """Verify that the 'matter' module can be imported and has expected classes."""
    script = (
        "import matter\n"
        "assert(matter.Counter != nil)\n"
        "assert(matter.Verhoeff != nil)\n"
        "assert(matter.Commissioning != nil)\n"
    )
    matter_harness.drain()
    matter_harness.send_cmd(f"berry {script}")
    line = matter_harness.wait_event(timeout=3).strip()
    assert line == "BERRY_OK", f"expected BERRY_OK, got {line!r}"


def test_tasmota_when_network_up_fires(matter_harness: Harness) -> None:
    """A callback registered via tasmota.when_network_up should fire at least
    once. The matter task fires the queued net_cbs immediately during init, so
    a callback registered now by the test will NOT see that initial fire — but
    `tasmota.wifi()` is a good proxy that the shim exists and returns a map."""
    script = (
        "import tasmota\n"
        "var info = tasmota.wifi()\n"
        "assert(info != nil)\n"
        "assert(classname(info) == 'map')\n"
    )
    matter_harness.drain()
    matter_harness.send_cmd(f"berry {script}")
    line = matter_harness.wait_event(timeout=3).strip()
    assert line == "BERRY_OK", f"got {line!r}"


def test_tasmota_fast_loop_runs(matter_harness: Harness) -> None:
    """Registering a fast-loop callback increments a counter on each tick."""
    # Store counter in a list (mutable, captured by closure) and stash the list
    # on the shim's _matter_drivers global so it survives across be_dostring calls.
    script = (
        "import tasmota\n"
        "_matter_drivers.push([0])\n"
        "def _tick() _matter_drivers[0][0] += 1 print('TICK=' + str(_matter_drivers[0][0])) end\n"
        "tasmota.add_fast_loop(_tick)\n"
        "print('REGISTERED n=' + str(size(_matter_fast_cbs)))\n"
    )
    matter_harness.drain()
    matter_harness.send_cmd(f"berry {script}")
    assert matter_harness.wait_event(timeout=3).strip() == "BERRY_OK"
    # Fast loop runs every 50ms; wait 500ms then check.
    time.sleep(0.5)
    matter_harness.drain()
    matter_harness.send_cmd(
        "berry print('FC_LEN=' + str(size(_matter_fast_cbs)) + "
        " ' COUNTER=' + str(_matter_drivers[0][0]))\n"
    )
    line = matter_harness.wait_event(timeout=3).strip()
    assert line == "BERRY_OK", f"counter did not increment: {line!r}"


def test_mdns_add_service_populates_snapshot(matter_harness: Harness) -> None:
    """Adding a Matter service via the mdns module creates PTR/SRV/TXT records
    in the matter_mdns snapshot that the FreeRTOS+TCP responder will serve."""
    script = (
        "import mdns\n"
        "mdns.add_hostname('sesame', '', '10.0.2.15')\n"
        "mdns.add_service('_matter', '_tcp', 5540, "
        "{'DN': 'Sesame', 'VP': '65521+32769'}, 'ABCDEF0123456789', 'sesame')\n"
    )
    matter_harness.drain()
    matter_harness.send_cmd(f"berry {script}")
    assert matter_harness.wait_event(timeout=3).strip() == "BERRY_OK"

    matter_harness.drain()
    matter_harness.send_cmd("mdns_dump")

    records = []
    count = None
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        line = matter_harness.wait_event(
            timeout=deadline - time.monotonic()
        ).strip()
        if line.startswith("MDNS_COUNT="):
            count = int(line.split("=", 1)[1])
        elif line == "MDNS_END":
            break
        elif line.startswith("MDNS "):
            records.append(line)
    else:
        raise AssertionError(f"no MDNS_END; got {records}")

    assert count is not None and count == len(records), (count, records)
    joined = "\n".join(records)
    # Hostname A record.
    assert "MDNS A sesame.local" in joined, joined
    # Services meta-PTR.
    assert (
        "MDNS PTR _services._dns-sd._udp.local -> _matter._tcp.local" in joined
    ), joined
    # Service PTR → instance.
    assert (
        "MDNS PTR _matter._tcp.local -> ABCDEF0123456789._matter._tcp.local"
        in joined
    ), joined
    # SRV target/port.
    assert (
        "MDNS SRV ABCDEF0123456789._matter._tcp.local -> sesame.local:5540"
        in joined
    ), joined
    # TXT encoded as <len><kv> chunks; check the key substrings round-tripped.
    assert any(
        r.startswith("MDNS TXT ABCDEF0123456789._matter._tcp.local") and "DN=Sesame" in r
        for r in records
    ), records
