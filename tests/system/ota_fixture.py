import contextlib
import re
import socket
import threading
import time
from pathlib import Path

import requests
from RangeHTTPServer import RangeRequestHandler


def get_system_test_versions():
    repo_root = Path(__file__).resolve().parents[2]
    content = (repo_root / "VERSION").read_text()
    major = int(re.search(r"VERSION_MAJOR\s*=\s*(\d+)", content).group(1))
    minor = int(re.search(r"VERSION_MINOR\s*=\s*(\d+)", content).group(1))
    patch = int(re.search(r"PATCHLEVEL\s*=\s*(\d+)", content).group(1))
    return {
        "base": f"v{major}.{minor}.{patch}",
        "b": f"v{major}.{minor}.{patch + 1}",
        "c": f"v{major}.{minor}.{patch + 2}",
    }


def get_mcuboot_test_versions():
    repo_root = Path(__file__).resolve().parents[2]
    version_file = repo_root / "bootloader" / "mcuboot" / "boot" / "zephyr" / "VERSION"
    if version_file.exists():
        content = version_file.read_text()
        major = int(re.search(r"VERSION_MAJOR\s*=\s*(\d+)", content).group(1))
        minor = int(re.search(r"VERSION_MINOR\s*=\s*(\d+)", content).group(1))
        patch = int(re.search(r"PATCHLEVEL\s*=\s*(\d+)", content).group(1))
        base_ver = f"v{major}.{minor}.{patch}"
        test_ver = f"v{major}.{minor}.{patch + 1}"
    else:
        base_ver = "v2.4.0"
        test_ver = "v2.4.1"
    return {
        "base": base_ver,
        "b": test_ver,
    }


@contextlib.contextmanager
def semihosting_keepalive(openocd):
    """Keepalive thread polling OpenOCD telnet so semihosting BKPT calls are serviced."""
    stop_event = threading.Event()
    s = socket.create_connection((openocd.host, openocd.telnet_port), timeout=5)
    time.sleep(0.1)
    s.recv(4096)
    s.sendall(b"arm semihosting enable\r")
    time.sleep(0.1)

    def _worker():
        s.settimeout(1.0)
        while not stop_event.is_set():
            try:
                s.sendall(b"poll\r")
                time.sleep(0.5)
                try:
                    s.recv(4096)
                except socket.timeout:
                    pass
            except OSError:
                break
        try:
            s.close()
        except Exception:
            pass

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    try:
        yield stop_event
    finally:
        stop_event.set()
        t.join(timeout=2)


def wait_for_boot(
    logs,
    start_idx,
    expected_slot,
    expected_version,
    expect_test_running=None,
    timeout=60,
):
    """Wait for firmware to complete boot, asserting active slot and version from MCUboot logs."""
    offset_str = "0x50000" if expected_slot == 0 else "0x1b0000"
    slot_name = "primary" if expected_slot == 0 else "secondary"
    version_str = f"Image version: {expected_version}"
    start_time = time.time()

    while time.time() - start_time < timeout:
        recent = logs[start_idx:]
        slot_found = any(
            f"Bootloader chainload address offset: {offset_str}" in line
            or f"loaded from the {slot_name} slot" in line
            for line in recent
        )
        version_found = any(version_str in line for line in recent)
        ready_found = any("System ready" in line for line in recent)
        test_running_found = any("OTA test image running" in line for line in recent)

        if slot_found and version_found and ready_found:
            if expect_test_running is not None:
                if expect_test_running == test_running_found:
                    return
            else:
                return

        time.sleep(0.5)

    recent_str = "\n".join(logs[start_idx:])
    raise TimeoutError(
        f"Timed out waiting for boot into slot={expected_slot} ({offset_str}), "
        f"version={expected_version}, test_running={expect_test_running}.\n"
        f"Logs since step start:\n{recent_str}"
    )


def wait_for_bootloader_boot(
    logs,
    start_idx,
    expected_bootloader_version,
    timeout=60,
):
    """Wait for MCUboot to complete boot, asserting its version from MCUboot logs."""
    bootloader_str = f"Starting Direct-XIP bootloader {expected_bootloader_version}"
    start_time = time.time()

    while time.time() - start_time < timeout:
        recent = logs[start_idx:]
        bootloader_found = any(bootloader_str in line for line in recent)
        ready_found = any("System ready" in line for line in recent)

        if bootloader_found and ready_found:
            return

        time.sleep(0.5)

    recent_str = "\n".join(logs[start_idx:])
    raise TimeoutError(
        f"Timed out waiting for MCUboot bootloader version={expected_bootloader_version}.\n"
        f"Logs since step start:\n{recent_str}"
    )


def wait_for_ip(logs, start_idx, family, timeout=120):
    """Wait until the device logs a fresh address of `family` ('v4' or 'v6').

    Returns the address (bare for v4, bracketed for v6). The device re-runs
    DHCP/SLAAC on every reboot, so this guarantees the control channel has a
    live, routable address before any HTTP request is issued against it.
    """
    pattern = (
        r"IPv4 address: ([0-9\.]+)"
        if family == "v4"
        else r"IPv6 address: ([0-9a-fA-F:]+)"
    )
    start_time = time.time()
    while time.time() - start_time < timeout:
        for line in reversed(logs[start_idx:]):
            m = re.search(pattern, line)
            if not m:
                continue
            addr = m.group(1)
            if family == "v6" and addr.startswith("fe80"):
                continue  # link-local isn't routable for the control channel
            return f"[{addr}]" if family == "v6" else addr
        time.sleep(0.5)
    raise TimeoutError(
        f"Timed out waiting for a fresh {family} address from the device.\n"
        f"Logs since step start:\n" + "\n".join(logs[start_idx:])
    )


def post_proto(ip, path, data, timeout=10):
    """Send a POST request with protobuf body in a single TCP segment."""
    host = ip.strip("[]")
    s = socket.create_connection((host, 80), timeout=timeout)
    msg = (
        f"POST {path} HTTP/1.1\r\n"
        f"Host: {ip}\r\n"
        f"Content-Type: application/protobuf\r\n"
        f"Content-Length: {len(data)}\r\n"
        f"Connection: close\r\n\r\n"
    ).encode() + data
    s.sendall(msg)
    resp = s.recv(1024).decode(errors="ignore")
    s.close()
    assert "200" in resp, f"POST {path} failed, response:\n{resp}"


def post_mcuboot(ip, bin_data, timeout=30):
    """Send a POST request to upgrade MCUboot."""
    url = f"http://{ip}/fwupgrade/mcuboot"
    return requests.post(
        url,
        data=bin_data,
        headers={"Content-Type": "application/octet-stream"},
        timeout=timeout,
    )


def reboot_device(openocd, device_ip, logs=None, start_idx=None, wait_reboot_timeout=5):
    """Reboot device via HTTP POST /restart or fallback to OpenOCD hardware reboot."""
    try:
        requests.post(f"http://{device_ip}/restart", timeout=2)
    except Exception:
        pass

    if logs is not None and start_idx is not None:
        t0 = time.time()
        while time.time() - t0 < wait_reboot_timeout:
            recent = logs[start_idx:]
            if any(
                "Starting Direct-XIP bootloader" in line or "Restarting system" in line
                for line in recent
            ):
                return
            time.sleep(0.2)
        print(
            "HTTP restart did not trigger reboot within timeout; falling back to OpenOCD hardware reset..."
        )

    openocd.reboot()


class ThrottledRangeRequestHandler(RangeRequestHandler):
    def copyfile(self, source, outputfile):
        if getattr(self, "range", None):
            start, stop = self.range
            if start is not None:
                source.seek(start)
            total = (stop - start + 1) if stop is not None else None
        else:
            total = None

        copied = 0
        bufsize = 512
        while True:
            to_read = min(bufsize, total - copied) if total is not None else bufsize
            if to_read <= 0:
                break
            buf = source.read(to_read)
            if not buf:
                break
            outputfile.write(buf)
            outputfile.flush()
            copied += len(buf)
            time.sleep(0.010)  # 10ms pacing prevents packet drops on device
