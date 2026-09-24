import contextlib
import functools
from http.server import HTTPServer
import os
from pathlib import Path
import re
import socket
import sys
import threading
import time

import requests
from RangeHTTPServer import RangeRequestHandler

# Ensure proto is importable
sys.path.insert(0, str(Path(__file__).parent))
from proto import api_pb2  # noqa: E402


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
    offset_str = "0x30000" if expected_slot == 0 else "0x1a0000"
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
        f"Timed out waiting for boot into slot={expected_slot} ({offset_str}), version={expected_version}, test_running={expect_test_running}.\n"
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
    # The device's HTTP server needs the request in a single segment with
 # Connection: close; create_connection takes a bare address (no IPv6 brackets)
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


def reboot_device(
    openocd, device_ip, logs=None, start_idx=None, wait_reboot_timeout=5
):
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
                "Starting Direct-XIP bootloader" in l or "Restarting system" in l
                for l in recent
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
            to_read = (
                min(bufsize, total - copied) if total is not None else bufsize
            )
            if to_read <= 0:
                break
            buf = source.read(to_read)
            if not buf:
                break
            outputfile.write(buf)
            outputfile.flush()
            copied += len(buf)
            time.sleep(0.010)  # 10ms pacing prevents packet drops on device


def test_ota(hardware_device, openocd):
    """
    This starts with a flashed mcuboot+sesame_test image (A).
    runs OTA upgrade to image B via http endpoint. verify that it boots B from image-1 slot.
    promote the image and reboot. verify that it boots B from image-1 slot again.
    run OTA upgrade to image C via http endpoint. verify that it boots C from image-0 slot.
    do not promote, reboot. verify that it reverts to image B from image-1 slot.
    """
    device_ipv4 = hardware_device.get("ipv4")
    device_ipv6 = hardware_device.get("ipv6")
    logs = hardware_device["logs"]
    assert device_ipv4 and device_ipv6, (
        "This test exercises a complete OTA step over each address family, so "
        f"the device must be dual-stack (got v4={device_ipv4}, v6={device_ipv6})"
    )

    versions = get_system_test_versions()

    print("\n[Step 1] Verifying initial boot of Image A from slot 0...")
    wait_for_boot(
        logs,
        0,
        expected_slot=0,
        expected_version=versions["base"],
        expect_test_running=False,
        timeout=10,
    )
    print("Initial boot into Image A verified successfully.")

    image_dir = os.path.abspath("build/sesame_test/zephyr")
    handler = functools.partial(ThrottledRangeRequestHandler, directory=image_dir)

    servers = []

    def host_for(device_addr, af):
        """Local address (of family af) that routes to the device's address."""
        s = socket.socket(af, socket.SOCK_DGRAM)
        s.connect((device_addr, 80))
        host_ip = s.getsockname()[0]
        s.close()
        return host_ip

    def start_server(host_ip, af):
        class FamilyHTTPServer(HTTPServer):
            address_family = af

        srv = FamilyHTTPServer((host_ip, 0), handler)
        port = srv.server_address[1]
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        servers.append(srv)
        url_host = f"[{host_ip}]" if af == socket.AF_INET6 else host_ip
        print(
            f"Started RangeHTTPServer at http://{url_host}:{port} serving {image_dir}"
        )
        return f"http://{url_host}:{port}"

    # One download server per family. A single run exercises both address
    # families: the A->B upgrade runs entirely over IPv4 (control + download)
    # and the B->C upgrade entirely over IPv6, so each protocol family is
    # validated for a complete OTA step.
    base = {
        "ipv4": start_server(host_for(device_ipv4, socket.AF_INET), socket.AF_INET),
        "ipv6": start_server(
            host_for(device_ipv6, socket.AF_INET6), socket.AF_INET6
        ),
    }

    try:
        # Step 2: OTA Upgrade to Image B, entirely over IPv4
        print("\n[Step 2] OTA A->B over IPv4 (control + download)...")
        step_start = len(logs)
        req = api_pb2.FirmwareUpgradeFetchRequest()
        req.url = base["ipv4"] + "/image_b.signed.bin"
        with semihosting_keepalive(openocd):
            post_proto(device_ipv4, "/fwupgrade", req.SerializeToString())
            print("FirmwareUpgradeFetchRequest sent over IPv4. Waiting for Image B...")
            wait_for_boot(
                logs,
                step_start,
                expected_slot=1,
                expected_version=versions["b"],
                expect_test_running=True,
                timeout=90,
            )
        # B re-ran DHCP on reboot; wait for its fresh IPv4 before controlling it
        ctrl_v4 = wait_for_ip(logs, step_start, "v4")
        print(f"Booted Image B (test mode) from slot 1; fresh IPv4={ctrl_v4}")

        # Step 3: Promote Image B and Reboot (over IPv4)
        print("\n[Step 3] Promoting Image B and rebooting (over IPv4)...")
        resp = requests.post(f"http://{ctrl_v4}/promote", timeout=5)
        assert resp.status_code == 200, f"promote failed with status {resp.status_code}"
        time.sleep(1)

        step_start = len(logs)
        with semihosting_keepalive(openocd):
            print("Rebooting device...")
            reboot_device(openocd, ctrl_v4, logs=logs, start_idx=step_start)
            wait_for_boot(
                logs,
                step_start,
                expected_slot=1,
                expected_version=versions["b"],
                expect_test_running=False,
                timeout=60,
            )
        # Next control step (B->C) runs over IPv6; wait for that address
        ctrl_v6 = wait_for_ip(logs, step_start, "v6")
        print(
            f"Rebooted into confirmed Image B from slot 1; fresh IPv6={ctrl_v6}"
        )

        # Step 4: OTA Upgrade to Image C, entirely over IPv6
        print("\n[Step 4] OTA B->C over IPv6 (control + download)...")
        step_start = len(logs)
        req = api_pb2.FirmwareUpgradeFetchRequest()
        req.url = base["ipv6"] + "/image_c.signed.bin"
        with semihosting_keepalive(openocd):
            post_proto(ctrl_v6, "/fwupgrade", req.SerializeToString())
            print("FirmwareUpgradeFetchRequest sent over IPv6. Waiting for Image C...")
            wait_for_boot(
                logs,
                step_start,
                expected_slot=0,
                expected_version=versions["c"],
                expect_test_running=True,
                timeout=90,
            )
        ctrl_v6 = wait_for_ip(logs, step_start, "v6")
        print(f"Booted Image C (test mode) from slot 0; fresh IPv6={ctrl_v6}")

        # Step 5: Do not promote, reboot and verify rollback to Image B (IPv6)
        print(
            "\n[Step 5] Rebooting without promoting to verify rollback to Image B (over IPv6)..."
        )
        step_start = len(logs)
        with semihosting_keepalive(openocd):
            reboot_device(openocd, ctrl_v6, logs=logs, start_idx=step_start)
            wait_for_boot(
                logs,
                step_start,
                expected_slot=1,
                expected_version=versions["b"],
                expect_test_running=False,
                timeout=60,
            )
        wait_for_ip(logs, step_start, "v6")
        print("Reverted to Image B from slot 1.")
        print("\nOTA test sequence passed completely! (A->B over IPv4, B->C over IPv6)")

    finally:
        for srv in servers:
            srv.shutdown()
            srv.server_close()
