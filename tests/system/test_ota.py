import functools
from http.server import HTTPServer
import os
from pathlib import Path
import socket
import sys
import threading
import time

from ota_fixture import (  # noqa: F401
    ThrottledRangeRequestHandler,
    get_system_test_versions,
    post_proto,
    reboot_device,
    semihosting_keepalive,
    wait_for_boot,
    wait_for_ip,
)
import requests

# Ensure proto is importable
sys.path.insert(0, str(Path(__file__).parent))
from proto import api_pb2  # noqa: E402


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
