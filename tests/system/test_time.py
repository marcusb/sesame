import datetime
import re
import sys
from pathlib import Path
import time

import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# Ensure system tests root is on sys.path
sys.path.insert(0, str(Path(__file__).parent))
from test_ota import reboot_device, semihosting_keepalive, wait_for_ip  # noqa: E402


def get_http_session():
    session = requests.Session()
    retry = Retry(connect=5, read=5, backoff_factor=0.5)
    adapter = HTTPAdapter(max_retries=retry)
    session.mount("http://", adapter)
    session.mount("https://", adapter)
    return session


def parse_iso8601(iso_str):
    # Parse e.g. "2026-09-24T00:30:00Z"
    return datetime.datetime.strptime(iso_str, "%Y-%m-%dT%H:%M:%SZ").replace(
        tzinfo=datetime.timezone.utc
    )


def test_rtc_sync_and_timekeeping(hardware_device):
    """
    Verifies that the RTC driver initializes, syncs time via SNTP,
    increments forward in real-time, and serves the time via GET /time.
    """
    ip = hardware_device["ip"]
    logs = hardware_device["logs"]
    session = get_http_session()

    print(f"\n[Test 1] Checking RTC time sync and timekeeping on http://{ip}...")

    # Wait up to 30s for SNTP sync or restored RTC time to be logged
    sync_logged = False
    start_wait = time.time()
    while time.time() - start_wait < 30:
        if any(
            "RTC time synchronized:" in line
            or "Restored time from RTC NVRAM:" in line
            or "SNTP sync success" in line
            for line in logs
        ):
            sync_logged = True
            break
        time.sleep(0.5)

    assert sync_logged, (
        f"RTC was neither restored nor synchronized within timeout.\n"
        f"Logs:\n{chr(10).join(logs[-30:])}"
    )

    # Query /time endpoint
    resp1 = session.get(f"http://{ip}/time", timeout=5)
    assert resp1.status_code == 200, f"/time endpoint returned status {resp1.status_code}"
    data1 = resp1.json()
    print(f"Initial time response: {data1}")

    epoch1 = data1["epoch"]
    iso1 = data1["time"]

    # Verify timestamp is valid real-world time (> year 2024, epoch > 1704067200)
    assert epoch1 > 1704067200, f"Epoch {epoch1} appears uninitialized (expected > 2024)"

    dt1 = parse_iso8601(iso1)
    assert dt1.year >= 2024, f"Parsed year {dt1.year} is invalid"

    # Sleep 3 seconds and query again to verify time increments correctly via crystal
    time.sleep(3)
    resp2 = session.get(f"http://{ip}/time", timeout=5)
    assert resp2.status_code == 200
    data2 = resp2.json()
    epoch2 = data2["epoch"]
    print(f"Time response after 3s: {data2}")

    delta = epoch2 - epoch1
    assert 2 <= delta <= 5, (
        f"RTC clock did not advance properly over 3s interval: "
        f"start={epoch1}, end={epoch2}, delta={delta}s"
    )

    print("RTC sync and timekeeping verified successfully!")


def test_time_restored_from_nvram_across_reboot(hardware_device, openocd):
    """
    Verifies that wall-clock time is periodically persisted to BBRAM/NVRAM,
    survives a reboot, and is restored by the boot sequence before SNTP syncs.
    """
    ip = hardware_device["ip"]
    logs = hardware_device["logs"]
    session = get_http_session()

    print(f"\n[Test 2] Verifying NVRAM time persistence across reboot on http://{ip}...")

    # Wait at least 11 seconds to guarantee the 10-second periodic save work has fired
    print("Waiting for periodic NVRAM save cycle...")
    time.sleep(11)

    # Query current time right before reboot
    resp = session.get(f"http://{ip}/time", timeout=5)
    assert resp.status_code == 200
    pre_reboot_data = resp.json()
    pre_reboot_epoch = pre_reboot_data["epoch"]
    pre_reboot_time = pre_reboot_data["time"]
    print(f"Pre-reboot time: epoch={pre_reboot_epoch}, time={pre_reboot_time}")

    step_start = len(logs)
    with semihosting_keepalive(openocd):
        print("Rebooting device...")
        reboot_device(openocd, ip, logs=logs, start_idx=step_start)

        # Wait for the device to boot and log its restored time
        restored_epoch = None
        restored_time_str = None
        boot_timeout = 60
        t0 = time.time()
        while time.time() - t0 < boot_timeout:
            recent_logs = logs[step_start:]
            for line in recent_logs:
                m = re.search(r"Restored time from RTC NVRAM: ([0-9T:-]+Z)", line)
                if m:
                    restored_time_str = m.group(1)
                    dt_restored = parse_iso8601(restored_time_str)
                    restored_epoch = int(dt_restored.timestamp())
                    break
            if restored_epoch is not None:
                break
            time.sleep(0.2)

        assert restored_epoch is not None, (
            f"Device did not log 'Restored time from RTC NVRAM' after reboot.\n"
            f"Post-reboot logs:\n{chr(10).join(logs[step_start:])}"
        )

        print(
            f"Device restored time from NVRAM at boot: {restored_time_str} (epoch={restored_epoch})"
        )

        # Verify restored time is close to pre-reboot time (saved within 10s of reboot)
        time_diff = abs(restored_epoch - pre_reboot_epoch)
        print(f"Time difference across reboot: {time_diff}s")
        assert time_diff <= 20, (
            f"Restored time differs too much from pre-reboot time: "
            f"pre={pre_reboot_epoch}, restored={restored_epoch}, diff={time_diff}s"
        )

        # Wait for fresh network IP to re-establish control
        fresh_ip = wait_for_ip(logs, step_start, "v4" if "." in ip else "v6")
        print(f"Reboot complete, fresh IP={fresh_ip}")

    # Verify /time on fresh boot reflects the restored and continuously progressing time
    resp_post = session.get(f"http://{fresh_ip}/time", timeout=10)
    assert resp_post.status_code == 200
    post_reboot_data = resp_post.json()
    post_reboot_epoch = post_reboot_data["epoch"]
    print(f"Post-reboot /time response: {post_reboot_data}")

    assert post_reboot_epoch >= restored_epoch, (
        f"Post-reboot epoch {post_reboot_epoch} went backwards compared to restored {restored_epoch}"
    )

    print("NVRAM time persistence across reboot verified successfully!")
