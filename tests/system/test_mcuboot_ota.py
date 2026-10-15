import subprocess
from pathlib import Path

import pytest
import requests
from ota_fixture import (get_mcuboot_test_versions, post_mcuboot,
                         reboot_device, semihosting_keepalive,
                         wait_for_bootloader_boot, wait_for_ip)


@pytest.fixture(scope="module")
def mcuboot_images():
    repo_root = Path(__file__).resolve().parents[2]
    prod_bin = repo_root / "build" / "mcuboot" / "zephyr" / "mcuboot.bin"
    test_bin = repo_root / "build" / "mcuboot_test" / "zephyr" / "mcuboot.bin"

    assert prod_bin.exists(), f"Production mcuboot.bin not found at {prod_bin}"

    versions = get_mcuboot_test_versions()

    # Ensure test mcuboot binary is up-to-date
    test_bin.parent.mkdir(parents=True, exist_ok=True)
    try:
        subprocess.run(["ninja", "-C", "build/mcuboot_test"], cwd=str(repo_root), check=True)
    except Exception:
        cmd = [
            "west",
            "build",
            "-b",
            "genie_idcm/88mw320/cpu0",
            "-d",
            "build/mcuboot_test",
            "bootloader/mcuboot/boot/zephyr",
            "--",
            f"-DBOARD_ROOT={repo_root}",
            f"-DSOC_ROOT={repo_root}",
            f"-DDTS_ROOT={repo_root}",
            f"-DZEPHYR_EXTRA_MODULES={repo_root}/modules/mw320_sdk;{repo_root}/mcuboot_module",
            f"-DEXTRA_CONF_FILE={repo_root}/sysbuild/mcuboot.conf",
            f"-DDTC_OVERLAY_FILE={repo_root}/sysbuild/mcuboot.overlay",
            f'-DCONFIG_BOOT_SIGNATURE_KEY_FILE="{repo_root}/bootloader/mcuboot/root-rsa-2048.pem"',
            "-DCONFIG_BOOT_DIRECT_XIP=y",
            "-DCONFIG_BOOT_DIRECT_XIP_REVERT=y",
            "-DCONFIG_BOOT_SIGNATURE_TYPE_RSA=y",
            f'-DEXTRA_CFLAGS="-DMCUBOOT_VERSION_STR=\\"{versions["b"]}\\""',
        ]
        subprocess.run(cmd, cwd=str(repo_root), check=True)

    assert test_bin.exists(), f"Test mcuboot.bin not found at {test_bin}"

    return {
        "prod_bin": prod_bin.read_bytes(),
        "test_bin": test_bin.read_bytes(),
        "versions": versions,
    }


def get_version(ip):
    resp = requests.get(f"http://{ip}/version", timeout=10)
    assert resp.status_code == 200, f"GET /version failed: {resp.status_code}"
    return resp.json()


def promote_mcuboot(ip):
    resp = requests.post(f"http://{ip}/promote/mcuboot", timeout=10)
    assert resp.status_code == 200, f"POST /promote/mcuboot failed: {resp.status_code}"
    assert resp.json().get("status") == "success"
    return resp.json()


def test_mcuboot_ota(hardware_device, openocd, mcuboot_images):
    """
    Test bootloader A/B upgrade lifecycle:
    1. Verify initial boot of production MCUboot (v2.4.0) in slot 0, confirmed.
    2. Attempt upgrade with invalid image payload -> expect HTTP 400 rejection.
    3. Upgrade to test MCUboot (v2.4.1) via POST /fwupgrade/mcuboot over IPv4 -> boots in test mode (confirmed: false).
    4. Reboot without promotion -> verify automatic rollback to production MCUboot (slot 0).
    5. Upgrade again to test MCUboot (v2.4.1), promote via POST /promote/mcuboot -> confirmed: true.
       Reboot device and verify it persists running test MCUboot (slot 1, confirmed: true).
    6. Restore production MCUboot (v2.4.0) over IPv6, promote, and verify clean return to slot 0.
    """
    device_ipv4 = hardware_device.get("ipv4")
    device_ipv6 = hardware_device.get("ipv6")
    logs = hardware_device["logs"]
    assert device_ipv4 and device_ipv6, (
        "This test exercises bootloader OTA over both address families, so "
        f"the device must be dual-stack (got v4={device_ipv4}, v6={device_ipv6})"
    )

    versions = mcuboot_images["versions"]
    prod_bin = mcuboot_images["prod_bin"]
    test_bin = mcuboot_images["test_bin"]

    print("\n[Step 1] Verifying initial boot with production MCUboot...")
    wait_for_bootloader_boot(
        logs,
        0,
        expected_bootloader_version=versions["base"],
        timeout=10,
    )
    v_info = get_version(device_ipv4)
    mb = v_info.get("mcuboot", {})
    assert (
        mb.get("version") == versions["base"]
    ), f"Expected {versions['base']}, got {mb.get('version')}"
    assert (
        mb.get("confirmed") is True
    ), f"Expected confirmed True, got {mb.get('confirmed')}"
    initial_slot = mb.get("slot", 0)
    target_slot = initial_slot ^ 1
    initial_gen = mb.get("gen_level", 1)
    print(
        f"Initial boot into MCUboot {versions['base']} verified: "
        f"slot={initial_slot}, gen_level={initial_gen}, confirmed={mb.get('confirmed')}"
    )

    # Step 2: Attempt upgrade with invalid image payload
    print("\n[Step 2] Testing rejection of invalid MCUboot image...")
    resp = requests.post(
        f"http://{device_ipv4}/fwupgrade/mcuboot",
        data=b"Invalid MCUboot payload without MRVL magic signature",
        headers={"Content-Type": "application/octet-stream"},
        timeout=10,
    )
    assert resp.status_code == 400, f"Expected 400 Bad Request, got {resp.status_code}"
    print("Invalid MCUboot payload rejected with HTTP 400 as expected.")

    # Step 3: Upgrade to test MCUboot over IPv4 (test boot, unconfirmed)
    print(
        f"\n[Step 3] Upgrading MCUboot to {versions['b']} (target slot {target_slot}) over IPv4 in test mode..."
    )
    step_start = len(logs)
    with semihosting_keepalive(openocd):
        resp = post_mcuboot(device_ipv4, test_bin)
        assert (
            resp.status_code == 200
        ), f"Upgrade failed with {resp.status_code}: {resp.text}"
        assert resp.json().get("status") == "success"
        print("Upload successful, waiting for device reboot into test MCUboot...")
        wait_for_bootloader_boot(
            logs,
            step_start,
            expected_bootloader_version=versions["b"],
            timeout=60,
        )

    ctrl_v4 = wait_for_ip(logs, step_start, "v4")
    ctrl_v6 = wait_for_ip(logs, step_start, "v6")
    v_info = get_version(ctrl_v4)
    mb = v_info.get("mcuboot", {})
    print(f"Booted test MCUboot: {mb}")
    assert (
        mb.get("version") == versions["b"]
    ), f"Expected {versions['b']}, got {mb.get('version')}"
    assert (
        mb.get("slot") == target_slot
    ), f"Expected slot {target_slot}, got {mb.get('slot')}"
    assert (
        mb.get("confirmed") is False
    ), f"Expected unconfirmed test boot, got confirmed={mb.get('confirmed')}"
    assert (
        mb.get("gen_level") == 0
    ), f"Expected gen_level 0 for unconfirmed test boot, got {mb.get('gen_level')}"

    # Step 4: Test rollback on unconfirmed reboot
    print("\n[Step 4] Testing rollback on unconfirmed reboot...")
    step_start = len(logs)
    with semihosting_keepalive(openocd):
        reboot_device(openocd, ctrl_v4, logs, step_start)
        print(
            "Reboot triggered, waiting for Boot2 to detect gen_level 0 and rollback..."
        )
        wait_for_bootloader_boot(
            logs,
            step_start,
            expected_bootloader_version=versions["base"],
            timeout=60,
        )

    ctrl_v4 = wait_for_ip(logs, step_start, "v4")
    ctrl_v6 = wait_for_ip(logs, step_start, "v6")
    v_info = get_version(ctrl_v4)
    mb = v_info.get("mcuboot", {})
    print(f"Rolled back to production MCUboot: {mb}")
    assert (
        mb.get("version") == versions["base"]
    ), f"Expected rollback to {versions['base']}, got {mb.get('version')}"
    assert (
        mb.get("slot") == initial_slot
    ), f"Expected rollback to slot {initial_slot}, got {mb.get('slot')}"
    assert (
        mb.get("confirmed") is True
    ), f"Expected confirmed True after rollback, got {mb.get('confirmed')}"

    # Step 5: Upgrade again to test MCUboot and promote
    print(f"\n[Step 5] Upgrading MCUboot to {versions['b']} and promoting...")
    step_start = len(logs)
    with semihosting_keepalive(openocd):
        resp = post_mcuboot(ctrl_v4, test_bin)
        assert resp.status_code == 200, f"Upgrade failed: {resp.text}"
        wait_for_bootloader_boot(
            logs,
            step_start,
            expected_bootloader_version=versions["b"],
            timeout=60,
        )

    ctrl_v4 = wait_for_ip(logs, step_start, "v4")
    ctrl_v6 = wait_for_ip(logs, step_start, "v6")
    v_info = get_version(ctrl_v4)
    mb = v_info.get("mcuboot", {})
    assert (
        mb.get("slot") == target_slot
    ), f"Expected slot {target_slot}, got {mb.get('slot')}"
    assert (
        mb.get("confirmed") is False
    ), f"Expected unconfirmed test boot, got {mb.get('confirmed')}"
    assert (
        mb.get("gen_level") == 0
    ), f"Expected gen_level 0 for unconfirmed test boot, got {mb.get('gen_level')}"

    print("Promoting test MCUboot via POST /promote/mcuboot...")
    promote_mcuboot(ctrl_v4)
    v_info = get_version(ctrl_v4)
    mb = v_info.get("mcuboot", {})
    assert (
        mb.get("confirmed") is True
    ), f"Expected confirmed True after promotion, got {mb.get('confirmed')}"
    assert (
        mb.get("gen_level") > 0
    ), f"Expected gen_level > 0 after promotion, got {mb.get('gen_level')}"
    print(f"Promotion confirmed: {mb}")

    print("Rebooting device to verify promoted MCUboot persists...")
    step_start = len(logs)
    with semihosting_keepalive(openocd):
        reboot_device(openocd, ctrl_v4, logs, step_start)
        wait_for_bootloader_boot(
            logs,
            step_start,
            expected_bootloader_version=versions["b"],
            timeout=60,
        )

    ctrl_v4 = wait_for_ip(logs, step_start, "v4")
    ctrl_v6 = wait_for_ip(logs, step_start, "v6")
    v_info = get_version(ctrl_v4)
    mb = v_info.get("mcuboot", {})
    assert (
        mb.get("version") == versions["b"]
    ), f"Expected {versions['b']}, got {mb.get('version')}"
    assert (
        mb.get("slot") == target_slot
    ), f"Expected slot {target_slot}, got {mb.get('slot')}"
    assert (
        mb.get("confirmed") is True
    ), f"Expected confirmed True after reboot, got {mb.get('confirmed')}"
    print("Promoted MCUboot persisted across reboot as expected.")

    # Step 6: Restore production MCUboot over IPv6 and promote it
    print(
        f"\n[Step 6] Restoring production MCUboot {versions['base']} over IPv6 and promoting..."
    )
    step_start = len(logs)
    with semihosting_keepalive(openocd):
        resp = post_mcuboot(ctrl_v6, prod_bin)
        assert resp.status_code == 200, f"Restore failed: {resp.text}"
        wait_for_bootloader_boot(
            logs,
            step_start,
            expected_bootloader_version=versions["base"],
            timeout=60,
        )

    ctrl_v4 = wait_for_ip(logs, step_start, "v4")
    ctrl_v6 = wait_for_ip(logs, step_start, "v6")
    promote_mcuboot(ctrl_v6)
    v_info = get_version(ctrl_v6)
    mb = v_info.get("mcuboot", {})
    assert (
        mb.get("version") == versions["base"]
    ), f"Expected {versions['base']}, got {mb.get('version')}"
    assert (
        mb.get("slot") == initial_slot
    ), f"Expected slot {initial_slot}, got {mb.get('slot')}"
    assert (
        mb.get("confirmed") is True
    ), f"Expected confirmed True, got {mb.get('confirmed')}"
    print(
        f"Booted and confirmed restored production MCUboot {versions['base']} in slot {initial_slot}."
    )

    print("Rebooting device to verify production MCUboot persists...")
    step_start = len(logs)
    with semihosting_keepalive(openocd):
        reboot_device(openocd, ctrl_v4, logs, step_start)
        wait_for_bootloader_boot(
            logs,
            step_start,
            expected_bootloader_version=versions["base"],
            timeout=60,
        )

    ctrl_v4 = wait_for_ip(logs, step_start, "v4")
    v_info = get_version(ctrl_v4)
    mb = v_info.get("mcuboot", {})
    assert (
        mb.get("version") == versions["base"]
    ), f"Expected {versions['base']}, got {mb.get('version')}"
    assert (
        mb.get("slot") == initial_slot
    ), f"Expected slot {initial_slot}, got {mb.get('slot')}"
    assert (
        mb.get("confirmed") is True
    ), f"Expected confirmed True, got {mb.get('confirmed')}"
    print("\nMCUboot A/B OTA upgrade and rollback system test passed completely!")
