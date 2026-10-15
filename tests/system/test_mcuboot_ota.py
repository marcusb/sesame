from pathlib import Path
import subprocess
import pytest
import requests

from ota_fixture import (
    get_mcuboot_test_versions,
    post_mcuboot,
    semihosting_keepalive,
    wait_for_bootloader_boot,
    wait_for_ip,
)


@pytest.fixture(scope="module")
def mcuboot_images():
    repo_root = Path(__file__).resolve().parents[2]
    prod_bin = repo_root / "build" / "mcuboot" / "zephyr" / "mcuboot.bin"
    test_bin = repo_root / "build" / "mcuboot_test" / "zephyr" / "mcuboot.bin"

    assert prod_bin.exists(), f"Production mcuboot.bin not found at {prod_bin}"

    versions = get_mcuboot_test_versions()

    if not test_bin.exists():
        # Build test mcuboot binary if not already present
        test_bin.parent.mkdir(parents=True, exist_ok=True)
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


def test_mcuboot_ota(hardware_device, openocd, mcuboot_images):
    """
    Test bootloader in-place upgrade:
    1. Verify initial boot of production MCUboot (e.g. v2.4.0).
    2. Attempt upgrade with invalid image payload -> expect HTTP 400 rejection.
    3. Upgrade to test MCUboot (e.g. v2.4.1) via POST /fwupgrade/mcuboot over IPv4.
    4. Verify device reboots and runs updated MCUboot (v2.4.1).
    5. Restore production MCUboot (v2.4.0) via POST /fwupgrade/mcuboot over IPv6.
    6. Verify device reboots and runs production MCUboot (v2.4.0).
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
    print(f"Initial boot into MCUboot {versions['base']} verified.")

    # Step 2: Attempt upgrade with invalid image payload
    print("\n[Step 2] Testing rejection of invalid MCUboot image...")
    resp = requests.post(
        f"http://{device_ipv4}/fwupgrade/mcuboot",
        data=b"Invalid MCUboot payload without MRVL magic signature",
        headers={"Content-Type": "application/octet-stream"},
        timeout=10,
    )
    assert (
        resp.status_code == 400
    ), f"Expected 400 Bad Request, got {resp.status_code}"
    print("Invalid MCUboot payload rejected with HTTP 400 as expected.")

    # Step 3: Upgrade to test MCUboot over IPv4
    print(f"\n[Step 3] Upgrading MCUboot to {versions['b']} over IPv4...")
    step_start = len(logs)
    with semihosting_keepalive(openocd):
        resp = post_mcuboot(device_ipv4, test_bin)
        assert (
            resp.status_code == 200
        ), f"Upgrade failed with {resp.status_code}: {resp.text}"
        assert resp.json().get("status") == "success"
        print(
            "Upload successful, waiting for device reboot into updated MCUboot..."
        )
        wait_for_bootloader_boot(
            logs,
            step_start,
            expected_bootloader_version=versions["b"],
            timeout=60,
        )

    ctrl_v4 = wait_for_ip(logs, step_start, "v4")
    ctrl_v6 = wait_for_ip(logs, step_start, "v6")
    print(
        f"Booted updated MCUboot {versions['b']}; fresh v4={ctrl_v4}, v6={ctrl_v6}"
    )

    # Step 4: Restore production MCUboot over IPv6
    print(
        f"\n[Step 4] Restoring production MCUboot {versions['base']} over IPv6..."
    )
    step_start = len(logs)
    with semihosting_keepalive(openocd):
        resp = post_mcuboot(ctrl_v6, prod_bin)
        assert (
            resp.status_code == 200
        ), f"Restore failed with {resp.status_code}: {resp.text}"
        assert resp.json().get("status") == "success"
        print(
            "Upload successful, waiting for device reboot into production MCUboot..."
        )
        wait_for_bootloader_boot(
            logs,
            step_start,
            expected_bootloader_version=versions["base"],
            timeout=60,
        )

    ctrl_v4 = wait_for_ip(logs, step_start, "v4")
    print(
        f"Booted restored production MCUboot {versions['base']}; fresh IPv4={ctrl_v4}"
    )
    print("\nMCUboot OTA upgrade system test passed completely!")
