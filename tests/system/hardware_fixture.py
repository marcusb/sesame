import os
import re
import threading
import time

import pytest


@pytest.fixture(scope="module")
def hardware_device(request, device_port, test_app_config, openocd):
    elf_paths = [
        "build/sesame_test/zephyr/zephyr.elf",
        "build/sesame/sesame_test/zephyr/zephyr.elf",
    ]
    elf_path = None
    for p in elf_paths:
        if os.path.exists(p):
            elf_path = p
            break

    if not elf_path:
        pytest.fail(
            "sesame_test zephyr.elf not found. Run: west build -b genie_idcm/88mw320/cpu0 -d build/sesame --sysbuild"
        )

    print("Connecting to hardware via OpenOCD semihost console...")

    import serial

    firmware_logs = []
    stop_reader = threading.Event()

    def log_reader(ser):
        try:
            while not stop_reader.is_set():
                line = ser.readline()
                if line:
                    try:
                        decoded = line.decode("utf-8", errors="ignore").strip()
                        if decoded:
                            firmware_logs.append(decoded)
                            print("LOG:", decoded, flush=True)
                    except Exception:
                        pass
        except Exception as e:
            print(f"Serial reader error: {e}")

    # Open the serial port first — before triggering the reset — so we capture
    # every byte from the moment the chip comes out of reset.
    # dsrdtr=False + explicit dtr/rts=False prevents pyserial's constructor from
    # toggling control lines, which would cause a spurious hardware reset.
    ser = serial.Serial(
        device_port,
        115200,
        timeout=0.1,
        dsrdtr=False,
        rtscts=False,
    )
    ser.dtr = False
    ser.rts = False

    reader_thread = threading.Thread(target=log_reader, args=(ser,), daemon=True)
    reader_thread.start()

    # Write NetworkConfig to file before reset so semihosting can read it
    # the instant the firmware starts executing.
    config_bytes = test_app_config.SerializeToString()
    with open("test_config.bin", "wb") as f:
        f.write(config_bytes)
    print("Wrote AppConfig to test_config.bin for semihosting")

    # Reset and keep the OpenOCD telnet connection alive so that it can service
    # the semihosting calls the firmware makes (reading test_config.bin).
    # Without a live telnet client the CPU halts at the semihosting BKPT and
    # never resumes.
    semihost_done = threading.Event()
    openocd.reboot_with_semihosting(
        semihost_done,
        on_halt=lambda: (ser.reset_input_buffer(), firmware_logs.clear()),
    )

    # Wait for Wi-Fi connection and IP
    device_ip = None
    ipv4_addr = None
    ipv6_addr = None
    second_stack_deadline = None
    start_time = time.time()
    while time.time() - start_time < 60:
        log_snapshot = list(firmware_logs)

        # Stop polling OpenOCD as soon as semihosting is confirmed complete —
        # keeping poll commands running during WiFi association or Matter
        # commissioning can briefly halt the CPU and disrupt timing-sensitive
        # code paths.
        if not semihost_done.is_set() and any(
            "Successfully decoded AppConfig" in line or "delaying boot" in line
            for line in log_snapshot
        ):
            semihost_done.set()
            print("Semihosting complete; stopping keep-alive poll.")

        for line in log_snapshot:
            m4 = re.search(r"IPv4 address: ([0-9\.]+)", line)
            if m4 and not ipv4_addr:
                ipv4_addr = m4.group(1)
            m6 = re.search(r"IPv6 address: ([0-9a-fA-F:]+)", line)
            if m6 and not ipv6_addr:
                ip6_str = m6.group(1)
                if not ip6_str.startswith("fe80"):
                    ipv6_addr = ip6_str  # bare; bracketed only when forming URLs

        mod_name = getattr(request.module, "__name__", "").lower()
        is_matter_test = "matter" in mod_name
        ipv6_bracketed = f"[{ipv6_addr}]" if ipv6_addr else None
        device_ip = (
            ipv6_bracketed
            if (is_matter_test and ipv6_bracketed)
            else (ipv4_addr or ipv6_bracketed)
        )
        if device_ip:
            if is_matter_test:
                ready = any(
                    "Commissioning window opened successfully" in l
                    for l in firmware_logs
                )
            else:
                ready = any("System ready" in l for l in firmware_logs)

            if ready:
                # Both stacks are expected; DHCPv4 is usually the slower stack to arrive.
                # Wait a bounded time for the second stack before proceeding.
                if ipv4_addr and ipv6_addr:
                    print(
                        f"Parsed IPs: v4={ipv4_addr} v6={ipv6_addr}."
                        f" Firmware logs so far:\n" + "\n".join(firmware_logs)
                    )
                    break
                if second_stack_deadline is None:
                    second_stack_deadline = time.time() + 20
                elif time.time() > second_stack_deadline:
                    print(
                        f"Proceeding with single stack: v4={ipv4_addr} v6={ipv6_addr}"
                    )
                    break

        # No fallback – if timeout expires the fixture will fail.

        time.sleep(0.1)
        if int((time.time() - start_time) * 10) % 10 == 0:
            print(f"Wait IP: {time.time()-start_time:.1f}s", flush=True)

    try:
        semihost_done.set()  # ensure keep-alive exits if we fell out of the loop
        assert device_ip, (
            f"Device failed to connect to Wi-Fi and acquire IP.\n"
            f"Logs:\n{chr(10).join(firmware_logs)}"
        )
        print(f"Device ready on Wi-Fi at IP {device_ip}")

        yield {
            "ip": device_ip,
            "ipv4": ipv4_addr,
            "ipv6": ipv6_addr,
            "logs": firmware_logs,
        }
    finally:
        stop_reader.set()
        reader_thread.join(timeout=2)
        ser.close()
