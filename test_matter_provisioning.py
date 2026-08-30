#!/usr/bin/env python3
import pexpect
import time
import sys
import subprocess

def test_provisioning():
    print("Starting QEMU...")
    qemu_cmd = "ninja -C build/qemu-build test/integration/sesame_tests-qemu.axf && qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -nographic -netdev user,id=net0 -device lan9220,netdev=net0 -kernel build/qemu-build/test/integration/sesame_tests-qemu.axf"
    # Actually wait, I should run the main zephyr.elf in QEMU?
    pass
