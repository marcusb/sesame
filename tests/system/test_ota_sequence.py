import pytest
import subprocess

def test_hardware_ota(port):
    """
    Placeholder test for Type 3 System tests.
    This starts with a factory-flashed MCUboot + firmware image.
    Uses black-box testing: connects to the AP, configures credentials,
    and runs a Matter OTA upgrade sequence.
    """
    print(f"System Test: Orchestrating on port {port}")
    # Provision via AP (HTTP)
    # Commission via Matter
    # Push OTA image
    # Verify success
    pass
