def test_hardware_ota(device_port):
    """
    Placeholder test for Type 3 System tests.
    This starts with a flashed mcuboot+sesame_test image.
    Uses NVS config injection to configure WiFi credentials,
    and runs a Matter commissioning followed by OTA upgrade sequence.
    """
    print(f"System Test: Orchestrating on port {device_port}")
