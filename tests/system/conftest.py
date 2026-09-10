import pytest
import json
import os

def pytest_addoption(parser):
    parser.addoption("--wifi-config", action="store", default="wifi.json",
                     help="Path to JSON file containing Wi-Fi credentials")
    parser.addoption("--device-port", action="store", default="/dev/ttyUSB0",
                     help="Serial port for device")

@pytest.fixture(scope="session")
def wifi_credentials(request):
    config_path = request.config.getoption("--wifi-config")
    if not os.path.exists(config_path):
        pytest.skip(f"Wi-Fi config file {config_path} not found. Skipping hardware network tests.")
    with open(config_path, "r") as f:
        return json.load(f)

@pytest.fixture
def test_network_config(wifi_credentials):
    from proto import app_config_pb2
    config = app_config_pb2.NetworkConfig()
    config.hostname = "sesame-test"
    config.ssid = wifi_credentials.get("ssid", "TEST_SSID")
    config.password = wifi_credentials.get("password", "TEST_PASS")
    
    # 2 is WPA2 in our enum (or WPA2_PSK depending on definition)
    # The user protobuf probably defines SecurityType
    security = wifi_credentials.get("security", 2)
    config.security = security
    return config
