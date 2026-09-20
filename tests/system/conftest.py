from openocd_fixture import openocd  # noqa: F401
from hardware_fixture import hardware_device  # noqa: F401


import os

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--app-config",
        action="store",
        default="test_config.json",
        help="Path to JSON file containing the full AppConfig",
    )
    parser.addoption(
        "--device-port",
        action="store",
        default="/dev/ttyUSB0",
        help="Serial port for device",
    )


@pytest.fixture(scope="module")
def device_port(request):
    return request.config.getoption("--device-port")


@pytest.fixture(scope="module")
def test_app_config(request):
    from google.protobuf import json_format

    from proto import app_config_pb2

    config_path = request.config.getoption("--app-config")
    if not os.path.exists(config_path):
        pytest.skip(
            f"App config file {config_path} not found. Skipping hardware network tests."
        )

    with open(config_path, "r") as f:
        json_data = f.read()

    config = app_config_pb2.AppConfig()
    json_format.Parse(json_data, config, ignore_unknown_fields=True)
    return config
