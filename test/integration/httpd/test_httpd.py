"""
Tests for src/httpd.c running on host.

The SUT listens on port 80 inside the FreeRTOS stack; libslirp forwards
127.0.0.1:<host_port> to guest:80. We POST real HTTP requests and watch for
ctrl_queue events via the inspector UDP socket.
"""

import pytest
import requests


def _post(harness, path, data=b"", content_type=None, timeout=5):
    headers = {}
    if content_type:
        headers["content-type"] = content_type
    return requests.post(
        f"http://{harness.host}:{harness.port}{path}",
        data=data,
        headers=headers,
        timeout=timeout,
    )


def test_open_triggers_door_control_open(httpd_harness):
    r = _post(httpd_harness, "/open")
    assert r.status_code == 200
    evt = httpd_harness.wait_event()
    assert evt.startswith("CTRL DOOR_CONTROL")
    # DOOR_CMD_OPEN = 1
    assert "cmd=1" in evt


def test_close_triggers_door_control_close(httpd_harness):
    r = _post(httpd_harness, "/close")
    assert r.status_code == 200
    evt = httpd_harness.wait_event()
    assert "DOOR_CONTROL" in evt and "cmd=2" in evt  # DOOR_CMD_CLOSE


def test_unknown_post_returns_404(httpd_harness):
    r = _post(httpd_harness, "/nope")
    assert r.status_code == 404


def test_get_returns_404(httpd_harness):
    r = requests.get(f"http://{httpd_harness.host}:{httpd_harness.port}/open", timeout=5)
    assert r.status_code == 404


def test_cfg_mqtt_decodes_protobuf(httpd_harness):
    from app_config_pb2 import MqttConfig

    body = MqttConfig(
        enabled=True,
        broker_host="broker.example",
        broker_port=8883,
        prefix="sesame",
    ).SerializeToString()
    r = _post(httpd_harness, "/cfg/mqtt", data=body, content_type="application/protobuf")
    assert r.status_code == 200
    evt = httpd_harness.wait_event()
    assert "MQTT_CONFIG" in evt
    assert "enabled=1" in evt
    assert "host=broker.example" in evt
    assert "port=8883" in evt
    assert "prefix=sesame" in evt


def test_cfg_network_decodes_protobuf(httpd_harness):
    from app_config_pb2 import NetworkConfig

    body = NetworkConfig(
        hostname="sesame",
        ssid="MyWifi",
        security=2,  # WIFI_SECURITY_WPA
    ).SerializeToString()
    r = _post(httpd_harness, "/cfg/network", data=body, content_type="application/protobuf")
    assert r.status_code == 200
    evt = httpd_harness.wait_event()
    assert "WIFI_CONFIG" in evt
    assert "hostname=sesame" in evt
    assert "ssid=MyWifi" in evt
    assert "security=2" in evt


def test_cfg_logging_decodes_protobuf(httpd_harness):
    from app_config_pb2 import LoggingConfig, SyslogConfig

    body = LoggingConfig(
        syslog_config=SyslogConfig(
            enabled=True,
            syslog_host="syslog.example",
            syslog_port=514,
        )
    ).SerializeToString()
    r = _post(httpd_harness, "/cfg/logging", data=body, content_type="application/protobuf")
    assert r.status_code == 200
    evt = httpd_harness.wait_event()
    assert "LOGGING_CONFIG" in evt
    assert "syslog_enabled=1" in evt
    assert "host=syslog.example" in evt
    assert "port=514" in evt


def test_promote_triggers_ota_promote(httpd_harness):
    r = _post(httpd_harness, "/promote")
    assert r.status_code == 200
    evt = httpd_harness.wait_event()
    assert "OTA_PROMOTE" in evt


def test_fwupgrade_decodes_protobuf(httpd_harness):
    from api_pb2 import FirmwareUpgradeFetchRequest

    body = FirmwareUpgradeFetchRequest(
        url="http://192.168.1.1/fw.bin"
    ).SerializeToString()
    r = _post(httpd_harness, "/fwupgrade", data=body, content_type="application/protobuf")
    assert r.status_code == 200
    evt = httpd_harness.wait_event()
    assert "OTA_UPGRADE" in evt


def test_cfg_mqtt_bad_body_returns_400(httpd_harness):
    # \xff is a truncated varint — nanopb decode fails → 400
    r = _post(httpd_harness, "/cfg/mqtt", data=b"\xff", content_type="application/protobuf")
    assert r.status_code == 400


def test_restart_calls_reboot(httpd_harness):
    r = _post(httpd_harness, "/restart")
    assert r.status_code == 200
    # reboot() stub calls _exit(0) after a 3-second vTaskDelay; allow up to 10s
    assert httpd_harness.proc.wait(timeout=10.0) == 0
