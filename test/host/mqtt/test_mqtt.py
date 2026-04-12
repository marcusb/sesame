"""
Tests for src/mqtt.c running on host.

Launches an embedded MQTT 3.1.1 broker (pure Python, no external dependency),
then tells the SUT to connect via a `start` harness command.  The SUT reaches
the broker via the libslirp gateway at 10.0.2.2.  We then assert
LWT/state/cmd topics behave as expected.
"""

import json
import os
import time

import pytest

paho = pytest.importorskip("paho.mqtt.client")

from broker import EmbeddedBroker


@pytest.fixture()
def broker():
    b = EmbeddedBroker()
    b.start()
    yield b
    b.stop()


def _subscriber(broker_port, topics):
    events = []
    client = paho.Client(paho.CallbackAPIVersion.VERSION2, client_id=f"test-{os.getpid()}")

    def on_connect(cli, userdata, flags, reason_code, properties):
        for t in topics:
            cli.subscribe(t)

    def on_message(cli, userdata, msg):
        events.append((msg.topic, msg.payload, bool(msg.retain)))

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("127.0.0.1", broker_port, keepalive=10)
    client.loop_start()
    return client, events


def test_connect_publishes_lwt_online(mqtt_harness, broker):
    client, events = _subscriber(broker.port, ["sesame/#"])
    try:
        mqtt_harness.send_cmd(f"start 10.0.2.2 {broker.port} sesame")
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not any(
            t == "sesame/availability" for t, _, _ in events
        ):
            time.sleep(0.1)
        assert any(
            t == "sesame/availability" and p == b"Online"
            for t, p, _ in events
        ), events
    finally:
        client.loop_stop()
        client.disconnect()


def test_cmd_topic_publishes_door_control(mqtt_harness, broker):
    client, events = _subscriber(broker.port, ["sesame/availability"])
    try:
        mqtt_harness.send_cmd(f"start 10.0.2.2 {broker.port} sesame")
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not events:
            time.sleep(0.1)

        pub = paho.Client(paho.CallbackAPIVersion.VERSION2)
        pub.connect("127.0.0.1", broker.port, keepalive=5)
        pub.publish("sesame/cmd", payload=b"1", qos=0).wait_for_publish()
        pub.disconnect()

        evt = mqtt_harness.wait_event(timeout=10)
        assert "DOOR_CONTROL" in evt and "cmd=1" in evt
    finally:
        client.loop_stop()
        client.disconnect()


def test_publish_state_emits_json(mqtt_harness, broker):
    client, events = _subscriber(broker.port, ["sesame/state"])
    try:
        mqtt_harness.send_cmd(f"start 10.0.2.2 {broker.port} sesame")
        time.sleep(1.5)
        # state=OPEN(1), dir=UP(1), pos=42
        mqtt_harness.send_cmd("pubstate 1 1 42")

        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not events:
            time.sleep(0.1)
        assert events, "no state publish received"
        topic, payload, _ = events[0]
        assert topic == "sesame/state"
        data = json.loads(payload.decode())
        assert data["contact"] == "OPEN"
        assert data["dir"] == "up"
        assert data["pos"] == 42
    finally:
        client.loop_stop()
        client.disconnect()
