import json
import time

from pic_protocol import (
    DcmAlertCmd,
    DcmAudioCmd,
    DcmDoorCmd,
    DcmMsgType,
)


def _wait_for_mqtt_state(mqtt_server, predicate, timeout=15.0):
    """Waits for a message on sesame/state matching predicate."""

    def _check(payload_bytes: bytes) -> bool:
        try:
            data = json.loads(payload_bytes.decode("utf-8"))
            return predicate(data)
        except Exception:
            return False

    raw = mqtt_server.wait_for_message(
        "sesame/state", timeout=timeout, predicate=_check
    )
    return json.loads(raw.decode("utf-8"))


def test_sesame_initiated_open_and_close(zephyr_app, config, paho_client):
    """
    Verifies Sesame-initiated open and close commands submitted via MQTT:
    1. Submits OPEN ("1") to sesame/cmd via MQTT.
    2. Verifies PIC receives DcmDoorCmd(val=1, OPEN).
    3. PIC simulates opening movement and sends status updates.
    4. Verifies MQTT receives state: contact=OPEN, pos=100, dir=stopped.
    5. Submits CLOSE ("0") to sesame/cmd via MQTT.
    6. Verifies PIC receives alert sequence (DcmAudioCmd val=2 and DcmAlertCmd val=1).
    7. Verifies PIC receives DcmDoorCmd(val=0, CLOSE) after the 6-second alert duration.
    8. PIC simulates closing movement and sends status updates.
    9. Verifies MQTT receives state: contact=CLOSED, pos=0, dir=stopped.
    """
    pic_sim = zephyr_app["pic_sim"]
    mqtt_server = zephyr_app["mqtt_server"]

    # Subscribe paho_client to topics
    paho_client.subscribe("sesame/state")
    paho_client.subscribe("sesame/availability")
    time.sleep(0.5)

    # 1. Submit OPEN command via MQTT
    print("\n--- Submitting OPEN command via MQTT ---")
    paho_client.publish("sesame/cmd", "1")

    # 2. Verify PIC simulator receives DCM_MSG_DOOR_CMD with val=1 (OPEN)
    door_cmd_frame = pic_sim.wait_for_door_cmd(val=1, timeout=8.0)
    assert isinstance(door_cmd_frame.payload, DcmDoorCmd)
    assert door_cmd_frame.payload.val == 1, "Expected OPEN command (val=1)"

    # 3. Simulate door opening motion
    print("Simulating door opening motion...")
    pic_sim.simulate_open(steps=5, step_delay=0.1)

    # 4. Verify MQTT state update for fully open door
    state_open = _wait_for_mqtt_state(
        mqtt_server,
        lambda d: d.get("contact") == "OPEN"
        and d.get("pos") == 100
        and d.get("dir") == "stopped",
        timeout=10.0,
    )
    print("Received MQTT state after open:", state_open)
    assert state_open["contact"] == "OPEN"
    assert state_open["pos"] == 100
    assert state_open["dir"] == "stopped"

    # Clear previously recorded frames before testing close
    pic_sim.received_frames.clear()

    # 5. Submit CLOSE command via MQTT
    print("\n--- Submitting CLOSE command via MQTT ---")
    paho_client.publish("sesame/cmd", "0")

    # 6. Verify PIC simulator receives alert audio and alert light commands
    audio_frame = pic_sim.wait_for_frame(
        DcmMsgType.AUDIO_CMD,
        timeout=5.0,
        predicate=lambda f: isinstance(f.payload, DcmAudioCmd) and f.payload.val == 2,
    )
    assert audio_frame is not None
    assert audio_frame.payload.val == 2, "Expected alert audio command val=2"

    alert_frame = pic_sim.wait_for_frame(
        DcmMsgType.ALERT_CMD,
        timeout=5.0,
        predicate=lambda f: isinstance(f.payload, DcmAlertCmd) and f.payload.val == 1,
    )
    assert alert_frame is not None
    assert alert_frame.payload.val == 1, "Expected alert light command val=1"

    # 7. Verify PIC simulator receives DcmDoorCmd(val=0, CLOSE) after the 6-second alert window
    print("Waiting for CLOSE command following 6-second alert period...")
    close_cmd_frame = pic_sim.wait_for_door_cmd(val=0, timeout=15.0)
    assert isinstance(close_cmd_frame.payload, DcmDoorCmd)
    assert close_cmd_frame.payload.val == 0, "Expected CLOSE command (val=0)"

    # 8. Simulate door closing motion
    print("Simulating door closing motion...")
    pic_sim.simulate_close(steps=5, step_delay=0.1)

    # 9. Verify MQTT state update for fully closed door
    state_closed = _wait_for_mqtt_state(
        mqtt_server,
        lambda d: d.get("contact") == "CLOSED"
        and d.get("pos") == 0
        and d.get("dir") == "stopped",
        timeout=10.0,
    )
    print("Received MQTT state after close:", state_closed)
    assert state_closed["contact"] == "CLOSED"
    assert state_closed["pos"] == 0
    assert state_closed["dir"] == "stopped"


def test_external_initiated_open_and_close(zephyr_app, config, paho_client):
    """
    Verifies externally initiated door movement (e.g. wall button or RF remote):
    1. PIC simulator sends unsolicited status updates for door opening.
    2. Verifies Sesame decodes updates and publishes to MQTT (contact=OPEN, pos=100).
    3. PIC simulator sends unsolicited status updates for door closing.
    4. Verifies Sesame decodes updates and publishes to MQTT (contact=CLOSED, pos=0).
    """
    pic_sim = zephyr_app["pic_sim"]
    mqtt_server = zephyr_app["mqtt_server"]

    # Subscribe paho_client to topics
    paho_client.subscribe("sesame/state")
    time.sleep(0.5)

    # 1. Trigger external open (unsolicited status updates from PIC)
    print("\n--- Triggering external OPEN (e.g. wall button) ---")
    pic_sim.trigger_external_open()

    # 2. Verify MQTT state update reflects open door
    state_open = _wait_for_mqtt_state(
        mqtt_server,
        lambda d: d.get("contact") == "OPEN"
        and d.get("pos") == 100
        and d.get("dir") == "stopped",
        timeout=10.0,
    )
    print("Received MQTT state for external open:", state_open)
    assert state_open["contact"] == "OPEN"
    assert state_open["pos"] == 100
    assert state_open["dir"] == "stopped"

    # 3. Trigger external close (unsolicited status updates from PIC)
    print("\n--- Triggering external CLOSE (e.g. wall button) ---")
    pic_sim.trigger_external_close()

    # 4. Verify MQTT state update reflects closed door
    state_closed = _wait_for_mqtt_state(
        mqtt_server,
        lambda d: d.get("contact") == "CLOSED"
        and d.get("pos") == 0
        and d.get("dir") == "stopped",
        timeout=10.0,
    )
    print("Received MQTT state for external close:", state_closed)
    assert state_closed["contact"] == "CLOSED"
    assert state_closed["pos"] == 0
    assert state_closed["dir"] == "stopped"
