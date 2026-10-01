import json
import time

from pic_protocol import (
    DcmAlertCmd,
    DcmAudioCmd,
    DcmDoorCmd,
    DcmMsgType,
    DoorDirection,
    DoorOpenState,
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


def test_redundant_commands_ignored(zephyr_app, config, paho_client):
    """
    Verifies redundant commands are ignored:
    - When door is CLOSED, CLOSE (0) and STOP (2) commands are ignored.
    - When door is OPEN (100%), OPEN (1) and STOP (2) commands are ignored.
    """
    pic_sim = zephyr_app["pic_sim"]
    paho_client.subscribe("sesame/state")
    time.sleep(0.5)

    # 1. Door is initially CLOSED and STOPPED.
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "0")  # CLOSE
    time.sleep(0.5)
    assert not pic_sim.has_door_cmd(), "CLOSE when already CLOSED should be ignored"
    assert not pic_sim.has_alert_cmds(), "Alerts should not trigger when already CLOSED"

    paho_client.publish("sesame/cmd", "2")  # STOP
    time.sleep(0.5)
    assert not pic_sim.has_door_cmd(), "STOP when already stopped should be ignored"

    # 2. Transition door to fully OPEN.
    pic_sim.trigger_external_open()
    _wait_for_mqtt_state(
        zephyr_app["mqtt_server"],
        lambda d: d.get("contact") == "OPEN"
        and d.get("pos") == 100
        and d.get("dir") == "stopped",
        timeout=10.0,
    )

    # 3. Door is now OPEN and STOPPED.
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "1")  # OPEN
    time.sleep(0.5)
    assert not pic_sim.has_door_cmd(), "OPEN when already OPEN should be ignored"

    paho_client.publish("sesame/cmd", "2")  # STOP
    time.sleep(0.5)
    assert not pic_sim.has_door_cmd(), "STOP when already stopped should be ignored"


def test_in_flight_commands_and_reversals(zephyr_app, config, paho_client):
    """
    Verifies behavior when door is in motion or stopped partway:
    - Moving UP:
      - OPEN (1) is ignored.
      - CLOSE (0) immediately sends door cmd 0 (stops door, no alert delay).
      - STOP (2) sends door cmd 1 (toggle to stop).
    - Moving DOWN:
      - CLOSE (0) is ignored.
      - OPEN (1) sends door cmd 1 (reverses door).
      - STOP (2) sends door cmd 1 (toggle to stop).
    - Stopped midway:
      - OPEN (1) sends door cmd 1.
    """
    pic_sim = zephyr_app["pic_sim"]
    mqtt_server = zephyr_app["mqtt_server"]
    paho_client.subscribe("sesame/state")
    time.sleep(0.5)

    # A. Door moving UP
    pic_sim.send_status_update(
        state=DoorOpenState.OPEN,
        direction=DoorDirection.UP,
        pos=32800,
    )
    _wait_for_mqtt_state(mqtt_server, lambda d: d.get("dir") == "up", timeout=5.0)

    # OPEN when moving UP is ignored
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "1")
    time.sleep(0.5)
    assert not pic_sim.has_door_cmd(), "OPEN when moving UP should be ignored"

    # CLOSE when moving UP immediately sends door cmd 0 (stops door)
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "0")
    close_cmd = pic_sim.wait_for_door_cmd(val=0, timeout=3.0)
    assert (
        close_cmd is not None
    ), "CLOSE when moving UP should immediately send door cmd 0"
    assert (
        not pic_sim.has_alert_cmds()
    ), "CLOSE when moving UP should not trigger alert sequence"

    # STOP when moving UP sends door cmd 1 (toggle to stop)
    pic_sim.send_status_update(
        state=DoorOpenState.OPEN,
        direction=DoorDirection.UP,
        pos=32820,
    )
    _wait_for_mqtt_state(mqtt_server, lambda d: d.get("dir") == "up", timeout=5.0)
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "2")
    stop_cmd = pic_sim.wait_for_door_cmd(val=1, timeout=3.0)
    assert stop_cmd is not None, "STOP when moving UP should send door cmd 1"

    # B. Door moving DOWN
    pic_sim.send_status_update(
        state=DoorOpenState.OPEN,
        direction=DoorDirection.DOWN,
        pos=32850,
    )
    _wait_for_mqtt_state(mqtt_server, lambda d: d.get("dir") == "down", timeout=5.0)

    # CLOSE when moving DOWN is ignored
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "0")
    time.sleep(0.5)
    assert not pic_sim.has_door_cmd(), "CLOSE when moving DOWN should be ignored"

    # OPEN when moving DOWN sends door cmd 1 (reverses door)
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "1")
    open_cmd = pic_sim.wait_for_door_cmd(val=1, timeout=3.0)
    assert open_cmd is not None, "OPEN when moving DOWN should send door cmd 1"

    # STOP when moving DOWN sends door cmd 1 (toggle to stop)
    pic_sim.send_status_update(
        state=DoorOpenState.OPEN,
        direction=DoorDirection.DOWN,
        pos=32840,
    )
    _wait_for_mqtt_state(mqtt_server, lambda d: d.get("dir") == "down", timeout=5.0)
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "2")
    stop_cmd2 = pic_sim.wait_for_door_cmd(val=1, timeout=3.0)
    assert stop_cmd2 is not None, "STOP when moving DOWN should send door cmd 1"

    # C. Stopped midway (e.g. 50%)
    pic_sim.send_status_update(
        state=DoorOpenState.OPEN,
        direction=DoorDirection.STOPPED,
        pos=32828,
    )
    _wait_for_mqtt_state(
        mqtt_server,
        lambda d: d.get("pos") == 50 and d.get("dir") == "stopped",
        timeout=5.0,
    )
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "1")
    open_cmd2 = pic_sim.wait_for_door_cmd(val=1, timeout=3.0)
    assert open_cmd2 is not None, "OPEN when stopped midway should send door cmd 1"


def _wait_for_firmware_log(zephyr_app, text, start_index=0, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        logs = zephyr_app["logs"]
        for i in range(start_index, len(logs)):
            if text in logs[i]:
                return i
        time.sleep(0.05)
    raise TimeoutError(
        f"Timed out waiting for log containing '{text}' after index {start_index}"
    )


def test_scheduled_close_cancellation(zephyr_app, config, paho_client):
    """
    Verifies that a scheduled close (alert period) is cancelled by:
    - An incoming OPEN command.
    - An incoming STOP command.
    - External door movement reported by PIC (direction change, position change, or CLOSED state).
    And verifies that no CLOSE command is transmitted after cancellation.
    """
    pic_sim = zephyr_app["pic_sim"]
    mqtt_server = zephyr_app["mqtt_server"]
    paho_client.subscribe("sesame/state")
    time.sleep(0.5)

    def _ensure_open_stopped():
        pic_sim.send_status_update(
            state=DoorOpenState.OPEN,
            direction=DoorDirection.STOPPED,
            pos=pic_sim.up_limit,
        )
        time.sleep(0.1)

    # 1. Cancel scheduled close on OPEN command
    _ensure_open_stopped()
    log_idx = len(zephyr_app["logs"])
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "0")  # CLOSE
    assert pic_sim.wait_for_frame(DcmMsgType.ALERT_CMD, timeout=8.0) is not None

    time.sleep(0.1)
    paho_client.publish("sesame/cmd", "1")  # OPEN cancels close
    _wait_for_firmware_log(
        zephyr_app,
        "Cancelling scheduled close: OPEN command received",
        start_index=log_idx,
    )

    # 2. Cancel scheduled close on STOP command
    _ensure_open_stopped()
    log_idx = len(zephyr_app["logs"])
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "0")  # CLOSE
    assert pic_sim.wait_for_frame(DcmMsgType.ALERT_CMD, timeout=8.0) is not None

    time.sleep(0.1)
    paho_client.publish("sesame/cmd", "2")  # STOP cancels close
    _wait_for_firmware_log(
        zephyr_app,
        "Cancelling scheduled close: STOP command received",
        start_index=log_idx,
    )

    # 3. Cancel scheduled close on direction change (external motion)
    _ensure_open_stopped()
    log_idx = len(zephyr_app["logs"])
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "0")  # CLOSE
    assert pic_sim.wait_for_frame(DcmMsgType.ALERT_CMD, timeout=8.0) is not None

    time.sleep(0.1)
    pic_sim.send_status_update(direction=DoorDirection.DOWN)
    _wait_for_firmware_log(
        zephyr_app,
        "Cancelling scheduled close: door direction not stopped",
        start_index=log_idx,
    )

    # 4. Cancel scheduled close on position change
    _ensure_open_stopped()
    log_idx = len(zephyr_app["logs"])
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "0")  # CLOSE
    assert pic_sim.wait_for_frame(DcmMsgType.ALERT_CMD, timeout=8.0) is not None

    time.sleep(0.1)
    pic_sim.send_status_update(pos=pic_sim.up_limit - 10)
    _wait_for_firmware_log(
        zephyr_app,
        "Cancelling scheduled close: door position changed",
        start_index=log_idx,
    )

    # 5. Cancel scheduled close on state becomes CLOSED
    _ensure_open_stopped()
    log_idx = len(zephyr_app["logs"])
    pic_sim.clear_frames()
    paho_client.publish("sesame/cmd", "0")  # CLOSE
    assert pic_sim.wait_for_frame(DcmMsgType.ALERT_CMD, timeout=8.0) is not None

    time.sleep(0.1)
    pic_sim.send_status_update(state=DoorOpenState.CLOSED)
    _wait_for_firmware_log(
        zephyr_app,
        "Cancelling scheduled close: door reached closed",
        start_index=log_idx,
    )

    # Finally, wait past the 6s alert timer to verify that the timer was cancelled
    # and no CLOSE command (0) was transmitted.
    pic_sim.clear_frames()
    time.sleep(6.5)
    assert not pic_sim.has_door_cmd(
        val=0
    ), "No CLOSE command should fire after cancellation"
