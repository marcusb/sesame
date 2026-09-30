import json
import urllib.request


def test_http_version(zephyr_app):
    """
    Verifies that the /version HTTP endpoint returns valid version info,
    running slot ("none" in native_sim), and image confirmation status.
    """
    req = urllib.request.Request("http://127.0.0.1:8080/version")
    with urllib.request.urlopen(req, timeout=5) as response:
        assert response.status == 200
        assert "application/json" in response.headers.get("Content-Type", "")
        data = json.loads(response.read().decode("utf-8"))

    assert "version" in data
    assert isinstance(data["version"], str) and len(data["version"]) > 0
    assert data["slot"] == "none"
    assert data["confirmed"] is False


def test_http_logs(zephyr_app):
    """
    Verifies that the /logs HTTP endpoint returns text/plain log buffer snapshot,
    starting from system boot with the Zephyr hello banner and valid timestamped lines.
    """
    req = urllib.request.Request("http://127.0.0.1:8080/logs")
    with urllib.request.urlopen(req, timeout=5) as response:
        assert response.status == 200
        assert "text/plain" in response.headers.get("Content-Type", "")
        body = response.read().decode("utf-8", errors="replace")

    lines = [line.strip() for line in body.splitlines() if line.strip()]
    assert len(lines) > 5, f"Expected multiple log lines in snapshot, got {len(lines)}"
    assert lines[0].startswith(
        "*** Booting Zephyr OS"
    ), f"First line should be Zephyr boot banner: {lines[0]}"
    assert any(
        line.startswith("[1970-01-01") for line in lines
    ), "Expected standard [YYYY-MM-DD timestamped log lines"


def test_http_logs_html(zephyr_app):
    """
    Verifies that the /logs.html endpoint serves the interactive web log viewer.
    """
    req = urllib.request.Request("http://127.0.0.1:8080/logs.html")
    with urllib.request.urlopen(req, timeout=5) as response:
        assert response.status == 200
        assert "text/html" in response.headers.get("Content-Type", "")
        body = response.read().decode("utf-8")
        assert "Sesame Logs" in body
        assert "ws/logs" in body


def test_ws_logs_streaming(zephyr_app):
    """
    Verifies that the WebSocket server accepts connections on /ws/logs,
    completes the RFC 6455 handshake, and streams well-formed initial log backlog.
    """
    import websocket

    ws = websocket.create_connection("ws://127.0.0.1:8080/ws/logs", timeout=5)
    try:
        backlog = ws.recv()
        assert (
            isinstance(backlog, str) and len(backlog) > 0
        ), "Expected non-empty backlog text message"
        lines = [line.strip() for line in backlog.splitlines() if line.strip()]
        assert (
            len(lines) > 5
        ), f"Expected multiple log lines in backlog, got {len(lines)}"
        assert lines[0].startswith(
            "*** Booting Zephyr OS"
        ), f"First backlog line should be Zephyr boot banner: {lines[0]}"
        assert any(
            line.startswith("[1970-01-01") for line in lines
        ), "Expected standard [YYYY-MM-DD timestamped log lines"
    finally:
        ws.close()
