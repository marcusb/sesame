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
    Verifies that the /logs HTTP endpoint returns text/plain log buffer snapshot.
    """
    req = urllib.request.Request("http://127.0.0.1:8080/logs")
    with urllib.request.urlopen(req, timeout=5) as response:
        assert response.status == 200
        assert "text/plain" in response.headers.get("Content-Type", "")
        body = response.read().decode("utf-8", errors="replace")
        assert len(body) > 0


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
    Verifies that the WebSocket server accepts connections on port 8081 (in native_sim),
    completes the RFC 6455 handshake, and streams log frames.
    """
    import base64
    import socket

    s = socket.create_connection(("127.0.0.1", 8081), timeout=5)
    key = base64.b64encode(b"0123456789abcdef").decode()
    handshake = (
        "GET /ws/logs HTTP/1.1\r\n"
        "Host: 127.0.0.1:8081\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    )
    s.sendall(handshake.encode("utf-8"))

    # Read handshake response
    resp = s.recv(1024).decode("utf-8", errors="ignore")
    assert "101 Switching Protocols" in resp
    assert "Sec-WebSocket-Accept:" in resp

    # Read backlog frame streamed immediately upon connect
    frame_data = s.recv(4096)
    assert len(frame_data) > 0
    # First byte should be WebSocket text frame opcode 0x81 (FIN + text)
    assert frame_data[0] == 0x81
    s.close()

