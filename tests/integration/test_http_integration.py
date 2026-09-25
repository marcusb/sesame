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
