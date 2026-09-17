import time

import requests


def test_http_door_endpoints(hardware_device):
    ip = hardware_device["ip"]
    logs = hardware_device["logs"]

    print(f"Testing POST http://{ip}/open")

    from requests.adapters import HTTPAdapter
    from urllib3.util.retry import Retry

    session = requests.Session()
    retry = Retry(connect=5, backoff_factor=0.5)
    adapter = HTTPAdapter(max_retries=retry)
    session.mount("http://", adapter)
    session.mount("https://", adapter)

    resp = session.post(f"http://{ip}/open", timeout=5)
    assert resp.status_code == 200

    time.sleep(0.5)
    assert any(
        "PIC: OPEN" in line for line in logs
    ), "Door open command not logged by firmware"

    print(f"Testing POST http://{ip}/close")
    resp = session.post(f"http://{ip}/close", timeout=5)
    assert resp.status_code == 200

    time.sleep(7.5)
    if not any("PIC: CLOSE" in line for line in logs):
        print("Last 20 log lines:")
        for line in logs[-20:]:
            print(line)
        assert False, "Door close command not logged by firmware"

    print("HTTP endpoints verified successfully!")
