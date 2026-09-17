import pytest
import subprocess
import time
import os
import socket
import tempfile

TELNET_HOST = "localhost"
TELNET_PORT = 4444


class OpenOCD:
    """Drives an OpenOCD instance over its telnet console.

    All target control goes through the telnet port. The GDB server (port
    3333) is never used: on this board attaching a GDB client drops the
    target, and OpenOCD tears itself down when that client disconnects.
    """

    def __init__(self, host=TELNET_HOST, telnet_port=TELNET_PORT):
        self.host = host
        self.telnet_port = telnet_port

    def run(self, cmd, wait=0.4):
        s = socket.create_connection((self.host, self.telnet_port), timeout=5)
        try:
            time.sleep(0.25)
            s.recv(4096)  # banner
            s.sendall(cmd.encode() + b"\r")
            time.sleep(wait)
            buf = b""
            s.settimeout(0.4)
            try:
                while True:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
            except socket.timeout:
                pass
            return buf.decode(errors="ignore")
        finally:
            s.close()

    def reboot(self):
        return self.run("reset run", wait=1.0)

    def halt(self):
        return self.run("halt", wait=2.0)

    def resume(self):
        return self.run("resume", wait=0.5)

    def target_up(self):
        self.halt()
        return "halted" in self.run("poll")


def _port_open(host, port, timeout=0.5):
    try:
        socket.create_connection((host, port), timeout=timeout).close()
        return True
    except OSError:
        return False


def _dump_log(path):
    if path and os.path.exists(path):
        with open(path, "r") as f:
            print("OpenOCD log:\n" + f.read())


def _remove(path):
    if path:
        try:
            os.remove(path)
        except OSError:
            pass


@pytest.fixture(scope="session")
def openocd():
    """Provide an OpenOCD with a halt-able JTAG target, controlled via telnet.

    Reuses an instance already listening on the telnet port; otherwise starts
    one and tears it down at the end of the session. Yields an OpenOCD client.
    """
    ocd = OpenOCD()
    proc = None
    log_path = None

    if _port_open(TELNET_HOST, TELNET_PORT):
        print(f"OpenOCD already running (telnet {TELNET_HOST}:{TELNET_PORT}); reusing.")
    else:
        print(f"Starting OpenOCD (telnet {TELNET_HOST}:{TELNET_PORT}) ...")
        log = tempfile.NamedTemporaryFile(mode="wb", suffix=".openocd.log", delete=False)
        log_path = log.name
        proc = subprocess.Popen(
            [
                "openocd",
                "-s", "tools/OpenOCD",
                "-f", "tools/OpenOCD/interface/ftdi.cfg",
                "-f", "tools/OpenOCD/openocd.cfg",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )

    ready = False
    deadline = time.time() + 20
    while time.time() < deadline:
        try:
            if ocd.target_up():
                ready = True
                break
        except OSError:
            pass
        if proc is not None and proc.poll() is not None:
            break
        time.sleep(0.5)

    if not ready:
        _dump_log(log_path)
        if proc is not None:
            proc.terminate()
        _remove(log_path)
        pytest.fail(
            "OpenOCD JTAG target not reachable on %s:%d "
            "(is the device connected to JTAG?)" % (TELNET_HOST, TELNET_PORT)
        )

    print("OpenOCD ready (JTAG target halt verified).")
    ocd.run("arm semihosting enable", wait=0.5)
    
    ocd.log_path = log_path

    try:
        yield ocd
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        _remove(log_path)
