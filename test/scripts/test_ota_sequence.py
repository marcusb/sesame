#!/usr/bin/env python3
import serial
import time
import subprocess
import sys
import threading
import http.server
import socketserver
import os
import socket

class RangeRequestHandler(http.server.SimpleHTTPRequestHandler):
    def send_head(self):
        if 'Range' not in self.headers:
            return super().send_head()
        
        try:
            # Very basic Range parsing: bytes=start-end
            range_str = self.headers['Range'].replace('bytes=', '')
            start_str, end_str = range_str.split('-')
            start = int(start_str) if start_str else 0
            
            path = self.translate_path(self.path)
            f = None
            try:
                f = open(path, 'rb')
            except OSError:
                self.send_error(http.HTTPStatus.NOT_FOUND, "File not found")
                return None
            
            fs = os.fstat(f.fileno())
            file_len = fs[6]
            end = int(end_str) if end_str else file_len - 1
            
            if start >= file_len:
                self.send_error(http.HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE, "Requested Range Not Satisfiable")
                f.close()
                return None
            
            self.send_response(http.HTTPStatus.PARTIAL_CONTENT)
            self.send_header("Content-type", self.guess_type(path))
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Range", f"bytes {start}-{end}/{file_len}")
            self.send_header("Content-Length", str(end - start + 1))
            self.send_header("Last-Modified", self.date_time_string(fs.st_mtime))
            self.end_headers()
            
            f.seek(start)
            # Monkey patch copyfile to only copy the requested range
            self.copyfile = lambda src, dst: dst.write(src.read(end - start + 1))
            return f
        except Exception as e:
            print("Range error:", e)
            return super().send_head()

def start_server(port):
    socketserver.TCPServer.allow_reuse_address = True
    httpd = socketserver.TCPServer(("", port), RangeRequestHandler)
    print(f"serving at port {port}")
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    return httpd

def get_local_ip(target_ip):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((target_ip, 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def read_until_match(ser, matches, timeout):
    import re
    start = time.time()
    buf = ""
    while time.time() - start < timeout:
        line = ser.readline()
        if line:
            s = line.decode(errors='ignore')
            buf += s
            print(s, end='', flush=True)
            for m in matches:
                if m in buf:
                    return m
    return None

def reset_board(ip=None):
    if ip:
        print(f"\n--- Restarting board via HTTP ({ip}) ---")
        subprocess.run(["curl", "-s", "-X", "POST", f"http://{ip}/restart"], check=False)
    else:
        print("\n--- Resetting board via OpenOCD ---")
        subprocess.run(["./tools/OpenOCD/flashprog.py", "-r"], check=False)

def read_until_match_or_ip(ser, timeout):
    import re
    start = time.time()
    buf = ""
    ip = None
    while time.time() - start < timeout:
        line = ser.readline()
        if line:
            s = line.decode(errors='ignore')
            buf += s
            print(s, end='', flush=True)
            match = re.search(r"IPv4 address: ([\d\.]+)", buf)
            if match:
                ip = match.group(1)
                return ip
    return None

def trigger_ota(ip, url):
    print(f"\n--- Triggering OTA from {url} ---")
    data = f'url: "{url}"'
    res = subprocess.run(
        f"echo '{data}' | protoc --encode=FirmwareUpgradeFetchRequest proto/api.proto | curl -v --data-binary @- -H content-type:application/protobuf 'http://{ip}/fwupgrade'",
        shell=True
    )
    print(f"curl exit code: {res.returncode}")
    time.sleep(2)
    
def promote_image(ip):
    print("\n--- Promoting Image ---")
    res = subprocess.run(
        f"curl -v -X POST 'http://{ip}/promote'",
        shell=True
    )
    print(f"curl exit code: {res.returncode}")
    time.sleep(2)

def test_flow():
    port = '/dev/ttyUSB0'
    ser = serial.Serial(port, 115200, timeout=1)

    httpd = start_server(8081)

    time.sleep(3)
    reset_board()

    print("\n--- Waiting for Boot A (0.2.1) ---")
    if not read_until_match(ser, ["Firmware version: 0.2.1"], 30):
        print("\nFailed to boot Image A")
        return False

    print("\n--- Waiting for IP Address ---")
    ip = read_until_match_or_ip(ser, 30)
    if not ip:
        print("\nFailed to get IP address")
        return False
    print(f"\nDevice IP: {ip}")

    local_ip = get_local_ip(ip)

    time.sleep(15)
    trigger_ota(ip, f"http://{local_ip}:8081/zephyr.signed.B.bin")
    
    print("\n--- Waiting for Boot B (0.2.2) ---")
    if not read_until_match(ser, ["Firmware version: 0.2.2"], 120):
        print("\nFailed to boot Image B")
        return False

    print("\n--- Verifying OTA LED blinks blue ---")
    if not read_until_match(ser, ["OTA test image running"], 15):
        print("\nFailed to see OTA LED pattern")
        return False

    print("\n--- Waiting for IP Address after B ---")
    ip = read_until_match_or_ip(ser, 30)
    if not ip:
        print("\nFailed to get IP address")
        return False
        
    time.sleep(15)
    promote_image(ip)
    
    time.sleep(2)
    time.sleep(3)
    reset_board(ip)
    ser.reset_input_buffer()
    
    print("\n--- Waiting for Boot B (0.2.2) again (persist after reboot) ---")
    if not read_until_match(ser, ["Firmware version: 0.2.2"], 30):
        print("\nFailed to keep Image B after reboot")
        return False
        
    print("\n--- Waiting for IP Address ---")
    ip = read_until_match_or_ip(ser, 30)
    
    time.sleep(15)
    trigger_ota(ip, f"http://{local_ip}:8081/zephyr.signed.C.bin")
    
    print("\n--- Waiting for Boot C (0.2.3) ---")
    if not read_until_match(ser, ["Firmware version: 0.2.3"], 120):
        print("\nFailed to boot Image C")
        return False

    print("\n--- Verifying OTA LED blinks blue for C ---")
    if not read_until_match(ser, ["OTA test image running"], 15):
        print("\nFailed to see OTA LED pattern")
        return False

    print("\n--- Waiting for IP Address before reset to avoid WiFi hang ---")
    ip = read_until_match_or_ip(ser, 30)

    time.sleep(2)
    print("\n--- Resetting WITHOUT promoting ---")
    time.sleep(3)
    reset_board(ip)
    
    print("\n--- Waiting for Boot B (0.2.2) (revert successful) ---")
    if not read_until_match(ser, ["Firmware version: 0.2.2"], 120):
        print("\nFailed to revert to Image B")
        return False
        
    print("\nSUCCESS! OTA and revert logic work perfectly.")
    return True

if __name__ == '__main__':
    test_flow()
