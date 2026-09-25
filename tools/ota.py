#!/usr/bin/env python3
"""OTA firmware upgrade tool for sesame devices.

Usage:
    uv run tools/ota.py upgrade HOSTNAME [--image FILE] [--promote]
    uv run tools/ota.py promote HOSTNAME
"""

import argparse
import http.server
import json
import re
import socket
import struct
import subprocess
import sys
import threading
import time

import requests


def device_url(hostname, path):
    """Format an HTTP URL for a device hostname or IPv4/IPv6 address."""
    clean = hostname.strip("[]")
    if ":" in clean:
        return f"http://[{clean}]{path}"
    return f"http://{clean}{path}"


def get_local_ip(target_host=None):
    """Get the local IP address used to reach the target host (or default)."""
    if target_host:
        try:
            clean_host = target_host.strip("[]")
            addrinfo = socket.getaddrinfo(
                clean_host, 80, socket.AF_UNSPEC, socket.SOCK_DGRAM
            )
            family, socktype, proto, canonname, sockaddr = addrinfo[0]
            s = socket.socket(family, socket.SOCK_DGRAM)
            try:
                s.connect(sockaddr)
                ip = s.getsockname()[0]
                if family == socket.AF_INET6:
                    return f"[{ip}]"
                return ip
            finally:
                s.close()
        except Exception:
            pass

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()


def read_image_version(path):
    """Read the MCUboot image version from the image header.

    MCUboot image header (IMAGE_HEADER_SIZE = 32 bytes):
      offset 0:  uint32 ih_magic   (0x96f3b83d)
      offset 4:  uint32 ih_load_addr
      offset 8:  uint16 ih_hdr_size
      offset 10: uint16 ih_protect_tlv_size
      offset 12: uint32 ih_img_size
      offset 16: uint32 ih_flags
      offset 20: uint8  iv_major
      offset 21: uint8  iv_minor
      offset 22: uint16 iv_revision
      offset 24: uint32 iv_build_num
    """
    with open(path, "rb") as f:
        hdr = f.read(32)

    if len(hdr) < 32:
        raise ValueError(f"Image too small: {len(hdr)} bytes")

    magic = struct.unpack_from("<I", hdr, 0)[0]
    if magic != 0x96F3B83D:
        raise ValueError(f"Bad MCUboot magic: 0x{magic:08x}")

    major, minor, revision, build_num = struct.unpack_from("<BBHI", hdr, 20)
    return f"{major}.{minor}.{revision}+{build_num}"


def get_device_version(hostname):
    """Query the device /version endpoint. Returns dict or None."""
    url = device_url(hostname, "/version")
    try:
        r = requests.get(url, timeout=5)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        print(f"  ⚠ Could not reach {url}: {e}")
        return None


def wait_for_device(hostname, timeout=90, expect_version=None):
    """Wait for device to come back online after reboot."""
    print(f"  Waiting for {hostname} to come back online (timeout {timeout}s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        info = get_device_version(hostname)
        if info is not None:
            if expect_version and info.get("version") != expect_version:
                print(
                    f"  ⚠ Device up but running {info['version']}"
                    f" (expected {expect_version})"
                )
            return info
        time.sleep(2)
    return None


class DualStackServer(http.server.ThreadingHTTPServer):
    address_family = socket.AF_INET6

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()


def serve_file(directory, port):
    """Start an HTTP server serving files from directory. Returns (server, thread)."""

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=directory, **kwargs)

        def log_message(self, format, *args):
            print(f"  [HTTP] {format % args}")

    try:
        server = DualStackServer(("::", port), QuietHandler)
    except Exception:
        # Fallback to IPv4 only if IPv6 dual-stack bind fails
        server = http.server.HTTPServer(("0.0.0.0", port), QuietHandler)

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def send_upgrade_request(hostname, url):
    """Send the OTA upgrade request to the device via protobuf POST."""
    url_bytes = url.encode("utf-8")
    length = len(url_bytes)
    varint = b""
    while length > 0x7F:
        varint += bytes([(length & 0x7F) | 0x80])
        length >>= 7
    varint += bytes([length])
    payload = b"\x0a" + varint + url_bytes

    target = device_url(hostname, "/fwupgrade")
    r = requests.post(
        target,
        data=payload,
        headers={"Content-Type": "application/protobuf"},
        timeout=10,
    )
    r.raise_for_status()
    return r.status_code


def send_promote_request(hostname):
    """Send the /promote POST to confirm the running image."""
    target = device_url(hostname, "/promote")
    r = requests.post(target, timeout=10)
    r.raise_for_status()
    return r.status_code


def find_default_image():
    """Find the default OTA image from the build directory."""
    import pathlib

    candidates = [
        pathlib.Path("build/sesame/zephyr/zephyr.signed.bin"),
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    return None


def cmd_upgrade(args):
    hostname = args.hostname
    image = args.image or find_default_image()
    port = args.port

    if not image:
        print("✗ No image specified and no default found in build/")
        sys.exit(1)

    print(f"═══ OTA Upgrade: {hostname} ═══")
    print()

    # Read target image version from MCUboot header
    try:
        target_version = read_image_version(image)
    except Exception as e:
        print(f"✗ Failed to read image version: {e}")
        sys.exit(1)

    import os

    image_size = os.path.getsize(image)
    print(f"  Image:   {image} ({image_size:,} bytes)")
    print(f"  Target:  {target_version}")
    print()

    # Check current device version
    print(f"  Querying {hostname}...")
    pre_info = get_device_version(hostname)
    if pre_info:
        print(f"  Current: {pre_info['version']}")
        print(f"  Slot:    {pre_info.get('slot', '?')}")
        print(f"  Confirmed: {pre_info.get('confirmed', '?')}")
        if pre_info["version"] == target_version:
            print(f"\n  ⚠ Device already running target version {target_version}")
            if not args.force:
                print("  Use --force to upgrade anyway")
                sys.exit(0)
    else:
        print("  ⚠ Could not query device version (continuing anyway)")
    print()

    # Start HTTP server
    import os

    image_dir = os.path.dirname(os.path.abspath(image))
    image_name = os.path.basename(image)
    local_ip = get_local_ip(hostname)

    print(f"  Starting HTTP server on {local_ip}:{port}...")
    server, thread = serve_file(image_dir, port)

    try:
        download_url = f"http://{local_ip}:{port}/{image_name}"
        print(f"  Serving: {download_url}")
        print()

        # Send upgrade request
        print(f"  Sending upgrade request to {hostname}...")
        try:
            status = send_upgrade_request(hostname, download_url)
            print(f"  → HTTP {status}")
        except Exception as e:
            print(f"  ✗ Upgrade request failed: {e}")
            sys.exit(1)

        # Wait for the device to download (watch HTTP server logs)
        # and then reboot. The device will disconnect.
        print()
        print("  Waiting for device to download and reboot...")
        time.sleep(5)

        # Wait for reboot
        post_info = wait_for_device(
            hostname, timeout=args.timeout, expect_version=target_version
        )

        if not post_info:
            print(f"\n  ✗ Device did not come back online within {args.timeout}s")
            sys.exit(1)

        print()
        print(f"  Device is back online!")
        print(f"  Version:   {post_info['version']}")
        print(f"  Slot:      {post_info.get('slot', '?')}")
        print(f"  Confirmed: {post_info.get('confirmed', '?')}")

        if post_info["version"] != target_version:
            print(
                f"\n  ✗ Version mismatch! Expected {target_version},"
                f" got {post_info['version']}"
            )
            print("    MCUboot may have reverted to the old image.")
            sys.exit(1)

        # Auto-promote if requested
        if args.promote:
            print()
            if post_info.get("confirmed"):
                print("  Image is already confirmed, skipping promote")
            else:
                print("  Promoting image...")
                try:
                    status = send_promote_request(hostname)
                    print(f"  → HTTP {status}")
                except Exception as e:
                    print(f"  ✗ Promote failed: {e}")
                    sys.exit(1)

                # Verify promotion
                time.sleep(1)
                verify = get_device_version(hostname)
                if verify and verify.get("confirmed"):
                    print(f"  ✓ Image confirmed!")
                else:
                    print(
                        f"  ⚠ Promote sent but confirmed={verify.get('confirmed') if verify else '?'}"
                    )

        print()
        print("  ✓ OTA upgrade complete!")

    finally:
        server.shutdown()


def cmd_promote(args):
    hostname = args.hostname

    print(f"═══ OTA Promote: {hostname} ═══")
    print()

    info = get_device_version(hostname)
    if not info:
        print(f"  ✗ Could not reach {hostname}")
        sys.exit(1)

    print(f"  Version:   {info['version']}")
    print(f"  Slot:      {info.get('slot', '?')}")
    print(f"  Confirmed: {info.get('confirmed', '?')}")

    if info.get("confirmed"):
        print(f"\n  Image is already confirmed, nothing to do")
        return

    print(f"\n  Promoting image...")
    try:
        status = send_promote_request(hostname)
        print(f"  → HTTP {status}")
    except Exception as e:
        print(f"  ✗ Promote failed: {e}")
        sys.exit(1)

    time.sleep(1)
    verify = get_device_version(hostname)
    if verify and verify.get("confirmed"):
        print(f"  ✓ Image confirmed!")
    else:
        print(
            f"  ⚠ Promote sent but confirmed={verify.get('confirmed') if verify else '?'}"
        )


def main():
    parser = argparse.ArgumentParser(
        description="OTA firmware upgrade tool for sesame devices"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # upgrade subcommand
    up = subparsers.add_parser("upgrade", help="Upload and flash new firmware via OTA")
    up.add_argument("hostname", help="Device hostname or IP")
    up.add_argument(
        "--image",
        help="Path to signed firmware image (default: build/sesame/zephyr/zephyr.signed.bin)",
    )
    up.add_argument(
        "--promote",
        action="store_true",
        help="Automatically promote (confirm) the image after successful boot",
    )
    up.add_argument(
        "--port",
        type=int,
        default=8000,
        help="HTTP server port (default: 8000)",
    )
    up.add_argument(
        "--timeout",
        type=int,
        default=90,
        help="Timeout in seconds to wait for device reboot (default: 90)",
    )
    up.add_argument(
        "--force",
        action="store_true",
        help="Upgrade even if device is already running the target version",
    )
    up.set_defaults(func=cmd_upgrade)

    # promote subcommand
    pr = subparsers.add_parser("promote", help="Confirm the currently running image")
    pr.add_argument("hostname", help="Device hostname or IP")
    pr.set_defaults(func=cmd_promote)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
