import serial
import time
import sys

def test_boot(port='/dev/ttyUSB0', timeout=15):
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(1)

    try:
        start = time.time()
        boot_logs = ""
        while time.time() - start < timeout:
            chunk = ser.read(512)
            if chunk:
                boot_logs += chunk.decode(errors='ignore')
                print(chunk.decode(errors='ignore'), end='', flush=True)

        if 'WiFi' in boot_logs and 'ready' in boot_logs:
            print("\n✓ Boot successful")
            return True
        else:
            print("\n✗ Boot incomplete")
            return False
    finally:
        ser.close()

if __name__ == '__main__':
    sys.exit(0 if test_boot() else 1)
