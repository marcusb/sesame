import logging
logging.basicConfig(level=logging.DEBUG)
import pytest
import subprocess
import time
import os
import signal
import pty
import re
import asyncio
import tempfile
import shutil
import threading

def test_matter_provisioning():
    zephyr_exe = "build/zephyr/zephyr.exe"
    if not os.path.exists(zephyr_exe):
        pytest.fail(f"{zephyr_exe} not found. Please build integration target first.")
    
    try:
        import matter.native
        matter.native.Init()
        from matter.CertificateAuthority import CertificateAuthorityManager
        from matter.ChipStack import ChipStack
        import matter.ChipDeviceCtrl
        import matter.storage
        import matter.clusters as Clusters
        from matter.setup_payload.setup_payload import SetupPayload
    except ImportError:
        pytest.fail("Matter Python bindings not found. Please build them first and activate the venv.")

    print("Starting Zephyr native_sim...")
    
    master, slave = pty.openpty()
    
    os.system("rm -f flash.bin")
    process = subprocess.Popen([
        zephyr_exe,
    ], stdout=slave, stderr=slave, text=True)
    
    os.close(slave)
    
    firmware_logs = []
    stop_reader = threading.Event()
    
    def log_reader():
        buf = b""
        while not stop_reader.is_set():
            try:
                # Use non-blocking read or select
                import select
                r, _, _ = select.select([master], [], [], 0.1)
                if r:
                    data = os.read(master, 1024)
                    if data:
                        buf += data
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            line_str = line.decode(errors="ignore").strip()
                            firmware_logs.append(line_str)
                            print(f"[FIRMWARE] {line_str}")
            except Exception:
                pass

    reader_thread = threading.Thread(target=log_reader, daemon=True)
    reader_thread.start()
    
    temp_dir = tempfile.mkdtemp(dir="build")
    try:
        storage = matter.storage.PersistentStorageJSON(os.path.join(temp_dir, 'repl_storage.json'))
        stack = ChipStack(persistentStorage=storage)
        certificate_authority_manager = CertificateAuthorityManager(stack, stack.GetStorageManager())
        certificate_authority_manager.LoadAuthoritiesFromStorage()
        if len(certificate_authority_manager.activeCaList) == 0:
            ca = certificate_authority_manager.NewCertificateAuthority()
            ca.NewFabricAdmin(vendorId=0xFFF1, fabricId=1)
        ca = certificate_authority_manager.activeCaList[0]
        admin = ca.adminList[0]
        controller = admin.NewController(nodeId=112233)
        
        booted = False
        pairing_code = None
        start_time = time.time()
        while time.time() - start_time < 30:
            # check logs
            for line in list(firmware_logs):
                match = re.search(r"Manual pairing code: \[([0-9]+)\]", line)
                if match and not pairing_code:
                    pairing_code = match.group(1)
                    print(f"--> Found pairing code: {pairing_code}")
                
                if "Network is UP. Opening commissioning window" in line:
                    booted = True
            
            if booted and pairing_code:
                break
            time.sleep(0.1)
                
        assert booted, "Firmware failed to boot or initialize network"
        assert pairing_code, "Firmware did not output a manual pairing code"
        print("Firmware booted successfully on native_sim!")
        time.sleep(2) # let it settle
        
        parser = SetupPayload()
        parser.ParseManualPairingCode(pairing_code)
        setup_pin = int(parser.attributes["SetUpPINCode"])
        
        print(f"Commissioning Node 1 with setup PIN {setup_pin} via IP...")
        async def commission():
            await controller.EstablishPASESessionIP("::1", setup_pin, 1)
            await controller.Commission(1)
            
        asyncio.run(commission())

        print("Commissioning successful! Sending Door Open command...")
        
        async def send_command():
            try:
                # 1 is endpoint 1
                await controller.SendCommand(1, 1, Clusters.WindowCovering.Commands.UpOrOpen())
                print("Command sent successfully!")
            except Exception as e:
                print(f"Failed to send command: {e}")
                raise

        asyncio.run(send_command())
        time.sleep(2) # Give it time to process and log

            
        
        command_received = False
        end_time = time.time() + 10
        while time.time() < end_time:
            for line in list(firmware_logs):
                if "Matter: Target=Open" in line:
                    command_received = True
                    break
            if command_received:
                break
            time.sleep(0.1)
                
        assert command_received, "Firmware did not log receipt of the Open command"
        print("SUCCESS! Integration test passed.")
        
    finally:
        stop_reader.set()
        reader_thread.join(timeout=2)
        os.close(master)
        process.kill()
        process.wait(timeout=5)
        try:
            controller.Shutdown()
            certificate_authority_manager.Shutdown()
            stack.Shutdown()
        except:
            pass
        import shutil
        shutil.rmtree(temp_dir)
