import asyncio
import json
import logging
import os
import shutil
import tempfile
import time
import urllib.request
from pathlib import Path

import pytest

logging.basicConfig(level=logging.DEBUG)

# The device signs its Matter attestation with the development (VID 0xFFF1)
# example creds, which chain to a dev PAA root. The Matter python controller
# defaults to a CWD-relative trust store that only resolves if `./credentials`
# is a symlink, so point it at the real PAA root-cert dir explicitly.
PAA_TRUST_STORE = str(
    Path(__file__).resolve().parents[2]
    / "third_party"
    / "connectedhomeip"
    / "credentials"
    / "development"
    / "paa-root-certs"
)


@pytest.fixture(scope="module")
def matter_classes():
    """
    Dynamically loads the Matter Python bindings.
    Skips tests if the bindings are not built or not in the Python path.
    """
    try:
        import matter.native

        matter.native.Init()
        import matter.ChipDeviceCtrl
        import matter.clusters as Clusters
        import matter.storage
        from matter.CertificateAuthority import CertificateAuthorityManager
        from matter.ChipStack import ChipStack
        from matter.setup_payload.setup_payload import SetupPayload

        return {
            "CertificateAuthorityManager": CertificateAuthorityManager,
            "ChipStack": ChipStack,
            "ChipDeviceCtrl": matter.ChipDeviceCtrl,
            "storage": matter.storage,
            "Clusters": Clusters,
            "SetupPayload": SetupPayload,
        }
    except ImportError:
        pytest.fail(
            "Matter Python bindings not found. Please build them first and activate the venv."
        )


@pytest.fixture
def matter_controller(matter_classes):
    """
    Initializes the Matter controller stack and provides a controller instance.
    Cleans up the stack and temporary persistent storage after the test.
    """
    temp_dir = tempfile.mkdtemp(dir="build")

    storage = matter_classes["storage"].PersistentStorageJSON(
        os.path.join(temp_dir, "repl_storage.json")
    )
    stack = matter_classes["ChipStack"](persistentStorage=storage)
    ca_manager = matter_classes["CertificateAuthorityManager"](
        stack, stack.GetStorageManager()
    )
    ca_manager.LoadAuthoritiesFromStorage()

    if len(ca_manager.activeCaList) == 0:
        ca = ca_manager.NewCertificateAuthority()
        ca.NewFabricAdmin(vendorId=0xFFF1, fabricId=1)

    ca = ca_manager.activeCaList[0]
    admin = ca.adminList[0]
    controller = admin.NewController(nodeId=112233, paaTrustStorePath=PAA_TRUST_STORE)

    yield controller

    # Teardown
    try:
        controller.Shutdown()
        ca_manager.Shutdown()
        stack.Shutdown()
    except Exception:
        pass
    shutil.rmtree(temp_dir)


def test_matter_provisioning(zephyr_app, matter_controller, matter_classes):
    """
    Verifies that the firmware can be commissioned over PASE
    and properly processes operational commands (like UpOrOpen).
    """
    pairing_code = zephyr_app["pairing_code"]
    firmware_logs = zephyr_app["logs"]

    parser = matter_classes["SetupPayload"]()
    parser.ParseManualPairingCode(pairing_code)
    setup_pin = int(parser.attributes["SetUpPINCode"])

    # Verify that /matter/info returns empty fabrics and setup details before commissioning
    req = urllib.request.Request("http://127.0.0.1:8080/matter/info")
    with urllib.request.urlopen(req, timeout=5) as response:
        assert response.status == 200
        assert "application/json" in response.headers.get("Content-Type", "")
        data = json.loads(response.read().decode("utf-8"))
        assert isinstance(data, dict)
        assert data.get("fabrics") == []
        assert "setup" in data
        assert "commissioning_open" in data["setup"]
        assert data["setup"]["manual_pairing_code"] == pairing_code
        assert "qr_code" in data["setup"]
        assert data["setup"]["qr_code"].startswith("MT:")

    print(f"Commissioning Node 1 with setup PIN {setup_pin} via IP...")
    time.sleep(1.0)

    async def commission():
        await matter_controller.EstablishPASESessionIP("::1", setup_pin, 1)

    asyncio.run(commission())

    print("PASE session established! Sending Door Open command...")

    async def send_command():
        Clusters = matter_classes["Clusters"]
        # 1 is endpoint 1
        await matter_controller.SendCommand(
            1, 1, Clusters.WindowCovering.Commands.UpOrOpen()
        )

    asyncio.run(send_command())
    time.sleep(2)  # Give it time to process and log

    command_received = False
    end_time = time.time() + 10
    while time.time() < end_time:
        if any("Matter: Target=Open" in line for line in firmware_logs):
            command_received = True
            break
        time.sleep(0.1)

    assert command_received, "Firmware did not log receipt of the Open command"

    print("Verifying NetworkCommissioning and BasicInformation attributes on Endpoint 0 over PASE...")

    async def verify_endpoint0_attributes():
        Clusters = matter_classes["Clusters"]
        res = await matter_controller.ReadAttribute(
            nodeId=1,
            attributes=[
                (0, Clusters.NetworkCommissioning.Attributes.FeatureMap),
                (0, Clusters.BasicInformation.Attributes.SoftwareVersionString),
            ],
            returnClusterObject=True,
        )
        assert 0 in res
        assert Clusters.NetworkCommissioning in res[0]
        feature_map = res[0][Clusters.NetworkCommissioning].featureMap
        assert feature_map == 4, f"Expected FeatureMap 4 (Ethernet), got {feature_map}"

        assert Clusters.BasicInformation in res[0]
        version_str = res[0][Clusters.BasicInformation].softwareVersionString
        print(f"Matter SoftwareVersionString: {version_str}")
        assert "+" in version_str and "-" not in version_str, f"Unexpected version string format: {version_str}"

    asyncio.run(verify_endpoint0_attributes())

    print("Commissioning node to provision fabric...")

    async def complete_commissioning():
        matter_controller.SetSkipCommissioningComplete(True)
        await matter_controller.Commission(1)

    asyncio.run(complete_commissioning())

    # Verify that /matter/info returns the provisioned fabric in "fabrics" and no "setup"
    req = urllib.request.Request("http://127.0.0.1:8080/matter/info")
    with urllib.request.urlopen(req, timeout=5) as response:
        assert response.status == 200
        assert "application/json" in response.headers.get("Content-Type", "")
        data = json.loads(response.read().decode("utf-8"))
        assert isinstance(data, dict)
        assert "setup" not in data
        fabrics = data.get("fabrics", [])
        assert len(fabrics) >= 1
        fabric = fabrics[0]
        assert "node_id" in fabric
        assert "fabric_id" in fabric
        assert "vendor_id" in fabric
        assert fabric["vendor_id"] == 0xFFF1

    print("SUCCESS! Integration test passed.")
