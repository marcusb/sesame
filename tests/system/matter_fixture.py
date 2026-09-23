import pytest


@pytest.fixture(scope="session")
def matter():
    """Lazily initialize the Matter (CHIP) controller runtime once per session.

    The CHIP native stack is a process-wide singleton that cannot be restarted
    after shutdown, so it is initialized exactly once (session scope) and shared
    by every Matter test in the run. Non-Matter system tests (OTA, HTTP) never
    request this fixture, so they never import or init the CHIP InetLayer (and
    avoid the process-exit teardown that aborts the interpreter).
    """
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
        "Clusters": Clusters,
        "SetupPayload": SetupPayload,
        "storage": matter.storage,
    }


@pytest.fixture(scope="session")
def chip_stack(matter, tmp_path_factory):
    """A single shared CHIP device-controller stack for the whole session.

    ChipStack is a process singleton: it must be created in exactly one place
    and kept alive for every Matter test. Shutting it down per-test would poison
    the singleton (the next ChipStack(...) returns the dead instance) and leave
    the native common-stack init/shutdown unbalanced. It is torn down exactly
    once here, at session end.
    """
    storage = matter["storage"].PersistentStorageJSON(
        str(tmp_path_factory.mktemp("matter_stack") / "storage.json")
    )
    stack = matter["ChipStack"](persistentStorage=storage)
    yield stack
    stack.Shutdown()


@pytest.fixture(scope="session")
def fabric_admin(chip_stack, matter):
    """A commissioning fabric admin shared by all Matter tests.

    Built once on the shared stack so the CA's OpCreds delegate is not freed and
    re-initialized between tests. Each test attaches its own controller via
    admin.NewController(nodeId=...) and re-commissions (rebooted) nodes onto this
    fabric. Both Matter tests use VID 0xFFF1 / fabric 1.
    """
    ca_manager = matter["CertificateAuthorityManager"](
        chip_stack, chip_stack.GetStorageManager()
    )
    ca_manager.LoadAuthoritiesFromStorage()
    if len(ca_manager.activeCaList) == 0:
        ca_manager.NewCertificateAuthority().NewFabricAdmin(vendorId=0xFFF1, fabricId=1)
    admin = ca_manager.activeCaList[0].adminList[0]
    yield admin
    # All tests are done; free the delegate while the stack is still alive
    # (chip_stack tears the stack down afterwards).
    ca_manager.Shutdown()
