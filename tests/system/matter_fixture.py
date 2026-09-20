import pytest


@pytest.fixture(scope="module")
def matter():
    """Lazily initialize the Matter (CHIP) controller runtime.

    The CHIP native stack is created only when a Matter test requests this
    fixture, so non-Matter system tests (OTA, HTTP) never import or init it.
    This avoids instantiating the CHIP InetLayer for those runs and the
    process-exit teardown that aborts the interpreter.
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
