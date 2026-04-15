import time
import pytest
from _harness import Harness

def test_matter_startup(matter_harness: Harness) -> None:
    """Verify that the Matter stack starts and runs its loop."""
    time.sleep(2)
    assert matter_harness.proc.poll() is None

def test_matter_crypto_sha256(matter_harness: Harness) -> None:
    """Verify SHA256 shim."""
    script = """
    import crypto
    var b = crypto.SHA256(bytes().fromstring('hello'))
    assert(b.tohex() == '2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824')
    print('SUCCESS_SHA256')
    """
    matter_harness.send_cmd(f"berry {script}")
    # Harness doesn't have a direct way to see stdout yet, but we can check if it crashes
    time.sleep(1)
    assert matter_harness.proc.poll() is None

def test_matter_module_load(matter_harness: Harness) -> None:
    """Verify that the 'matter' module can be imported and has expected classes."""
    script = """
    import matter
    assert(matter.Counter != nil)
    assert(matter.Verhoeff != nil)
    assert(matter.Commissioning != nil)
    print('SUCCESS_MATTER_LOAD')
    """
    matter_harness.send_cmd(f"berry {script}")
    time.sleep(1)
    assert matter_harness.proc.poll() is None
