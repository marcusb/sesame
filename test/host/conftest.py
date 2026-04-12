"""
Shared pytest infrastructure for the Sesame host tests.

Each module's own conftest.py defines a fixture that calls _run_harness() and
handles any protocol-specific readiness checks before yielding.
"""

import sys
from pathlib import Path

import pytest

from _harness import _BUILD_DIRS


@pytest.fixture(scope="session", autouse=True)
def _proto_py_path() -> None:
    """Add the build-time generated protobuf Python bindings to sys.path."""
    for bd in _BUILD_DIRS:
        p = bd / "proto_py"
        if (p / "app_config_pb2.py").exists():
            sys.path.insert(0, str(p))
            return
    pytest.fail(
        "proto_py not found; run `ninja -C build host_tests` first.\n"
        "Looked in:\n  " + "\n  ".join(str(bd / "proto_py") for bd in _BUILD_DIRS)
    )
