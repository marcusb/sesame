"""
Shared pytest infrastructure for the Sesame integration tests.

Each module's own conftest.py defines a fixture that calls _run_harness() and
handles any protocol-specific readiness checks before yielding.
"""

import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).parent.parent.parent
_BUILD_DIR = _REPO_ROOT / "build"

@pytest.fixture(scope="session", autouse=True)
def _proto_py_path() -> None:
    """Add the build-time generated protobuf Python bindings to sys.path."""
    p = _BUILD_DIR / "test" / "integration" / "proto_py"
    if (p / "app_config_pb2.py").exists():
        sys.path.insert(0, str(p))
        return
    pytest.fail(
        "proto_py not found; run `ninja -C build integration_tests` first.\n"
        f"Looked in: {p}"
    )
