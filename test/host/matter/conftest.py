from typing import Iterator
import pytest
from _harness import _run_harness, Harness

@pytest.fixture
def matter_harness() -> Iterator[Harness]:
    """Fixture to run the Matter host test binary."""
    yield from _run_harness("HOST_TEST_BIN", "matter")
