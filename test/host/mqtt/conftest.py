from __future__ import annotations

from typing import Iterator

import pytest

from _harness import Harness, _run_harness


@pytest.fixture()
def mqtt_harness() -> Iterator[Harness]:
    yield from _run_harness("HOST_TEST_BIN", "mqtt")
