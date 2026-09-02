"""Unit tests for :class:`CarlaCameraSensor` frame retrieval.

The draining behaviour is the subtle part: the sensor delivers frames faster than
the policy consumes them, so ``get_image`` must return the *newest* queued frame
rather than the oldest, or the caller is fed steadily staler images while the
bounded FIFO stays full and blocks the delivery callback.
"""

from __future__ import annotations

import queue
from types import SimpleNamespace

import numpy as np
import pytest

from autoware_carla_scenario.sensor import carla_camera
from autoware_carla_scenario.sensor.carla_camera import (
    CarlaCameraSensor,
    CarlaCameraSensorConfig,
)


def _frame(value: int) -> SimpleNamespace:
    """A 1x1 BGRA frame whose single pixel encodes *value*, as a fake carla.Image."""
    return SimpleNamespace(
        width=1,
        height=1,
        raw_data=bytes([value, value, value, 255]),  # BGRA
    )


def _sensor() -> CarlaCameraSensor:
    return CarlaCameraSensor(CarlaCameraSensorConfig(image_width=1, image_height=1))


def test_get_image_returns_the_only_queued_frame() -> None:
    sensor = _sensor()
    sensor._frame_queue.put(_frame(7))
    image = sensor.get_image()
    assert image is not None
    assert image.shape == (1, 1, 3)
    assert int(image[0, 0, 0]) == 7


def test_get_image_drains_to_the_newest_frame() -> None:
    """Two frames arrive per policy step; the older ones must be dropped, not the
    newest one delivered late."""
    sensor = _sensor()
    sensor._frame_queue = queue.Queue()  # unbounded, to stage several frames
    for value in (1, 2, 3, 4):
        sensor._frame_queue.put(_frame(value))

    image = sensor.get_image()
    assert image is not None
    assert int(image[0, 0, 0]) == 4  # the newest
    assert sensor._frame_queue.empty()  # everything drained


def test_get_image_returns_none_on_timeout(monkeypatch: pytest.MonkeyPatch) -> None:
    # Shrink the blocking wait so an empty queue reports "no frame" quickly.
    monkeypatch.setattr(carla_camera, "_FRAME_TIMEOUT", 0.01)
    sensor = _sensor()
    assert sensor.get_image() is None


def test_get_image_produces_a_bgr_array() -> None:
    sensor = _sensor()
    sensor._frame_queue.put(_frame(5))
    image = sensor.get_image()
    assert image is not None
    assert image.dtype == np.uint8
    assert image.shape[2] == 3  # alpha stripped
