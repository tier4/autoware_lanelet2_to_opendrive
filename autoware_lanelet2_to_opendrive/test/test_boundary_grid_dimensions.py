"""The boundary grid must say how coarse it turned out.

`_build_boundary_grid` sets its cell to the largest bounding-box extent in
the input, so a single long boundary sizes the cell for the whole pool. That
keeps the grid at O(N) entries -- which is what the docstring claims -- but it
also decides how many boundaries a query returns, and nothing reported it.

On the nishishinjuku pool the cell comes out at 328 m over a map of about
1112 x 1086 m, which is 14 occupied cells: a query returns most of the pool.
The grid is still correct, and the expensive part of the comparison is bounded
by the road's own bounding box, so the cost is not visible as a slowdown until
the extent grows -- and then it arrives as a cliff rather than a gradient.

This pins the one line that makes that visible before it is paid for.
"""

import logging

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    _build_boundary_grid,
)

LOGGER_NAME = "autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping"


def _pool(boundaries):
    """``(boundaries, bboxes)`` for polylines given as point lists."""
    arrays = {
        lid: np.asarray(points, dtype=float) for lid, points in boundaries.items()
    }
    bboxes = {
        lid: (
            float(points[:, 0].min()),
            float(points[:, 1].min()),
            float(points[:, 0].max()),
            float(points[:, 1].max()),
        )
        for lid, points in arrays.items()
    }
    return arrays, bboxes


@pytest.mark.xfail(strict=True, reason="the grid reports no dimensions")
def test_the_grid_reports_its_cell_size_and_occupancy(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """One short boundary and one long one: the long one sizes the cell."""
    boundaries, bboxes = _pool(
        {
            1: [(0.0, 0.0), (5.0, 0.0)],
            2: [(0.0, 10.0), (400.0, 10.0)],
            3: [(10.0, 20.0), (15.0, 20.0)],
        }
    )

    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        grid = _build_boundary_grid(boundaries, bboxes)

    assert grid.cell_size == 400.0
    message = "\n".join(record.getMessage() for record in caplog.records)
    assert message, "the grid logged nothing about its own dimensions"
    assert "400" in message, f"the cell size is not in the message: {message}"
    assert "3" in message, f"the boundary count is not in the message: {message}"


@pytest.mark.xfail(strict=True, reason="the grid reports no dimensions")
def test_the_message_names_the_occupancy_not_just_the_size(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Two boundaries in one cell must read differently from two in two.

    The cell size alone does not say whether the grid separates anything; the
    number a query returns does.
    """
    together, together_boxes = _pool(
        {1: [(0.0, 0.0), (1.0, 0.0)], 2: [(2.0, 0.0), (3.0, 0.0)]}
    )
    apart, apart_boxes = _pool(
        {1: [(0.0, 0.0), (1.0, 0.0)], 2: [(900.0, 0.0), (901.0, 0.0)]}
    )

    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        _build_boundary_grid(together, together_boxes)
        first = "\n".join(record.getMessage() for record in caplog.records)
        caplog.clear()
        _build_boundary_grid(apart, apart_boxes)
        second = "\n".join(record.getMessage() for record in caplog.records)

    assert first != second, (
        "the same message for a grid holding both boundaries in one cell and"
        f" one holding them apart: {first}"
    )
