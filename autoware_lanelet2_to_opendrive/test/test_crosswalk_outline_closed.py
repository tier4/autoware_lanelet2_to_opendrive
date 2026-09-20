"""Crosswalk outlines must close with a byte-identical repeated first corner.

CARLA finds the end of one crosswalk polygon by looking for a *repeated
vertex*: ``Map::GetAllCrosswalkMesh()`` concatenates every crosswalk's raw
corner list into one flat array and starts a new triangle fan only when
``crosswalk_vertex[start_vertex_index] == crosswalk_vertex[i]``.  If no
vertex ever repeats, ``AddTriangleFan`` is never reached and the crosswalk
contributes **no geometry at all** to the mesh that Recast turns into the
pedestrian navmesh -- silently, with no parse error.

That comparison is ``geom::Vector3D::operator==`` at tag 0.10.0::

    constexpr bool operator==(const Vector3D &rhs) const {
      return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
    }

Exact floating-point equality on ``float`` members, no tolerance.  So a
closing corner that is merely *geometrically coincident* with the first is
not good enough: it has to survive serialisation and reparsing as the same
value.  ``_compute_corner_locals`` guarantees that by appending the first
``CornerLocal`` object itself rather than a recomputed copy.

These tests pin the property at the level that matters -- identical
attribute *strings* -- so that a future change to number formatting cannot
weaken it without failing here.  Verified against both reference outputs at
the time of writing: 84 of 84 outlines closed with byte-identical strings.

Source read, not run: there is no CARLA server in this environment, so the
navmesh consequence above is derived from CARLA 0.10.0 source and from the
pinned client wheel, not observed.
"""

from __future__ import annotations

import contextlib
import io
from typing import List, Tuple

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
    ParamPoly3Config,
)
from autoware_lanelet2_to_opendrive.main import convert_lanelet2_to_opendrive
from autoware_lanelet2_to_opendrive.opendrive.objects import _compute_corner_locals

import numpy as np


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _bound(points: List[Tuple[float, float]]) -> lanelet2.core.LineString3d:
    return lanelet2.core.LineString3d(
        lanelet2.core.getId(),
        [lanelet2.core.Point3d(lanelet2.core.getId(), x, y, 0.0) for x, y in points],
    )


def _map_with_road_and_crosswalk() -> lanelet2.core.LaneletMap:
    """A straight road with a crosswalk lying across it."""
    lanelet_map = lanelet2.core.LaneletMap()

    road = lanelet2.core.Lanelet(
        lanelet2.core.getId(),
        _bound([(0.0, 0.0), (20.0, 0.0), (40.0, 0.0)]),
        _bound([(0.0, -3.5), (20.0, -3.5), (40.0, -3.5)]),
    )
    road.attributes["subtype"] = "road"
    lanelet_map.add(road)

    # Crosswalk across the road at x ~ 20, running in +y.
    crosswalk = lanelet2.core.Lanelet(
        lanelet2.core.getId(),
        _bound([(18.0, -6.0), (18.0, 2.0)]),
        _bound([(22.0, -6.0), (22.0, 2.0)]),
    )
    crosswalk.attributes["subtype"] = "crosswalk"
    lanelet_map.add(crosswalk)

    return lanelet_map


def _convert(lanelet_map: lanelet2.core.LaneletMap):
    config = ConversionConfig(
        origin=OriginSpec(mgrs_code="54SUE"),
        parampoly3=ParamPoly3Config(enabled=False),
    )
    with (
        contextlib.redirect_stdout(io.StringIO()),
        contextlib.redirect_stderr(io.StringIO()),
    ):
        opendrive, *_ = convert_lanelet2_to_opendrive(lanelet_map, config)
    return opendrive.to_xml()


def _crosswalk_outlines(root) -> List[List[Tuple[str, str, str]]]:
    """Each crosswalk's corner ``(u, v, z)`` attribute strings, verbatim."""
    outlines = []
    for obj in root.iter("object"):
        if obj.get("type") != "crosswalk":
            continue
        outline = obj.find("outline")
        if outline is None:
            continue
        outlines.append(
            [
                (c.get("u"), c.get("v"), c.get("z"))
                for c in outline.findall("cornerLocal")
            ]
        )
    return outlines


# --------------------------------------------------------------------------
# the closure property
# --------------------------------------------------------------------------


def test_compute_corner_locals_repeats_the_first_corner_object() -> None:
    """Closure is explicit, not incidental to the input geometry.

    The last entry must be the *same object* as the first, which is what
    makes the serialised strings identical no matter how floats are
    formatted.
    """
    centroid = np.array([0.0, 0.0])
    cw_dir = np.array([1.0, 0.0])
    vertices = [
        np.array([-2.0, -1.0]),
        np.array([-2.0, 1.0]),
        np.array([2.0, 1.0]),
        np.array([2.0, -1.0]),
    ]

    corners = _compute_corner_locals(centroid, cw_dir, vertices)

    assert len(corners) == len(vertices) + 1
    assert corners[-1] is corners[0], (
        "the closing corner must be the first corner object itself; a "
        "recomputed copy can serialise differently and CARLA compares "
        "these values with exact float equality"
    )


def test_emitted_crosswalk_outline_closes_with_identical_strings() -> None:
    """End to end: last cornerLocal's u/v/z strings equal the first's."""
    root = _convert(_map_with_road_and_crosswalk())

    outlines = _crosswalk_outlines(root)
    assert outlines, "no crosswalk object with an outline was emitted"

    for corners in outlines:
        assert len(corners) >= 4, f"degenerate crosswalk outline: {corners}"
        assert corners[0] == corners[-1], (
            "crosswalk outline is not closed with a byte-identical repeated "
            f"first corner: first={corners[0]} last={corners[-1]}. CARLA "
            "delimits polygons by exact float equality, so an outline that "
            "is merely geometrically coincident produces no navmesh geometry."
        )


def test_no_crosswalk_outline_is_left_open() -> None:
    """Every emitted outline closes -- none may be skipped."""
    root = _convert(_map_with_road_and_crosswalk())

    outlines = _crosswalk_outlines(root)
    unclosed = [c for c in outlines if c[0] != c[-1]]

    assert not unclosed, f"{len(unclosed)} of {len(outlines)} outlines unclosed"


@pytest.mark.parametrize("attribute_index,name", [(0, "u"), (1, "v"), (2, "z")])
def test_closure_holds_per_coordinate(attribute_index: int, name: str) -> None:
    """Each coordinate individually matches, since CARLA ANDs all three."""
    root = _convert(_map_with_road_and_crosswalk())

    for corners in _crosswalk_outlines(root):
        assert (
            corners[0][attribute_index] == corners[-1][attribute_index]
        ), f"crosswalk outline closing corner differs in {name}"
