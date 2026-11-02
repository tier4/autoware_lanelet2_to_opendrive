"""The resolved origin must belong to the map it is resolved for.

Nothing compares the origin against the map. A preset copied to a new map,
or a `map_projector_info.yaml` left behind from another one, resolves without
complaint and the conversion runs to completion: every coordinate is written
in a frame that is wrong by whole MGRS squares, and the only visible symptom
is the `geoReference` -- which no gate we own reads.

The map's node lat/lon is the one thing always present (the `mgrs_code` tag
is optional), so the check is built on it: the configured grid must be a grid
the map's own nodes fall in.
"""

from pathlib import Path

import pytest
from omegaconf import OmegaConf

from autoware_lanelet2_to_opendrive.projection_resolver import (
    resolve_projection,
    verify_origin_matches_map,
)

TOKYO = (35.68856, 139.69260)
"""A node of the ``walkway_mini`` fixture: MGRS square 54SUE."""

OSAKA = (34.69, 135.50)
"""MGRS square 53SNU -- a different zone, so no prefix can match."""


def _osm(tmp_path: Path, name: str, points) -> Path:
    nodes = "\n".join(
        f'  <node id="{index + 1}" lat="{lat}" lon="{lon}"/>'
        for index, (lat, lon) in enumerate(points)
    )
    path = tmp_path / name
    path.write_text(
        f'<?xml version="1.0"?>\n<osm version="0.6">\n{nodes}\n</osm>\n',
        encoding="utf-8",
    )
    return path


def _resolved(map_dict, map_path: Path):
    return resolve_projection(OmegaConf.create({"map": map_dict}), map_path)


def test_an_origin_in_the_map_s_own_grid_is_accepted(tmp_path: Path) -> None:
    """The nishishinjuku preset's grid against a map of Tokyo nodes."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])
    with pytest.warns(DeprecationWarning):
        resolved = _resolved(
            {"mgrs_grid": "54SUE", "offset": {"x": 81655.73, "y": 50137.43}}, osm
        )

    verify_origin_matches_map(resolved, osm)


def test_a_higher_precision_grid_is_compared_on_its_prefix(tmp_path: Path) -> None:
    """``example.yaml`` names ``54SUE815501``; the map's square is ``54SUE``."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])
    with pytest.warns(DeprecationWarning):
        resolved = _resolved({"mgrs_grid": "54SUE815501"}, osm)

    verify_origin_matches_map(resolved, osm)


@pytest.mark.xfail(strict=True, reason="nothing compares the origin with the map")
def test_an_origin_from_another_square_is_rejected(tmp_path: Path) -> None:
    """A grid the map's nodes do not fall in must stop the conversion."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])
    with pytest.warns(DeprecationWarning):
        resolved = _resolved({"mgrs_grid": "53SNU"}, osm)

    with pytest.raises(ValueError, match="53SNU"):
        verify_origin_matches_map(resolved, osm)


@pytest.mark.xfail(strict=True, reason="nothing compares the origin with the map")
def test_a_lat_lon_origin_far_from_the_map_is_rejected(tmp_path: Path) -> None:
    """Without a grid the origin itself has to sit near the map."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])
    with pytest.warns(DeprecationWarning):
        resolved = _resolved(
            {"lat_lon": {"latitude": OSAKA[0], "longitude": OSAKA[1]}}, osm
        )

    with pytest.raises(ValueError, match="34.69"):
        verify_origin_matches_map(resolved, osm)


def test_a_lat_lon_origin_inside_the_map_is_accepted(tmp_path: Path) -> None:
    """The ordinary case: the origin is a point of the map."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO, (35.69, 139.70)])
    with pytest.warns(DeprecationWarning):
        resolved = _resolved(
            {"lat_lon": {"latitude": TOKYO[0], "longitude": TOKYO[1]}}, osm
        )

    verify_origin_matches_map(resolved, osm)


def test_a_map_with_no_readable_nodes_is_not_rejected(tmp_path: Path) -> None:
    """The check must not be able to fail a map it could not read.

    A Lanelet2 ``.osm`` that lanelet2 loads is not always well-formed XML to
    a stricter parser, and a map whose nodes cannot be read says nothing
    about the origin either way.
    """
    path = tmp_path / "unreadable.osm"
    path.write_text("<osm><!-- a -- b --></osm>", encoding="utf-8")
    with pytest.warns(DeprecationWarning):
        resolved = _resolved({"mgrs_grid": "53SNU"}, path)

    verify_origin_matches_map(resolved, path)


def test_the_shipped_fixture_passes_its_own_preset() -> None:
    """``walkway_mini`` with ``example_mgrs_offset``'s grid, end to end."""
    fixture = (Path(__file__).parent / "data" / "walkway_mini.osm").resolve()
    with pytest.warns(DeprecationWarning):
        resolved = _resolved({"mgrs_grid": "54SUE"}, fixture)

    verify_origin_matches_map(resolved, fixture)
