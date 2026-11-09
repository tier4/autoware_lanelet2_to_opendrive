"""A declared coordinate frame must belong to the map it is declared for.

Resolution has two steps: a `map_projector_info.yaml` next to the map, or --
with no such file -- a frame derived from the map's own node coordinates. The
derived frame cannot be wrong about the map. The file can: it is a file next
to the map, and a copy left behind from another one resolved without
complaint, after which the conversion completed with every coordinate in the
wrong frame and the `geoReference` as the only symptom. Nothing read it.

The node lat/lon is the basis, because it is the one thing always present --
the `mgrs_code` tag is optional, so it is not consulted.
"""

from pathlib import Path

import lanelet2
import pytest
from omegaconf import OmegaConf

from autoware_lanelet2_to_opendrive.projection_resolver import (
    ResolvedProjection,
    resolve_projection,
    verify_origin_matches_map,
)

TOKYO = (35.68856, 139.69260)
"""A node of the ``walkway_mini`` fixture: MGRS square 54SUE."""

OSAKA = (34.69, 135.50)
"""MGRS square 53SNU -- a different zone, so no square can match."""


def _osm(tmp_path: Path, points) -> Path:
    nodes = "\n".join(
        f'  <node id="{index + 1}" lat="{lat}" lon="{lon}"/>'
        for index, (lat, lon) in enumerate(points)
    )
    path = tmp_path / "map.osm"
    path.write_text(
        f'<?xml version="1.0"?>\n<osm version="0.6">\n{nodes}\n</osm>\n',
        encoding="utf-8",
    )
    return path


def _projector_info(tmp_path: Path, body: str) -> None:
    (tmp_path / "map_projector_info.yaml").write_text(body, encoding="utf-8")


def _mgrs_frame(grid: str) -> ResolvedProjection:
    return ResolvedProjection(
        origin=lanelet2.io.Origin(0.0, 0.0),
        mgrs_code=grid,
        origin_lat=None,
        origin_lon=None,
    )


def _tmerc_frame(lat: float, lon: float) -> ResolvedProjection:
    return ResolvedProjection(
        origin=lanelet2.io.Origin(lat, lon),
        mgrs_code=None,
        origin_lat=lat,
        origin_lon=lon,
        scale_factor=0.9996,
        projector_type="TransverseMercator",
    )


# ---------------------------------------------------------------------------
# The check itself
# ---------------------------------------------------------------------------


def test_a_grid_the_map_lies_in_is_accepted(tmp_path: Path) -> None:
    osm = _osm(tmp_path, [TOKYO])

    verify_origin_matches_map(_mgrs_frame("54SUE"), osm)


def test_a_higher_precision_grid_is_compared_as_a_square(tmp_path: Path) -> None:
    """A code like ``54SUE815501`` names the square ``54SUE``."""
    osm = _osm(tmp_path, [TOKYO])

    verify_origin_matches_map(_mgrs_frame("54SUE815501"), osm)


def test_a_grid_from_another_square_is_rejected(tmp_path: Path) -> None:
    osm = _osm(tmp_path, [TOKYO])

    with pytest.raises(ValueError, match="53SNU"):
        verify_origin_matches_map(_mgrs_frame("53SNU"), osm)


def test_a_frame_without_a_grid_is_checked_on_its_coordinates(
    tmp_path: Path,
) -> None:
    """Transverse Mercator carries no square, only the origin itself."""
    osm = _osm(tmp_path, [TOKYO])

    verify_origin_matches_map(_tmerc_frame(*TOKYO), osm)
    with pytest.raises(ValueError, match="135.5"):
        verify_origin_matches_map(_tmerc_frame(*OSAKA), osm)


def test_a_map_with_no_readable_nodes_is_not_rejected(tmp_path: Path) -> None:
    """The check must not be able to fail a map it could not read.

    A map whose nodes cannot be read says nothing about the frame either
    way, and the loader is more tolerant than a strict XML parser.
    """
    path = tmp_path / "unreadable.osm"
    path.write_text("<osm></osm>", encoding="utf-8")

    verify_origin_matches_map(_mgrs_frame("53SNU"), path)
    verify_origin_matches_map(_tmerc_frame(*OSAKA), path)


def test_the_shipped_fixture_lies_in_the_square_it_resolves_to() -> None:
    """``walkway_mini`` against the square its own nodes derive."""
    fixture = (Path(__file__).parent / "data" / "walkway_mini.osm").resolve()

    verify_origin_matches_map(_mgrs_frame("54SUE"), fixture)


# ---------------------------------------------------------------------------
# Wiring: the projector-info path is the one that can be wrong
# ---------------------------------------------------------------------------


def test_resolution_rejects_a_projector_info_from_another_map(
    tmp_path: Path,
) -> None:
    """The check runs during resolution, before anything is converted."""
    osm = _osm(tmp_path, [TOKYO])
    _projector_info(
        tmp_path, "projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 53SNU\n"
    )
    cfg = OmegaConf.create({"map": {}})

    with pytest.raises(ValueError, match="53SNU"):
        resolve_projection(cfg, osm)


def test_a_projector_info_for_this_map_resolves(tmp_path: Path) -> None:
    """The ordinary case must be untouched."""
    osm = _osm(tmp_path, [TOKYO])
    _projector_info(
        tmp_path, "projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 54SUE\n"
    )
    cfg = OmegaConf.create({"map": {}})

    assert resolve_projection(cfg, osm).mgrs_code == "54SUE"


def test_the_check_can_be_turned_off(tmp_path: Path) -> None:
    """`check_origin_against_map=false` resolves the mismatch anyway."""
    osm = _osm(tmp_path, [TOKYO])
    _projector_info(
        tmp_path, "projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 53SNU\n"
    )
    cfg = OmegaConf.create({"map": {"check_origin_against_map": False}})

    assert resolve_projection(cfg, osm).mgrs_code == "53SNU"


def test_a_frame_derived_from_the_map_is_not_re_checked(tmp_path: Path) -> None:
    """With no projector-info file there is nothing to disagree with."""
    osm = _osm(tmp_path, [TOKYO])
    cfg = OmegaConf.create({"map": {}})

    assert resolve_projection(cfg, osm).mgrs_code == "54SUE"
