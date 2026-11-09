"""The resolved origin must belong to the map it is resolved for.

Nothing compared the origin against the map. A preset copied to a new map, or
a `map_projector_info.yaml` left behind from another one, resolved without
complaint and the conversion ran to completion: every coordinate written in a
frame that is wrong by whole MGRS squares, with the `geoReference` as the only
symptom, and no gate we own reads it.

The map's node lat/lon is the basis, because it is the one thing always
present -- the `mgrs_code` tag is optional, so it is not consulted.
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


def _resolved(map_dict, map_path: Path, *, check: bool = False):
    """Resolve, with the automatic check off unless a test wants it.

    The tests that exercise :func:`verify_origin_matches_map` directly need a
    resolved frame to hand it, which they cannot get while resolution itself
    rejects the mismatch.
    """
    cfg = OmegaConf.create({"map": dict(map_dict, check_origin_against_map=check)})
    return resolve_projection(cfg, map_path)


# ---------------------------------------------------------------------------
# The check itself
# ---------------------------------------------------------------------------


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


def test_an_origin_from_another_square_is_rejected(tmp_path: Path) -> None:
    """A grid the map's nodes do not fall in must stop the conversion."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])
    with pytest.warns(DeprecationWarning):
        resolved = _resolved({"mgrs_grid": "53SNU"}, osm)

    with pytest.raises(ValueError, match="53SNU"):
        verify_origin_matches_map(resolved, osm)


def test_a_lat_lon_origin_is_checked_through_the_grid_it_resolves_to(
    tmp_path: Path,
) -> None:
    """Under the MGRS projector a ``lat_lon`` origin still yields a grid."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])
    with pytest.warns(DeprecationWarning):
        resolved = _resolved(
            {"lat_lon": {"latitude": OSAKA[0], "longitude": OSAKA[1]}}, osm
        )

    assert resolved.mgrs_code == "53SNU"
    with pytest.raises(ValueError, match="53SNU"):
        verify_origin_matches_map(resolved, osm)


def test_an_origin_without_a_grid_is_checked_on_its_coordinates(
    tmp_path: Path,
) -> None:
    """The Transverse Mercator frame carries no grid, only the origin itself."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])

    def _frame(lat: float, lon: float) -> ResolvedProjection:
        return ResolvedProjection(
            origin=lanelet2.io.Origin(lat, lon),
            mgrs_code=None,
            origin_lat=lat,
            origin_lon=lon,
            scale_factor=0.9996,
            projector_type="TransverseMercator",
        )

    verify_origin_matches_map(_frame(*TOKYO), osm)
    with pytest.raises(ValueError, match="135.5"):
        verify_origin_matches_map(_frame(*OSAKA), osm)


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


# ---------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------


def test_resolution_itself_rejects_a_mismatched_origin(tmp_path: Path) -> None:
    """The check runs during resolution, before anything is converted."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])
    cfg = OmegaConf.create({"map": {"mgrs_grid": "53SNU"}})

    with pytest.warns(DeprecationWarning), pytest.raises(ValueError, match="53SNU"):
        resolve_projection(cfg, osm)


def test_the_check_can_be_turned_off(tmp_path: Path) -> None:
    """`check_origin_against_map=false` resolves the mismatch anyway."""
    osm = _osm(tmp_path, "tokyo.osm", [TOKYO])
    cfg = OmegaConf.create(
        {"map": {"mgrs_grid": "53SNU", "check_origin_against_map": False}}
    )

    with pytest.warns(DeprecationWarning):
        resolved = resolve_projection(cfg, osm)

    assert resolved.mgrs_code == "53SNU"
