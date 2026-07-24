"""Tests for the projector-resolution layer (``projection_resolver``).

These lock the behavior extracted from ``main.py`` in the projector-resolution
refactor (issue #540): origin parsing, geoReference generation, and the
coordinate offset are all resolved from one place, with no change to the
previously emitted values.
"""

import pytest
from omegaconf import OmegaConf

from autoware_lanelet2_to_opendrive.conversion_config import OriginSpec
from autoware_lanelet2_to_opendrive.projection import (
    latlon_to_proj_string,
    mgrs_grid_with_offset_to_latlon,
    mgrs_to_proj_string,
)
from autoware_lanelet2_to_opendrive.projection_resolver import (
    ResolvedProjection,
    geo_reference_for_origin,
    resolve_projection,
    resolve_projection_from_hydra,
)


def _cfg(map_dict):
    return OmegaConf.create({"map": map_dict})


# ---------------------------------------------------------------------------
# resolve_projection_from_hydra
# ---------------------------------------------------------------------------


def test_resolve_mgrs_grid_with_offset():
    """MGRS grid + offset resolves origin, offset, and lat/lon geoReference."""
    resolved = resolve_projection_from_hydra(
        _cfg(
            {"mgrs_grid": "54SUE", "offset": {"x": 81655.73, "y": 50137.43, "z": 42.5}}
        )
    )

    assert resolved.mgrs_code == "54SUE"
    assert resolved.offset == (81655.73, 50137.43, 42.5)
    assert resolved.mgrs_offset == (81655.73, 50137.43)

    expected_lat, expected_lon = mgrs_grid_with_offset_to_latlon(
        "54SUE", 81655.73, 50137.43
    )
    assert resolved.origin_lat == expected_lat
    assert resolved.origin_lon == expected_lon

    # geoReference is derived from the offset-adjusted lat/lon.
    assert resolved.geo_reference == latlon_to_proj_string(expected_lat, expected_lon)


def test_resolve_mgrs_grid_without_offset():
    """MGRS grid alone yields a zero offset and grid-corner lat/lon."""
    resolved = resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"}))

    assert resolved.mgrs_code == "54SUE"
    assert resolved.offset == (0.0, 0.0, 0.0)
    expected_lat, expected_lon = mgrs_grid_with_offset_to_latlon("54SUE", 0.0, 0.0)
    assert resolved.origin_lat == expected_lat
    assert resolved.origin_lon == expected_lon


def test_resolve_lat_lon():
    """lat_lon origin is passed through and yields a derived MGRS grid."""
    resolved = resolve_projection_from_hydra(
        _cfg({"lat_lon": {"latitude": 35.6895, "longitude": 139.6917}})
    )

    assert resolved.origin_lat == 35.6895
    assert resolved.origin_lon == 139.6917
    assert resolved.offset == (0.0, 0.0, 0.0)
    # Derived grid zone for Tokyo is 54S...
    assert resolved.mgrs_code.startswith("54S")
    assert resolved.geo_reference == latlon_to_proj_string(35.6895, 139.6917)


def test_resolve_legacy_mgrs_code_alias():
    """Legacy ``mgrs_code`` map field is treated as ``mgrs_grid``."""
    resolved = resolve_projection_from_hydra(_cfg({"mgrs_code": "54SUE"}))
    assert resolved.mgrs_code == "54SUE"


def test_resolve_lat_lon_mgrs_fallback(monkeypatch):
    """If toMGRS returns an unexpected format, fall back to its first 5 chars."""
    import autoware_lanelet2_to_opendrive.projection_resolver as pr

    class _FakeMGRS:
        def toMGRS(self, lat, lon):  # noqa: N802 (mirrors mgrs lib API)
            return "ABCDEFG"  # no leading digits -> the grid-zone regex misses

    monkeypatch.setattr(pr.mgrs_lib, "MGRS", _FakeMGRS)
    resolved = resolve_projection_from_hydra(
        _cfg({"lat_lon": {"latitude": 35.0, "longitude": 139.0}})
    )
    assert resolved.mgrs_code == "ABCDE"


def test_resolve_requires_an_origin():
    with pytest.raises(ValueError, match="Origin must be specified"):
        resolve_projection_from_hydra(_cfg({}))


def test_resolve_rejects_multiple_methods():
    with pytest.raises(ValueError, match="Multiple origin specification"):
        resolve_projection_from_hydra(
            _cfg({"mgrs_grid": "54SUE", "lat_lon": {"latitude": 1.0, "longitude": 2.0}})
        )


def test_resolve_rejects_offset_without_mgrs_grid():
    with pytest.raises(ValueError, match="only be used together with 'mgrs_grid'"):
        resolve_projection_from_hydra(
            _cfg(
                {
                    "lat_lon": {"latitude": 1.0, "longitude": 2.0},
                    "offset": {"x": 1.0, "y": 2.0},
                }
            )
        )


# ---------------------------------------------------------------------------
# geo_reference_for_origin
# ---------------------------------------------------------------------------


def test_geo_reference_prefers_lat_lon():
    spec = OriginSpec(mgrs_code="54SUE", lat=35.6895, lon=139.6917)
    assert geo_reference_for_origin(spec) == latlon_to_proj_string(35.6895, 139.6917)


def test_geo_reference_falls_back_to_mgrs_code():
    spec = OriginSpec(mgrs_code="54SUE")
    assert geo_reference_for_origin(spec) == mgrs_to_proj_string("54SUE")


def test_geo_reference_requires_origin():
    with pytest.raises(ValueError, match="must have lat/lon or mgrs_code"):
        geo_reference_for_origin(OriginSpec())


# ---------------------------------------------------------------------------
# ResolvedProjection helpers
# ---------------------------------------------------------------------------


def test_resolved_projection_helpers():
    resolved = ResolvedProjection(
        origin=resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"})).origin,
        mgrs_code="54SUE",
        origin_lat=35.6895,
        origin_lon=139.6917,
        offset_x=1.0,
        offset_y=2.0,
        offset_z=3.0,
    )

    assert resolved.offset == (1.0, 2.0, 3.0)
    assert resolved.mgrs_offset == (1.0, 2.0)

    spec = resolved.to_origin_spec()
    assert spec == OriginSpec(mgrs_code="54SUE", lat=35.6895, lon=139.6917)

    # make_projector returns a usable MGRSProjector for the resolved origin.
    projector = resolved.make_projector()
    assert projector is not None


# ---------------------------------------------------------------------------
# resolve_projection – map_projector_info.yaml
# ---------------------------------------------------------------------------


def _write_map(tmp_path, *, projector_info: str | None = None):
    """Create a dummy .osm and optional map_projector_info.yaml; return osm path.

    ``resolve_projection`` only reads the projector-info file (not the .osm),
    so the .osm content is irrelevant here.
    """
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text("", encoding="utf-8")
    if projector_info is not None:
        (tmp_path / "map_projector_info.yaml").write_text(
            projector_info, encoding="utf-8"
        )
    return osm


def test_resolve_projection_uses_map_projector_info_mgrs(tmp_path):
    """An MGRS map_projector_info.yaml reproduces the explicit mgrs_grid frame."""
    osm = _write_map(
        tmp_path,
        projector_info="projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 54SUE\n",
    )

    # cfg carries no origin keys – the projector-info file supplies the origin.
    resolved = resolve_projection(_cfg({}), osm)
    explicit = resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"}))

    assert resolved.mgrs_code == explicit.mgrs_code == "54SUE"
    assert resolved.origin_lat == explicit.origin_lat
    assert resolved.origin_lon == explicit.origin_lon
    assert resolved.offset == explicit.offset == (0.0, 0.0, 0.0)
    # Equal geoReference => byte-identical .xodr for this origin.
    assert resolved.geo_reference == explicit.geo_reference


def test_resolve_projection_falls_back_without_info(tmp_path):
    """With no projector-info file, explicit cfg keys drive the frame."""
    osm = _write_map(tmp_path)  # no map_projector_info.yaml
    resolved = resolve_projection(_cfg({"mgrs_grid": "54SUE"}), osm)
    assert resolved.mgrs_code == "54SUE"


def test_map_projector_info_wins_over_explicit_keys(tmp_path):
    """When both are present, the projector-info file is the champion."""
    osm = _write_map(
        tmp_path, projector_info="projector_type: MGRS\nmgrs_grid: 54SUE\n"
    )
    # Explicit keys point somewhere else entirely; they must be ignored.
    resolved = resolve_projection(
        _cfg({"lat_lon": {"latitude": 10.0, "longitude": 20.0}}), osm
    )
    assert resolved.mgrs_code == "54SUE"


def test_resolve_projection_non_mgrs_type_falls_back(tmp_path):
    """A not-yet-supported projector type falls back to explicit keys.

    ``LocalCartesian`` is one of Autoware's three standard projector types
    (MGRS / TransverseMercator / LocalCartesian, see #537) but is not yet
    wired into this converter, making it a good stand-in for "unsupported
    type -> silent fallback" now that TransverseMercator is supported
    (issue #541).
    """
    osm = _write_map(
        tmp_path,
        projector_info=(
            "projector_type: LocalCartesian\n"
            "map_origin:\n  latitude: 35.0\n  longitude: 139.0\n"
        ),
    )
    resolved = resolve_projection(_cfg({"mgrs_grid": "54SUE"}), osm)
    assert resolved.mgrs_code == "54SUE"


def test_map_projector_info_mgrs_without_grid_raises(tmp_path):
    osm = _write_map(tmp_path, projector_info="projector_type: MGRS\n")
    with pytest.raises(ValueError, match="requires a 'mgrs_grid'"):
        resolve_projection(_cfg({}), osm)
