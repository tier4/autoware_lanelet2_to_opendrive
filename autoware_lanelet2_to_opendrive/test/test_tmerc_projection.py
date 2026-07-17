"""Tests for the TransverseMercator projector (issue #541).

Autoware's Python binding for ``TransverseMercatorProjector`` does not expose
``scale_factor`` -- it is locked at ``k=0.9996`` in C++. These tests lock:

* Resolution of a TransverseMercator ``map_projector_info.yaml`` into a
  :class:`ResolvedProjection`.
* The exact PROJ geoReference string produced for it (contractual).
* A pyproj round trip through that PROJ string, as an independent check that
  the string is a well-formed, self-consistent Transverse Mercator
  definition.
* Rejection of unsupported ``scale_factor`` values (pointing at follow-up
  issue #548) and of a missing ``map_origin``.
* That the MGRS resolution path is unaffected (additive-contract guard).
* That the cached map-resolution path (``map_resolver``) refuses a
  TransverseMercator map rather than silently mis-projecting it.
"""

from pathlib import Path

import pytest
from omegaconf import OmegaConf
from pyproj import Transformer

from autoware_lanelet2_to_opendrive.projection_resolver import (
    resolve_projection,
    resolve_projection_from_hydra,
)

TEST_DATA_DIR = Path(__file__).parent / "data"
TMERC_MINI_DIR = TEST_DATA_DIR / "tmerc_mini"
TMERC_MINI_OSM = TMERC_MINI_DIR / "lanelet2_map.osm"

EXPECTED_LAT = 35.61739731
EXPECTED_LON = 139.7797546
EXPECTED_SCALE_FACTOR = 0.9996
EXPECTED_GEO_REFERENCE = (
    "+proj=tmerc +lat_0=35.61739731 +lon_0=139.7797546 +k=0.9996 "
    "+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
)


def _cfg(map_dict=None):
    return OmegaConf.create({"map": map_dict or {}})


def _write_projector_info(tmp_path: Path, contents: str) -> Path:
    """Write a ``map_projector_info.yaml`` + dummy sibling ``.osm``; return the osm path."""
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text("", encoding="utf-8")
    (tmp_path / "map_projector_info.yaml").write_text(contents, encoding="utf-8")
    return osm


# ---------------------------------------------------------------------------
# 1. Resolution from map_projector_info.yaml
# ---------------------------------------------------------------------------


def test_tm_resolved_projection_from_projector_info():
    """The tmerc_mini fixture resolves to a TransverseMercator projection."""
    resolved = resolve_projection(_cfg(), TMERC_MINI_OSM)

    assert resolved.projector_type == "TransverseMercator"
    assert resolved.scale_factor == EXPECTED_SCALE_FACTOR
    assert resolved.origin_lat == EXPECTED_LAT
    assert resolved.origin_lon == EXPECTED_LON
    assert resolved.mgrs_code is None
    assert resolved.offset == (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# 2. geoReference PROJ string -- exact match (contractual)
# ---------------------------------------------------------------------------


def test_tm_geo_reference_string_exact():
    resolved = resolve_projection(_cfg(), TMERC_MINI_OSM)
    assert resolved.geo_reference == EXPECTED_GEO_REFERENCE


# ---------------------------------------------------------------------------
# 3. pyproj round trip through the generated PROJ string
# ---------------------------------------------------------------------------


def test_tm_round_trip_closes_within_tolerance():
    resolved = resolve_projection(_cfg(), TMERC_MINI_OSM)

    forward = Transformer.from_crs("EPSG:4326", resolved.geo_reference, always_xy=True)
    inverse = Transformer.from_crs(resolved.geo_reference, "EPSG:4326", always_xy=True)

    lon_in = EXPECTED_LON + 0.01
    lat_in = EXPECTED_LAT + 0.01

    x, y = forward.transform(lon_in, lat_in)
    lon_out, lat_out = inverse.transform(x, y)

    assert lat_out == pytest.approx(lat_in, abs=1e-9)
    assert lon_out == pytest.approx(lon_in, abs=1e-9)

    # Re-project the round-tripped lat/lon and confirm the xy also closes.
    x2, y2 = forward.transform(lon_out, lat_out)
    assert x2 == pytest.approx(x, abs=1e-6)
    assert y2 == pytest.approx(y, abs=1e-6)


# ---------------------------------------------------------------------------
# 4. Unsupported scale_factor -> ValueError citing #548
# ---------------------------------------------------------------------------


def test_tm_unsupported_scale_factor_raises(tmp_path):
    osm = _write_projector_info(
        tmp_path,
        "projector_type: TransverseMercator\n"
        "vertical_datum: WGS84\n"
        "map_origin:\n"
        f"  latitude: {EXPECTED_LAT}\n"
        f"  longitude: {EXPECTED_LON}\n"
        "scale_factor: 0.9999\n",
    )
    with pytest.raises(ValueError, match="#548"):
        resolve_projection(_cfg(), osm)


# ---------------------------------------------------------------------------
# 5. Missing map_origin -> ValueError
# ---------------------------------------------------------------------------


def test_tm_missing_map_origin_raises(tmp_path):
    osm = _write_projector_info(
        tmp_path,
        "projector_type: TransverseMercator\n"
        "vertical_datum: WGS84\n"
        "scale_factor: 0.9996\n",
    )
    with pytest.raises(ValueError, match="map_origin"):
        resolve_projection(_cfg(), osm)


# ---------------------------------------------------------------------------
# 6. MGRS regression -- additive-contract guard at the API boundary
# ---------------------------------------------------------------------------


def test_no_mgrs_regression():
    """The existing MGRS resolution path is unaffected by TM support."""
    resolved = resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"}))

    assert resolved.projector_type == "MGRS"
    assert resolved.scale_factor is None
    assert resolved.geo_reference.startswith("+proj=utm")


# ---------------------------------------------------------------------------
# 7. map_resolver cache path rejects TransverseMercator maps
# ---------------------------------------------------------------------------


def test_map_resolver_rejects_tm_yaml(tmp_path):
    from autoware_lanelet2_to_opendrive.map_resolver import (
        _convert_lanelet2_to_xodr_cached,
    )

    osm = _write_projector_info(
        tmp_path,
        "projector_type: TransverseMercator\n"
        "vertical_datum: WGS84\n"
        "map_origin:\n"
        f"  latitude: {EXPECTED_LAT}\n"
        f"  longitude: {EXPECTED_LON}\n"
        "scale_factor: 0.9996\n",
    )
    with pytest.raises(RuntimeError, match="#541"):
        _convert_lanelet2_to_xodr_cached(osm)
