"""Tests for the TransverseMercator projector.

Covers resolving a TransverseMercator ``map_projector_info.yaml`` (for both
a UTM-style ``k_0=0.9996`` fixture and a Japan plane rectangular coordinate
system ``k_0=0.9999`` fixture), the exact geoReference PROJ string, a pyproj
round trip, end-to-end ``.osm`` -> ``.xodr`` conversion, that the C++
projector binding actually applies ``scale_factor``, error handling for
invalid ``map_origin``/``scale_factor``, that the MGRS path is unaffected,
and that the cached map-resolution path and the post-conversion analysis
step resolve the same frame through the shared projector-resolution layer.
"""

import logging
import math
import subprocess
from pathlib import Path

import lanelet2
import pytest
from lxml import etree as ET
from omegaconf import OmegaConf
from pyproj import Transformer

from autoware_lanelet2_to_opendrive import projection_resolver
from autoware_lanelet2_to_opendrive.projection_resolver import (
    resolve_projection,
    resolve_projection_from_hydra,
)

TEST_DATA_DIR = Path(__file__).parent / "data"
TMERC_MINI_DIR = TEST_DATA_DIR / "tmerc_mini"
TMERC_MINI_OSM = TMERC_MINI_DIR / "lanelet2_map.osm"
TMERC_MINI_K9999_DIR = TEST_DATA_DIR / "tmerc_mini_k9999"
TMERC_MINI_K9999_OSM = TMERC_MINI_K9999_DIR / "lanelet2_map.osm"

EXPECTED_LAT = 35.61739731
EXPECTED_LON = 139.7797546
EXPECTED_SCALE_FACTOR = 0.9996
EXPECTED_GEO_REFERENCE = (
    "+proj=tmerc +lat_0=35.61739731 +lon_0=139.7797546 +k_0=0.9996 "
    "+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
)

EXPECTED_K9999_SCALE_FACTOR = 0.9999
EXPECTED_K9999_GEO_REFERENCE = (
    "+proj=tmerc +lat_0=35.61739731 +lon_0=139.7797546 +k_0=0.9999 "
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
# 4. scale_factor=0.9999 (Japan plane rectangular coordinate system)
# ---------------------------------------------------------------------------


def test_tm_k9999_resolved_projection_from_projector_info():
    """The tmerc_mini_k9999 fixture resolves with scale_factor=0.9999."""
    resolved = resolve_projection(_cfg(), TMERC_MINI_K9999_OSM)

    assert resolved.projector_type == "TransverseMercator"
    assert resolved.scale_factor == EXPECTED_K9999_SCALE_FACTOR
    assert resolved.origin_lat == EXPECTED_LAT
    assert resolved.origin_lon == EXPECTED_LON
    assert resolved.mgrs_code is None
    assert resolved.offset == (0.0, 0.0, 0.0)


def test_tm_k9999_geo_reference_string_exact():
    resolved = resolve_projection(_cfg(), TMERC_MINI_K9999_OSM)
    assert resolved.geo_reference == EXPECTED_K9999_GEO_REFERENCE


def test_tm_k9999_round_trip_closes_within_tolerance():
    resolved = resolve_projection(_cfg(), TMERC_MINI_K9999_OSM)

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


def test_tm_k9999_conversion_to_xodr_succeeds(tmp_path):
    """A full .osm -> .xodr conversion succeeds for the k=0.9999 fixture.

    ``map=example_mgrs_offset`` is a placeholder Hydra origin config: the
    sibling ``map_projector_info.yaml`` next to the fixture takes priority
    (see :func:`resolve_projection`), so the actual origin/scale_factor used
    is the fixture's, not the placeholder's.
    """
    out = tmp_path / "tmerc_mini_k9999.xodr"

    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=example_mgrs_offset",
            "target=carla",
            f"input_map_path={TMERC_MINI_K9999_OSM}",
            f"output_map_path={out}",
        ],
        check=True,
    )

    tree = ET.parse(str(out))
    geo_ref_elem = tree.find(".//geoReference")
    assert geo_ref_elem is not None and geo_ref_elem.text is not None
    assert geo_ref_elem.text.strip() == EXPECTED_K9999_GEO_REFERENCE
    assert tree.findall(".//road"), "conversion should emit at least one road"


def test_tm_projector_binding_actually_uses_scale_factor():
    """The C++ ``TransverseMercatorProjector`` binding must honor scale_factor.

    The geo-reference tests above only check PROJ-string formatting,
    independent of the Autoware binding, so this exercises the actual C++
    projector via ``make_projector()``. The two fixtures share the same
    origin and differ only in ``scale_factor`` (0.9996 vs 0.9999), isolating
    its effect on a forward-projected point.
    """
    resolved_k9996 = resolve_projection(_cfg(), TMERC_MINI_OSM)
    resolved_k9999 = resolve_projection(_cfg(), TMERC_MINI_K9999_OSM)
    assert resolved_k9996.origin_lat == resolved_k9999.origin_lat
    assert resolved_k9996.origin_lon == resolved_k9999.origin_lon

    projector_k9996 = resolved_k9996.make_projector()
    projector_k9999 = resolved_k9999.make_projector()

    point = lanelet2.core.GPSPoint(EXPECTED_LAT + 0.01, EXPECTED_LON + 0.01, 0.0)
    fwd_k9996 = projector_k9996.forward(point)
    fwd_k9999 = projector_k9999.forward(point)

    dist_k9996 = math.hypot(fwd_k9996.x, fwd_k9996.y)
    dist_k9999 = math.hypot(fwd_k9999.x, fwd_k9999.y)

    # If the binding ignored scale_factor, both distances would be identical.
    assert dist_k9999 != pytest.approx(dist_k9996, rel=1e-6)
    # A larger scale_factor stretches map distances away from the origin, so
    # k=0.9999 must place the point farther from (0, 0) than k=0.9996 does,
    # roughly in proportion to the scale ratio.
    assert dist_k9999 > dist_k9996
    assert dist_k9999 / dist_k9996 == pytest.approx(0.9999 / 0.9996, rel=1e-3)


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
# 5a. Missing scale_factor -> defaults to UTM k0 = 0.9996
# ---------------------------------------------------------------------------


def test_tm_missing_scale_factor_defaults_to_utm_k0(tmp_path):
    """A TM yaml without ``scale_factor`` resolves with ``k=0.9996``.

    Real Autoware TransverseMercator yamls usually omit ``scale_factor``;
    Autoware's projector uses ``GeographicLib::TransverseMercator::UTM()``
    (k0 = 0.9996), so the converter must default to the same value.
    """
    osm = _write_projector_info(
        tmp_path,
        "projector_type: TransverseMercator\n"
        "vertical_datum: WGS84\n"
        "map_origin:\n"
        f"  latitude: {EXPECTED_LAT}\n"
        f"  longitude: {EXPECTED_LON}\n",
    )
    resolved = resolve_projection(_cfg(), osm)

    assert resolved.projector_type == "TransverseMercator"
    assert resolved.scale_factor == EXPECTED_SCALE_FACTOR
    assert resolved.geo_reference == EXPECTED_GEO_REFERENCE
    assert resolved.to_origin_spec().lat == EXPECTED_LAT
    assert resolved.to_origin_spec().lon == EXPECTED_LON


# ---------------------------------------------------------------------------
# 5b. Invalid scale_factor (non-positive or non-finite) -> ValueError
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "scale_factor_yaml",
    [
        "scale_factor: 0\n",
        "scale_factor: -0.9996\n",
        "scale_factor: .nan\n",
    ],
    ids=["zero", "negative", "nan"],
)
def test_tm_invalid_scale_factor_raises(tmp_path, scale_factor_yaml):
    osm = _write_projector_info(
        tmp_path,
        "projector_type: TransverseMercator\n"
        "vertical_datum: WGS84\n"
        "map_origin:\n"
        f"  latitude: {EXPECTED_LAT}\n"
        f"  longitude: {EXPECTED_LON}\n"
        f"{scale_factor_yaml}",
    )
    with pytest.raises(ValueError, match="must be a positive, finite number"):
        resolve_projection(_cfg(), osm)


# ---------------------------------------------------------------------------
# 6. MGRS regression -- additive-contract guard at the API boundary
# ---------------------------------------------------------------------------


def test_no_mgrs_regression():
    """The existing MGRS resolution path is unaffected by TM support."""
    with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
        resolved = resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"}))

    assert resolved.projector_type == "MGRS"
    assert resolved.scale_factor is None
    # Issue #550: the MGRS geoReference is built as an explicit +proj=tmerc
    # (grid-relative false easting/northing), not +proj=utm.
    assert resolved.geo_reference.startswith("+proj=tmerc")


def test_tm_yaml_emits_no_deprecation_or_unsupported_warning(recwarn, caplog):
    """A TransverseMercator yaml is a supported, canonical origin source.

    It must neither trigger the legacy-origin-key deprecation nor the
    "not supported yet" fallback warning, even when explicit keys are set.
    """
    with caplog.at_level(logging.WARNING, logger=projection_resolver.__name__):
        resolved = resolve_projection(_cfg({"mgrs_grid": "54SUE"}), TMERC_MINI_OSM)

    assert resolved.projector_type == "TransverseMercator"
    assert not any(issubclass(w.category, DeprecationWarning) for w in recwarn.list)
    messages = [r.getMessage() for r in caplog.records]
    assert not any("deprecated" in m for m in messages), messages
    assert not any("not supported yet" in m for m in messages), messages


# ---------------------------------------------------------------------------
# 7. map_resolver cache path resolves TransverseMercator maps
# ---------------------------------------------------------------------------


class _StopBeforeConversion(Exception):
    """Sentinel raised from the patched converter to end the test early."""


def _capture_cached_conversion_config(tmp_path, monkeypatch, osm, **kwargs):
    """Run the cached path far enough to capture the config it would convert with.

    The conversion itself is replaced: this is about which frame the cached
    entry point resolves, and converting a map would take minutes. The
    ``/tmp`` cache is redirected into *tmp_path* so the cache-miss path --
    the only one that resolves an origin -- always runs.
    """
    from autoware_lanelet2_to_opendrive import main as main_module
    from autoware_lanelet2_to_opendrive import map_resolver

    monkeypatch.setattr(map_resolver, "_CACHE_DIR", tmp_path / "cache")

    captured = {}

    def _fake_convert(lanelet_map, config):
        captured["config"] = config
        raise _StopBeforeConversion

    monkeypatch.setattr(main_module, "convert_lanelet2_to_opendrive", _fake_convert)

    with pytest.raises(_StopBeforeConversion):
        map_resolver._convert_lanelet2_to_xodr_cached(osm, **kwargs)

    return captured["config"]


@pytest.mark.xfail(strict=True, reason="#547: the cached path rejects TM instead")
def test_map_resolver_uses_tm_projector_info(tmp_path, monkeypatch):
    """The cached path must resolve the frame like the Hydra path does.

    It derived the origin from ``mgrs_code``/``lat``/``lon`` only and never
    read ``map_projector_info.yaml`` (issue #547), so a TransverseMercator
    map had to be rejected outright rather than silently mis-projected as
    MGRS. The origin, scale factor and geoReference handed to the converter
    must come from the projector-info file instead.
    """
    origin = _capture_cached_conversion_config(
        tmp_path, monkeypatch, TMERC_MINI_OSM
    ).origin

    assert origin.lat == EXPECTED_LAT
    assert origin.lon == EXPECTED_LON
    assert origin.scale_factor == EXPECTED_SCALE_FACTOR
    assert origin.geo_reference == EXPECTED_GEO_REFERENCE
    assert origin.mgrs_code is None


@pytest.mark.xfail(
    strict=True, reason="the cached path hands lanelet2.io.load a Path, not a str"
)
def test_map_resolver_without_projector_info_keeps_legacy_origin(tmp_path, monkeypatch):
    """Without a projector-info file the legacy mgrs_code origin still wins.

    Reaching the converter at all is part of what this pins: the cached path
    calls ``lanelet2.io.load(lanelet2_path, projector)`` with a
    :class:`pathlib.Path`, and the Boost.Python binding only accepts a
    ``std::string``, so every cached conversion raises
    ``Boost.Python.ArgumentError`` before the config is ever used. Nothing
    exercised it until now, because the TransverseMercator rejection fired
    first for the only map the suite sent down this path.
    """
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_bytes(TMERC_MINI_OSM.read_bytes())  # no sibling projector info

    origin = _capture_cached_conversion_config(
        tmp_path, monkeypatch, osm, mgrs_code="54SUE"
    ).origin

    assert origin.mgrs_code == "54SUE"
    assert origin.scale_factor is None
    assert origin.geo_reference is None


@pytest.mark.parametrize(
    "contents", ["- projector_type: TransverseMercator\n", "just a string\n"]
)
def test_map_resolver_non_mapping_yaml_raises_value_error(tmp_path, contents):
    """A top-level non-mapping yaml fails with a clear ValueError."""
    from autoware_lanelet2_to_opendrive.map_resolver import (
        _convert_lanelet2_to_xodr_cached,
    )

    osm = _write_projector_info(tmp_path, contents)
    with pytest.raises(ValueError, match="expected a YAML mapping"):
        _convert_lanelet2_to_xodr_cached(osm)


# ---------------------------------------------------------------------------
# 8. The analysis step reloads the OSM in the converter's frame
# ---------------------------------------------------------------------------


@pytest.mark.xfail(strict=True, reason="#547: the analysis step assumes MGRS")
def test_analysis_projection_resolved_from_projector_info():
    """``analyze`` must not assume MGRS when reloading the OSM.

    The cross-validation pass reloads the source OSM and compares the
    resulting lanelet geometry against the emitted roads. Reloading a
    TransverseMercator map with an ``MGRSProjector`` puts the lanelets in a
    rotated, rescaled frame -- at lon 139.78 in UTM zone 54 that is about
    0.7 deg of meridian convergence plus a scale mismatch -- so the
    comparison would be between mismatched frames.
    """
    from autoware_lanelet2_to_opendrive.analyze_xodr import (
        _resolve_analysis_projection,
    )

    # lat_0/lon_0 as they would be parsed from the XODR geoReference.
    resolved = _resolve_analysis_projection(TMERC_MINI_OSM, EXPECTED_LAT, EXPECTED_LON)

    assert resolved.projector_type == "TransverseMercator"
    assert resolved.scale_factor == EXPECTED_SCALE_FACTOR
    assert resolved.origin_lat == EXPECTED_LAT
    assert resolved.origin_lon == EXPECTED_LON

    # The map origin is the frame origin, so its forward projection is (0, 0).
    # An MGRSProjector would put it at the grid square's offset instead.
    fwd = resolved.make_projector().forward(
        lanelet2.core.GPSPoint(EXPECTED_LAT, EXPECTED_LON, 0.0)
    )
    assert fwd.x == pytest.approx(0.0, abs=1e-6)
    assert fwd.y == pytest.approx(0.0, abs=1e-6)
