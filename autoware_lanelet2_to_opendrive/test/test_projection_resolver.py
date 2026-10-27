"""Tests for the projector-resolution layer (``projection_resolver``).

These lock the behavior extracted from ``main.py`` in the projector-resolution
refactor (issue #540): origin parsing, geoReference generation, and the
coordinate offset are all resolved from one place, with no change to the
previously emitted values.
"""

import logging

import pytest
from omegaconf import OmegaConf

import autoware_lanelet2_to_opendrive.projection_resolver as projection_resolver

from autoware_lanelet2_to_opendrive.conversion_config import OriginSpec
from autoware_lanelet2_to_opendrive.projection import (
    latlon_to_proj_string,
    mgrs_grid_with_offset_to_latlon,
    mgrs_grid_with_offset_to_proj_string,
    mgrs_to_proj_string,
)
from autoware_lanelet2_to_opendrive.projection_resolver import (
    ResolvedProjection,
    geo_reference_for_origin,
    resolve_projection,
    resolve_projection_from_hydra,
    resolve_projection_from_map_info,
)


@pytest.fixture(autouse=True)
def _reset_legacy_origin_log_guard(monkeypatch):
    """The legacy-key logger.warning fires once per process; reset that guard
    so each test observes it independently of test order."""
    monkeypatch.setattr(
        projection_resolver, "_legacy_origin_keys_logged", False, raising=False
    )


def _cfg(map_dict):
    return OmegaConf.create({"map": map_dict})


# ---------------------------------------------------------------------------
# resolve_projection_from_hydra
# ---------------------------------------------------------------------------


def test_resolve_mgrs_grid_with_offset():
    """MGRS grid + offset resolves origin, offset, and lat/lon geoReference."""
    with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
        resolved = resolve_projection_from_hydra(
            _cfg(
                {
                    "mgrs_grid": "54SUE",
                    "offset": {"x": 81655.73, "y": 50137.43, "z": 42.5},
                }
            )
        )

    assert resolved.mgrs_code == "54SUE"
    assert resolved.offset == (81655.73, 50137.43, 42.5)
    assert resolved.mgrs_offset == (81655.73, 50137.43)

    # origin_lat/origin_lon go through mgrs_grid_with_offset_to_latlon, which
    # truncates the offset to whole meters when round-tripping through a
    # 10-digit MGRS string; they are unaffected by this fix and are only
    # used to build the lanelet2 Origin (which MGRSProjector ignores when
    # computing exported coordinates).
    expected_lat, expected_lon = mgrs_grid_with_offset_to_latlon(
        "54SUE", 81655.73, 50137.43
    )
    assert resolved.origin_lat == expected_lat
    assert resolved.origin_lon == expected_lon

    # geoReference bypasses that truncation: it folds the full-precision
    # offset directly into the grid square's UTM corner (issue #550).
    assert resolved.geo_reference == mgrs_grid_with_offset_to_proj_string(
        "54SUE", 81655.73, 50137.43
    )


def test_resolve_mgrs_grid_without_offset():
    """MGRS grid alone yields a zero offset and grid-corner lat/lon."""
    with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
        resolved = resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"}))

    assert resolved.mgrs_code == "54SUE"
    assert resolved.offset == (0.0, 0.0, 0.0)
    expected_lat, expected_lon = mgrs_grid_with_offset_to_latlon("54SUE", 0.0, 0.0)
    assert resolved.origin_lat == expected_lat
    assert resolved.origin_lon == expected_lon


def test_resolve_lat_lon():
    """lat_lon origin is passed through and yields a derived MGRS grid."""
    with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
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
    with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
        resolved = resolve_projection_from_hydra(_cfg({"mgrs_code": "54SUE"}))
    assert resolved.mgrs_code == "54SUE"


def test_resolve_lat_lon_mgrs_fallback(monkeypatch):
    """If toMGRS returns an unexpected format, fall back to its first 5 chars."""
    import autoware_lanelet2_to_opendrive.projection_resolver as pr

    class _FakeMGRS:
        def toMGRS(self, lat, lon):  # noqa: N802 (mirrors mgrs lib API)
            return "ABCDEFG"  # no leading digits -> the grid-zone regex misses

    monkeypatch.setattr(pr.mgrs_lib, "MGRS", _FakeMGRS)
    with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
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


def test_geo_reference_uses_lat_lon_without_mgrs_code():
    spec = OriginSpec(lat=35.6895, lon=139.6917)
    assert geo_reference_for_origin(spec) == latlon_to_proj_string(35.6895, 139.6917)


def test_geo_reference_prefers_mgrs_code():
    # lat/lon here is the grid square's corner, which sits on a grid
    # boundary; the mgrs_code is authoritative.
    spec = OriginSpec(mgrs_code="54SUE", lat=35.6895, lon=139.6917)
    assert geo_reference_for_origin(spec) == mgrs_to_proj_string("54SUE")


def test_geo_reference_folds_origin_spec_offset():
    spec = OriginSpec(mgrs_code="54SUE", offset_x=81655.73, offset_y=50137.43)
    assert geo_reference_for_origin(spec) == mgrs_grid_with_offset_to_proj_string(
        "54SUE", 81655.73, 50137.43
    )


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


def test_resolve_projection_uses_map_projector_info_mgrs(tmp_path, recwarn):
    """An MGRS map_projector_info.yaml reproduces the explicit mgrs_grid frame."""
    osm = _write_map(
        tmp_path,
        projector_info="projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 54SUE\n",
    )

    # cfg carries no origin keys – the projector-info file supplies the origin.
    # This is the canonical path: no DeprecationWarning should be emitted.
    resolved = resolve_projection(_cfg({}), osm)
    assert not any(issubclass(w.category, DeprecationWarning) for w in recwarn.list)

    with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
        explicit = resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"}))

    assert resolved.mgrs_code == explicit.mgrs_code == "54SUE"
    assert resolved.origin_lat == explicit.origin_lat
    assert resolved.origin_lon == explicit.origin_lon
    assert resolved.offset == explicit.offset == (0.0, 0.0, 0.0)
    # Equal geoReference => byte-identical .xodr for this origin.
    assert resolved.geo_reference == explicit.geo_reference


def test_resolve_projection_falls_back_without_info(tmp_path):
    """With no projector-info file, explicit cfg keys drive the frame (legacy,
    so a DeprecationWarning is expected)."""
    osm = _write_map(tmp_path)  # no map_projector_info.yaml
    with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
        resolved = resolve_projection(_cfg({"mgrs_grid": "54SUE"}), osm)
    assert resolved.mgrs_code == "54SUE"


def test_map_projector_info_wins_over_explicit_keys(tmp_path, recwarn):
    """When both are present, the projector-info file is the champion, and
    since the explicit (legacy) keys are never consulted, no warning fires."""
    osm = _write_map(
        tmp_path, projector_info="projector_type: MGRS\nmgrs_grid: 54SUE\n"
    )
    # Explicit keys point somewhere else entirely; they must be ignored.
    resolved = resolve_projection(
        _cfg({"lat_lon": {"latitude": 10.0, "longitude": 20.0}}), osm
    )
    assert resolved.mgrs_code == "54SUE"
    assert not any(issubclass(w.category, DeprecationWarning) for w in recwarn.list)


@pytest.mark.parametrize("projector_type", ["LocalCartesianUTM", "LocalCartesian"])
def test_resolve_projection_unsupported_type_falls_back_without_deprecation(
    tmp_path, recwarn, caplog, projector_type
):
    """A recognised-but-unsupported projector type falls back to the explicit
    keys, but must NOT tell the user to "provide a map_projector_info.yaml"
    (they already did, and have no non-deprecated way to resolve the origin).
    """
    osm = _write_map(
        tmp_path,
        projector_info=(
            f"projector_type: {projector_type}\n"
            "map_origin:\n  latitude: 35.0\n  longitude: 139.0\n"
        ),
    )
    with caplog.at_level(logging.WARNING, logger=projection_resolver.__name__):
        resolved = resolve_projection(_cfg({"mgrs_grid": "54SUE"}), osm)
    assert resolved.mgrs_code == "54SUE"

    assert not any(issubclass(w.category, DeprecationWarning) for w in recwarn.list)
    messages = [r.getMessage() for r in caplog.records]
    assert not any("deprecated" in m for m in messages), messages
    assert not any("Prefer map_projector_info.yaml" in m for m in messages), messages
    assert any(
        projector_type in m
        and "not supported yet" in m
        and "remain required for this map" in m
        for m in messages
    ), messages


def test_resolve_projection_without_info_emits_deprecation(tmp_path, caplog):
    """With no projector-info file the normal deprecation is still emitted,
    both as a DeprecationWarning and as a logger warning."""
    osm = _write_map(tmp_path)
    with caplog.at_level(logging.WARNING, logger=projection_resolver.__name__):
        with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
            resolve_projection(_cfg({"mgrs_grid": "54SUE"}), osm)
    assert any(
        "deprecated" in r.getMessage() and "map_projector_info.yaml" in r.getMessage()
        for r in caplog.records
    )


def test_legacy_origin_log_warning_fires_once(caplog):
    """The deprecation logger.warning is emitted once per process."""
    with caplog.at_level(logging.WARNING, logger=projection_resolver.__name__):
        with pytest.warns(DeprecationWarning):
            resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"}))
            resolve_projection_from_hydra(_cfg({"mgrs_grid": "54SUE"}))
    deprecations = [r for r in caplog.records if "deprecated" in r.getMessage()]
    assert len(deprecations) == 1


def test_map_projector_info_mgrs_without_grid_raises(tmp_path):
    osm = _write_map(tmp_path, projector_info="projector_type: MGRS\n")
    with pytest.raises(ValueError, match="requires a 'mgrs_grid'"):
        resolve_projection(_cfg({}), osm)


def test_map_projector_info_malformed_field_raises(tmp_path):
    """A wrongly-typed field is rejected at load time, naming the file."""
    osm = _write_map(tmp_path, projector_info="projector_type: MGRS\nmgrs_grid: 54\n")
    with pytest.raises(ValueError, match="'mgrs_grid' must be a string"):
        resolve_projection(_cfg({}), osm)


# ---------------------------------------------------------------------------
# resolve_projection_from_map_info -- resolution without a Hydra config
# ---------------------------------------------------------------------------


def test_resolve_from_map_info_returns_none_without_file(tmp_path):
    """No projector-info file means "nothing declared", not an error.

    Callers that have no Hydra config to fall back on use the ``None`` to
    keep whatever origin they already had.
    """
    assert resolve_projection_from_map_info(_write_map(tmp_path)) is None


def test_resolve_from_map_info_returns_none_for_unsupported_type(tmp_path):
    """An unsupported projector_type is also "nothing this layer can do"."""
    osm = _write_map(tmp_path, projector_info="projector_type: LocalCartesian\n")
    assert resolve_projection_from_map_info(osm) is None


def test_resolve_from_map_info_matches_resolve_projection(tmp_path):
    """The config-free helper and the Hydra entry point resolve one frame.

    This is the property that makes the helper usable by the cached
    map-resolution path and the post-conversion analysis step: what they
    resolve has to be what the converter itself resolved.
    """
    osm = _write_map(
        tmp_path, projector_info="projector_type: MGRS\nmgrs_grid: 54SUE\n"
    )
    from_map_info = resolve_projection_from_map_info(osm)
    from_hydra_entry_point = resolve_projection(_cfg({}), osm)

    assert from_map_info is not None
    assert from_map_info.mgrs_code == from_hydra_entry_point.mgrs_code
    assert from_map_info.origin_lat == from_hydra_entry_point.origin_lat
    assert from_map_info.origin_lon == from_hydra_entry_point.origin_lon
    assert from_map_info.geo_reference == from_hydra_entry_point.geo_reference
