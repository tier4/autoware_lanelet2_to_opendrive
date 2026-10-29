"""Tests for the projector-resolution layer (``projection_resolver``).

These lock the behavior extracted from ``main.py`` in the projector-resolution
refactor (issue #540): origin parsing, geoReference generation, and the
coordinate offset are all resolved from one place, with no change to the
previously emitted values.
"""

import logging
from pathlib import Path

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


def test_resolve_rejects_the_offset_key():
    """Declaring ``offset`` must fail loudly rather than relocate the map.

    The export offset is gone (#564 follow-up). Ignoring a declared offset
    would move the exported coordinates by its own magnitude -- 82 km for
    ``nishishinjuku.yaml`` -- so the key is rejected instead.
    """
    with pytest.raises(ValueError, match="'offset' is no longer supported"):
        resolve_projection_from_hydra(
            _cfg(
                {
                    "mgrs_grid": "54SUE",
                    "offset": {"x": 81655.73, "y": 50137.43, "z": 42.5},
                }
            )
        )


def test_no_shipped_map_config_declares_an_offset():
    """Guard: the key must not creep back into the bundled map configs."""
    conf_map = Path(projection_resolver.__file__).parent / "conf" / "map"
    # Loaded the way Hydra loads them, so this sees exactly what the resolver
    # would see.
    offenders = []
    for path in sorted(conf_map.glob("*.yaml")):
        map_cfg = OmegaConf.load(path)
        if "offset" in map_cfg and map_cfg.offset is not None:
            offenders.append(path.name)
    assert offenders == [], f"map configs still declaring an offset: {offenders}"


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


def test_resolve_rejects_the_offset_key_with_a_lat_lon_origin():
    """The rejection does not depend on which origin key accompanies it."""
    with pytest.raises(ValueError, match="'offset' is no longer supported"):
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


def _write_map(tmp_path, *, projector_info: str | None = None, nodes=None):
    """Create an .osm and optional map_projector_info.yaml; return the osm path.

    With ``nodes`` (an iterable of ``(lat, lon)``), the .osm is a minimal
    georeferenced document, which is what the OSM-derived frame reads. Without
    it the file is empty and unparseable, so derivation declines and
    resolution falls back to the explicit origin keys -- the case most of
    these tests want.
    """
    osm = tmp_path / "lanelet2_map.osm"
    if nodes is None:
        osm.write_text("", encoding="utf-8")
    else:
        elements = "\n".join(
            f'  <node id="{i + 1}" lat="{lat}" lon="{lon}"/>'
            for i, (lat, lon) in enumerate(nodes)
        )
        osm.write_text(
            f'<?xml version="1.0" encoding="UTF-8"?>\n'
            f'<osm version="0.6">\n{elements}\n</osm>\n',
            encoding="utf-8",
        )
    if projector_info is not None:
        (tmp_path / "map_projector_info.yaml").write_text(
            projector_info, encoding="utf-8"
        )
    return osm


#: Two points inside MGRS grid square 54SUE (Nishishinjuku).
_TOKYO_NODES = [(35.68856, 139.69260), (35.68865, 139.69256)]


def test_resolve_projection_derives_the_grid_from_the_osm_nodes(tmp_path, recwarn):
    """With no projector-info file the frame comes from the map's own nodes.

    Every Lanelet2 OSM node carries ``lat``/``lon``; the per-node
    ``mgrs_code`` tag is optional, so the grid square is derived from the
    coordinates rather than read off a tag. This is a non-deprecated route,
    so no DeprecationWarning fires -- and it needs no origin keys at all.
    """
    osm = _write_map(tmp_path, nodes=_TOKYO_NODES)

    resolved = resolve_projection(_cfg({}), osm)

    assert resolved.projector_type == "MGRS"
    assert resolved.mgrs_code == "54SUE"
    assert resolved.geo_reference == mgrs_to_proj_string("54SUE")
    assert not any(issubclass(w.category, DeprecationWarning) for w in recwarn.list)


def test_osm_derivation_wins_over_the_legacy_origin_keys(tmp_path):
    """The map's own coordinates beat a stale key.

    ``MGRSProjector`` emits coordinates relative to each point's own grid
    square whatever origin it is given, so a key naming a different square
    only ever mislabelled the geoReference.
    """
    osm = _write_map(tmp_path, nodes=_TOKYO_NODES)

    resolved = resolve_projection(_cfg({"mgrs_grid": "53SPU"}), osm)

    assert resolved.mgrs_code == "54SUE"


def test_osm_derivation_declines_a_map_spanning_two_grid_squares(tmp_path, caplog):
    """A map in more than one 100 km square has no single grid origin.

    Derivation declines and says which squares it found, rather than picking
    one; resolution then falls back to the (deprecated) explicit keys.
    """
    osm = _write_map(tmp_path, nodes=[_TOKYO_NODES[0], (35.68856, 140.69260)])

    with caplog.at_level(logging.WARNING, logger=projection_resolver.__name__):
        with pytest.warns(DeprecationWarning, match="map_projector_info.yaml"):
            resolved = resolve_projection(_cfg({"mgrs_grid": "54SUE"}), osm)

    assert resolved.mgrs_code == "54SUE"  # from the key, not derived
    messages = [r.getMessage() for r in caplog.records]
    assert any(
        "54SUE" in m and "54SVE" in m and "grid square" in m for m in messages
    ), messages


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


def test_resolve_rejects_the_offset_key_with_projector_info(tmp_path):
    """The rejection fires on the ``map_projector_info.yaml`` path as well.

    That path never consulted ``offset``, so a map declaring one used to be
    converted 82 km from where its own config said, announced by nothing but
    "explicit origin keys, if any, are ignored".
    """
    osm = _write_map(
        tmp_path, projector_info="projector_type: MGRS\nmgrs_grid: 54SUE\n"
    )
    with pytest.raises(ValueError, match="'offset' is no longer supported"):
        resolve_projection(_cfg({"offset": {"x": 81655.73, "y": 50137.43}}), osm)


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
