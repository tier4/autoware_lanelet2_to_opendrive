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
    mgrs_to_lanelet2_origin,
    mgrs_to_proj_string,
)
from autoware_lanelet2_to_opendrive.projection_resolver import (
    ResolvedProjection,
    geo_reference_for_origin,
    reject_removed_map_keys,
    resolve_projection,
    resolve_projection_from_osm,
)


def _cfg(map_dict):
    return OmegaConf.create({"map": map_dict})


# ---------------------------------------------------------------------------
# reject_removed_map_keys
# ---------------------------------------------------------------------------


_ORIGIN_KEYS = ("mgrs_grid", "mgrs_code", "lat_lon")


def test_no_shipped_map_config_declares_an_origin_key():
    """No bundled config may declare an origin -- the keys are gone.

    Not even as documentation: a config that declares one is now rejected,
    so an example carrying it would be an example that does not run.
    """
    conf_map = Path(projection_resolver.__file__).parent / "conf" / "map"
    offenders = []
    for path in sorted(conf_map.glob("*.yaml")):
        map_cfg = OmegaConf.load(path)
        if any(k in map_cfg and map_cfg[k] is not None for k in _ORIGIN_KEYS):
            offenders.append(path.name)
    assert offenders == [], f"map configs still declaring an origin: {offenders}"


@pytest.mark.parametrize(
    "removed",
    [
        {"mgrs_grid": "54SUE"},
        {"mgrs_code": "54SUE"},
        {"lat_lon": {"latitude": 35.0, "longitude": 139.0}},
        {"offset": {"x": 1.0, "y": 2.0}},
    ],
)
def test_removed_map_keys_are_rejected(removed):
    """Every key that used to declare the frame is now an error.

    Ignoring them would be worse than refusing them: a config naming a grid
    square the map is not in would silently keep converting, with the
    geoReference it names no longer having any effect.
    """
    with pytest.raises(ValueError, match="no longer supported"):
        reject_removed_map_keys(OmegaConf.create(removed))


def test_reject_removed_map_keys_accepts_a_modern_config():
    """A config with operations and a traffic rule and no origin is fine."""
    reject_removed_map_keys(
        OmegaConf.create({"traffic_rule": "LHT", "merge_operations": None})
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
        origin=mgrs_to_lanelet2_origin("54SUE"),
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


def test_osm_derivation_tolerates_a_double_hyphen_comment(tmp_path):
    """The derivation parser must not be stricter than the map loader.

    ``walkway_mini.osm`` carries a ``--`` inside an XML comment: lanelet2's
    loader accepts it and strict lxml rejects it as "Double hyphen within
    comment". A map the converter can convert must have a derivable frame,
    so the parser recovers instead of giving up.
    """
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<!--
  Synthetic fixture -- note the double hyphen
-->
<osm version="0.6">
  <node id="1" lat="35.68856" lon="139.69260"/>
</osm>
""",
        encoding="utf-8",
    )

    resolved = resolve_projection_from_osm(osm)

    assert resolved is not None, "a loadable map must have a derivable frame"
    assert resolved.mgrs_code == "54SUE"


def test_osm_derivation_declines_a_map_spanning_two_grid_squares(tmp_path, caplog):
    """A map in more than one 100 km square has no single grid origin.

    Derivation declines and says which squares it found, rather than picking
    one. With no origin keys left to fall back to, resolution then fails --
    and the log is the only place that says why, so it is asserted here.
    """
    osm = _write_map(tmp_path, nodes=[_TOKYO_NODES[0], (35.68856, 140.69260)])

    with caplog.at_level(logging.WARNING, logger=projection_resolver.__name__):
        with pytest.raises(ValueError, match="Cannot determine the coordinate frame"):
            resolve_projection(_cfg({}), osm)

    messages = [r.getMessage() for r in caplog.records]
    assert any(
        "54SUE" in m and "54SVE" in m and "grid square" in m for m in messages
    ), messages


def test_resolve_projection_uses_map_projector_info_mgrs(tmp_path, recwarn):
    """An MGRS map_projector_info.yaml resolves to the grid square it names."""
    osm = _write_map(
        tmp_path,
        projector_info="projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 54SUE\n",
    )

    resolved = resolve_projection(_cfg({}), osm)
    assert not any(issubclass(w.category, DeprecationWarning) for w in recwarn.list)

    # Compared against the projection helpers, not against a second
    # resolution path: for an MGRS grid square there is only one left.
    expected_lat, expected_lon = mgrs_grid_with_offset_to_latlon("54SUE", 0.0, 0.0)
    assert resolved.mgrs_code == "54SUE"
    assert resolved.origin_lat == expected_lat
    assert resolved.origin_lon == expected_lon
    assert resolved.offset == (0.0, 0.0, 0.0)
    # Equal geoReference => byte-identical .xodr for this origin.
    assert resolved.geo_reference == mgrs_to_proj_string("54SUE")


def test_resolve_projection_rejects_a_removed_key_before_resolving(tmp_path):
    """A sidecar projector-info file does not excuse a removed key.

    The check runs before the branch, so a map that would otherwise resolve
    fine still reports the stale key, rather than converting and leaving the
    user to wonder why the key had no effect.
    """
    osm = _write_map(
        tmp_path, projector_info="projector_type: MGRS\nmgrs_grid: 54SUE\n"
    )
    with pytest.raises(ValueError, match="'offset' is no longer supported"):
        resolve_projection(_cfg({"offset": {"x": 81655.73, "y": 50137.43}}), osm)


def test_resolve_projection_raises_when_nothing_can_supply_the_frame(tmp_path):
    """No projector-info file and no usable coordinates is an error.

    There is no third source left, so this reports the two it tried instead
    of resolving something arbitrary.
    """
    osm = _write_map(tmp_path)  # empty, unparseable, no projector-info file
    with pytest.raises(ValueError, match="Cannot determine the coordinate frame"):
        resolve_projection(_cfg({}), osm)


@pytest.mark.parametrize("projector_type", ["LocalCartesianUTM", "LocalCartesian"])
def test_resolve_projection_unsupported_type_raises(tmp_path, projector_type):
    """A recognised-but-unsupported projector type is now an error.

    Deriving an MGRS frame instead would export the map in a projection it
    does not declare, and there are no origin keys left to fall back to.
    The message names the type so the user knows what is missing, rather
    than reporting an absent origin.
    """
    osm = _write_map(
        tmp_path,
        nodes=_TOKYO_NODES,
        projector_info=(
            f"projector_type: {projector_type}\n"
            "map_origin:\n  latitude: 35.0\n  longitude: 139.0\n"
        ),
    )
    with pytest.raises(ValueError, match=f"{projector_type}.*not supported"):
        resolve_projection(_cfg({}), osm)


def test_map_projector_info_mgrs_without_grid_raises(tmp_path):
    osm = _write_map(tmp_path, projector_info="projector_type: MGRS\n")
    with pytest.raises(ValueError, match="requires a 'mgrs_grid'"):
        resolve_projection(_cfg({}), osm)


def test_map_projector_info_malformed_field_raises(tmp_path):
    """A wrongly-typed field is rejected at load time, naming the file."""
    osm = _write_map(tmp_path, projector_info="projector_type: MGRS\nmgrs_grid: 54\n")
    with pytest.raises(ValueError, match="'mgrs_grid' must be a string"):
        resolve_projection(_cfg({}), osm)
