"""Tests for the typed ``map_projector_info.yaml`` reader.

These exercise only :mod:`autoware_lanelet2_to_opendrive.map_projector_info`,
which depends on PyYAML alone, so they need neither ``lanelet2`` nor the
Autoware extension bindings.
"""

from pathlib import Path

import pytest
import yaml  # type: ignore[import-untyped]

from autoware_lanelet2_to_opendrive.map_projector_info import (
    MAP_PROJECTOR_INFO_FILENAME,
    MapOrigin,
    MapProjectorInfo,
)


def _write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / MAP_PROJECTOR_INFO_FILENAME
    path.write_text(text, encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# Happy paths: the three layouts Autoware's map_projection_loader emits
# ---------------------------------------------------------------------------


def test_mgrs_layout(tmp_path):
    info = MapProjectorInfo.from_yaml(
        _write(
            tmp_path, "projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 54SUE\n"
        )
    )
    assert info == MapProjectorInfo(
        projector_type="MGRS", vertical_datum="WGS84", mgrs_grid="54SUE"
    )
    assert info.map_origin is None
    assert info.scale_factor is None


def test_transverse_mercator_layout(tmp_path):
    info = MapProjectorInfo.from_yaml(
        _write(
            tmp_path,
            "projector_type: TransverseMercator\n"
            "vertical_datum: WGS84\n"
            "scale_factor: 0.9996\n"
            "map_origin:\n"
            "  latitude: 35.6762\n"
            "  longitude: 139.6503\n"
            "  altitude: 12.5\n",
        )
    )
    assert info.projector_type == "TransverseMercator"
    assert info.scale_factor == pytest.approx(0.9996)
    assert info.map_origin == MapOrigin(
        latitude=35.6762, longitude=139.6503, altitude=12.5
    )
    assert info.mgrs_grid is None


def test_local_layout_has_only_a_type(tmp_path):
    info = MapProjectorInfo.from_yaml(_write(tmp_path, "projector_type: Local\n"))
    assert info == MapProjectorInfo(projector_type="Local")


def test_altitude_defaults_to_zero_and_ints_become_floats(tmp_path):
    info = MapProjectorInfo.from_yaml(
        _write(
            tmp_path,
            "projector_type: LocalCartesianUTM\n"
            "map_origin:\n  latitude: 35\n  longitude: 139\n",
        )
    )
    assert info.map_origin == MapOrigin(latitude=35.0, longitude=139.0, altitude=0.0)
    assert isinstance(info.map_origin.latitude, float)


def test_unknown_keys_are_ignored(tmp_path):
    """Newer Autoware releases may add fields; the reader must not break."""
    info = MapProjectorInfo.from_yaml(
        _write(tmp_path, "projector_type: MGRS\nmgrs_grid: 54SUE\nfuture_field: 1\n")
    )
    assert info.mgrs_grid == "54SUE"


def test_whitespace_is_stripped(tmp_path):
    info = MapProjectorInfo.from_yaml(
        _write(tmp_path, "projector_type: '  MGRS '\nmgrs_grid: ' 54SUE '\n")
    )
    assert info.projector_type == "MGRS"
    assert info.mgrs_grid == "54SUE"


def test_is_frozen(tmp_path):
    info = MapProjectorInfo.from_yaml(_write(tmp_path, "projector_type: MGRS\n"))
    with pytest.raises(AttributeError):
        info.projector_type = "Local"  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Malformed input: every error names the file and the offending key
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("text", "match"),
    [
        ("", "missing required field 'projector_type'"),
        ("vertical_datum: WGS84\n", "missing required field 'projector_type'"),
        ("projector_type: 42\n", "'projector_type' must be a string, got int"),
        ("projector_type: ''\n", "'projector_type' must be a non-empty string"),
        ("projector_type: MGRS\nmgrs_grid: 54\n", "'mgrs_grid' must be a string"),
        ("projector_type: MGRS\nmgrs_grid: ''\n", "'mgrs_grid' must be a non-empty"),
        ("projector_type: MGRS\nvertical_datum: [a]\n", "'vertical_datum' must be"),
        ("projector_type: TM\nscale_factor: fast\n", "'scale_factor' must be a number"),
        ("projector_type: TM\nscale_factor: true\n", "'scale_factor' must be a number"),
        ("projector_type: TM\nmap_origin: 35,139\n", "'map_origin' must be a mapping"),
        (
            "projector_type: TM\nmap_origin:\n  longitude: 139.0\n",
            "map_origin: missing required field 'latitude'",
        ),
        (
            "projector_type: TM\nmap_origin:\n  latitude: 35.0\n",
            "map_origin: missing required field 'longitude'",
        ),
        (
            "projector_type: TM\nmap_origin:\n  latitude: north\n  longitude: 139.0\n",
            "map_origin: 'latitude' must be a number, got str",
        ),
        ("- projector_type: MGRS\n", "expected a YAML mapping at the top level"),
    ],
)
def test_malformed_input_raises_value_error(tmp_path, text, match):
    path = _write(tmp_path, text)
    with pytest.raises(ValueError, match=match) as excinfo:
        MapProjectorInfo.from_yaml(path)
    # The message always starts with the file so the user knows which map.
    assert str(excinfo.value).startswith(str(path))


def test_invalid_yaml_propagates_yaml_error(tmp_path):
    path = _write(tmp_path, "projector_type: [unterminated\n")
    with pytest.raises(yaml.YAMLError):
        MapProjectorInfo.from_yaml(path)


def test_from_mapping_uses_default_source_label():
    with pytest.raises(ValueError, match=f"^{MAP_PROJECTOR_INFO_FILENAME}: "):
        MapProjectorInfo.from_mapping({})
