"""Typed reader for Autoware's ``map_projector_info.yaml``.

Autoware's ``map_projection_loader`` ships this file next to the Lanelet2
``.osm`` map to declare the projector the map was built with. The known
layouts are::

    projector_type: MGRS
    vertical_datum: WGS84
    mgrs_grid: 54SUE

    projector_type: TransverseMercator      # or LocalCartesianUTM
    vertical_datum: WGS84
    scale_factor: 0.9996                    # TransverseMercator only
    map_origin:
      latitude: 35.6762
      longitude: 139.6503
      altitude: 0.0

    projector_type: Local

:class:`MapProjectorInfo` is the typed view of that file. Loading validates
the *shape* of each field (type, presence of ``latitude``/``longitude`` under
``map_origin``) and raises :class:`ValueError` naming the offending file and
key. Which fields a given ``projector_type`` *requires* is a projector
concern and is checked by the caller that builds the projector.

This module depends only on PyYAML so callers that must stay light can use
it without importing ``lanelet2``.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional

import yaml

#: Autoware ships this file next to the ``.osm`` map to declare the projector.
MAP_PROJECTOR_INFO_FILENAME = "map_projector_info.yaml"


@dataclass(frozen=True)
class MapOrigin:
    """The ``map_origin`` block: a geodetic origin in degrees / metres."""

    latitude: float
    longitude: float
    altitude: float = 0.0


@dataclass(frozen=True)
class MapProjectorInfo:
    """Typed contents of a ``map_projector_info.yaml``.

    Only ``projector_type`` is mandatory; the other fields are present or
    absent depending on the projector. Unknown keys are ignored so newer
    Autoware releases can add fields without breaking this reader.
    """

    projector_type: str
    vertical_datum: Optional[str] = None
    mgrs_grid: Optional[str] = None
    map_origin: Optional[MapOrigin] = None
    scale_factor: Optional[float] = None

    @classmethod
    def from_mapping(
        cls, data: Mapping[str, Any], *, source: str = MAP_PROJECTOR_INFO_FILENAME
    ) -> "MapProjectorInfo":
        """Build from an already-parsed mapping.

        Args:
            data: Parsed YAML content (a mapping).
            source: Label used in error messages, normally the file path.

        Raises:
            ValueError: If a field is missing or has the wrong type.
        """
        projector_type = _required_str(data, "projector_type", source)

        map_origin: Optional[MapOrigin] = None
        raw_origin = data.get("map_origin")
        if raw_origin is not None:
            if not isinstance(raw_origin, Mapping):
                raise ValueError(f"{source}: 'map_origin' must be a mapping")
            origin_source = f"{source}: map_origin"
            altitude = _optional_float(raw_origin, "altitude", origin_source)
            map_origin = MapOrigin(
                latitude=_required_float(raw_origin, "latitude", origin_source),
                longitude=_required_float(raw_origin, "longitude", origin_source),
                altitude=0.0 if altitude is None else altitude,
            )

        return cls(
            projector_type=projector_type,
            vertical_datum=_optional_str(data, "vertical_datum", source),
            mgrs_grid=_optional_str(data, "mgrs_grid", source),
            map_origin=map_origin,
            scale_factor=_optional_float(data, "scale_factor", source),
        )

    @classmethod
    def from_yaml(cls, path: Path) -> "MapProjectorInfo":
        """Load and validate the file at *path*.

        Raises:
            ValueError: If the document is not a mapping, or a field is
                missing or has the wrong type.
            yaml.YAMLError: If the file is not valid YAML.
        """
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if data is None:
            data = {}
        if not isinstance(data, Mapping):
            raise ValueError(f"{path}: expected a YAML mapping at the top level")
        return cls.from_mapping(data, source=str(path))


def _required_str(data: Mapping[str, Any], key: str, source: str) -> str:
    value = _optional_str(data, key, source)
    if value is None:
        raise ValueError(f"{source}: missing required field '{key}'")
    return value


def _optional_str(data: Mapping[str, Any], key: str, source: str) -> Optional[str]:
    value = data.get(key)
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError(
            f"{source}: '{key}' must be a string, got {type(value).__name__}"
        )
    value = value.strip()
    if not value:
        raise ValueError(f"{source}: '{key}' must be a non-empty string")
    return value


def _required_float(data: Mapping[str, Any], key: str, source: str) -> float:
    value = _optional_float(data, key, source)
    if value is None:
        raise ValueError(f"{source}: missing required field '{key}'")
    return value


def _optional_float(data: Mapping[str, Any], key: str, source: str) -> Optional[float]:
    value = data.get(key)
    if value is None:
        return None
    # bool is an int subclass; ``scale_factor: true`` is a mistake, not 1.0.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(
            f"{source}: '{key}' must be a number, got {type(value).__name__}"
        )
    return float(value)
