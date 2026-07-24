"""Single projector-resolution layer for Lanelet2 -> OpenDRIVE conversion.

Resolves the map's coordinate frame in one place: origin parsing (from a Hydra
config or an :class:`OriginSpec`), projector construction, geoReference
PROJ-string generation, and the coordinate offset applied during export.

Behavior-preserving extraction of logic that previously lived inline in
``main.py`` (issue #540): the resolved values are identical to before, so the
emitted ``.xodr`` is byte-identical.
"""

import logging
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import lanelet2
import mgrs as mgrs_lib
import yaml
from autoware_lanelet2_extension_python.projection import (
    MGRSProjector,
    TransverseMercatorProjector,
)
from omegaconf import DictConfig

from .conversion_config import OriginSpec
from .projection import (
    latlon_to_lanelet2_origin,
    latlon_to_proj_string,
    latlon_to_tmerc_proj_string,
    mgrs_grid_with_offset_to_lanelet2_origin,
    mgrs_grid_with_offset_to_latlon,
    mgrs_to_lanelet2_origin,
    mgrs_to_proj_string,
)

logger = logging.getLogger(__name__)


def geo_reference_for_origin(origin_spec: OriginSpec) -> str:
    """Build the OpenDRIVE geoReference PROJ string for an origin spec.

    Prefers ``lat``/``lon`` (set for lat/lon origins and for MGRS + offset) and
    falls back to ``mgrs_code`` (the grid square's south-west corner).

    Raises:
        ValueError: If neither lat/lon nor mgrs_code is set.
    """
    if origin_spec.lat is not None and origin_spec.lon is not None:
        return latlon_to_proj_string(origin_spec.lat, origin_spec.lon)
    if origin_spec.mgrs_code is not None:
        return mgrs_to_proj_string(origin_spec.mgrs_code)
    raise ValueError(
        "Cannot generate geoReference: config.origin must have lat/lon "
        "or mgrs_code set."
    )


@dataclass(frozen=True)
class ResolvedProjection:
    """Resolved coordinate frame: origin, projector, offset, and geoReference.

    ``mgrs_code`` and ``origin_lat``/``origin_lon`` may be ``None`` depending on
    how the origin was specified; the offset defaults to zero.

    ``projector_type`` selects the projector :meth:`make_projector` builds and
    how :attr:`geo_reference` is derived; it defaults to ``"MGRS"`` so
    existing call sites keep their prior behavior. ``scale_factor`` only
    applies when ``projector_type == "TransverseMercator"``.
    """

    origin: lanelet2.io.Origin
    mgrs_code: Optional[str]
    origin_lat: Optional[float]
    origin_lon: Optional[float]
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0
    scale_factor: Optional[float] = None
    projector_type: str = "MGRS"

    def make_projector(self):
        if self.projector_type == "TransverseMercator":
            return TransverseMercatorProjector(self.origin, self.scale_factor)
        return MGRSProjector(self.origin)

    @property
    def offset(self) -> Tuple[float, float, float]:
        return (self.offset_x, self.offset_y, self.offset_z)

    @property
    def mgrs_offset(self) -> Tuple[float, float]:
        return (self.offset_x, self.offset_y)

    def to_origin_spec(self) -> OriginSpec:
        return OriginSpec(
            mgrs_code=self.mgrs_code,
            lat=self.origin_lat,
            lon=self.origin_lon,
        )

    @property
    def geo_reference(self) -> str:
        if self.projector_type == "TransverseMercator":
            return latlon_to_tmerc_proj_string(
                self.origin_lat, self.origin_lon, self.scale_factor
            )
        return geo_reference_for_origin(self.to_origin_spec())


def resolve_projection_from_hydra(cfg: DictConfig) -> ResolvedProjection:
    """Resolve the coordinate frame from a Hydra config.

    Supports three mutually exclusive origin specifications (identical to the
    previous ``parse_origin_from_config`` behavior):

    1. ``mgrs_grid``: MGRS grid code (e.g. ``"54SUE"``), optionally combined
       with ``offset`` ``{x, y, z}``.
    2. ``lat_lon``: ``{latitude, longitude, altitude}``.

    The legacy ``mgrs_code`` map field is accepted as an alias for
    ``mgrs_grid``.

    Args:
        cfg: Hydra configuration object with a ``map`` section.

    Returns:
        A :class:`ResolvedProjection`.

    Raises:
        ValueError: If the origin specification is missing, ambiguous, or
            invalid.
    """
    map_cfg = cfg.map

    has_mgrs_grid = "mgrs_grid" in map_cfg and map_cfg.mgrs_grid is not None
    has_mgrs_code = (
        "mgrs_code" in map_cfg and map_cfg.mgrs_code is not None
    )  # Legacy support
    has_offset = "offset" in map_cfg and map_cfg.offset is not None
    has_lat_lon = "lat_lon" in map_cfg and map_cfg.lat_lon is not None

    # Support legacy mgrs_code field
    if has_mgrs_code and not has_mgrs_grid:
        has_mgrs_grid = True
        map_cfg.mgrs_grid = map_cfg.mgrs_code

    # Count how many base methods are specified (offset is an optional
    # modifier for mgrs_grid).
    specified_methods = sum([has_mgrs_grid, has_lat_lon])

    if specified_methods == 0:
        raise ValueError(
            "Origin must be specified using one of: mgrs_grid (with optional "
            "offset), or lat_lon"
        )

    if specified_methods > 1:
        raise ValueError(
            "Multiple origin specification methods detected. "
            "Please specify only one of: mgrs_grid (with optional offset), or "
            "lat_lon"
        )

    # Offset can only be used with mgrs_grid
    if has_offset and not has_mgrs_grid:
        raise ValueError(
            "The 'offset' field can only be used together with 'mgrs_grid'"
        )

    if has_mgrs_grid:
        mgrs_grid = map_cfg.mgrs_grid

        if has_offset:
            offset_cfg = map_cfg.offset
            offset_x = offset_cfg.x
            offset_y = offset_cfg.y
            offset_z = offset_cfg.get("z", 0.0)

            logger.info(
                f"Using MGRS grid with offset: {mgrs_grid}, "
                f"offset x={offset_x} y={offset_y} z={offset_z}"
            )
            origin_lat, origin_lon = mgrs_grid_with_offset_to_latlon(
                mgrs_grid, offset_x, offset_y
            )
            origin = mgrs_grid_with_offset_to_lanelet2_origin(
                mgrs_grid, offset_x, offset_y, offset_z
            )
            logger.info(f"Origin coordinates: lat={origin_lat}, lon={origin_lon}")
            return ResolvedProjection(
                origin=origin,
                mgrs_code=mgrs_grid,
                origin_lat=origin_lat,
                origin_lon=origin_lon,
                offset_x=offset_x,
                offset_y=offset_y,
                offset_z=offset_z,
            )

        logger.info(f"Using MGRS grid origin: {mgrs_grid}")
        origin = mgrs_to_lanelet2_origin(mgrs_grid)
        origin_lat, origin_lon = mgrs_grid_with_offset_to_latlon(mgrs_grid, 0.0, 0.0)
        logger.info(f"Origin coordinates: lat={origin_lat}, lon={origin_lon}")
        return ResolvedProjection(
            origin=origin,
            mgrs_code=mgrs_grid,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
        )

    # has_lat_lon
    latlon_cfg = map_cfg.lat_lon
    latitude = latlon_cfg.latitude
    longitude = latlon_cfg.longitude
    altitude = latlon_cfg.get("altitude", 0.0)

    logger.info(
        f"Using lat/lon origin: latitude={latitude}, longitude={longitude}, "
        f"altitude={altitude}"
    )
    origin = latlon_to_lanelet2_origin(latitude, longitude, altitude)

    # For a lat/lon origin, derive an approximate MGRS grid for the PROJ
    # string by converting lat/lon back to MGRS and keeping the grid zone.
    m = mgrs_lib.MGRS()
    mgrs_code = m.toMGRS(latitude, longitude)
    match = re.match(r"^(\d+[A-Z][A-Z][A-Z])", mgrs_code)
    if match:
        mgrs_grid = match.group(1)
    else:
        mgrs_grid = mgrs_code[:5]  # Fallback to first 5 chars

    logger.info(f"Derived MGRS grid for PROJ string: {mgrs_grid}")
    return ResolvedProjection(
        origin=origin,
        mgrs_code=mgrs_grid,
        origin_lat=latitude,
        origin_lon=longitude,
    )


#: Autoware ships this file next to the ``.osm`` map to declare the projector.
MAP_PROJECTOR_INFO_FILENAME = "map_projector_info.yaml"


def _resolve_transverse_mercator(info_path: Path, data: dict) -> ResolvedProjection:
    """Build a :class:`ResolvedProjection` for ``projector_type: TransverseMercator``.

    Any positive ``scale_factor`` is accepted: the Autoware Python binding
    for ``TransverseMercatorProjector`` accepts an explicit ``scale_factor``
    argument (defaulting to ``0.9996``), so it is threaded straight through
    to the projector.

    Args:
        info_path: Path to the ``map_projector_info.yaml`` file (for error
            messages).
        data: Parsed YAML content of the file.

    Returns:
        A :class:`ResolvedProjection` with ``projector_type="TransverseMercator"``.

    Raises:
        ValueError: If ``map_origin.latitude``/``.longitude`` or
            ``scale_factor`` are missing, or if ``scale_factor`` is not a
            positive, finite number.
    """
    map_origin = data.get("map_origin") or {}
    if "latitude" not in map_origin or "longitude" not in map_origin:
        raise ValueError(
            f"{info_path}: projector_type 'TransverseMercator' requires "
            "'map_origin.latitude' and 'map_origin.longitude' fields"
        )
    origin_lat = float(map_origin["latitude"])
    origin_lon = float(map_origin["longitude"])

    if "scale_factor" not in data:
        raise ValueError(
            f"{info_path}: projector_type 'TransverseMercator' requires a "
            "'scale_factor' field"
        )
    scale_factor = float(data["scale_factor"])
    if not (math.isfinite(scale_factor) and scale_factor > 0):
        raise ValueError(
            f"{info_path}: TransverseMercator scale_factor={scale_factor!r} must be "
            "a positive, finite number"
        )

    origin = lanelet2.io.Origin(origin_lat, origin_lon)
    return ResolvedProjection(
        origin=origin,
        mgrs_code=None,
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        scale_factor=scale_factor,
        projector_type="TransverseMercator",
    )


def _resolve_from_map_projector_info(info_path: Path) -> Optional[ResolvedProjection]:
    """Build a :class:`ResolvedProjection` from a ``map_projector_info.yaml``.

    ``projector_type: MGRS`` and ``projector_type: TransverseMercator`` are
    wired. For an MGRS grid this reproduces the exact projector/geoReference
    of the equivalent explicit ``mgrs_grid`` config, so existing MGRS
    outputs stay byte-identical. Other projector types return ``None`` so
    the caller falls back to the explicit origin keys.

    Args:
        info_path: Path to the ``map_projector_info.yaml`` file.

    Returns:
        A :class:`ResolvedProjection` for MGRS or TransverseMercator, or
        ``None`` for unsupported projector types.

    Raises:
        ValueError: If ``projector_type`` is MGRS but ``mgrs_grid`` is
            missing, or if ``projector_type`` is TransverseMercator but
            ``map_origin``/``scale_factor`` are missing (see
            :func:`_resolve_transverse_mercator`).
    """
    data = yaml.safe_load(info_path.read_text(encoding="utf-8")) or {}
    projector_type = str(data.get("projector_type", "")).strip()

    if projector_type.upper() == "MGRS":
        mgrs_grid = data.get("mgrs_grid")
        if not mgrs_grid:
            raise ValueError(
                f"{info_path}: projector_type 'MGRS' requires a 'mgrs_grid' field"
            )
        origin = mgrs_to_lanelet2_origin(mgrs_grid)
        origin_lat, origin_lon = mgrs_grid_with_offset_to_latlon(mgrs_grid, 0.0, 0.0)
        return ResolvedProjection(
            origin=origin,
            mgrs_code=mgrs_grid,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
        )

    if projector_type == "TransverseMercator":
        return _resolve_transverse_mercator(info_path, data)

    logger.warning(
        "map_projector_info.yaml projector_type=%r is not yet supported; "
        "falling back to explicit origin keys",
        projector_type,
    )
    return None


def resolve_projection(cfg: DictConfig, input_map_path: Path) -> ResolvedProjection:
    """Resolve the coordinate frame, preferring ``map_projector_info.yaml``.

    When that file sits next to ``input_map_path`` and declares a supported
    projector, it drives the frame and any explicit ``cfg.map`` origin keys
    are ignored. Otherwise resolution falls back to the explicit-key path
    (:func:`resolve_projection_from_hydra`).

    Args:
        cfg: Hydra configuration object with a ``map`` section.
        input_map_path: Path to the input ``.osm`` map. The
            ``map_projector_info.yaml`` is looked up in its directory.

    Returns:
        A :class:`ResolvedProjection`.
    """
    info_path = Path(input_map_path).parent / MAP_PROJECTOR_INFO_FILENAME
    if info_path.is_file():
        resolved = _resolve_from_map_projector_info(info_path)
        if resolved is not None:
            logger.info(
                "Using %s as the canonical origin source (explicit origin keys, "
                "if any, are ignored)",
                info_path,
            )
            return resolved

    return resolve_projection_from_hydra(cfg)
