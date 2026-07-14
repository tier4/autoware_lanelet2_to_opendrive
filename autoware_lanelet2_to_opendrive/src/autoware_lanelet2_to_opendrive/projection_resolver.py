"""Single projector-resolution layer for Lanelet2 -> OpenDRIVE conversion.

Resolves the map's coordinate frame in one place: origin parsing (from a Hydra
config or an :class:`OriginSpec`), projector construction, geoReference
PROJ-string generation, and the coordinate offset applied during export.

Behavior-preserving extraction of logic that previously lived inline in
``main.py`` (issue #540): the resolved values are identical to before, so the
emitted ``.xodr`` is byte-identical.
"""

import logging
import re
from dataclasses import dataclass
from typing import Optional, Tuple

import lanelet2
import mgrs as mgrs_lib
from autoware_lanelet2_extension_python.projection import MGRSProjector
from omegaconf import DictConfig

from .conversion_config import OriginSpec
from .projection import (
    latlon_to_lanelet2_origin,
    latlon_to_proj_string,
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
    """

    origin: lanelet2.io.Origin
    mgrs_code: Optional[str]
    origin_lat: Optional[float]
    origin_lon: Optional[float]
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0

    def make_projector(self) -> MGRSProjector:
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
