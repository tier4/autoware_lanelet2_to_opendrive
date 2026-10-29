"""Single projector-resolution layer for Lanelet2 -> OpenDRIVE conversion.

Resolves the map's coordinate frame in one place: the origin (from a
``map_projector_info.yaml``, the map's own node coordinates, or an
:class:`OriginSpec` built by a library caller), projector construction and
geoReference PROJ-string generation.

A map config declares no frame at all. The ``mgrs_grid`` / ``mgrs_code`` /
``lat_lon`` / ``offset`` keys that used to are removed, and are rejected if
still present (:func:`reject_removed_map_keys`, #564). Exported coordinates
are always expressed in the frame the projector itself defines.

The ``offset_x``/``offset_y``/``offset_z`` fields of
:class:`ResolvedProjection` remain, at zero, for library callers that build
an :class:`OriginSpec` directly and for reading back the geoReference of
previously exported files.
"""

import logging
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import lanelet2
import lxml.etree as ET
import mgrs as mgrs_lib
from autoware_lanelet2_extension_python.projection import (
    MGRSProjector,
    TransverseMercatorProjector,
)
from omegaconf import DictConfig

from .config import DEFAULT_CONFIG
from .conversion_config import OriginSpec
from .map_projector_info import MAP_PROJECTOR_INFO_FILENAME, MapProjectorInfo
from .projection import (
    latlon_to_proj_string,
    latlon_to_tmerc_proj_string,
    mgrs_grid_with_offset_to_latlon,
    mgrs_grid_with_offset_to_proj_string,
    mgrs_to_lanelet2_origin,
    mgrs_to_proj_string,
)

logger = logging.getLogger(__name__)

#: ``projector_type`` values are matched case-sensitively, exactly as
#: Autoware's ``map_projection_loader`` writes and compares them.
_MGRS = "MGRS"
_TRANSVERSE_MERCATOR = "TransverseMercator"

#: An MGRS code's leading 100 km grid square designator, e.g. ``"54SUE"``.
_GRID_SQUARE_RE = re.compile(r"^(\d+[A-Z]{3})")


def _grid_square(mgrs_code: str) -> str:
    """Return the 100 km grid square designator of an MGRS code.

    Falls back to the code's first five characters when it does not start
    with a zone number plus three letters, which is the behaviour the
    ``lat_lon`` origin branch has always had.
    """
    match = _GRID_SQUARE_RE.match(mgrs_code)
    return match.group(1) if match else mgrs_code[:5]


#: The map config keys that used to declare the coordinate frame, and what
#: replaced each one. They are rejected rather than ignored: a config naming
#: a grid square the map is not in would otherwise keep converting while the
#: geoReference it names no longer had any effect, and ``offset`` was
#: *subtracted from every exported coordinate*, so ignoring it relocated the
#: map by its own magnitude.
_REMOVED_MAP_KEYS: dict[str, str] = {
    "mgrs_grid": (
        "the coordinate frame now comes from a map_projector_info.yaml next "
        "to the .osm map, or from the map's own node coordinates"
    ),
    "mgrs_code": (
        "the coordinate frame now comes from a map_projector_info.yaml next "
        "to the .osm map, or from the map's own node coordinates"
    ),
    "lat_lon": (
        "the coordinate frame now comes from a map_projector_info.yaml next "
        "to the .osm map, or from the map's own node coordinates"
    ),
    "offset": (
        "exported coordinates are always expressed in the frame the "
        "projector itself defines; removing the block moves them by exactly "
        "the offset it declared"
    ),
}


def reject_removed_map_keys(map_cfg: DictConfig) -> None:
    """Reject map config keys that no longer declare anything.

    Args:
        map_cfg: The ``map`` section of a Hydra config.

    Raises:
        ValueError: If any key in :data:`_REMOVED_MAP_KEYS` is present and
            not null, naming the key and what replaced it (see #564).
    """
    for key, replacement in _REMOVED_MAP_KEYS.items():
        if key in map_cfg and map_cfg[key] is not None:
            raise ValueError(
                f"The map config key {key!r} is no longer supported: "
                f"{replacement}. Remove it from the map config (see #564)."
            )


def _mgrs_geo_reference(mgrs_code: str, offset_x: float, offset_y: float) -> str:
    """Build the geoReference for an MGRS grid square plus an optional offset.

    Without an offset, ``mgrs_code`` is the exact, authoritative grid square
    and is used directly. Re-deriving the grid from its own south-west corner
    via lat/lon sits exactly on a grid boundary and can round to the
    neighboring square, breaking frame consistency between the mgrs_grid and
    lat_lon origin specs (issue #550).

    With an offset, it is folded directly into the grid square's UTM corner
    instead of going through a lat/lon derived via
    ``mgrs_grid_with_offset_to_latlon``, which truncates the offset to whole
    meters (a ~1m error). The exported coordinates, which ``MGRSProjector``
    computes independently of the projector's origin, are unaffected.
    """
    if offset_x == 0.0 and offset_y == 0.0:
        return mgrs_to_proj_string(mgrs_code)
    return mgrs_grid_with_offset_to_proj_string(mgrs_code, offset_x, offset_y)


def geo_reference_for_origin(origin_spec: OriginSpec) -> str:
    """Build the OpenDRIVE geoReference PROJ string for an origin spec.

    Prefers ``mgrs_code`` (with ``offset_x``/``offset_y`` folded in, so the
    frame matches the offset-subtracted coordinates that are exported) and
    falls back to ``lat``/``lon`` for lat/lon origins.

    Raises:
        ValueError: If neither mgrs_code nor lat/lon is set.
    """
    if origin_spec.mgrs_code is not None:
        return _mgrs_geo_reference(
            origin_spec.mgrs_code, origin_spec.offset_x, origin_spec.offset_y
        )
    if origin_spec.lat is not None and origin_spec.lon is not None:
        return latlon_to_proj_string(origin_spec.lat, origin_spec.lon)
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

    def make_projector(self) -> "MGRSProjector | TransverseMercatorProjector":
        if self.projector_type == _TRANSVERSE_MERCATOR:
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
        if self.projector_type == _TRANSVERSE_MERCATOR:
            return latlon_to_tmerc_proj_string(
                self.origin_lat, self.origin_lon, self.scale_factor
            )
        if self.mgrs_code is not None:
            return _mgrs_geo_reference(self.mgrs_code, self.offset_x, self.offset_y)
        return geo_reference_for_origin(self.to_origin_spec())


def _resolve_transverse_mercator(
    info_path: Path, info: MapProjectorInfo
) -> ResolvedProjection:
    """Build a :class:`ResolvedProjection` for ``projector_type: TransverseMercator``.

    ``scale_factor`` is optional. When absent it defaults to
    ``DEFAULT_CONFIG.projection.transverse_mercator_default_scale_factor``
    (0.9996): real Autoware TransverseMercator yamls usually omit it, and
    Autoware's projector uses ``GeographicLib::TransverseMercator::UTM()``
    (k0 = 0.9996). Any positive, finite ``scale_factor`` is accepted and
    threaded straight through to the ``TransverseMercatorProjector`` binding.

    Args:
        info_path: Path to the ``map_projector_info.yaml`` file (for error
            messages).
        info: Typed contents of the file.

    Returns:
        A :class:`ResolvedProjection` with ``projector_type="TransverseMercator"``.

    Raises:
        ValueError: If ``map_origin`` is missing, or if a present
            ``scale_factor`` is not a positive, finite number.
    """
    if info.map_origin is None:
        raise ValueError(
            f"{info_path}: projector_type 'TransverseMercator' requires "
            "'map_origin.latitude' and 'map_origin.longitude' fields"
        )
    origin_lat = info.map_origin.latitude
    origin_lon = info.map_origin.longitude

    scale_factor = (
        DEFAULT_CONFIG.projection.transverse_mercator_default_scale_factor
        if info.scale_factor is None
        else info.scale_factor
    )
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
        projector_type=_TRANSVERSE_MERCATOR,
    )


def _resolve_from_map_projector_info(
    info: MapProjectorInfo, info_path: Path
) -> Optional[ResolvedProjection]:
    """Build a :class:`ResolvedProjection` from a ``map_projector_info.yaml``.

    ``projector_type: MGRS`` and ``projector_type: TransverseMercator`` are
    wired. For an MGRS grid this reproduces the exact projector/geoReference
    of the equivalent explicit ``mgrs_grid`` config, so existing MGRS
    outputs stay byte-identical. Other projector types return ``None`` so
    the caller falls back to the explicit origin keys.

    Args:
        info: The parsed ``map_projector_info.yaml``.
        info_path: Path to the file (used in error messages).

    Returns:
        A :class:`ResolvedProjection` for MGRS or TransverseMercator, or
        ``None`` for unsupported projector types.

    Raises:
        ValueError: If ``projector_type`` is MGRS but ``mgrs_grid`` is
            missing, or if ``projector_type`` is
            TransverseMercator but ``map_origin`` is missing or
            ``scale_factor`` is invalid (see
            :func:`_resolve_transverse_mercator`). ``projector_type`` is
            matched case-sensitively, as Autoware writes it.
    """
    if info.projector_type == _MGRS:
        if info.mgrs_grid is None:
            raise ValueError(
                f"{info_path}: projector_type 'MGRS' requires a 'mgrs_grid' field"
            )
        origin = mgrs_to_lanelet2_origin(info.mgrs_grid)
        origin_lat, origin_lon = mgrs_grid_with_offset_to_latlon(
            info.mgrs_grid, 0.0, 0.0
        )
        return ResolvedProjection(
            origin=origin,
            mgrs_code=info.mgrs_grid,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
        )

    if info.projector_type == _TRANSVERSE_MERCATOR:
        return _resolve_transverse_mercator(info_path, info)

    return None


def _mgrs_grid_squares_in_osm(input_map_path: Path) -> Optional[set[str]]:
    """Collect the 100 km MGRS grid squares the map's nodes fall in.

    Streams the ``.osm`` with ``iterparse`` so an 11 MB map does not have to
    be held in memory. Returns ``None`` when the file cannot be read or
    parsed, or carries no georeferenced nodes -- i.e. "this layer has nothing
    to say", not "the map is wrong".

    Entity resolution and network access are off: the map is input data.
    Recovery is on, because this parser must not be stricter than the one
    that loads the map: ``walkway_mini.osm`` carries a ``--`` inside an XML
    comment, which lanelet2's loader accepts and strict lxml rejects, and a
    map the converter can convert must have a derivable frame.
    """
    squares: set[str] = set()
    projector = mgrs_lib.MGRS()
    try:
        for _event, element in ET.iterparse(
            str(input_map_path),
            events=("end",),
            tag="node",
            resolve_entities=False,
            no_network=True,
            recover=True,
        ):
            latitude = element.get("lat")
            longitude = element.get("lon")
            element.clear()
            if latitude is None or longitude is None:
                continue
            squares.add(
                _grid_square(
                    projector.toMGRS(float(latitude), float(longitude), MGRSPrecision=0)
                )
            )
    except (OSError, ET.LxmlError, ValueError) as exc:
        logger.warning(
            "Could not read node coordinates from %s (%s); the coordinate "
            "frame cannot be derived from the map itself",
            input_map_path,
            exc,
        )
        return None

    if not squares:
        logger.info(
            "%s carries no georeferenced <node> elements; the coordinate "
            "frame cannot be derived from the map itself",
            input_map_path,
        )
        return None
    return squares


def resolve_projection_from_osm(input_map_path: Path) -> Optional[ResolvedProjection]:
    """Derive the MGRS coordinate frame from the map's own node coordinates.

    Every Lanelet2 ``<node>`` carries ``lat``/``lon``, while the per-node
    ``mgrs_code`` tag Autoware tooling writes is optional -- so the grid
    square is derived from the coordinates rather than read off a tag. This
    is the method Autoware's own tooling uses, and it needs no origin keys
    and no sidecar file.

    Args:
        input_map_path: Path to the input ``.osm`` map.

    Returns:
        A :class:`ResolvedProjection` for the single grid square the map
        occupies, or ``None`` when the frame cannot be derived: the file is
        unreadable or unparseable, it carries no georeferenced nodes, or it
        spans more than one 100 km square (which has no single grid origin).
    """
    squares = _mgrs_grid_squares_in_osm(input_map_path)
    if squares is None:
        return None

    if len(squares) > 1:
        logger.warning(
            "%s spans %d MGRS grid squares (%s), so it has no single grid "
            "origin; the coordinate frame cannot be derived from the map "
            "itself",
            input_map_path,
            len(squares),
            ", ".join(sorted(squares)),
        )
        return None

    (mgrs_grid,) = squares
    origin = mgrs_to_lanelet2_origin(mgrs_grid)
    origin_lat, origin_lon = mgrs_grid_with_offset_to_latlon(mgrs_grid, 0.0, 0.0)
    return ResolvedProjection(
        origin=origin,
        mgrs_code=mgrs_grid,
        origin_lat=origin_lat,
        origin_lon=origin_lon,
    )


def resolve_projection(cfg: DictConfig, input_map_path: Path) -> ResolvedProjection:
    """Resolve the coordinate frame, preferring ``map_projector_info.yaml``.

    Resolution order:

    1. a ``map_projector_info.yaml`` next to ``input_map_path`` declaring a
       supported projector;
    2. no such file: the frame is derived from the map's own node
       coordinates (:func:`resolve_projection_from_osm`).

    There is no third step. The ``mgrs_grid`` / ``mgrs_code`` / ``lat_lon``
    config keys that used to be one are gone, and ``cfg`` is now inspected
    only to reject them (:func:`reject_removed_map_keys`) rather than to
    resolve anything.

    A ``map_projector_info.yaml`` whose ``projector_type`` this layer cannot
    build does **not** fall through to step 2: the map declares a projection,
    and deriving an MGRS frame instead would export it in a different one.
    That raises, naming the type.

    Args:
        cfg: Hydra configuration object with a ``map`` section.
        input_map_path: Path to the input ``.osm`` map. The
            ``map_projector_info.yaml`` is looked up in its directory.

    Returns:
        A :class:`ResolvedProjection`.

    Raises:
        ValueError: If the map config declares a removed key, if the
            projector-info file is malformed or declares an unsupported
            ``projector_type``, or if the frame cannot be derived from the
            map itself.
    """
    reject_removed_map_keys(cfg.map)

    info_path = Path(input_map_path).parent / MAP_PROJECTOR_INFO_FILENAME
    if info_path.is_file():
        info = MapProjectorInfo.from_yaml(info_path)
        resolved = _resolve_from_map_projector_info(info, info_path)
        if resolved is not None:
            logger.info("Using %s as the origin source", info_path)
            return resolved
        raise ValueError(
            f"{info_path}: projector_type {info.projector_type!r} is not "
            "supported. Supported types are "
            f"{_MGRS!r} and {_TRANSVERSE_MERCATOR!r}. The frame is not "
            "derived from the map instead, because that would export it in a "
            "projection it does not declare."
        )

    resolved = resolve_projection_from_osm(input_map_path)
    if resolved is not None:
        logger.info(
            "Derived the coordinate frame from %s's own node coordinates "
            "(MGRS grid square %s)",
            input_map_path,
            resolved.mgrs_code,
        )
        return resolved

    raise ValueError(
        f"Cannot determine the coordinate frame of {input_map_path}: it has "
        f"no sibling {MAP_PROJECTOR_INFO_FILENAME} and no usable node "
        "coordinates. See the log above for why derivation declined."
    )
