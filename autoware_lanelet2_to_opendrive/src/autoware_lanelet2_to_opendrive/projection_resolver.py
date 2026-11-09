"""Single projector-resolution layer for Lanelet2 -> OpenDRIVE conversion.

Resolves the map's coordinate frame in one place: origin parsing (from a Hydra
config or an :class:`OriginSpec`), projector construction, geoReference
PROJ-string generation, and the coordinate offset applied during export.

The MGRS / explicit-key paths are a behavior-preserving extraction of logic
that previously lived inline in ``main.py`` (issue #540): their outputs remain
byte-identical. TransverseMercator support and the soft deprecation of the
explicit origin keys were added on top.
"""

import logging
import math
import re
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Set, Tuple

import lanelet2
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
    latlon_to_lanelet2_origin,
    latlon_to_proj_string,
    latlon_to_tmerc_proj_string,
    mgrs_grid_with_offset_to_lanelet2_origin,
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

# Once-per-process guard for the legacy-origin-key logger.warning (the
# DeprecationWarning is already de-duplicated per call site by the warnings
# module's default filters).
_legacy_origin_keys_logged = False


_ORIGIN_MARGIN_DEG = 1.0
"""How far outside the map a lat/lon origin may sit, in degrees.

About 110 km of latitude. Wide enough for an origin placed at a site datum
rather than inside the mapped area, narrow enough that a different city
fails.
"""


def _warn_legacy_origin_keys(stacklevel: int) -> None:
    """Emit the soft-deprecation notice for the explicit origin keys.

    Soft-deprecation (issue #539): map_projector_info.yaml is the canonical
    origin source (see :func:`resolve_projection`). Emits a
    DeprecationWarning (suppressed by default per PEP 565 outside
    ``__main__``) and, once per process, a logger.warning so CLI users
    actually see it. Removal is tracked in a future major version (#564).

    Args:
        stacklevel: Passed to :func:`warnings.warn`, counted from this
            helper; callers pass ``3`` so the warning points at the code that
            called the public resolver function.
    """
    global _legacy_origin_keys_logged
    warnings.warn(
        "Specifying the map origin via 'mgrs_grid'/'lat_lon' config keys is "
        "deprecated; provide a map_projector_info.yaml next to the .osm map "
        "instead. This fallback will be removed in a future major version "
        "(see #564).",
        DeprecationWarning,
        stacklevel=stacklevel,
    )
    if not _legacy_origin_keys_logged:
        _legacy_origin_keys_logged = True
        logger.warning(
            "Origin resolved via legacy 'mgrs_grid'/'lat_lon' config keys "
            "(deprecated). Prefer map_projector_info.yaml. See #564."
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


def resolve_projection_from_hydra(cfg: DictConfig) -> ResolvedProjection:
    """Resolve the coordinate frame from the explicit Hydra origin keys.

    Deprecated entry point: emits the legacy-origin-key deprecation notice
    (see :func:`_warn_legacy_origin_keys`). Prefer :func:`resolve_projection`
    with a ``map_projector_info.yaml`` next to the map. See
    :func:`_resolve_from_explicit_keys` for the accepted keys and errors.
    """
    resolved = _resolve_from_explicit_keys(cfg)
    _warn_legacy_origin_keys(stacklevel=3)
    return resolved


def _resolve_from_explicit_keys(cfg: DictConfig) -> ResolvedProjection:
    """Resolve the coordinate frame from a Hydra config (no deprecation notice).

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


def _map_lat_lon_bounds(
    map_path: Path,
) -> Optional[Tuple[float, float, float, float]]:
    """The lat/lon bounding box of the map's nodes, or ``None`` if unreadable.

    Read with the XML parser in recovering mode rather than by loading the
    map: a Lanelet2 ``.osm`` that lanelet2 loads is not always well-formed to
    a stricter parser, and loading it needs the very projector being checked.

    Returns:
        ``(min_lat, min_lon, max_lat, max_lon)``, or ``None`` when no node
        carried a readable lat/lon.
    """
    import lxml.etree as ET

    lats: List[float] = []
    lons: List[float] = []
    try:
        parser = ET.XMLParser(recover=True)
        tree = ET.parse(str(map_path), parser)
        root = tree.getroot()
        if root is None:
            return None
        for node in root.iter("node"):
            lat, lon = node.get("lat"), node.get("lon")
            if lat is None or lon is None:
                continue
            try:
                lats.append(float(lat))
                lons.append(float(lon))
            except ValueError:
                continue
    except (OSError, ET.XMLSyntaxError):
        return None

    if not lats:
        return None
    return (min(lats), min(lons), max(lats), max(lons))


def _mgrs_grids_of(
    bounds: Tuple[float, float, float, float],
) -> Set[str]:
    """The 100 km MGRS squares the bounding box's corners and centre fall in.

    The corners as well as the centre, because a map may straddle a square
    boundary and then no single probe answers for the whole map.
    """
    min_lat, min_lon, max_lat, max_lon = bounds
    probes = [
        (min_lat, min_lon),
        (min_lat, max_lon),
        (max_lat, min_lon),
        (max_lat, max_lon),
        ((min_lat + max_lat) / 2.0, (min_lon + max_lon) / 2.0),
    ]
    grids: Set[str] = set()
    for lat, lon in probes:
        projector = MGRSProjector(lanelet2.io.Origin(lat, lon))
        projector.forward(lanelet2.core.GPSPoint(lat, lon, 0.0))
        grid = projector.getProjectedMGRSGrid()
        if grid:
            grids.add(grid)
    return grids


def verify_origin_matches_map(resolved: ResolvedProjection, map_path: Path) -> None:
    """Fail when the resolved origin does not belong to this map.

    Nothing else compares the two. A preset copied to a new map, or a
    ``map_projector_info.yaml`` left behind from another one, resolves
    without complaint and the conversion completes with every coordinate in
    the wrong frame.

    The map's node lat/lon is the basis, because it is the one thing always
    present -- the ``mgrs_code`` tag is optional, so it is not consulted.

    * With an MGRS grid: the configured square must be one the map's own
      nodes fall in. Compared on the shorter of the two prefixes, so a
      higher-precision code such as ``54SUE815501`` matches the square
      ``54SUE`` it names.
    * Otherwise: the origin itself must lie within the map's bounding box
      expanded by :data:`_ORIGIN_MARGIN_DEG`.

    A map whose nodes cannot be read is accepted: it says nothing about the
    origin either way, and a check that can fail a map it never read is
    worse than no check.

    Raises:
        ValueError: The origin belongs to a different part of the world.
    """
    bounds = _map_lat_lon_bounds(map_path)
    if bounds is None:
        logger.debug(
            "No node lat/lon read from %s; origin not checked against the map",
            map_path,
        )
        return

    min_lat, min_lon, max_lat, max_lon = bounds

    if resolved.mgrs_code:
        grids = _mgrs_grids_of(bounds)
        length = min([len(resolved.mgrs_code)] + [len(grid) for grid in grids])
        if grids and resolved.mgrs_code[:length] not in {
            grid[:length] for grid in grids
        }:
            raise ValueError(
                f"Origin grid {resolved.mgrs_code!r} is not a grid this map "
                f"lies in. {map_path.name} spans lat {min_lat:.5f}..{max_lat:.5f},"
                f" lon {min_lon:.5f}..{max_lon:.5f}, which is MGRS"
                f" {sorted(grids)}. Fix the origin, or set"
                " check_origin_against_map=false to proceed anyway."
            )
        return

    if resolved.origin_lat is None or resolved.origin_lon is None:
        return

    if not (
        min_lat - _ORIGIN_MARGIN_DEG
        <= resolved.origin_lat
        <= max_lat + _ORIGIN_MARGIN_DEG
        and min_lon - _ORIGIN_MARGIN_DEG
        <= resolved.origin_lon
        <= max_lon + _ORIGIN_MARGIN_DEG
    ):
        raise ValueError(
            f"Origin ({resolved.origin_lat}, {resolved.origin_lon}) lies"
            f" outside {map_path.name}, which spans lat"
            f" {min_lat:.5f}..{max_lat:.5f}, lon {min_lon:.5f}..{max_lon:.5f}"
            f" (margin {_ORIGIN_MARGIN_DEG} deg). Fix the origin, or set"
            " check_origin_against_map=false to proceed anyway."
        )


def resolve_projection(cfg: DictConfig, input_map_path: Path) -> ResolvedProjection:
    """Resolve the coordinate frame, preferring ``map_projector_info.yaml``.

    When that file sits next to ``input_map_path`` and declares a supported
    projector, it drives the frame and any explicit ``cfg.map`` origin keys
    are ignored. Otherwise resolution falls back to the explicit origin keys:

    * no ``map_projector_info.yaml``: the keys are used and the soft
      deprecation notice is emitted (the yaml is the non-deprecated route);
    * a ``map_projector_info.yaml`` with a not-yet-supported
      ``projector_type``: the keys are used *without* the deprecation notice,
      since the user already provided the yaml and has no non-deprecated way
      to resolve the origin; a warning says the keys remain required.

    Args:
        cfg: Hydra configuration object with a ``map`` section.
        input_map_path: Path to the input ``.osm`` map. The
            ``map_projector_info.yaml`` is looked up in its directory.

    Returns:
        A :class:`ResolvedProjection`.

    Raises:
        ValueError: If the projector-info file is malformed, or the explicit
            origin keys are missing/invalid on the fallback path.
    """
    resolved: Optional[ResolvedProjection] = None
    info_path = Path(input_map_path).parent / MAP_PROJECTOR_INFO_FILENAME
    if info_path.is_file():
        info = MapProjectorInfo.from_yaml(info_path)
        resolved = _resolve_from_map_projector_info(info, info_path)
        if resolved is not None:
            logger.info(
                "Using %s as the canonical origin source (explicit origin keys, "
                "if any, are ignored)",
                info_path,
            )
        else:
            logger.warning(
                "projector_type %r in %s is not supported yet; the explicit "
                "origin keys ('mgrs_grid'/'lat_lon') remain required for this "
                "map",
                info.projector_type,
                info_path,
            )
            resolved = _resolve_from_explicit_keys(cfg)
    else:
        resolved = _resolve_from_explicit_keys(cfg)
        _warn_legacy_origin_keys(stacklevel=3)

    # Whichever source won, it has to belong to this map. A yaml left behind
    # from another map is as wrong as a copied preset.
    if _check_origin_against_map(cfg):
        verify_origin_matches_map(resolved, Path(input_map_path))
    return resolved


def _check_origin_against_map(cfg: DictConfig) -> bool:
    """Whether to verify the resolved origin against the map's own nodes.

    Precedence map > target > global, matching the other per-map overrides.
    ``or`` cannot be used because the value is a boolean and ``False`` is a
    meaningful setting.
    """
    return bool(
        cfg.get("map", {}).get(
            "check_origin_against_map",
            cfg.get("target", {}).get(
                "check_origin_against_map",
                cfg.get("check_origin_against_map", True),
            ),
        )
    )
