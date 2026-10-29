"""Single projector-resolution layer for Lanelet2 -> OpenDRIVE conversion.

Resolves the map's coordinate frame in one place: origin parsing (from a
``map_projector_info.yaml``, the map's own node coordinates, a Hydra config or
an :class:`OriginSpec`), projector construction and geoReference PROJ-string
generation.

Exported coordinates are always expressed in the frame the projector itself
defines; the ``offset`` map config key that used to shift them is gone and is
rejected if still present (:func:`_reject_offset_key`). The
``offset_x``/``offset_y``/``offset_z`` fields of :class:`ResolvedProjection`
remain, at zero, for library callers that build an :class:`OriginSpec`
directly and for reading back the geoReference of previously exported files.

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
    latlon_to_lanelet2_origin,
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

# Once-per-process guard for the legacy-origin-key logger.warning (the
# DeprecationWarning is already de-duplicated per call site by the warnings
# module's default filters).
_legacy_origin_keys_logged = False


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


#: Message for a map config that still declares the removed ``offset`` key.
#: The key was *subtracted from every exported coordinate*, so quietly
#: ignoring a declared one would relocate the map by its own magnitude -- and
#: the ``map_projector_info.yaml`` path did exactly that, because it never
#: consulted the key at all.
_OFFSET_REMOVED_MESSAGE = (
    "The map config key 'offset' is no longer supported: exported coordinates "
    "are always expressed in the frame the projector itself defines. Remove "
    "the 'offset' block from the map config -- the exported coordinates will "
    "move by exactly the offset you removed (see #564)."
)


def _reject_offset_key(map_cfg: DictConfig) -> None:
    """Reject a map config that still declares the removed ``offset`` key.

    Called from every resolution entry point, including the
    ``map_projector_info.yaml`` path, so the key cannot be silently dropped
    on any of them.

    Raises:
        ValueError: If ``map_cfg`` declares a non-null ``offset``.
    """
    if "offset" in map_cfg and map_cfg.offset is not None:
        raise ValueError(_OFFSET_REMOVED_MESSAGE)


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

    1. ``mgrs_grid``: MGRS grid code (e.g. ``"54SUE"``).
    2. ``lat_lon``: ``{latitude, longitude, altitude}``.

    The legacy ``mgrs_code`` map field is accepted as an alias for
    ``mgrs_grid``. The removed ``offset`` key is rejected (see
    :func:`_reject_offset_key`).

    Args:
        cfg: Hydra configuration object with a ``map`` section.

    Returns:
        A :class:`ResolvedProjection`, always with a zero offset.

    Raises:
        ValueError: If the origin specification is missing, ambiguous, or
            invalid, or if ``offset`` is declared.
    """
    map_cfg = cfg.map
    _reject_offset_key(map_cfg)

    has_mgrs_grid = "mgrs_grid" in map_cfg and map_cfg.mgrs_grid is not None
    has_mgrs_code = (
        "mgrs_code" in map_cfg and map_cfg.mgrs_code is not None
    )  # Legacy support
    has_lat_lon = "lat_lon" in map_cfg and map_cfg.lat_lon is not None

    # Support legacy mgrs_code field
    if has_mgrs_code and not has_mgrs_grid:
        has_mgrs_grid = True
        map_cfg.mgrs_grid = map_cfg.mgrs_code

    # Count how many base methods are specified.
    specified_methods = sum([has_mgrs_grid, has_lat_lon])

    if specified_methods == 0:
        raise ValueError("Origin must be specified using one of: mgrs_grid, or lat_lon")

    if specified_methods > 1:
        raise ValueError(
            "Multiple origin specification methods detected. "
            "Please specify only one of: mgrs_grid, or lat_lon"
        )

    if has_mgrs_grid:
        mgrs_grid = map_cfg.mgrs_grid

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
    mgrs_grid = _grid_square(m.toMGRS(latitude, longitude))

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
       supported projector: it drives the frame and any explicit ``cfg.map``
       origin keys are ignored;
    2. no such file: the frame is derived from the map's own node
       coordinates (:func:`resolve_projection_from_osm`). This is also a
       non-deprecated route, so no deprecation notice is emitted, and it
       takes precedence over the explicit keys -- ``MGRSProjector`` emits
       coordinates relative to each point's own grid square whatever origin
       it is given, so a key naming a different square only ever mislabelled
       the geoReference;
    3. neither available: the explicit origin keys, with the soft
       deprecation notice.

    One case skips step 2 deliberately: a ``map_projector_info.yaml`` with a
    not-yet-supported ``projector_type``. The map declares a projection this
    layer cannot build, and deriving an MGRS frame instead would silently
    export in a different projection from the one declared. The explicit
    keys are used there *without* the deprecation notice, since the user
    already provided the yaml and has no non-deprecated way to resolve the
    origin; a warning says the keys remain required.

    Args:
        cfg: Hydra configuration object with a ``map`` section.
        input_map_path: Path to the input ``.osm`` map. The
            ``map_projector_info.yaml`` is looked up in its directory.

    Returns:
        A :class:`ResolvedProjection`.

    Raises:
        ValueError: If the projector-info file is malformed, if the removed
            ``offset`` key is declared, or the frame can be derived from
            neither the map nor the explicit origin keys.
    """
    # Before the branch, not inside it: the projector-info path never read
    # this key, so a map that declared one used to be converted by its own
    # magnitude away from where the config said.
    _reject_offset_key(cfg.map)

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
            return resolved
        logger.warning(
            "projector_type %r in %s is not supported yet; the explicit origin "
            "keys ('mgrs_grid'/'lat_lon') remain required for this map",
            info.projector_type,
            info_path,
        )
        return _resolve_from_explicit_keys(cfg)

    resolved = resolve_projection_from_osm(input_map_path)
    if resolved is not None:
        logger.info(
            "Derived the coordinate frame from %s's own node coordinates "
            "(MGRS grid square %s); explicit origin keys, if any, are ignored",
            input_map_path,
            resolved.mgrs_code,
        )
        return resolved

    resolved = _resolve_from_explicit_keys(cfg)
    _warn_legacy_origin_keys(stacklevel=3)
    return resolved
