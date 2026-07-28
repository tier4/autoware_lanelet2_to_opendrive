"""Coordinate projection and MGRS conversion utilities."""

import logging
import re

import lanelet2
import mgrs

from .config import DEFAULT_CONFIG

logger = logging.getLogger(__name__)


def _normalize_mgrs_grid(mgrs_grid: str) -> str:
    """Normalize a partial MGRS grid string by padding coordinates with zeros.

    Handles partial MGRS grids (e.g., "54SUE" without meter coordinates)
    by zero-padding to get the origin coordinates of that grid.

    Args:
        mgrs_grid: MGRS grid reference string, may be partial

    Returns:
        Normalized MGRS string with full 10-digit coordinate suffix
    """
    processed_mgrs = mgrs_grid.strip()
    match = re.match(r"^(\d+[A-Z][A-Z][A-Z])(.*)$", processed_mgrs)
    if match:
        grid_square = match.group(1)
        coordinates = match.group(2)

        if len(coordinates) == 0:
            # No coordinates provided, use origin (00000 00000)
            processed_mgrs = grid_square + "0000000000"
        elif len(coordinates) < 10:
            # Partial coordinates provided - pad to 10 digits
            if len(coordinates) % 2 == 1:
                coordinates += "0"
            padded_coords = coordinates.ljust(10, "0")
            processed_mgrs = grid_square + padded_coords

    return processed_mgrs


def _utm_central_meridian(zone: int) -> float:
    """Return the central meridian (decimal degrees) of a UTM zone."""
    return (zone - 1) * 6 - 180 + 3


def _build_mgrs_frame_proj_string(
    zone: int, is_south: bool, grid_easting: float, grid_northing: float
) -> str:
    """Build the PROJ string for the frame ``MGRSProjector`` actually emits.

    Autoware's ``MGRSProjector::forward()`` ignores the projector's origin:
    for every point it computes the standard UTM easting/northing and
    returns ``fmod(easting, 1e5)`` / ``fmod(northing, 1e5)`` -- the offset
    from the south-west corner of that point's 100 km MGRS grid square.
    ``+proj=utm`` hardcodes its false easting/northing and silently ignores
    any ``+x_0``/``+y_0``/``+lat_0``/``+lon_0`` override (verified
    empirically against PROJ), so it cannot express this grid-relative
    frame. Building the projection explicitly as ``+proj=tmerc`` with the
    standard UTM scale factor and central meridian, and a false
    easting/northing shifted by the grid square's south-west corner,
    reproduces it exactly.

    Args:
        zone: UTM zone number.
        is_south: True if the grid square is in the southern hemisphere.
        grid_easting: Standard UTM easting (m) of the grid square's
            south-west corner.
        grid_northing: Standard UTM northing (m) of the grid square's
            south-west corner.

    Returns:
        PROJ string for the MGRS-grid-relative frame.
    """
    utm = DEFAULT_CONFIG.utm
    false_northing = utm.false_northing_south if is_south else 0.0
    # Round to micrometer precision to strip float-arithmetic repr artifacts
    # (e.g. 118344.27000000002) while fully preserving the 0.01 m-granularity
    # fractional offsets from issue #550.
    x_0 = round(utm.false_easting - grid_easting, 6)
    y_0 = round(false_northing - grid_northing, 6)
    return (
        f"+proj=tmerc +lat_0=0 +lon_0={_utm_central_meridian(zone)} "
        f"+k_0={utm.scale_factor} +x_0={x_0} +y_0={y_0} "
        f"+datum=WGS84 +units=m +no_defs"
    )


def mgrs_to_lanelet2_origin(mgrs_grid: str) -> lanelet2.io.Origin:
    """Convert MGRS grid name to lanelet2.io.Origin.

    If the input is a partial MGRS grid (e.g., "54SUE" without meter coordinates),
    it will be zero-padded to get the origin coordinates of that grid.

    Args:
        mgrs_grid: MGRS grid reference string (e.g., "54SUE1234567890" or "54SUE")

    Returns:
        lanelet2.io.Origin object with coordinates converted from MGRS

    Raises:
        ValueError: If the MGRS grid string is invalid
    """
    try:
        processed_mgrs = _normalize_mgrs_grid(mgrs_grid)

        # Convert MGRS to latitude/longitude
        m = mgrs.MGRS()
        lat, lon = m.toLatLon(processed_mgrs)

        # Create lanelet2 Origin with the converted coordinates
        origin = lanelet2.io.Origin(lat, lon)

        logger.debug(
            f"Origin from MGRS grid: mgrs_grid={mgrs_grid}, "
            f"processed_mgrs={processed_mgrs}, lat={lat}, lon={lon}"
        )

        return origin

    except Exception as e:
        raise ValueError(f"Invalid MGRS grid string '{mgrs_grid}': {e}") from e


def mgrs_grid_with_offset_to_latlon(
    mgrs_grid: str, offset_x: float, offset_y: float
) -> tuple[float, float]:
    """Convert MGRS grid + offset to latitude/longitude coordinates.

    Args:
        mgrs_grid: MGRS grid reference string (e.g., "54SUE")
        offset_x: Easting offset in meters from the grid origin
        offset_y: Northing offset in meters from the grid origin

    Returns:
        Tuple of (latitude, longitude) in decimal degrees

    Raises:
        ValueError: If the MGRS grid string is invalid

    Example:
        >>> mgrs_grid_with_offset_to_latlon("54SUE", 81655.73, 50137.43)
        (-33.123456, 151.234567)
    """
    try:
        processed_mgrs = _normalize_mgrs_grid(mgrs_grid)

        m = mgrs.MGRS()

        # Extract the grid square identifier (zone + band + square)
        match = re.match(r"^(\d+[A-Z][A-Z][A-Z])", processed_mgrs)
        if not match:
            raise ValueError(f"Invalid MGRS format: {mgrs_grid}")
        grid_square = match.group(1)

        # Build MGRS string with the offset coordinates
        # Format as 5-digit easting and northing
        easting = int(offset_x)
        northing = int(offset_y)
        mgrs_with_offset = f"{grid_square}{easting:05d}{northing:05d}"

        # Convert this MGRS coordinate to lat/lon
        lat, lon = m.toLatLon(mgrs_with_offset)

        return lat, lon

    except Exception as e:
        raise ValueError(
            f"Invalid MGRS grid string '{mgrs_grid}' or offset values: {e}"
        ) from e


def mgrs_grid_with_offset_to_lanelet2_origin(
    mgrs_grid: str, offset_x: float, offset_y: float, offset_z: float = 0.0
) -> lanelet2.io.Origin:
    """Convert MGRS grid + offset to lanelet2.io.Origin.

    Args:
        mgrs_grid: MGRS grid reference string (e.g., "54SUE")
        offset_x: Easting offset in meters from the grid origin
        offset_y: Northing offset in meters from the grid origin
        offset_z: Altitude offset in meters (optional, default 0.0)

    Returns:
        lanelet2.io.Origin object with coordinates converted from MGRS + offset

    Raises:
        ValueError: If the MGRS grid string or offset values are invalid

    Example:
        >>> origin = mgrs_grid_with_offset_to_lanelet2_origin("54SUE", 81655.73, 50137.43, 42.49998)
    """
    lat, lon = mgrs_grid_with_offset_to_latlon(mgrs_grid, offset_x, offset_y)
    origin = lanelet2.io.Origin(lat, lon, offset_z)

    logger.debug(
        f"Origin from MGRS grid with offset: "
        f"mgrs_grid={mgrs_grid}, offset_x={offset_x}, offset_y={offset_y}, offset_z={offset_z}, "
        f"lat={lat}, lon={lon}"
    )

    return origin


def latlon_to_lanelet2_origin(
    latitude: float, longitude: float, altitude: float = 0.0
) -> lanelet2.io.Origin:
    """Convert latitude/longitude to lanelet2.io.Origin.

    Args:
        latitude: Latitude in decimal degrees
        longitude: Longitude in decimal degrees
        altitude: Altitude in meters (optional, default 0.0)

    Returns:
        lanelet2.io.Origin object with the specified coordinates

    Example:
        >>> origin = latlon_to_lanelet2_origin(-33.123456, 151.234567, 42.5)
    """
    origin = lanelet2.io.Origin(latitude, longitude, altitude)

    logger.debug(
        f"Origin from lat/lon: lat={latitude}, lon={longitude}, altitude={altitude}"
    )

    return origin


def mgrs_to_proj_string(mgrs_grid: str) -> str:
    """Convert MGRS grid to PROJ string for OpenDRIVE geoReference.

    Assumes the map's coordinates all fall within the single 100 km MGRS
    grid square identified by ``mgrs_grid`` -- the same assumption
    ``MGRSProjector`` itself relies on (it warns if a projected point lands
    in a different grid square than the previous one).

    Args:
        mgrs_grid: MGRS grid reference string (e.g., "54SUE" or "54SUE1234567890")

    Returns:
        PROJ string for the MGRS-grid-relative frame ``MGRSProjector``
        actually emits (see :func:`_build_mgrs_frame_proj_string`).

    Raises:
        ValueError: If the MGRS grid string is invalid

    Example:
        >>> mgrs_to_proj_string("54SUE")
        '+proj=tmerc +lat_0=0 +lon_0=141 +k_0=0.9996 +x_0=200000.0 +y_0=-3900000.0 +datum=WGS84 +units=m +no_defs'
    """
    try:
        processed_mgrs = _normalize_mgrs_grid(mgrs_grid)
        m = mgrs.MGRS()
        zone, hemisphere, grid_easting, grid_northing = m.MGRSToUTM(processed_mgrs)

        proj_string = _build_mgrs_frame_proj_string(
            zone, hemisphere == "S", grid_easting, grid_northing
        )

        logger.debug(
            f"PROJ string from MGRS grid: mgrs_grid={mgrs_grid}, "
            f"zone={zone}, hemisphere={hemisphere}, "
            f"grid_easting={grid_easting}, grid_northing={grid_northing}, "
            f"proj={proj_string}"
        )

        return proj_string

    except Exception as e:
        raise ValueError(f"Invalid MGRS grid string '{mgrs_grid}': {e}") from e


def mgrs_grid_with_offset_to_proj_string(
    mgrs_grid: str, offset_x: float, offset_y: float
) -> str:
    """Convert an MGRS grid square + full-precision offset to a PROJ string.

    Unlike :func:`mgrs_grid_with_offset_to_latlon`, this does not round-trip
    the offset through a 10-digit MGRS string (which truncates it to whole
    meters). Instead, the offset is folded directly into the grid square's
    SW-corner UTM coordinates before building the ``+proj=tmerc`` frame, so
    the resulting geoReference is exact for fractional-meter offsets such as
    those in ``nishishinjuku.yaml`` (issue #550).

    Args:
        mgrs_grid: MGRS grid reference string (e.g., "54SUE" or "54SUE1234567890")
        offset_x: Offset in meters (easting direction) from the grid square's
            south-west corner.
        offset_y: Offset in meters (northing direction) from the grid square's
            south-west corner.

    Returns:
        PROJ string for the MGRS-grid-relative frame, shifted so the offset
        point (not the grid square's own SW corner) sits at the frame's
        origin -- matching the convention ``MGRSProjector`` itself emits.

    Raises:
        ValueError: If the MGRS grid string is invalid.
    """
    try:
        processed_mgrs = _normalize_mgrs_grid(mgrs_grid)
        m = mgrs.MGRS()
        zone, hemisphere, grid_easting, grid_northing = m.MGRSToUTM(processed_mgrs)

        proj_string = _build_mgrs_frame_proj_string(
            zone,
            hemisphere == "S",
            grid_easting + offset_x,
            grid_northing + offset_y,
        )

        logger.debug(
            f"PROJ string from MGRS grid + offset: mgrs_grid={mgrs_grid}, "
            f"offset_x={offset_x}, offset_y={offset_y}, "
            f"zone={zone}, hemisphere={hemisphere}, "
            f"grid_easting={grid_easting}, grid_northing={grid_northing}, "
            f"proj={proj_string}"
        )

        return proj_string

    except Exception as e:
        raise ValueError(
            f"Invalid MGRS grid string '{mgrs_grid}' or offset values: {e}"
        ) from e


def latlon_to_tmerc_proj_string(lat_0: float, lon_0: float, scale_factor: float) -> str:
    """Build a Transverse Mercator PROJ string for OpenDRIVE geoReference.

    Pure string formatting -- no pyproj call. Mirrors the projection applied
    by Autoware's ``TransverseMercatorProjector`` C++/Python binding, which
    accepts an explicit central-meridian scale factor (``k``).

    Args:
        lat_0: Origin latitude in decimal degrees.
        lon_0: Origin longitude in decimal degrees.
        scale_factor: Central-meridian scale factor (k).

    Returns:
        PROJ string for a Transverse Mercator projection centered at
        (lat_0, lon_0).

    Example:
        >>> latlon_to_tmerc_proj_string(35.61739731, 139.7797546, 0.9996)
        '+proj=tmerc +lat_0=35.61739731 +lon_0=139.7797546 +k_0=0.9996 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs'
    """
    return (
        f"+proj=tmerc +lat_0={lat_0} +lon_0={lon_0} +k_0={scale_factor} "
        f"+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
    )


def latlon_to_proj_string(lat: float, lon: float) -> str:
    """Convert latitude/longitude to PROJ string for OpenDRIVE geoReference.

    Both the ``mgrs_grid`` and ``lat_lon`` origin specifications resolve to
    an ``MGRSProjector`` (see :meth:`.projection_resolver.ResolvedProjection.make_projector`),
    which emits coordinates relative to the point's 100 km MGRS grid square
    regardless of the configured origin. This function therefore derives the
    grid square containing ``(lat, lon)`` and delegates to
    :func:`mgrs_to_proj_string`, so a ``lat_lon`` origin and an equivalent
    ``mgrs_grid`` origin in the same grid square produce an identical
    geoReference. As with :func:`mgrs_to_proj_string`, this assumes the map's
    coordinates all fall within that single 100 km grid square.

    Args:
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees

    Returns:
        PROJ string for the MGRS-grid-relative frame containing (lat, lon).

    Example:
        >>> latlon_to_proj_string(35.6895, 139.6917)
        '+proj=tmerc +lat_0=0 +lon_0=141 +k_0=0.9996 +x_0=200000.0 +y_0=-3900000.0 +datum=WGS84 +units=m +no_defs'
    """
    m = mgrs.MGRS()
    mgrs_code = m.toMGRS(lat, lon)
    match = re.match(r"^(\d+[A-Z][A-Z][A-Z])", mgrs_code)
    grid = match.group(1) if match else mgrs_code[:5]

    proj_string = mgrs_to_proj_string(grid)

    logger.debug(
        f"PROJ string from lat/lon: lat={lat}, lon={lon}, grid={grid}, proj={proj_string}"
    )

    return proj_string
