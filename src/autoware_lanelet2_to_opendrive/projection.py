"""Coordinate projection and MGRS conversion utilities."""

import logging
import re

import lanelet2
import mgrs

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


def _build_utm_proj_string(zone: int | str, lat: float, lon: float, is_south: bool) -> str:
    """Build a UTM PROJ string from zone, origin coordinates, and hemisphere.

    Args:
        zone: UTM zone number
        lat: Origin latitude in decimal degrees
        lon: Origin longitude in decimal degrees
        is_south: True if the location is in the southern hemisphere

    Returns:
        PROJ string for UTM projection
    """
    hemisphere = "+south" if is_south else ""
    proj_string = (
        f"+proj=utm +zone={zone} {hemisphere} "
        f"+lat_0={lat} +lon_0={lon} "
        f"+datum=WGS84 +units=m +no_defs"
    ).replace("  ", " ")  # Remove double spaces if hemisphere is empty
    return proj_string


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

    Args:
        mgrs_grid: MGRS grid reference string (e.g., "54SUE" or "54SUE1234567890")

    Returns:
        PROJ string with UTM projection and origin coordinates from MGRS grid

    Raises:
        ValueError: If the MGRS grid string is invalid

    Example:
        >>> mgrs_to_proj_string("54SUE")
        '+proj=utm +zone=54 +south +lat_0=-28.0 +lon_0=141.0 +datum=WGS84 +units=m +no_defs'
    """
    try:
        # Extract UTM zone number and latitude band
        zone_match = re.match(r"^(\d+)([A-Z])", mgrs_grid)
        if not zone_match:
            raise ValueError(f"Invalid MGRS format: {mgrs_grid}")

        zone = zone_match.group(1)
        band = zone_match.group(2)

        # Determine hemisphere from latitude band
        # Latitude bands: C-M are south, N-X are north
        is_south = band < "N"

        # Convert MGRS to latitude/longitude
        processed_mgrs = _normalize_mgrs_grid(mgrs_grid)
        m = mgrs.MGRS()
        lat, lon = m.toLatLon(processed_mgrs)

        return _build_utm_proj_string(zone, lat, lon, is_south)

    except Exception as e:
        raise ValueError(f"Invalid MGRS grid string '{mgrs_grid}': {e}") from e


def latlon_to_proj_string(lat: float, lon: float) -> str:
    """Convert latitude/longitude to PROJ string for OpenDRIVE geoReference.

    Args:
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees

    Returns:
        PROJ string with UTM projection and the specified origin coordinates

    Example:
        >>> latlon_to_proj_string(35.6895, 139.6917)
        '+proj=utm +zone=54 +lat_0=35.6895 +lon_0=139.6917 +datum=WGS84 +units=m +no_defs'
    """
    # Calculate UTM zone from longitude
    # UTM zones are 6 degrees wide, starting at -180
    zone = int((lon + 180) / 6) + 1

    # Determine hemisphere from latitude
    is_south = lat < 0

    proj_string = _build_utm_proj_string(zone, lat, lon, is_south)

    logger.debug(
        f"PROJ string from lat/lon: lat={lat}, lon={lon}, zone={zone}, proj={proj_string}"
    )

    return proj_string
