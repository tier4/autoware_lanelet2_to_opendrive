"""Tests for the MGRS-grid-relative geoReference PROJ string (issue #550).

Autoware's ``MGRSProjector::forward()`` ignores the projector's origin and
always emits coordinates relative to the south-west corner of the point's
100 km MGRS grid square. Covers exact-match PROJ strings for northern and
southern hemisphere grid squares, a pyproj round trip, the real
``MGRSProjector`` C++ binding vs. the geoReference PROJ string (the
definitive regression guard proving the fix matches Autoware's actual
behavior), and that ``lat_lon``/``mgrs_grid`` origin resolution converge to
the same geoReference for the same grid square.
"""

import lanelet2
import pytest
from autoware_lanelet2_extension_python.projection import MGRSProjector
from pyproj import Transformer

from autoware_lanelet2_to_opendrive.projection import (
    latlon_to_proj_string,
    mgrs_grid_with_offset_to_proj_string,
    mgrs_to_proj_string,
    parse_mgrs_frame_proj_string,
)

TOKYO_GRID = "54SUE"
EXPECTED_TOKYO_GEO_REFERENCE = (
    "+proj=tmerc +lat_0=0 +lon_0=141 +k_0=0.9996 "
    "+x_0=200000.0 +y_0=-3900000.0 +datum=WGS84 +units=m +no_defs"
)

SYDNEY_GRID = "56HLH"
EXPECTED_SYDNEY_GEO_REFERENCE = (
    "+proj=tmerc +lat_0=0 +lon_0=153 +k_0=0.9996 "
    "+x_0=200000.0 +y_0=3800000.0 +datum=WGS84 +units=m +no_defs"
)


# ---------------------------------------------------------------------------
# 1. geoReference PROJ string -- exact match (contractual)
# ---------------------------------------------------------------------------


def test_mgrs_to_proj_string_exact():
    assert mgrs_to_proj_string(TOKYO_GRID) == EXPECTED_TOKYO_GEO_REFERENCE


def test_mgrs_to_proj_string_southern_hemisphere_exact():
    assert mgrs_to_proj_string(SYDNEY_GRID) == EXPECTED_SYDNEY_GEO_REFERENCE


# ---------------------------------------------------------------------------
# 2. pyproj round trip through the generated PROJ string
# ---------------------------------------------------------------------------


def test_mgrs_round_trip_closes_within_tolerance():
    proj_string = mgrs_to_proj_string(TOKYO_GRID)
    forward = Transformer.from_crs("EPSG:4326", proj_string, always_xy=True)
    inverse = Transformer.from_crs(proj_string, "EPSG:4326", always_xy=True)

    lon_in, lat_in = 139.75, 35.7
    x, y = forward.transform(lon_in, lat_in)
    lon_out, lat_out = inverse.transform(x, y)

    assert lon_out == pytest.approx(lon_in, abs=1e-9)
    assert lat_out == pytest.approx(lat_in, abs=1e-9)


def test_mgrs_round_trip_southern_hemisphere():
    proj_string = mgrs_to_proj_string(SYDNEY_GRID)
    forward = Transformer.from_crs("EPSG:4326", proj_string, always_xy=True)
    inverse = Transformer.from_crs(proj_string, "EPSG:4326", always_xy=True)

    lon_in, lat_in = 151.21, -33.87
    x, y = forward.transform(lon_in, lat_in)
    lon_out, lat_out = inverse.transform(x, y)

    assert lon_out == pytest.approx(lon_in, abs=1e-9)
    assert lat_out == pytest.approx(lat_in, abs=1e-9)


# ---------------------------------------------------------------------------
# 3. Real binding regression guard: MGRSProjector.forward() vs geoReference
# ---------------------------------------------------------------------------


def test_geo_reference_matches_real_mgrs_projector_binding():
    """The geoReference PROJ string must reproduce ``MGRSProjector::forward()``.

    This is the definitive regression guard for issue #550: it exercises the
    actual C++ ``MGRSProjector`` binding (which ignores its own ``Origin``
    and returns ``fmod(utm_easting, 1e5)`` / ``fmod(utm_northing, 1e5)``) and
    checks that forward-transforming the same point through the geoReference
    PROJ string (via pyproj) yields the identical (x, y) -- not merely that
    the same formula was re-derived in Python.
    """
    lat, lon = 35.7, 139.75  # inside grid "54SUE"

    # MGRSProjector ignores its Origin entirely, so an arbitrary one suffices.
    projector = MGRSProjector(lanelet2.io.Origin(35.0, 139.0))
    fwd = projector.forward(lanelet2.core.GPSPoint(lat, lon, 0.0))

    proj_string = mgrs_to_proj_string(TOKYO_GRID)
    transformer = Transformer.from_crs("EPSG:4326", proj_string, always_xy=True)
    x, y = transformer.transform(lon, lat)

    assert x == pytest.approx(fwd.x, abs=1e-6)
    assert y == pytest.approx(fwd.y, abs=1e-6)


# ---------------------------------------------------------------------------
# 4. Frame-consistency invariant across origin resolution paths
# ---------------------------------------------------------------------------


def test_lat_lon_and_its_grid_square_give_the_same_geo_reference():
    """A lat/lon and the grid square containing it converge to one PROJ string.

    ``MGRSProjector`` emits coordinates relative to each point's own 100 km
    square whatever origin it is given, so describing the frame by a lat/lon
    inside that square and by the square itself must agree (issue #550
    frame-consistency invariant).

    Asserted on the projection helpers rather than through a config, because
    the ``lat_lon`` and ``mgrs_grid`` keys that used to express the two sides
    are removed (#564).
    """
    lat, lon = 35.7, 139.75  # inside grid "54SUE"

    assert latlon_to_proj_string(lat, lon) == mgrs_to_proj_string(TOKYO_GRID)


# ---------------------------------------------------------------------------
# 5. Fractional-meter offset precision (issue #550 follow-up)
# ---------------------------------------------------------------------------


def test_mgrs_grid_with_offset_to_proj_string_preserves_fractional_offset():
    """geoReference for an offset MGRS origin must not truncate the offset.

    ``mgrs_grid_with_offset_to_latlon`` truncates ``offset_x``/``offset_y`` to
    whole meters when round-tripping through a 10-digit MGRS string (a ~1 m
    error, see ``nishishinjuku.yaml``/``example_mgrs_offset.yaml``).
    ``mgrs_grid_with_offset_to_proj_string`` bypasses that round trip
    entirely, so forward-transforming an arbitrary point through it must
    reproduce the grid-relative coordinate minus the *full-precision*
    offset, not the truncated one.
    """
    offset_x, offset_y = 81655.73, 50137.43
    lat, lon = 35.7, 139.75  # inside grid "54SUE"

    grid_transformer = Transformer.from_crs(
        "EPSG:4326", mgrs_to_proj_string(TOKYO_GRID), always_xy=True
    )
    grid_x, grid_y = grid_transformer.transform(lon, lat)

    offset_proj_string = mgrs_grid_with_offset_to_proj_string(
        TOKYO_GRID, offset_x, offset_y
    )
    offset_transformer = Transformer.from_crs(
        "EPSG:4326", offset_proj_string, always_xy=True
    )
    x, y = offset_transformer.transform(lon, lat)

    # The fractional part must survive: a truncated offset (81655, 50137)
    # would be off by ~0.27 m / 0.43 m, well outside this tolerance.
    assert x == pytest.approx(grid_x - offset_x, abs=1e-6)
    assert y == pytest.approx(grid_y - offset_y, abs=1e-6)


def test_full_precision_grid_anchors_at_grid_square_corner():
    """In-square digits must not move the frame: MGRSProjector emits
    coordinates relative to the 100 km square's corner, not to that point."""
    assert mgrs_to_proj_string("54SUE1234567890") == mgrs_to_proj_string("54SUE")
    assert mgrs_grid_with_offset_to_proj_string(
        "54SUE1234567890", 81655.73, 50137.43
    ) == mgrs_grid_with_offset_to_proj_string("54SUE", 81655.73, 50137.43)


@pytest.mark.parametrize(
    "grid, offset_x, offset_y",
    [
        (TOKYO_GRID, 81655.73, 50137.43),
        (TOKYO_GRID, 0.0, 0.0),
        (SYDNEY_GRID, 12345.67, 89012.34),
    ],
)
def test_parse_mgrs_frame_proj_string_inverts_geo_reference(grid, offset_x, offset_y):
    """The parsed frame origin is the inverse projection of output (0, 0),
    and its offset is the one folded into the geoReference."""
    proj_string = mgrs_grid_with_offset_to_proj_string(grid, offset_x, offset_y)
    frame = parse_mgrs_frame_proj_string(proj_string)
    assert frame is not None
    assert frame.offset_x == pytest.approx(offset_x, abs=1e-6)
    assert frame.offset_y == pytest.approx(offset_y, abs=1e-6)

    inverse = Transformer.from_crs(proj_string, "EPSG:4326", always_xy=True)
    lon, lat = inverse.transform(0.0, 0.0)
    # Frame lat/lon carry 1 m MGRS precision (~1e-5 deg).
    assert frame.latitude == pytest.approx(lat, abs=2e-5)
    assert frame.longitude == pytest.approx(lon, abs=2e-5)


@pytest.mark.parametrize(
    "proj_string",
    [
        # TransverseMercator geoReference: lat_0/lon_0 is the real origin.
        "+proj=tmerc +lat_0=35.68 +lon_0=139.69 +k_0=0.9999 +x_0=0 +y_0=0 "
        "+datum=WGS84 +units=m +no_defs",
        # Pre-#550 UTM geoReference.
        "+proj=utm +zone=54 +lat_0=35.68 +lon_0=139.69 +datum=WGS84 +units=m +no_defs",
        # tmerc with a non-UTM central meridian is not an MGRS frame.
        "+proj=tmerc +lat_0=0 +lon_0=139.69 +k_0=0.9996 +x_0=1000 +y_0=-1000 "
        "+datum=WGS84 +units=m +no_defs",
    ],
)
def test_parse_mgrs_frame_proj_string_rejects_real_origin_strings(proj_string):
    assert parse_mgrs_frame_proj_string(proj_string) is None
