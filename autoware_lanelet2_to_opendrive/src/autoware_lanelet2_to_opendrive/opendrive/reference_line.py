"""ReferenceLine implementation for OpenDRIVE conversion."""

import logging
from typing import TYPE_CHECKING, List, Optional, Set, Tuple, Union

import lanelet2
import lxml.etree as ET
import numpy as np
from lanelet2.routing import RoutingGraph

from ..config import DEFAULT_CONFIG, CoordinateOffset
from ..cubic_spline_1d import CubicSpline1D
from ..spline import Splines

# Import enums directly to avoid circular import
from .elevation import Elevation, ElevationProfile
from .enums import LaneType
from .lane_elements import LaneWidth, road_mark_from_linestring_attrs

logger = logging.getLogger(__name__)


def _guard_heading_override(
    hdg_override: Optional[float], own_vel: np.ndarray, end_name: str
) -> Optional[float]:
    """Drop a heading override that strays too far from the boundary's tangent.

    Returns ``hdg_override`` unchanged when it lies within
    ``SplineConstants.max_heading_override_rad`` of ``own_vel``'s direction,
    and ``None`` otherwise so the fit falls back to its own tangent.
    """
    if hdg_override is None:
        return None
    own_hdg = float(np.arctan2(own_vel[1], own_vel[0]))
    dpsi = abs((hdg_override - own_hdg + np.pi) % (2.0 * np.pi) - np.pi)
    limit = DEFAULT_CONFIG.spline.max_heading_override_rad
    if dpsi > limit:
        logger.debug(
            "Ignoring %s heading override: %.2f deg from the boundary tangent "
            "(limit %.2f deg); keeping the position pin only",
            end_name,
            np.degrees(dpsi),
            np.degrees(limit),
        )
        return None
    return hdg_override


if TYPE_CHECKING:
    pass


def _cut_at_arc_length(
    points_3d: np.ndarray, distance: float, from_end: bool
) -> np.ndarray:
    """Return ``points_3d`` with ``distance`` of 2D arc length removed.

    The cut point is interpolated on the segment it falls in -- linearly in
    x, y **and** z against the 2D arc length, which is the interpolation
    ``resample_points_3d`` and the height spline both use, so the elevation
    the shortened line reports is the elevation the original reported there.

    Vertices in the kept stretch are returned verbatim. The caller has
    already established that the polyline is long enough.
    """
    work = points_3d[::-1] if from_end else points_3d

    segments = np.linalg.norm(np.diff(work[:, :2], axis=0), axis=1)
    travelled = np.cumsum(segments)

    # The first vertex at or beyond the cut. ``searchsorted`` with "left"
    # gives the segment the cut falls in, which may be the first.
    index = int(np.searchsorted(travelled, distance, side="left"))

    before = travelled[index - 1] if index > 0 else 0.0
    segment_length = segments[index]
    remainder = distance - before

    kept = work[index + 1 :]
    if remainder >= segment_length:
        # The cut lands exactly on a vertex, so no point is interpolated.
        return kept[::-1] if from_end else kept

    fraction = remainder / segment_length
    cut_point = work[index] + fraction * (work[index + 1] - work[index])
    trimmed = np.vstack([cut_point[np.newaxis, :], kept])
    return trimmed[::-1] if from_end else trimmed


def trim_polyline(
    points_3d: np.ndarray,
    trim_start: float = 0.0,
    trim_end: float = 0.0,
) -> Optional[np.ndarray]:
    """Shorten a boundary polyline from either end by a 2D arc length.

    This is the geometric half of the junction carve: an incoming road is
    ended short of the junction so the freed stretch can be emitted as a
    connecting road with real extent. Shortening the polyline *before* the
    spline is fitted costs no re-parameterisation of the emitted geometry --
    the fitter runs on a shorter line -- at the price that the fit changes
    along the whole road rather than only at the trimmed end.

    Lengths are 2D (XY) arc length, matching :func:`resample_points_3d`, so
    the trim and the resampling agree on what a metre is.

    Args:
        points_3d: (N, 3) array of [x, y, z] boundary points.
        trim_start: Metres to remove from the s=0 end.
        trim_end: Metres to remove from the s=length end.

    Returns:
        The shortened (M, 3) array, the input itself when neither trim is
        positive, or ``None`` when the polyline cannot spare the lengths
        asked for -- which is how a road too short to carve is reported.
        A road reduced to a single point, or to nothing, is never returned:
        three roads on Nishishinjuku are under a metre long, and a
        degenerate reference line is worse than an untrimmed one.
    """
    if trim_start <= 0.0 and trim_end <= 0.0:
        return points_3d
    if len(points_3d) < 2:
        return None

    total = float(np.sum(np.linalg.norm(np.diff(points_3d[:, :2], axis=0), axis=1)))
    requested = max(trim_start, 0.0) + max(trim_end, 0.0)
    if requested >= total:
        return None

    trimmed = points_3d
    if trim_end > 0.0:
        trimmed = _cut_at_arc_length(trimmed, trim_end, from_end=True)
    if trim_start > 0.0:
        trimmed = _cut_at_arc_length(trimmed, trim_start, from_end=False)

    if len(trimmed) < 2:
        return None
    return trimmed


def resample_points_3d(points_3d: np.ndarray, spacing: float) -> np.ndarray:
    """Subdivide a boundary polyline so no segment is longer than ``spacing``.

    Every segment whose 2D (XY) length exceeds ``spacing`` is split into the
    smallest number of equal parts that brings it below the limit, and the
    inserted points are interpolated linearly in x, y **and** z against the
    2D cumulative arc length.  Because the interpolation is exactly the one
    the height spline performs on its own input
    (``np.interp`` over the XY arc length), adding these points leaves the
    elevation profile unchanged.

    Original vertices -- including the first and the last, which may carry a
    junction endpoint override -- are preserved verbatim, and points are only
    ever added, never dropped, so a densely digitised boundary is returned
    unchanged.

    Args:
        points_3d: (N, 3) array of [x, y, z] boundary points.
        spacing: Maximum allowed 2D distance between consecutive points (m).

    Returns:
        (M, 3) array with ``M >= N``, starting and ending on the input's
        first and last point.
    """
    if len(points_3d) < 2 or spacing <= 0:
        return points_3d

    segment_lengths = np.linalg.norm(np.diff(points_3d[:, :2], axis=0), axis=1)
    subdivisions = np.maximum(np.ceil(segment_lengths / spacing).astype(int), 1)
    if not np.any(subdivisions > 1):
        return points_3d

    pieces: List[np.ndarray] = []
    for start, end, count in zip(points_3d[:-1], points_3d[1:], subdivisions):
        fractions = (np.arange(count, dtype=float) / count)[:, np.newaxis]
        pieces.append(start + fractions * (end - start))
    pieces.append(points_3d[-1:])

    return np.concatenate(pieces, axis=0)


class ReferenceLine:
    """
    OpenDRIVE reference line representation.

    The reference line is the center line of a road and serves as the basis
    for defining lane geometry in OpenDRIVE format. It contains a Lane
    instance for compatibility but is specifically designed for reference line purposes.
    """

    def __init__(
        self,
        centerline_2d: Splines,
        height_spline: CubicSpline1D,
        elevation_offset: float = 0.0,
    ):
        """
        Initialize a ReferenceLine with a 2D centerline spline and height spline.

        Args:
            centerline_2d: 2D spline (XY coordinates only) representing the reference line
            height_spline: 1D spline mapping XY arc length (s) to relative elevation (z)
            elevation_offset: Absolute elevation (z coordinate) at the road start point (s=0)
        """
        self.centerline_2d: Splines = centerline_2d
        self.height_spline: CubicSpline1D = height_spline

        # Store the absolute elevation at the road start point (s=0)
        # This is needed for calculating signal z_offsets correctly
        self.elevation_offset = elevation_offset

        # Create a Lane instance for the reference line
        # Import here to avoid circular import
        from .lane import Lane

        # Reference line always has lane_id = 0 and type = none for center lane
        self._lane = Lane(
            lane_id=0,
            lane_type=LaneType.NONE,
            level=False,
        )

    @staticmethod
    def construct_from_lanelet_groups(
        lanelet_map: lanelet2.core.LaneletMap,
        lanelet_group: Union[
            Set[lanelet2.core.Lanelet],
            List[lanelet2.core.Lanelet],
            lanelet2.core.LaneletLayer,
        ],
        traffic_rule: Optional[str] = None,
        routing_graph: Optional[RoutingGraph] = None,
        start_xyz_override: Optional[Tuple[float, float, float]] = None,
        end_xyz_override: Optional[Tuple[float, float, float]] = None,
        offset: CoordinateOffset = CoordinateOffset(),
        start_hdg_override: Optional[float] = None,
        end_hdg_override: Optional[float] = None,
    ) -> "ReferenceLine":
        """
        Construct a ReferenceLine from a group of Lanelet2 lanelets.

        Args:
            lanelet_map: The Lanelet2 map containing the lanelets
            lanelet_group: List of lanelets representing lanes in a road
            traffic_rule: Traffic handedness - "RHT" (Right-Hand Traffic) or "LHT" (Left-Hand Traffic).
                         Defaults to "RHT" if not specified.
                         Both RHT and LHT use leftmost lanelet's left boundary as reference line.
                         The road@rule attribute indicates the traffic direction.
            routing_graph: Optional pre-built routing graph for the same
                ``lanelet_map``. Reused when supplied; a fresh graph is built
                only when ``None``.
            start_xyz_override: Optional (x, y, z) world-frame coordinate that
                replaces the first sampled point of the reference line before
                spline fitting.  Used by the junction phase to pin a
                connecting road's s=0 endpoint to the incoming road's
                endpoint (junction endpoint fidelity).
            end_xyz_override: Optional (x, y, z) world-frame coordinate that
                replaces the last sampled point of the reference line before
                spline fitting.  Used by the junction phase to pin a
                connecting road's s=length endpoint to the outgoing road's
                endpoint.
            offset: Coordinate offset to subtract from extracted boundary
                points. Defaults to a zero offset.
            start_hdg_override: Optional world-frame tangent angle (radians)
                that replaces the *direction* of the fitted spline's s=0
                boundary constraint.  The position override above makes the
                connecting road start at the right point; without this it
                then leaves along its own lanelet's tangent, and OpenDRIVE
                lane borders (``reference(s) + t * normal(s)``) fan out from
                the shared point by ``2 * w * sin(dpsi / 2)`` — the whole of
                the residual ``lane_smoothness`` gap at junction boundaries.
            end_hdg_override: The s=length counterpart of
                ``start_hdg_override``.

        Returns:
            ReferenceLine instance constructed from the center of the lanelet group
        """
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        # Sort the lanelets from left to right
        from ..util import sort_adjacent_groups

        sorted_lanelets = sort_adjacent_groups(
            lanelet_map, lanelet_group, routing_graph
        )

        # Normalize traffic_rule to uppercase, default to RHT
        traffic_rule_normalized = (traffic_rule or "RHT").upper()

        # Validate traffic_rule
        if traffic_rule_normalized not in ("RHT", "LHT"):
            raise ValueError(
                f"Invalid traffic_rule: '{traffic_rule}'. Must be 'RHT' or 'LHT'."
            )

        # Select reference lanelet and boundary based on traffic rule
        if traffic_rule_normalized == "RHT":
            # RHT: Use leftmost lanelet's left boundary.
            # All lanes extend to the right (negative IDs) from this line.
            reference_lanelet = sorted_lanelets[0]
            border = "left"
            boundary = reference_lanelet.leftBound
        else:  # LHT
            # LHT: Use rightmost (innermost) lanelet's right boundary.
            # In LHT, traffic is on the left side of the road.  The innermost lane is
            # adjacent to the road centre divider, whose edge is the rightBound of the
            # rightmost lanelet.  All lanes extend to the left (positive IDs) from
            # this line, matching the OpenDRIVE convention for LHT roads.
            reference_lanelet = sorted_lanelets[-1]
            border = "right"
            boundary = reference_lanelet.rightBound

        # Log reference line selection
        logger.debug(f"Traffic rule: {traffic_rule_normalized}")
        logger.debug(f"Reference lanelet ID: {reference_lanelet.id}")
        logger.debug(f"Using boundary: {border}")
        logger.debug(f"Number of sorted lanelets: {len(sorted_lanelets)}")

        # Extract 3D points directly from the selected boundary (no 3D spline needed)
        # This avoids the 3D-to-2D arc length mapping issues on steep slopes
        from ..util import extract_points_3d

        points_3d = extract_points_3d(boundary, offset=offset)
        logger.debug(f"Boundary points count: {len(points_3d)}")
        logger.debug(
            f"First point (original): [{points_3d[0, 0]:.3f}, {points_3d[0, 1]:.3f}, {points_3d[0, 2]:.3f}]"
        )
        logger.debug(
            f"Last point (original): [{points_3d[-1, 0]:.3f}, {points_3d[-1, 1]:.3f}, {points_3d[-1, 2]:.3f}]"
        )

        # Track if we reversed the boundary points (needed later for velocity vectors)
        boundary_reversed = False

        # Verify and correct boundary direction for LHT (legacy check, no longer needed)
        # Both RHT and LHT now use leftBound, which is in correct direction
        if traffic_rule_normalized == "LHT" and border == "right":
            # Get lanelet centerline direction vector (road direction)
            # Use first and last points of centerline
            centerline_points_list = []
            for point in reference_lanelet.centerline:
                centerline_points_list.append([point.x, point.y])
            centerline_points: np.ndarray = np.array(centerline_points_list)

            if len(centerline_points) >= 2:
                # Centerline direction: from first to last point
                centerline_dir = centerline_points[-1] - centerline_points[0]
                centerline_dir_norm = centerline_dir / (
                    np.linalg.norm(centerline_dir) + 1e-10
                )

                # Boundary direction: from first to last point
                boundary_dir = points_3d[-1, :2] - points_3d[0, :2]
                boundary_dir_norm = boundary_dir / (
                    np.linalg.norm(boundary_dir) + 1e-10
                )

                # Check if directions align (dot product should be positive)
                dot_product = np.dot(centerline_dir_norm, boundary_dir_norm)

                logger.debug(f"Centerline direction: {centerline_dir_norm}")
                logger.debug(f"Boundary direction: {boundary_dir_norm}")
                logger.debug(f"Direction alignment (dot product): {dot_product:.4f}")

                # If dot product is negative, boundary is reversed
                if dot_product < 0:
                    logger.warning(
                        f"LHT rightBound direction is reversed (dot product={dot_product:.4f}). "
                        "Reversing point order to align with road direction."
                    )
                    points_3d = points_3d[::-1]  # Reverse point order
                    boundary_reversed = True
                    logger.debug(
                        f"First point (corrected): [{points_3d[0, 0]:.3f}, {points_3d[0, 1]:.3f}, {points_3d[0, 2]:.3f}]"
                    )
                    logger.debug(
                        f"Last point (corrected): [{points_3d[-1, 0]:.3f}, {points_3d[-1, 1]:.3f}, {points_3d[-1, 2]:.3f}]"
                    )
                else:
                    logger.debug(
                        f"LHT rightBound direction is correct (dot product={dot_product:.4f})"
                    )

        # Apply world-frame endpoint overrides (junction endpoint fidelity).
        # These are set by the junction phase so that a connecting road's first
        # and last points become exactly the world-frame endpoint of the
        # linked incoming / outgoing road reference line.  The spline fitter
        # enforces hard position constraints at s=0 and s=length, so the
        # resulting paramPoly3 will land exactly on these world coordinates.
        if start_xyz_override is not None:
            override = np.asarray(start_xyz_override, dtype=float)
            if override.shape != (3,):
                raise ValueError(
                    f"start_xyz_override must have shape (3,), got {override.shape}"
                )
            points_3d = points_3d.copy()
            points_3d[0] = override
        if end_xyz_override is not None:
            override = np.asarray(end_xyz_override, dtype=float)
            if override.shape != (3,):
                raise ValueError(
                    f"end_xyz_override must have shape (3,), got {override.shape}"
                )
            if start_xyz_override is None:
                points_3d = points_3d.copy()
            points_3d[-1] = override

        # Densify the polyline before fitting.  The spline below is fitted to
        # these points and its control-point count is derived from their
        # number, so a coarsely digitised boundary (Lanelet2 gaps of tens of
        # metres are common) produces an under-parameterised fit that is
        # unconstrained between vertices and bows metres off the polyline.
        # Resampling only inserts points, so the endpoint overrides applied
        # above survive verbatim and dense boundaries are untouched.
        points_3d = resample_points_3d(
            points_3d, DEFAULT_CONFIG.spline.resample_spacing
        )
        logger.debug(f"Boundary points after resampling: {len(points_3d)}")

        # Calculate XY cumulative distances (2D arc length) directly from points
        xy_distances = np.linalg.norm(np.diff(points_3d[:, :2], axis=0), axis=1)
        xy_arc_lengths = np.concatenate(([0], np.cumsum(xy_distances)))

        # Get elevation offset (absolute Z at s=0)
        elevation_offset = points_3d[0, 2]

        # Extract the selected boundary as 2D spline (XY only) for reference line geometry
        # Use corrected points_3d for spline fitting
        from ..centerline import _calculate_centerline_velocity_vector

        # Get velocity vectors for boundary endpoints
        start_vel = np.array(
            _calculate_centerline_velocity_vector(reference_lanelet, at_start=True)[:2]
        )
        end_vel = np.array(
            _calculate_centerline_velocity_vector(reference_lanelet, at_start=False)[:2]
        )

        # If we reversed the boundary points, also reverse the velocity vectors
        if boundary_reversed:
            start_vel, end_vel = -end_vel, -start_vel
            logger.debug("Reversed velocity vectors to match reversed boundary points")

        # Apply the heading overrides (direction half of the junction contact
        # contract).  These must come *after* the reversal above: they are
        # stated in the frame of the emitted reference line, which is what
        # ``boundary_reversed`` has just settled.  ``Splines`` scales its
        # velocity targets by ``t_max`` internally and its own estimates are
        # unit vectors, so a unit vector is the right magnitude to hand it.
        # An override too far from the boundary's own tangent is dropped
        # (the position pin stays): forcing it would bend the line into an
        # S-curve, which is worse than the border gap it would close.
        start_hdg_override = _guard_heading_override(
            start_hdg_override, start_vel, "start"
        )
        end_hdg_override = _guard_heading_override(end_hdg_override, end_vel, "end")
        if start_hdg_override is not None:
            start_vel = np.array(
                [np.cos(start_hdg_override), np.sin(start_hdg_override)],
                dtype=float,
            )
        if end_hdg_override is not None:
            end_vel = np.array(
                [np.cos(end_hdg_override), np.sin(end_hdg_override)], dtype=float
            )

        # Create B-spline directly from corrected points.  When the caller
        # overrode start/end points (junction endpoint fidelity) the
        # override point can be up to a few decimetres away from the second
        # or second-to-last input point.  With the default weight mix
        # (hard=80, soft=20) the LSQ solver trades ~10-20 cm of boundary
        # error for better data fit, which re-introduces the gap we are
        # trying to close.  Boost the hard-constraint weight enough to
        # land well inside the 5 cm test tolerance without distorting the
        # interior fit so much that the resulting reference line strays
        # outside the corridor of the source lanelets — the geometric
        # lanelet→road matcher (road_lanelet_geo_mapping) compares lanelet
        # boundaries against the rendered road geometry and a too-aggressive
        # weight (e.g. 1e6) shifts neighbouring lanelets to a different
        # road in the matching, breaking conversion-vs-geo cross-validation.
        # ``endpoint_pin_weight`` (1e4) gives sub-millimetre endpoint accuracy
        # while keeping the interior within ~1 cm of the default-weight fit.
        has_override = (
            start_xyz_override is not None
            or end_xyz_override is not None
            or start_hdg_override is not None
            or end_hdg_override is not None
        )
        spline_hard_weight: Optional[float] = (
            DEFAULT_CONFIG.spline.endpoint_pin_weight if has_override else None
        )
        centerline_2d = Splines(
            points_3d[:, :2],  # Use corrected XY points
            start_vel=start_vel,
            end_vel=end_vel,
            num_control_points=None,
            hard_constraint_weight=spline_hard_weight,
        )

        # Generate height_spline: mapping from XY arc length (s) to relative elevation (z)
        # Use the original point Z-coordinates directly (no 3D spline sampling needed)
        z_values = points_3d[:, 2]
        relative_elevations_raw = z_values - elevation_offset

        # Resample to desired density (0.1m intervals) for height spline
        total_length_2d = centerline_2d.total_length
        num_samples = max(10, int(np.ceil(total_length_2d / 0.1)) + 1)
        xy_arc_lengths_resampled = np.linspace(0, total_length_2d, num_samples)

        # Interpolate z-coordinates at resampled positions
        relative_elevations = np.interp(
            xy_arc_lengths_resampled, xy_arc_lengths, relative_elevations_raw
        )

        # Create height_spline mapping XY arc length -> relative elevation
        height_spline = CubicSpline1D(
            xy_arc_lengths_resampled, relative_elevations, bc_type="not-a-knot"
        )

        # Create the ReferenceLine instance with 2D centerline, height spline, and elevation offset
        reference_line = ReferenceLine(
            centerline_2d=centerline_2d,
            height_spline=height_spline,
            elevation_offset=elevation_offset,
        )

        # Derive the center-lane road mark from the boundary LineString used
        # for this reference line. For RHT the reference is the leftmost
        # lanelet's leftBound; for LHT it is the rightmost lanelet's
        # rightBound. Fall back to solid/white when no attributes are
        # available.
        try:
            boundary_attrs = dict(boundary.attributes)
        except (AttributeError, TypeError):  # pragma: no cover - defensive
            boundary_attrs = {}
        rm = road_mark_from_linestring_attrs(
            s_offset=0.0,
            attrs=boundary_attrs,
            is_lht=(traffic_rule_normalized == "LHT"),
        )
        reference_line._lane._add_road_mark(rm)

        return reference_line

    @property
    def widths(self) -> List[LaneWidth]:
        """Access to lane widths."""
        return self._lane.widths

    @property
    def road_marks(self) -> List:
        """Access to road marks."""
        return self._lane.road_marks

    @property
    def lane_id(self) -> int:
        """Access to lane ID."""
        return self._lane.lane_id

    @property
    def lane_type(self) -> LaneType:
        """Access to lane type."""
        return self._lane.lane_type

    @property
    def level(self) -> bool:
        """Access to lane level."""
        return self._lane.level

    def get_elevation_profile(
        self, geometry_s_values: list[float] | None = None
    ) -> ElevationProfile:
        """
        Get elevation profile from the reference line.

        The elevation profile uses XY-plane arc length (2D projection) to match ParamPoly3
        geometry coordinates, ensuring that s-coordinates are consistent between the
        reference line geometry and elevation profile.

        Args:
            geometry_s_values: Optional list of s-coordinates for segment boundaries.
                If provided, elevation segments will be aligned with these boundaries
                (typically from ParamPoly3 geometries) to ensure consistency.
                If None, uses the internal height_spline segment boundaries.

        Returns:
            ElevationProfile object containing elevation segments with polynomial coefficients
        """
        if geometry_s_values is None:
            # Fallback: use height_spline segments directly
            elevation_segments = self.height_spline.get_segments()
            elevations = [
                Elevation(s=s_offset, a=a + self.elevation_offset, b=b, c=c, d=d)
                for s_offset, a, b, c, d in elevation_segments
            ]
        else:
            # Create elevation segments aligned with geometry boundaries
            # Use cubic Hermite interpolation to compute polynomial coefficients
            elevations = []
            for i in range(len(geometry_s_values)):
                s_start = geometry_s_values[i]
                s_end = (
                    geometry_s_values[i + 1]
                    if i + 1 < len(geometry_s_values)
                    else self.height_spline.total_length
                )
                segment_length = s_end - s_start

                if segment_length <= 0:
                    continue

                # Evaluate height_spline at segment boundaries
                # Get relative elevation and derivatives
                z_start = self.height_spline.evaluate(s_start, derivative=0)
                dz_start = self.height_spline.evaluate(s_start, derivative=1)
                z_end = self.height_spline.evaluate(s_end, derivative=0)
                dz_end = self.height_spline.evaluate(s_end, derivative=1)

                # Compute cubic polynomial coefficients using Hermite interpolation
                # z(ds) = a + b*ds + c*ds^2 + d*ds^3, where ds = s - s_start
                # Boundary conditions:
                #   z(0) = z_start, z(L) = z_end
                #   z'(0) = dz_start, z'(L) = dz_end
                L = segment_length

                a = z_start
                b = dz_start
                c = (3 * (z_end - z_start) / L - 2 * dz_start - dz_end) / L
                d = (2 * (z_start - z_end) / L + dz_start + dz_end) / (L * L)

                # Add elevation_offset to 'a' to convert to absolute z-coordinate
                elevations.append(
                    Elevation(
                        s=s_start,
                        a=a + self.elevation_offset,
                        b=b,
                        c=c,
                        d=d,
                    )
                )

        return ElevationProfile(elevations=elevations)

    def to_xml(self) -> ET.Element:
        """Convert to XML element via the internal lane."""
        return self._lane.to_xml()

    def __repr__(self) -> str:
        """String representation of the reference line."""
        return f"ReferenceLine(widths={len(self.widths)}, marks={len(self.road_marks)})"
