"""Configuration dataclasses for conversion parameters and constants.

This module centralizes all magic numbers and configurable parameters used
throughout the Lanelet2 to OpenDRIVE conversion process. All constants are
organized into logical dataclasses for easy access and modification.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class GeometryConstants:
    """Constants for geometry calculations and numerical stability.

    Attributes:
        epsilon: Tolerance for numerical stability in geometry calculations
        point_distance_threshold: Minimum distance between distinct points
        divergence_endpoint_tolerance: Maximum allowed distance (m) between a
            diverging/merging road's reference-line endpoint and each candidate
            regular road's mirrored endpoint. Sanity gate for issue #291.
        divergence_min_segment_length: Floor (m) on synthetic divergence/merge
            connecting-road geometry length. Issue #291.
        divergence_default_lane_width: Default width (m) emitted on synthetic
            divergence/merge connecting-road lanes so the rendered XODR
            ``<width>`` element is always present. Issue #291.
        sidewalk_height: Height (m) emitted on ``lane[type=sidewalk]`` and
            ``lane[type=shoulder]`` when the inner boundary LineString has
            ``type=curbstone`` or ``type=road_border``. Both ``inner`` and
            ``outer`` of the emitted ``<height>`` are set to this value, so
            the sidewalk surface sits flat on top of the curb. Issue #469.
        spatial_grid_min_cell_size: Floor (m) on the cell size of the uniform
            grid used to pre-filter candidate lanelet pairs before running an
            exact geometric intersection test. Only guards against a
            degenerate (zero-extent) input; it never affects the result,
            because the grid is a filter and every surviving pair is still
            confirmed geometrically.
        nearest_road_rerank_ulps: Relative width, in units of float64
            machine epsilon, of the band above the vectorised minimum distance
            whose candidates the nearest-road index re-ranks with
            ``math.hypot``. ``np.hypot`` (libm) and ``math.hypot`` (CPython)
            may disagree by about one ulp, so every candidate that could be
            the ``math.hypot`` minimum lies inside this band.
    """

    epsilon: float = 1e-10
    point_distance_threshold: float = 0.001
    divergence_endpoint_tolerance: float = 0.5
    divergence_min_segment_length: float = 0.01
    divergence_default_lane_width: float = 3.5
    sidewalk_height: float = 0.15
    spatial_grid_min_cell_size: float = 1.0
    nearest_road_rerank_ulps: float = 8.0


@dataclass(frozen=True)
class SplineConstants:
    """Constants for B-spline fitting and interpolation.

    Attributes:
        speed_epsilon: Tolerance for speed/velocity magnitude checks
        hard_constraint_weight: Weight for boundary position/velocity constraints
        soft_constraint_weight: Weight for intermediate point data fitting
        knot_alpha_weight: Weight for uniform knot distribution
        knot_beta_weight: Weight for curvature-adaptive knot placement
        position_tolerance: Maximum error for endpoint position constraints
        velocity_tolerance: Maximum error for endpoint velocity constraints
        max_avg_error: Maximum acceptable average fitting error
        max_point_error: Maximum acceptable single-point fitting error
        warn_percentile: Percentile for error reporting in warnings
        arc_length_table_samples: Number of samples for arc length lookup table
        min_control_points: Minimum number of control points for spline fitting
        max_control_points: Maximum number of control points for spline fitting
        control_points_ratio: Ratio of control points to input points (base calculation)
        curvature_threshold: Threshold for high curvature detection (radians)
        curvature_multiplier: Multiplier for control points in high curvature regions
        endpoint_pin_weight: Hard-constraint weight used instead of
            ``hard_constraint_weight`` when a reference-line endpoint position
            or heading is pinned (junction contact).  Gives sub-millimetre
            endpoint accuracy; much larger values (e.g. 1e6) distort the
            interior fit enough to confuse the lanelet-to-road matcher.
        max_heading_override_rad: Largest angle (radians, default ~20 deg)
            by which a pinned endpoint heading may differ from the
            reference line's own boundary tangent.  Beyond it the heading
            pin is dropped for that end (the position pin is kept), since
            forcing it would bend a short connector into an S-curve.
        resample_spacing: Maximum spacing (m) between consecutive input points
            handed to the reference-line spline fitter. Raw Lanelet2
            boundaries are digitised with an arbitrary vertex density -- gaps
            of tens of metres occur in practice -- while both the arc length
            and the dynamic control-point count
            (:func:`~autoware_lanelet2_to_opendrive.spline.compute_dynamic_control_points`)
            are derived from the vertex *count*. A coarsely digitised
            boundary therefore yields an under-parameterised fit that is
            unconstrained between vertices and can bow metres away from the
            polyline it is meant to reproduce. Subdividing every segment
            longer than this value before fitting bounds that error; 1.0 m is
            well below a lane width, so the inserted points pin the curve to
            the polyline without adding meaningful shape of their own. Only ever adds points, so densely digitised boundaries
            are unaffected.
    """

    speed_epsilon: float = 1e-12
    hard_constraint_weight: float = 80.0
    soft_constraint_weight: float = 20.0
    knot_alpha_weight: float = 2.0
    knot_beta_weight: float = 2.0
    position_tolerance: float = 5.0
    velocity_tolerance: float = 15.0
    max_avg_error: float = 2.0
    max_point_error: float = 8.0
    warn_percentile: float = 95.0
    arc_length_table_samples: int = 1000
    min_control_points: int = 5
    max_control_points: int = 50
    control_points_ratio: float = 0.4
    curvature_threshold: float = 0.2
    curvature_multiplier: float = 1.5
    endpoint_pin_weight: float = 1e4
    max_heading_override_rad: float = 0.35
    resample_spacing: float = 1.0


@dataclass(frozen=True)
class CenterlineConstants:
    """Constants for centerline extraction and width estimation.

    Attributes:
        min_sample_points_for_centerline: Minimum number of sample points for centerline extraction
        sample_point_multiplier: Multiplier for calculating sample points based on control points
        boundary_condition_default: Default boundary condition for cubic spline interpolation
        width_interpolation_method: Interpolant fitted to the lane-width
            samples. ``"pchip"`` cannot leave the envelope of its own
            samples; ``"cubic"`` (``not-a-knot``) overshoots it between
            knots. Measured on nishishinjuku, the width emitted beyond the
            range of the fit's own knots fell from p95 428.5 mm, max
            1,886.5 mm to exactly 0.0 mm, and ``"pchip"`` was not worse on
            any of the 892 lanes compared. Elevation keeps ``"cubic"``: it
            is reported as accurate (p95 2.1 mm) and is fitted elsewhere
            (``opendrive/reference_line.py``).
    """

    min_sample_points_for_centerline: int = 20
    sample_point_multiplier: int = 2
    boundary_condition_default: str = "not-a-knot"
    width_interpolation_method: str = "cubic"


@dataclass(frozen=True)
class PreprocessingConstants:
    """Constants for lanelet preprocessing operations.

    Attributes:
        merge_tolerance_default: Default tolerance for lanelet merging
        replace_tolerance_default: Default tolerance for lanelet replacement
        validate_tolerance_default: Default tolerance for continuity validation
    """

    merge_tolerance_default: float = 1e-3
    replace_tolerance_default: float = 1e-3
    validate_tolerance_default: float = 1e-3


@dataclass(frozen=True)
class OpenDriveConstants:
    """Constants for OpenDRIVE format and ID management.

    Issue #132 fix: OpenDRIVE specification allows roads and junctions to share
    the same ID space, but some tools (e.g., CARLA MapBuilder) cannot distinguish
    between them, causing ID collisions. To prevent this, we add an offset to
    junction IDs to ensure they never conflict with road IDs.

    Attributes:
        junction_id_offset: Offset added to all junction IDs to avoid conflicts
                           with road IDs. Default is 1000, which means:
                           - Junction 0 becomes ID 1000
                           - Junction 1 becomes ID 1001
                           - etc.
                           Once the highest road ID reaches the offset, junction
                           IDs start at the next multiple of the offset above it
                           instead, so they never collide with road IDs.
    """

    junction_id_offset: int = 1000


@dataclass(frozen=True)
class ParamPoly3Constants:
    """Constants for ParamPoly3 geometry generation.

    Controls how B-splines are converted to ParamPoly3 segments for OpenDRIVE
    output, preventing issues like zero-length segments that cause CARLA crashes.

    Attributes:
        min_segment_length: Minimum allowed segment length in meters (0.5m default)
                           Segments shorter than this will be rejected; a whole
                           spline shorter than this is emitted as a single
                           straight segment (with a warning) instead.
                           Floor for splitting roads into segments (CARLA, #203)
        default_segment_length: Target segment length for dynamic calculation (1.0m)
                               Used when num_segments is not explicitly specified
        max_segments: Maximum number of segments per road (100 default)
                     Prevents excessive segmentation of very long roads
        min_segments: Minimum number of segments per road (1 default)
                     Ensures at least one segment is created
        coefficient_epsilon: Threshold for rounding small coefficients to zero (1e-8)
                            Prevents numerical instability in paramPoly3
        knot_aligned: Emit one paramPoly3 per B-spline knot span instead of
                     re-approximating the fitted curve on an arbitrary uniform
                     grid.  Inside a knot span the fitted cubic B-spline *is* a
                     cubic polynomial, so the emitted paramPoly3 reproduces it
                     exactly (position error at machine precision, no curvature
                     jump at the piece boundaries).  Subdivision is only ever
                     applied *inside* a span, which keeps that exactness.
        knot_span_max_length: Optional cap (m) on the arc length of a single
                     emitted piece.  ``None`` means "never subdivide a knot
                     span"; a float subdivides a longer span into equal
                     parameter slices (still exact).  A cap costs file size for
                     no accuracy gain, so the default is ``None``.
        arc_length_panels / arc_length_nodes: composite Gauss-Legendre rule used
                     to compute the emitted ``@length`` as the true arc length
                     of the emitted cubic.  This is what makes the declared
                     length agree with the ASAM
                     ``road.geometry.parampoly3.*_range`` checkers, which
                     integrate |(u', v')| over the declared parameter range.
        param_inversion_iterations: Newton steps used to invert "distance along
                     the piece" -> normalized parameter p when sampling an
                     emitted paramPoly3 (see
                     ``opendrive.geometry.param_for_offset``).
    """

    min_segment_length: float = 0.5
    default_segment_length: float = 1.0
    max_segments: int = 100
    min_segments: int = 1
    coefficient_epsilon: float = 1e-8
    knot_aligned: bool = True
    knot_span_max_length: Optional[float] = None
    arc_length_panels: int = 8
    arc_length_nodes: int = 10
    param_inversion_iterations: int = 3


@dataclass(frozen=True)
class ArcSpiralConstants:
    """Internal numeric tunables for the arc/spiral classifier.

    Not user-facing — these govern numeric stability and sampling
    granularity of the curvature classification pass introduced for
    issue #466 (arc emission).

    Attributes:
        classification_step: Curvature sampling stride along arc length (m).
        lookahead_steps: How many strides the paramPoly3 fallback advances
            before emitting one run.
        arc_fit_max_bisect: Number of window-shrink retries when arc fit
            validation fails.
    """

    classification_step: float = 0.5
    lookahead_steps: int = 4
    arc_fit_max_bisect: int = 8


@dataclass(frozen=True)
class UtmConstants:
    """Constants for standard UTM projection parameters.

    Used to build the MGRS-grid-relative geoReference PROJ string (issue
    #550): Autoware's ``MGRSProjector`` emits coordinates relative to the
    south-west corner of the point's 100 km MGRS grid square
    (``fmod(utm_easting, 1e5)`` / ``fmod(utm_northing, 1e5)``), which
    ``+proj=utm`` cannot express -- it silently ignores ``+x_0``/``+y_0``
    (and even ``+lat_0``/``+lon_0``) overrides. Building the projection
    explicitly as ``+proj=tmerc`` with these standard UTM parameters plus a
    grid-square-relative false easting/northing reproduces it exactly.

    Attributes:
        scale_factor: Central-meridian scale factor (k) for standard UTM.
        false_easting: Standard UTM false easting (m).
        false_northing_south: Standard UTM false northing for the southern
            hemisphere (m); the northern hemisphere uses 0.
        mgrs_grid_square_size: Side length (m) of an MGRS 100 km grid
            square; ``MGRSProjector`` emits ``fmod(utm, size)``.
        zone_width_deg: Width (deg) of a UTM longitude zone.
        zone_count: Number of UTM longitude zones.
        param_tolerance: Absolute tolerance used when matching parsed PROJ
            parameters (``+lat_0``, ``+lon_0``, ``+k_0``) against the
            standard UTM values above.
    """

    scale_factor: float = 0.9996
    false_easting: float = 500_000.0
    false_northing_south: float = 10_000_000.0
    mgrs_grid_square_size: float = 100_000.0
    zone_width_deg: float = 6.0
    zone_count: int = 60
    param_tolerance: float = 1e-9


@dataclass(frozen=True)
class ProjectionConstants:
    """Constants for resolving the map projection.

    Attributes:
        transverse_mercator_default_scale_factor: Central-meridian scale
            factor (k0) used when a TransverseMercator
            ``map_projector_info.yaml`` omits ``scale_factor``. Matches
            Autoware's projector, which uses
            ``GeographicLib::TransverseMercator::UTM()`` (k0 = 0.9996), and
            the default of the pinned lanelet2 ``TransverseMercatorProjector``
            binding. Dimensionless.
    """

    transverse_mercator_default_scale_factor: float = 0.9996


@dataclass(frozen=True)
class GeoMappingConstants:
    """Constants for the geometric lanelet -> (road_id, lane_id) mapping.

    These govern the post-conversion cross-validation pass in
    :mod:`autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping`, which
    re-derives the mapping from raw geometry and compares it against the
    mapping produced during conversion.

    Attributes:
        match_threshold: Maximum mean distance (m) between a road reference
            line and a lanelet boundary for them to be considered a match.
            It must be larger than the spline fitting error
            (:attr:`SplineConstants.max_avg_error`, 2.0 m) and smaller than
            a typical lane width so that an adjacent boundary is never
            mistaken for the correct one.  The current value sits at the
            upper end of that window: Japanese urban lanes are ~3.0 m wide
            in practice, so 3.5 m does admit the neighbouring boundary as a
            candidate.  Narrowing it trades false matches for unmatched
            roads, which is why it is exposed here rather than silently
            retuned — see issue discussion in the module docstring.
        rescue_threshold_factor: Multiplier applied to ``match_threshold``
            for the relaxed second-chance search over roads dropped during
            conflict resolution.
    """

    match_threshold: float = 3.5
    rescue_threshold_factor: float = 1.5


@dataclass(frozen=True)
class ConversionConstants:
    """Main container for all internal constants used in the conversion process.

    This dataclass provides centralized access to all numerical constants and
    thresholds used throughout the Lanelet2 to OpenDRIVE conversion. These are
    internal algorithm parameters, not user-configurable settings.

    Example:
        ```python
        from autoware_lanelet2_to_opendrive.config import ConversionConstants

        config = ConversionConstants()

        # Access geometry constants
        if distance < config.geometry.epsilon:
            # handle near-zero case
            pass

        # Access spline constants
        spline = Splines(
            points,
            num_control_points=10,
            hard_weight=config.spline.hard_constraint_weight,
            soft_weight=config.spline.soft_constraint_weight,
        )

        # Access preprocessing constants
        merged = merge_lanelets(
            map, ids, tolerance=config.preprocessing.merge_tolerance_default
        )
        ```

    Attributes:
        geometry: Geometry calculation constants
        spline: Spline fitting constants
        centerline: Centerline extraction constants
        preprocessing: Preprocessing operation constants
        opendrive: OpenDRIVE format constants
        parampoly3: ParamPoly3 geometry generation constants
        arcspiral: Arc/spiral classifier internal tunables
        utm: Standard UTM projection constants
        projection: Map projection resolution constants
        geo_mapping: Geometric mapping cross-validation constants
    """

    geometry: GeometryConstants = GeometryConstants()
    spline: SplineConstants = SplineConstants()
    centerline: CenterlineConstants = CenterlineConstants()
    preprocessing: PreprocessingConstants = PreprocessingConstants()
    opendrive: OpenDriveConstants = OpenDriveConstants()
    parampoly3: ParamPoly3Constants = ParamPoly3Constants()
    arcspiral: ArcSpiralConstants = ArcSpiralConstants()
    utm: UtmConstants = UtmConstants()
    projection: ProjectionConstants = ProjectionConstants()
    geo_mapping: GeoMappingConstants = GeoMappingConstants()


@dataclass(frozen=True)
class CoordinateOffset:
    """Immutable coordinate offset applied to Lanelet2 coordinates.

    This value holds the coordinate offset that is subtracted from
    Lanelet2 coordinates during OpenDRIVE export to convert them to local
    coordinates. It is threaded explicitly through the conversion call
    chain (originating from :class:`~.projection_resolver.ResolvedProjection`
    / :class:`~.conversion_config.OriginSpec`) rather than read from a
    mutable global.

    When the offset is non-zero (e.g., from MGRS grid + offset in config),
    all coordinates in the output xodr file are shifted by subtracting
    these offset values, making them relative to the offset point.

    Attributes:
        x: Easting offset in meters to subtract from X coordinates
        y: Northing offset in meters to subtract from Y coordinates
        z: Altitude offset in meters to subtract from Z coordinates
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @property
    def is_active(self) -> bool:
        """Check if any offset is active (non-zero)."""
        return self.x != 0.0 or self.y != 0.0 or self.z != 0.0


# Global default configuration instance
# This can be imported and used directly throughout the codebase
DEFAULT_CONFIG = ConversionConstants()
