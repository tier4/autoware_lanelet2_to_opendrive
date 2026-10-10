"""OpenDRIVE geometry definitions."""

from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, List, Optional, Sequence, Tuple, TYPE_CHECKING
import lxml.etree as ET
import numpy as np

from .enums import GeometryType
from .xml_utils import replace_subnormal

if TYPE_CHECKING:
    from ..spline import Splines
    from ..conversion_config import ParamPoly3Config


#: ``paramPoly3@pRange`` values defined by ASAM OpenDRIVE.
PARAM_RANGE_ARC_LENGTH = "arcLength"
PARAM_RANGE_NORMALIZED = "normalized"


@lru_cache(maxsize=8)
def _gauss_legendre_unit(panels: int, nodes: int) -> Tuple[np.ndarray, np.ndarray]:
    """Composite Gauss-Legendre abscissae/weights for the unit interval."""
    x, w = np.polynomial.legendre.leggauss(nodes)
    edges = np.linspace(0.0, 1.0, panels + 1)
    half = 0.5 / panels
    centers = 0.5 * (edges[:-1] + edges[1:])
    abscissae = (centers[:, None] + half * x[None, :]).ravel()
    weights = np.tile(w * half, panels)
    return abscissae, weights


def param_poly3_speed(
    coeffs: Sequence[float], p: "np.ndarray | float"
) -> "np.ndarray | float":
    """Return ``|(du/dp, dv/dp)|`` of a paramPoly3 at parameter ``p``.

    Args:
        coeffs: ``(aU, bU, cU, dU, aV, bV, cV, dV)``.
        p: Parameter value(s) in the geometry's own ``pRange``.

    Returns:
        The parametric speed, i.e. the integrand of the arc-length integral
        that ASAM's ``road.geometry.parampoly3.*_range`` checkers evaluate.
    """
    _, bU, cU, dU, _, bV, cV, dV = coeffs
    du = bU + 2.0 * cU * p + 3.0 * dU * p * p
    dv = bV + 2.0 * cV * p + 3.0 * dV * p * p
    return np.hypot(du, dv)


def param_poly3_arc_length(
    coeffs: Sequence[float],
    p_end: float,
    p_start: float = 0.0,
    panels: Optional[int] = None,
    nodes: Optional[int] = None,
) -> float:
    """Return the true arc length of a paramPoly3 between two parameters.

    Computed with a composite Gauss-Legendre rule, which converges to machine
    precision on the smooth ``sqrt(quartic)`` integrand.  The emitted
    ``geometry@length`` is set from this function so that the declared length
    matches the curve the coefficients actually describe -- the condition both
    ``road.geometry.parampoly3.arclength_range`` (``pRange="arcLength"``) and
    ``road.geometry.parampoly3.normalized_range`` (``pRange="normalized"``)
    check to a 1 mm tolerance.
    """
    from ..config import DEFAULT_CONFIG

    if panels is None:
        panels = DEFAULT_CONFIG.parampoly3.arc_length_panels
    if nodes is None:
        nodes = DEFAULT_CONFIG.parampoly3.arc_length_nodes

    span = float(p_end) - float(p_start)
    if span == 0.0:
        return 0.0
    abscissae, weights = _gauss_legendre_unit(panels, nodes)
    p = p_start + span * abscissae
    return float(np.dot(weights, param_poly3_speed(coeffs, p)) * span)


def end_param(geom: "GeometryBase") -> float:
    """Return the parameter value at the *end* of a planView geometry.

    ``length`` for every primitive whose parameter is arc length, and ``1.0``
    for a ``paramPoly3`` declared ``pRange="normalized"``.
    """
    if isinstance(geom, ParamPoly3) and geom.pRange == PARAM_RANGE_NORMALIZED:
        return 1.0
    return float(geom.length)


def param_for_offset(geom: "GeometryBase", ds: float) -> float:
    """Return the curve parameter ``p`` reached ``ds`` metres into ``geom``.

    For arc-length-parameterised primitives this is the identity.  For a
    ``pRange="normalized"`` paramPoly3 the mapping is genuinely non-linear, so
    it is inverted by Newton iteration on the arc-length integral (the same
    thing CARLA does internally by tabulating ``s`` in
    ``GeometryParamPoly3::PreComputeSpline``).  Sampling with the naive
    ``ds / length`` guess instead would reintroduce exactly the mis-location
    that ``pRange="arcLength"`` was falsely promising not to have.
    """
    if not isinstance(geom, ParamPoly3) or geom.pRange != PARAM_RANGE_NORMALIZED:
        return float(ds)

    from ..config import DEFAULT_CONFIG

    length = float(geom.length)
    if length <= 0.0:
        return 0.0

    coeffs = geom.coefficients()
    target = float(np.clip(ds, 0.0, length))
    p = target / length
    for _ in range(DEFAULT_CONFIG.parampoly3.param_inversion_iterations):
        travelled = param_poly3_arc_length(coeffs, p, panels=1)
        speed = float(param_poly3_speed(coeffs, p))
        if speed <= DEFAULT_CONFIG.geometry.epsilon:
            break
        p = float(np.clip(p + (target - travelled) / speed, 0.0, 1.0))
    return p


def evaluate_plan_view_world(
    x: float,
    y: float,
    hdg: float,
    p: float,
    param_poly3_coeffs: Optional[
        Tuple[float, float, float, float, float, float, float, float]
    ] = None,
    arc_curvature: Optional[float] = None,
) -> Tuple[float, float]:
    """Evaluate a planView geometry at parameter ``p`` in the world XY frame.

    Supports ``<line/>``, ``<paramPoly3>`` and ``<arc>`` geometries. The
    ``arc_curvature`` and ``param_poly3_coeffs`` arguments are mutually
    exclusive; passing both raises ``ValueError``. With both set to
    ``None`` the function falls back to a straight line along ``hdg``
    (``<line/>`` semantics). ``arc_curvature`` magnitudes below
    ``DEFAULT_CONFIG.geometry.epsilon`` are also treated as a straight
    line to avoid 1/κ singularity.

    Args:
        x: Geometry start X (world frame).
        y: Geometry start Y (world frame).
        hdg: Geometry heading at start (radians).
        p: Curve parameter at which to evaluate. ``0`` gives the start;
            the end is ``length`` for ``<line/>``/``<arc>`` and for a
            ``pRange="arcLength"`` paramPoly3, but ``1`` for a
            ``pRange="normalized"`` one -- use :func:`end_param` /
            :func:`param_for_offset` rather than assuming.
        param_poly3_coeffs: Optional ``(aU, bU, cU, dU, aV, bV, cV, dV)``.
        arc_curvature: Optional constant curvature κ (1/m). Positive κ
            curves to the left of the start heading; negative to the right.

    Returns:
        Tuple ``(wx, wy)`` with the evaluated world coordinates.
    """
    from ..config import DEFAULT_CONFIG

    if param_poly3_coeffs is not None and arc_curvature is not None:
        raise ValueError(
            "evaluate_plan_view_world: param_poly3_coeffs and arc_curvature "
            "are mutually exclusive"
        )

    cos_hdg = np.cos(hdg)
    sin_hdg = np.sin(hdg)

    if (
        arc_curvature is not None
        and abs(arc_curvature) > DEFAULT_CONFIG.geometry.epsilon
    ):
        kappa = arc_curvature
        dhdg = kappa * p
        local_u = np.sin(dhdg) / kappa
        local_v = (1.0 - np.cos(dhdg)) / kappa
        wx = x + local_u * cos_hdg - local_v * sin_hdg
        wy = y + local_u * sin_hdg + local_v * cos_hdg
        return (wx, wy)

    if param_poly3_coeffs is not None:
        aU, bU, cU, dU, aV, bV, cV, dV = param_poly3_coeffs
        local_u = aU + bU * p + cU * p * p + dU * p * p * p
        local_v = aV + bV * p + cV * p * p + dV * p * p * p
        wx = x + local_u * cos_hdg - local_v * sin_hdg
        wy = y + local_u * sin_hdg + local_v * cos_hdg
        return (wx, wy)

    # Fallback: straight line along heading (also covers <line/>).
    wx = x + p * cos_hdg
    wy = y + p * sin_hdg
    return (wx, wy)


def evaluate_road_endpoints(
    root: ET._Element,
) -> Dict[int, Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """Evaluate the 3D start and end position of every ``<road>`` in a tree.

    For each road under the given OpenDRIVE root element this function walks
    the ``planView`` (paramPoly3 / line geometries) to obtain the XY reference
    line start and end, and then samples the ``elevationProfile`` at the same
    s-coordinates to obtain the matching Z.

    The helper exists primarily for tests that verify junction endpoint
    fidelity: the connecting-road start and end should land on the linked
    incoming / outgoing road endpoints in world frame.

    Args:
        root: ``<OpenDRIVE>`` element (or any element whose direct ``road``
            children are OpenDRIVE roads).

    Returns:
        Mapping ``road_id -> ((x0, y0, z0), (x1, y1, z1))`` where the first
        tuple is the s=0 endpoint and the second tuple is the s=length
        endpoint.  Roads without any geometry are omitted.
    """
    results: Dict[
        int, Tuple[Tuple[float, float, float], Tuple[float, float, float]]
    ] = {}

    for road_elem in root.findall("road"):
        road_id_str = road_elem.get("id")
        if road_id_str is None:
            continue
        road_id = int(road_id_str)

        plan_view = road_elem.find("planView")
        if plan_view is None:
            continue

        geometries = plan_view.findall("geometry")
        if not geometries:
            continue

        first_geom = geometries[0]
        start_xy = _eval_geometry_world(first_geom, p=0.0)
        last_geom = geometries[-1]
        last_length = float(last_geom.get("length", "0.0"))
        end_xy = _eval_geometry_world(last_geom, p=element_end_param(last_geom))
        if start_xy is None or end_xy is None:
            continue

        # Total s range of the reference line.
        s_start = float(first_geom.get("s", "0.0"))
        s_end = float(last_geom.get("s", "0.0")) + last_length

        elevation_profile = road_elem.find("elevationProfile")
        z_start = _eval_elevation_at_s(elevation_profile, s_start)
        z_end = _eval_elevation_at_s(elevation_profile, s_end)

        results[road_id] = (
            (start_xy[0], start_xy[1], z_start),
            (end_xy[0], end_xy[1], z_end),
        )

    return results


def element_end_param(geom_elem: ET._Element) -> float:
    """``end_param`` for a raw lxml ``<geometry>`` element.

    Use this instead of ``float(geom.get("length"))`` whenever a parsed
    ``<geometry>`` is evaluated at its end: a ``pRange="normalized"``
    paramPoly3 ends at ``p = 1``, and feeding it ``length`` instead
    extrapolates the cubic far outside the road.
    """
    param_poly3 = geom_elem.find("paramPoly3")
    if (
        param_poly3 is not None
        and param_poly3.get("pRange", PARAM_RANGE_ARC_LENGTH) == PARAM_RANGE_NORMALIZED
    ):
        return 1.0
    return float(geom_elem.get("length", "0.0"))


def _eval_geometry_world(
    geom_elem: ET._Element, p: float
) -> Optional[Tuple[float, float]]:
    """Evaluate a lxml planView ``<geometry>`` element at arc-length ``p``.

    Thin lxml-attribute adapter around :func:`evaluate_plan_view_world`.
    Supports ``<paramPoly3>``, ``<line>`` and ``<arc>`` geometries; spiral
    would need the shared kernel further extended.
    """
    try:
        x = float(geom_elem.get("x"))
        y = float(geom_elem.get("y"))
        hdg = float(geom_elem.get("hdg"))
    except (TypeError, ValueError):
        return None

    arc = geom_elem.find("arc")
    param_poly3 = geom_elem.find("paramPoly3")

    arc_curvature: Optional[float] = None
    if arc is not None:
        try:
            arc_curvature = float(arc.get("curvature"))
        except (TypeError, ValueError):
            arc_curvature = None

    coeffs: Optional[Tuple[float, float, float, float, float, float, float, float]] = (
        None
    )
    if param_poly3 is not None:
        coeffs = (
            float(param_poly3.get("aU", "0.0")),
            float(param_poly3.get("bU", "0.0")),
            float(param_poly3.get("cU", "0.0")),
            float(param_poly3.get("dU", "0.0")),
            float(param_poly3.get("aV", "0.0")),
            float(param_poly3.get("bV", "0.0")),
            float(param_poly3.get("cV", "0.0")),
            float(param_poly3.get("dV", "0.0")),
        )

    return evaluate_plan_view_world(x, y, hdg, p, coeffs, arc_curvature)


def _eval_elevation_at_s(elevation_profile: Optional[ET._Element], s: float) -> float:
    """Evaluate the absolute road-surface elevation at ``s``.

    Uses the piecewise cubic ``<elevation>`` segments under the given
    profile.  Returns 0.0 when no profile is present.
    """
    if elevation_profile is None:
        return 0.0

    z = 0.0
    for elev in elevation_profile.findall("elevation"):
        s_off = float(elev.get("s", "0.0"))
        if s_off > s:
            break
        a = float(elev.get("a", "0.0"))
        b = float(elev.get("b", "0.0"))
        c = float(elev.get("c", "0.0"))
        d = float(elev.get("d", "0.0"))
        ds = s - s_off
        z = a + b * ds + c * ds * ds + d * ds * ds * ds

    return z


@dataclass
class GeometryBase:
    """Base class for geometry records in planView."""

    s: float = 0.0  # s-coordinate of the start position
    x: float = 0.0  # x-coordinate of the start position
    y: float = 0.0  # y-coordinate of the start position
    hdg: float = 0.0  # heading at the start position (radians)
    length: float = 0.0  # length of the geometry segment

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("geometry")
        elem.set("s", str(self.s))
        elem.set("x", str(self.x))
        elem.set("y", str(self.y))
        elem.set("hdg", str(self.hdg))
        elem.set("length", str(self.length))
        return elem


@dataclass
class Line(GeometryBase):
    """Straight line geometry."""

    geometry_type = GeometryType.LINE

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = super().to_xml()
        ET.SubElement(elem, "line")
        return elem

    @classmethod
    def from_spline_window(
        cls, spline: "Splines", s_start: float, s_end: float
    ) -> "Line":
        """Build a Line covering arc-length [s_start, s_end] of ``spline``.

        Heading is taken from the spline tangent at ``s_start`` so that
        adjacent primitives are G1-continuous when their bounds match.
        """
        start = spline.evaluate(s_start, derivative=0)
        tangent = spline.evaluate(s_start, derivative=1)
        return cls(
            s=s_start,
            x=float(start[0]),
            y=float(start[1]),
            hdg=float(np.arctan2(tangent[1], tangent[0])),
            length=float(s_end - s_start),
        )


@dataclass
class Arc(GeometryBase):
    """Arc geometry with constant curvature."""

    curvature: float = 0.0  # constant curvature (1/radius)
    geometry_type = GeometryType.ARC

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = super().to_xml()
        arc_elem = ET.SubElement(elem, "arc")
        arc_elem.set("curvature", str(self.curvature))
        return elem

    @classmethod
    def from_spline_window(
        cls,
        spline: "Splines",
        s_start: float,
        s_end: float,
        curvature: float,
    ) -> "Arc":
        """Build an Arc covering arc-length [s_start, s_end] of ``spline``.

        ``curvature`` must be the constant κ chosen by the classifier
        (typically ``Splines.evaluate(s_start, derivative=2)`` projected
        appropriately). Heading and start position come from the spline.
        """
        start = spline.evaluate(s_start, derivative=0)
        tangent = spline.evaluate(s_start, derivative=1)
        return cls(
            s=s_start,
            x=float(start[0]),
            y=float(start[1]),
            hdg=float(np.arctan2(tangent[1], tangent[0])),
            length=float(s_end - s_start),
            curvature=float(curvature),
        )


@dataclass
class Spiral(GeometryBase):
    """Spiral/clothoid geometry."""

    curvStart: float = 0.0  # curvature at start
    curvEnd: float = 0.0  # curvature at end
    geometry_type = GeometryType.SPIRAL

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = super().to_xml()
        spiral_elem = ET.SubElement(elem, "spiral")
        spiral_elem.set("curvStart", str(self.curvStart))
        spiral_elem.set("curvEnd", str(self.curvEnd))
        return elem


@dataclass
class ParamPoly3(GeometryBase):
    """Parametric cubic polynomial geometry."""

    aU: float = 0.0  # coefficient a for u coordinate
    bU: float = 0.0  # coefficient b for u coordinate
    cU: float = 0.0  # coefficient c for u coordinate
    dU: float = 0.0  # coefficient d for u coordinate
    aV: float = 0.0  # coefficient a for v coordinate
    bV: float = 0.0  # coefficient b for v coordinate
    cV: float = 0.0  # coefficient c for v coordinate
    dV: float = 0.0  # coefficient d for v coordinate
    pRange: str = PARAM_RANGE_ARC_LENGTH  # arcLength or normalized
    geometry_type = GeometryType.PARAMPOLY3

    def coefficients(
        self,
    ) -> Tuple[float, float, float, float, float, float, float, float]:
        """Return ``(aU, bU, cU, dU, aV, bV, cV, dV)``."""
        return (
            self.aU,
            self.bU,
            self.cU,
            self.dU,
            self.aV,
            self.bV,
            self.cV,
            self.dV,
        )

    def arc_length(self) -> float:
        """Return the true arc length of this segment over its ``pRange``."""
        return param_poly3_arc_length(self.coefficients(), end_param(self))

    @staticmethod
    def _calculate_optimal_num_segments(
        total_length: float,
        min_segment_length: Optional[float] = None,
        default_segment_length: Optional[float] = None,
        max_segments: Optional[int] = None,
        min_segments: Optional[int] = None,
    ) -> int:
        """
        Calculate optimal number of segments based on road length.

        Ensures segments are never shorter than min_segment_length by
        dynamically adjusting the number of segments.

        Args:
            total_length: Total arc length of the spline
            min_segment_length: Minimum allowed segment length (default from config)
            default_segment_length: Target segment length (default from config)
            max_segments: Maximum allowed segments (default from config)
            min_segments: Minimum required segments (default from config)

        Returns:
            Optimal number of segments (clamped to [min_segments, max_segments])

        Examples:
            >>> _calculate_optimal_num_segments(10.0)
            10  # 10 segments of 1.0m each

            >>> _calculate_optimal_num_segments(0.53)  # Problematic case
            1  # 1 segment of 0.53m (above 0.5m minimum)

            >>> _calculate_optimal_num_segments(150.0)
            100  # Capped at max_segments
        """
        from ..config import DEFAULT_CONFIG

        # Use config defaults if not provided
        if min_segment_length is None:
            min_segment_length = DEFAULT_CONFIG.parampoly3.min_segment_length
        if default_segment_length is None:
            default_segment_length = DEFAULT_CONFIG.parampoly3.default_segment_length
        if max_segments is None:
            max_segments = DEFAULT_CONFIG.parampoly3.max_segments
        if min_segments is None:
            min_segments = DEFAULT_CONFIG.parampoly3.min_segments

        # Edge case: zero or negative length
        if total_length <= 0:
            return min_segments

        # Calculate based on target segment length
        num_segments_by_target = int(np.ceil(total_length / default_segment_length))

        # Calculate maximum segments that maintain minimum length
        max_segments_by_min_length = int(np.floor(total_length / min_segment_length))

        # Take the minimum of the two constraints
        num_segments = min(num_segments_by_target, max_segments_by_min_length)

        # Clamp to valid range
        num_segments = max(min_segments, min(num_segments, max_segments))

        return num_segments

    @staticmethod
    def _normalize_coefficients(
        aU: float,
        bU: float,
        cU: float,
        dU: float,
        aV: float,
        bV: float,
        cV: float,
        dV: float,
        epsilon: Optional[float] = None,
    ) -> tuple:
        """
        Normalize paramPoly3 coefficients by rounding very small values to zero.

        Prevents numerical instability and improves output quality by eliminating
        coefficients that are effectively zero due to floating-point precision.

        Args:
            aU, bU, cU, dU: U-coordinate polynomial coefficients
            aV, bV, cV, dV: V-coordinate polynomial coefficients
            epsilon: Threshold below which coefficients are set to zero

        Returns:
            Tuple of normalized coefficients (aU, bU, cU, dU, aV, bV, cV, dV)
        """
        from ..config import DEFAULT_CONFIG

        if epsilon is None:
            epsilon = DEFAULT_CONFIG.parampoly3.coefficient_epsilon

        def normalize(val: float) -> float:
            return 0.0 if abs(val) < epsilon else val

        return (
            normalize(aU),
            normalize(bU),
            normalize(cU),
            normalize(dU),
            normalize(aV),
            normalize(bV),
            normalize(cV),
            normalize(dV),
        )

    @staticmethod
    def _validate_segment(
        segment: "ParamPoly3", min_segment_length: Optional[float] = None
    ) -> tuple:
        """
        Validate a ParamPoly3 segment for numerical stability and correctness.

        Args:
            segment: ParamPoly3 segment to validate
            min_segment_length: Minimum allowed segment length (default from config)

        Returns:
            Tuple of (is_valid, error_message)
            - is_valid: True if segment passes all checks
            - error_message: Description of failure (empty if valid)

        Validation checks:
            1. Length is positive and above minimum threshold
            2. All coefficients are finite (not NaN or Inf)
            3. Heading is within valid range [-2π, 2π]
            4. Position coordinates are finite
        """
        # Load default config if not provided
        if min_segment_length is None:
            from ..config import DEFAULT_CONFIG

            min_segment_length = DEFAULT_CONFIG.parampoly3.min_segment_length

        # Check length
        min_length = min_segment_length
        if segment.length < min_length:
            return (
                False,
                f"Segment length {segment.length:.6f}m below minimum {min_length}m",
            )

        if not np.isfinite(segment.length):
            return False, f"Segment length is not finite: {segment.length}"

        # Check coefficients are finite
        coeffs = [
            segment.aU,
            segment.bU,
            segment.cU,
            segment.dU,
            segment.aV,
            segment.bV,
            segment.cV,
            segment.dV,
        ]
        if not all(np.isfinite(c) for c in coeffs):
            return False, "One or more coefficients are not finite (NaN or Inf)"

        # Check heading is reasonable
        if not np.isfinite(segment.hdg):
            return False, f"Heading is not finite: {segment.hdg}"

        if abs(segment.hdg) > 2 * np.pi:
            return (
                False,
                f"Heading {segment.hdg:.3f} outside valid range [-2π, 2π]",
            )

        # Check position is finite
        if not (np.isfinite(segment.x) and np.isfinite(segment.y)):
            return False, f"Position ({segment.x}, {segment.y}) is not finite"

        return True, ""

    @classmethod
    def exact_from_spline_span(
        cls,
        spline: "Splines",
        t_start: float,
        t_end: float,
        s_start: float,
        coefficient_epsilon: Optional[float] = None,
    ) -> "ParamPoly3":
        """Build the ParamPoly3 that *reproduces* ``spline`` on ``[t_start, t_end]``.

        ``t_start`` and ``t_end`` must lie inside a single knot span of the
        fitted cubic B-spline (see :meth:`Splines.breakpoints`).  There the
        spline is one cubic polynomial in ``t``, so its restriction to the
        window, reparameterised by ``p = (t - t_start) / (t_end - t_start)``,
        is again exactly cubic and is written down by a 3-term Taylor
        expansion -- no fitting, no residual:

            u(p), v(p) = R(-hdg) * sum_{n=1..3} C^(n)(t_start) * dt^n * p^n / n!

        The emitted geometry therefore has *zero* position error against the
        fitted reference line and, because neighbouring windows share the
        spline's own C2 continuity, no curvature jump at the seam.  This
        replaces the previous cubic-Hermite re-approximation, which discarded
        the second derivatives and split independently of the knots.

        ``pRange`` is ``"normalized"`` because that is what the coefficients
        express: ``p`` is the spline's own parameter mapped to ``[0, 1]``, not
        travelled distance.  ``length`` is the true arc length of the emitted
        cubic, so the declaration is consistent with ASAM's
        ``road.geometry.parampoly3.normalized_range`` /
        ``...length_match`` checkers.

        Args:
            spline: Fitted reference-line spline.
            t_start: Window start in the spline's normalized parameter.
            t_end: Window end in the spline's normalized parameter.
            s_start: Arc-length offset to record in ``geometry@s``.
            coefficient_epsilon: Small-coefficient rounding threshold.
        """
        from ..config import DEFAULT_CONFIG

        if coefficient_epsilon is None:
            coefficient_epsilon = DEFAULT_CONFIG.parampoly3.coefficient_epsilon

        dt = float(t_end) - float(t_start)
        origin = spline.evaluate_param(t_start, derivative=0)
        d1 = spline.evaluate_param(t_start, derivative=1)
        d2 = spline.evaluate_param(t_start, derivative=2)
        d3 = spline.evaluate_param(t_start, derivative=3)

        if float(np.hypot(d1[0], d1[1])) <= DEFAULT_CONFIG.geometry.epsilon:
            # Degenerate tangent: fall back to the chord direction so the
            # local frame is still well defined.
            end = spline.evaluate_param(t_end, derivative=0)
            hdg = float(np.arctan2(end[1] - origin[1], end[0] - origin[0]))
        else:
            hdg = float(np.arctan2(d1[1], d1[0]))
        cos_hdg, sin_hdg = np.cos(hdg), np.sin(hdg)

        scaled = [
            (float(d1[0]) * dt, float(d1[1]) * dt),
            (float(d2[0]) * dt * dt / 2.0, float(d2[1]) * dt * dt / 2.0),
            (float(d3[0]) * dt**3 / 6.0, float(d3[1]) * dt**3 / 6.0),
        ]
        local = [
            (gx * cos_hdg + gy * sin_hdg, -gx * sin_hdg + gy * cos_hdg)
            for gx, gy in scaled
        ]

        aU, bU, cU, dU, aV, bV, cV, dV = cls._normalize_coefficients(
            0.0,
            local[0][0],
            local[1][0],
            local[2][0],
            0.0,
            local[0][1],
            local[1][1],
            local[2][1],
            epsilon=coefficient_epsilon,
        )

        # @length is computed from the *emitted* (already rounded) coefficients
        # so the XML is self-consistent.
        length = param_poly3_arc_length((aU, bU, cU, dU, aV, bV, cV, dV), 1.0)

        return cls(
            s=float(s_start),
            x=float(origin[0]),
            y=float(origin[1]),
            hdg=hdg,
            length=length,
            aU=aU,
            bU=bU,
            cU=cU,
            dU=dU,
            aV=aV,
            bV=bV,
            cV=cV,
            dV=dV,
            pRange=PARAM_RANGE_NORMALIZED,
        )

    @classmethod
    def from_spline_windows(
        cls,
        spline: "Splines",
        s_start: float,
        s_end: float,
        config: "ParamPoly3Config",
    ) -> List["ParamPoly3"]:
        """Emit the paramPoly3 chain covering arc length ``[s_start, s_end]``.

        With ``DEFAULT_CONFIG.parampoly3.knot_aligned`` (the default) the window
        is cut at the fitted spline's own breakpoints, so every emitted piece
        stays inside one knot span and reproduces the fitted curve exactly
        (:meth:`exact_from_spline_span`).  ``knot_span_max_length`` optionally
        subdivides a long span further -- still inside the span, so still
        exact.

        With ``knot_aligned`` disabled the legacy uniform Hermite split is used
        instead (kept for A/B comparison and for the regression tests that pin
        the old behaviour).
        """
        from ..config import DEFAULT_CONFIG

        if not DEFAULT_CONFIG.parampoly3.knot_aligned:
            return cls._legacy_uniform_windows(spline, s_start, s_end, config)

        t_start = spline.param_at_arc_length(s_start)
        t_end = spline.param_at_arc_length(s_end)
        if t_end <= t_start:
            return []

        breaks = spline.breakpoints()
        cuts = [t_start]
        cuts.extend(float(t) for t in breaks if t_start < t < t_end)
        cuts.append(t_end)

        cap = DEFAULT_CONFIG.parampoly3.knot_span_max_length
        if cap is not None and cap > 0.0:
            refined: List[float] = [cuts[0]]
            for a, b in zip(cuts, cuts[1:]):
                span_len = spline.arc_length_at_param(b) - spline.arc_length_at_param(a)
                n = max(1, int(np.ceil(span_len / cap)))
                for i in range(1, n):
                    refined.append(a + (b - a) * i / n)
                refined.append(b)
            cuts = refined

        segments: List["ParamPoly3"] = []
        s_cursor = float(s_start)
        for a, b in zip(cuts, cuts[1:]):
            if b <= a:
                continue
            segment = cls.exact_from_spline_span(
                spline,
                t_start=a,
                t_end=b,
                s_start=s_cursor,
                coefficient_epsilon=config.coefficient_epsilon,
            )
            is_valid, error_msg = cls._validate_segment(segment, min_segment_length=0.0)
            if not is_valid:
                import warnings

                warnings.warn(
                    f"Skipping invalid segment at s={s_cursor:.3f}: {error_msg}",
                    UserWarning,
                )
                continue
            segments.append(segment)
            s_cursor += segment.length
        return segments

    @classmethod
    def _legacy_uniform_windows(
        cls,
        spline: "Splines",
        s_start: float,
        s_end: float,
        config: "ParamPoly3Config",
    ) -> List["ParamPoly3"]:
        """Uniform-grid cubic-Hermite split (pre-knot-alignment behaviour)."""
        import warnings

        length = float(s_end - s_start)
        target = config.default_segment_length if config.enabled else length
        n = max(1, int(np.ceil(length / max(target, config.min_segment_length))))
        out: List["ParamPoly3"] = []
        for i in range(n):
            s0 = s_start + (i / n) * length
            s1 = s_start + ((i + 1) / n) * length
            if s1 - s0 < config.min_segment_length:
                warnings.warn(
                    f"Skipping segment with length {s1 - s0:.6f}m "
                    f"(below minimum {config.min_segment_length}m) at s={s0:.3f}",
                    UserWarning,
                )
                continue
            out.append(
                cls.from_spline_window(
                    spline,
                    s0,
                    s1,
                    coefficient_epsilon=config.coefficient_epsilon,
                )
            )
        return out

    @classmethod
    def from_spline_window(
        cls,
        spline: "Splines",
        s_start: float,
        s_end: float,
        coefficient_epsilon: Optional[float] = None,
    ) -> "ParamPoly3":
        """Build a single ParamPoly3 covering [s_start, s_end] of ``spline``.

        Uses cubic Hermite interpolation between spline-derived position
        and tangent at the two endpoints. The caller is responsible for
        upstream length / segment-validity checks.

        Legacy path.  It re-approximates the fitted curve (second derivatives
        are discarded) and declares ``pRange="arcLength"`` although the Hermite
        cubic is not unit-speed, so ``@length`` and the integral of
        ``|(u', v')|`` disagree -- measured at up to 0.98 m on nishishinjuku.
        Prefer :meth:`exact_from_spline_span`, which reproduces the fitted
        curve exactly and declares the parameter range it really uses.
        """
        from ..config import DEFAULT_CONFIG

        if coefficient_epsilon is None:
            coefficient_epsilon = DEFAULT_CONFIG.parampoly3.coefficient_epsilon

        L = float(s_end - s_start)
        start_pos = spline.evaluate(s_start, derivative=0)
        start_tan = spline.evaluate(s_start, derivative=1)
        end_pos = spline.evaluate(s_end, derivative=0)
        end_tan = spline.evaluate(s_end, derivative=1)

        x0, y0 = float(start_pos[0]), float(start_pos[1])
        hdg = float(np.arctan2(start_tan[1], start_tan[0]))
        cos_hdg, sin_hdg = np.cos(hdg), np.sin(hdg)

        dx = end_pos[0] - x0
        dy = end_pos[1] - y0
        u_end = dx * cos_hdg + dy * sin_hdg
        v_end = -dx * sin_hdg + dy * cos_hdg

        du_start = start_tan[0] * cos_hdg + start_tan[1] * sin_hdg
        dv_start = -start_tan[0] * sin_hdg + start_tan[1] * cos_hdg
        du_end = end_tan[0] * cos_hdg + end_tan[1] * sin_hdg
        dv_end = -end_tan[0] * sin_hdg + end_tan[1] * cos_hdg

        aU = 0.0
        bU = float(du_start)
        cU = (3.0 * u_end - 2.0 * du_start * L - du_end * L) / (L * L)
        dU = (-2.0 * u_end + (du_start + du_end) * L) / (L * L * L)

        aV = 0.0
        bV = float(dv_start)
        cV = (3.0 * v_end - 2.0 * dv_start * L - dv_end * L) / (L * L)
        dV = (-2.0 * v_end + (dv_start + dv_end) * L) / (L * L * L)

        aU, bU, cU, dU, aV, bV, cV, dV = cls._normalize_coefficients(
            aU, bU, cU, dU, aV, bV, cV, dV, epsilon=coefficient_epsilon
        )

        return cls(
            s=float(s_start),
            x=x0,
            y=y0,
            hdg=hdg,
            length=L,
            aU=aU,
            bU=bU,
            cU=cU,
            dU=dU,
            aV=aV,
            bV=bV,
            cV=cV,
            dV=dV,
            pRange="arcLength",
        )

    @classmethod
    def from_spline(
        cls,
        spline: "Splines",
        num_segments: Optional[int] = None,
        config: Optional["ParamPoly3Config"] = None,
    ) -> List["ParamPoly3"]:
        """
        Convert a B-spline to a list of ParamPoly3 segments.

        This method divides the spline into segments and fits a cubic polynomial
        to each segment. The number of segments is automatically calculated based
        on road length to ensure no segment is shorter than minimum threshold.

        Args:
            spline: The Splines object to convert
            num_segments: Number of ParamPoly3 segments to create.
                          If None (default), automatically calculated to ensure
                          segments are >= min_segment_length (0.5m).
                          If specified, uses the provided value (backward compatible).
            config: ParamPoly3Config for customizing segment generation parameters.
                   If None, uses defaults from config.py.

        Returns:
            List of ParamPoly3 objects representing the spline

        Configuration:
            Uses ParamPoly3Config (from YAML or defaults):
            - min_segment_length: 0.5m (CARLA requirement)
            - default_segment_length: 1.0m (target length)
            - max_segments: 100 (prevents excessive segmentation)
            - enabled: True (use dynamic calculation)
        """
        from ..config import DEFAULT_CONFIG

        segments = []
        total_length = spline.total_length
        explicit_num_segments = num_segments

        if total_length <= 0:
            # Handle degenerate case
            return []

        # Load config if not provided
        if config is None:
            from ..conversion_config import ParamPoly3Config

            config = ParamPoly3Config()

        # Knot-aligned exact emission (default). The uniform-grid path below is
        # kept for `num_segments=` callers (its whole point is a fixed count)
        # and for `knot_aligned = False`.
        if (
            DEFAULT_CONFIG.parampoly3.knot_aligned
            and explicit_num_segments is None
            and total_length >= config.min_segment_length
        ):
            return cls.from_spline_windows(spline, 0.0, total_length, config)

        # Calculate optimal num_segments if not provided and dynamic mode is enabled
        if num_segments is None and config.enabled:
            num_segments = cls._calculate_optimal_num_segments(
                total_length,
                min_segment_length=config.min_segment_length,
                default_segment_length=config.default_segment_length,
                max_segments=config.max_segments,
                min_segments=config.min_segments,
            )
        elif num_segments is None:
            # Legacy behavior: fixed 10 segments if dynamic mode is disabled
            num_segments = 10

        # Divide the spline into segments
        segment_length = total_length / num_segments

        import warnings

        for i in range(num_segments):
            # Arc length bounds for this segment
            s_start = i * segment_length
            s_end = min((i + 1) * segment_length, total_length)
            actual_segment_length = s_end - s_start

            if actual_segment_length <= 0:
                continue

            # Skip segments that are too short
            min_length = config.min_segment_length

            if actual_segment_length < min_length:
                warnings.warn(
                    f"Skipping segment with length {actual_segment_length:.6f}m "
                    f"(below minimum {min_length}m) at s={s_start:.3f}",
                    UserWarning,
                )
                continue

            # Build the segment via the single-window helper.
            segment = cls.from_spline_window(
                spline,
                s_start=s_start,
                s_end=s_end,
                coefficient_epsilon=config.coefficient_epsilon,
            )

            # Validate segment before adding
            is_valid, error_msg = cls._validate_segment(
                segment, min_segment_length=config.min_segment_length
            )
            if not is_valid:
                warnings.warn(
                    f"Skipping invalid segment at s={s_start:.3f}: {error_msg}",
                    UserWarning,
                )
                continue

            segments.append(segment)

        return segments

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = super().to_xml()
        poly_elem = ET.SubElement(elem, "paramPoly3")
        for attr in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV"):
            poly_elem.set(attr, str(replace_subnormal(getattr(self, attr))))
        poly_elem.set("pRange", self.pRange)
        return elem


@dataclass
class PlanView:
    """Plan view container for geometry."""

    geometries: List[GeometryBase]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("planView")
        for geometry in self.geometries:
            elem.append(geometry.to_xml())
        return elem
