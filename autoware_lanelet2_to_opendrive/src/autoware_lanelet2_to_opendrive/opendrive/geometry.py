"""OpenDRIVE geometry definitions."""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, TYPE_CHECKING
import lxml.etree as ET
import numpy as np

from .enums import GeometryType
from .xml_utils import replace_subnormal

if TYPE_CHECKING:
    from ..spline import Splines
    from ..conversion_config import ParamPoly3Config


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
        p: Arc-length parameter at which to evaluate (``0`` gives the
            start, ``length`` gives the end).
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
        end_xy = _eval_geometry_world(last_geom, p=last_length)
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
    pRange: str = "arcLength"  # range of parameter p (arcLength or normalized)
    geometry_type = GeometryType.PARAMPOLY3

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
        segments = []
        total_length = spline.total_length

        if total_length <= 0:
            # Handle degenerate case
            return []

        # Load config if not provided
        if config is None:
            from ..conversion_config import ParamPoly3Config

            config = ParamPoly3Config()

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

        for i in range(num_segments):
            # Arc length bounds for this segment
            s_start = i * segment_length
            s_end = min((i + 1) * segment_length, total_length)
            actual_segment_length = s_end - s_start

            if actual_segment_length <= 0:
                continue

            # Skip segments that are too short
            import warnings

            min_length = config.min_segment_length

            if actual_segment_length < min_length:
                # Log warning for debugging
                warnings.warn(
                    f"Skipping segment with length {actual_segment_length:.6f}m "
                    f"(below minimum {min_length}m) at s={s_start:.3f}",
                    UserWarning,
                )
                continue

            # Get position and derivatives at segment start
            start_pos = spline.evaluate(s_start, derivative=0)
            start_tangent = spline.evaluate(s_start, derivative=1)

            # Get position and derivatives at segment end
            end_pos = spline.evaluate(s_end, derivative=0)
            end_tangent = spline.evaluate(s_end, derivative=1)

            # Extract 2D coordinates (assuming z is constant or negligible for road geometry)
            x0, y0 = start_pos[0], start_pos[1]

            # Calculate heading from tangent vector
            hdg = np.arctan2(start_tangent[1], start_tangent[0])

            # Transform to local coordinate system
            # Local u-axis is along the tangent, v-axis is perpendicular
            cos_hdg = np.cos(hdg)
            sin_hdg = np.sin(hdg)

            # Transform end position to local coordinates
            dx = end_pos[0] - x0
            dy = end_pos[1] - y0
            u_end = dx * cos_hdg + dy * sin_hdg
            v_end = -dx * sin_hdg + dy * cos_hdg

            # Transform tangent vectors to local coordinates
            # Start tangent in local coords (should be [1, 0] ideally)
            du_start = start_tangent[0] * cos_hdg + start_tangent[1] * sin_hdg
            dv_start = -start_tangent[0] * sin_hdg + start_tangent[1] * cos_hdg

            # End tangent in local coords
            du_end = end_tangent[0] * cos_hdg + end_tangent[1] * sin_hdg
            dv_end = -end_tangent[0] * sin_hdg + end_tangent[1] * cos_hdg

            # Fit cubic polynomials using boundary conditions
            # For paramPoly3: u(p) = aU + bU*p + cU*p^2 + dU*p^3
            #                 v(p) = aV + bV*p + cV*p^2 + dV*p^3
            # where p is the parameter (arc length in this case)

            # Boundary conditions:
            # u(0) = 0, u(L) = u_end
            # u'(0) = du_start, u'(L) = du_end
            # v(0) = 0, v(L) = v_end
            # v'(0) = dv_start, v'(L) = dv_end

            L = actual_segment_length

            # Solve for u coefficients using cubic Hermite interpolation
            aU = 0.0
            bU = du_start
            cU = (3 * u_end - 2 * du_start * L - du_end * L) / (L * L)
            dU = (-2 * u_end + (du_start + du_end) * L) / (L * L * L)

            # Solve for v coefficients
            aV = 0.0
            bV = dv_start
            cV = (3 * v_end - 2 * dv_start * L - dv_end * L) / (L * L)
            dV = (-2 * v_end + (dv_start + dv_end) * L) / (L * L * L)

            # Normalize coefficients to prevent numerical instability
            aU, bU, cU, dU, aV, bV, cV, dV = cls._normalize_coefficients(
                aU, bU, cU, dU, aV, bV, cV, dV, epsilon=config.coefficient_epsilon
            )

            # Create ParamPoly3 segment
            segment = cls(
                s=s_start,
                x=x0,
                y=y0,
                hdg=hdg,
                length=actual_segment_length,
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
