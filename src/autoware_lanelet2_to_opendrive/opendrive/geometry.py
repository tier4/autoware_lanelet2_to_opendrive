"""OpenDRIVE geometry definitions."""

from dataclasses import dataclass
from typing import List, Optional, TYPE_CHECKING
import lxml.etree as ET
import numpy as np

from .enums import GeometryType

if TYPE_CHECKING:
    from ..spline import Splines


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
    def _validate_segment(segment: "ParamPoly3") -> tuple:
        """
        Validate a ParamPoly3 segment for numerical stability and correctness.

        Args:
            segment: ParamPoly3 segment to validate

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
        from ..config import DEFAULT_CONFIG

        # Check length
        min_length = DEFAULT_CONFIG.parampoly3.min_segment_length
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
        cls, spline: "Splines", num_segments: Optional[int] = None
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

        Returns:
            List of ParamPoly3 objects representing the spline

        Configuration:
            Uses ParamPoly3Constants from config.py:
            - min_segment_length: 0.5m (CARLA requirement)
            - default_segment_length: 1.0m (target length)
            - max_segments: 100 (prevents excessive segmentation)
        """
        segments = []
        total_length = spline.total_length

        if total_length <= 0:
            # Handle degenerate case
            return []

        # Calculate optimal num_segments if not provided
        if num_segments is None:
            num_segments = cls._calculate_optimal_num_segments(total_length)

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
            from ..config import DEFAULT_CONFIG
            import warnings

            min_length = DEFAULT_CONFIG.parampoly3.min_segment_length

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
                aU, bU, cU, dU, aV, bV, cV, dV
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
            is_valid, error_msg = cls._validate_segment(segment)
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
        poly_elem.set("aU", str(self.aU))
        poly_elem.set("bU", str(self.bU))
        poly_elem.set("cU", str(self.cU))
        poly_elem.set("dU", str(self.dU))
        poly_elem.set("aV", str(self.aV))
        poly_elem.set("bV", str(self.bV))
        poly_elem.set("cV", str(self.cV))
        poly_elem.set("dV", str(self.dV))
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
