"""OpenDRIVE geometry definitions."""

from dataclasses import dataclass
from typing import List, Optional, TYPE_CHECKING
import lxml.etree as ET
import numpy as np

from .enums import GeometryType
from ..config import get_param_poly3_num_segments

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

    @classmethod
    def from_spline(
        cls, spline: "Splines", num_segments: Optional[int] = None
    ) -> List["ParamPoly3"]:
        """
        Convert a B-spline to a list of ParamPoly3 segments.

        This method divides the spline into segments and fits a cubic polynomial
        to each segment using local coordinate systems.

        Args:
            spline: The Splines object to convert
            num_segments: Number of ParamPoly3 segments to create. If None, uses
                         DEFAULT_CONFIG.geometry.param_poly3_num_segments

        Returns:
            List of ParamPoly3 objects representing the spline
        """
        # Use config default if not specified
        # get_param_poly3_num_segments() returns runtime override if set,
        # otherwise returns DEFAULT_CONFIG.geometry.param_poly3_num_segments
        if num_segments is None:
            num_segments = get_param_poly3_num_segments()

        segments = []
        total_length = spline.total_length

        if total_length <= 0:
            # Handle degenerate case
            return []

        # Divide the spline into segments
        segment_length = total_length / num_segments

        # Track previous heading at segment END for continuity (unwrap)
        prev_end_hdg = None

        for i in range(num_segments):
            # Arc length bounds for this segment
            s_start = i * segment_length
            s_end = min((i + 1) * segment_length, total_length)
            actual_segment_length = s_end - s_start

            if actual_segment_length <= 0:
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
            hdg_raw = np.arctan2(start_tangent[1], start_tangent[0])

            # Unwrap heading to ensure continuity with previous segment's END
            if prev_end_hdg is not None:
                # Calculate the difference and find the smallest equivalent angle
                # This ensures continuity by choosing the angle closest to prev segment's end
                diff = hdg_raw - prev_end_hdg

                # Wrap diff to [-π, π] range
                diff = np.arctan2(np.sin(diff), np.cos(diff))

                # Cumulative heading: add the wrapped difference to previous end heading
                hdg = prev_end_hdg + diff
            else:
                # First segment: use raw heading
                hdg = hdg_raw

            # Calculate heading at segment END for next iteration
            # Use end tangent to determine where this segment ends up
            end_hdg_raw = np.arctan2(end_tangent[1], end_tangent[0])

            if prev_end_hdg is not None:
                # Unwrap end heading relative to start heading of this segment
                diff_end = end_hdg_raw - hdg
                diff_end = np.arctan2(np.sin(diff_end), np.cos(diff_end))
                end_hdg = hdg + diff_end
            else:
                # First segment: use raw end heading
                end_hdg = end_hdg_raw

            # Update previous end heading for next iteration
            prev_end_hdg = end_hdg

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

            segments.append(segment)

        return segments

    @staticmethod
    def calculate_heading_change(segment: "ParamPoly3") -> float:
        """
        Calculate the heading change within a ParamPoly3 segment.

        Args:
            segment: ParamPoly3 segment to analyze

        Returns:
            Heading change in radians (can be > π for tight curves)
        """
        # Tangent at start (p=0): du/dp = bU, dv/dp = bV
        du_start = segment.bU  # Should be 1.0 for arcLength
        dv_start = segment.bV  # Should be 0.0 at start

        # Tangent at end (p=L): du/dp = bU + 2*cU*L + 3*dU*L^2
        L = segment.length
        du_end = segment.bU + 2 * segment.cU * L + 3 * segment.dU * L * L
        dv_end = segment.bV + 2 * segment.cV * L + 3 * segment.dV * L * L

        # Calculate heading change in local frame
        heading_start = np.arctan2(dv_start, du_start)
        heading_end = np.arctan2(dv_end, du_end)

        return heading_end - heading_start

    @classmethod
    def subdivide_spline_range(
        cls,
        spline: "Splines",
        s_start: float,
        s_end: float,
        num_subsegments: int = 2,
    ) -> List["ParamPoly3"]:
        """
        Subdivide a spline range into multiple ParamPoly3 segments.

        Args:
            spline: The source spline
            s_start: Start arc length
            s_end: End arc length
            num_subsegments: Number of subsegments to create

        Returns:
            List of ParamPoly3 segments covering [s_start, s_end]
        """
        subsegments = []
        segment_length = (s_end - s_start) / num_subsegments

        for i in range(num_subsegments):
            sub_s_start = s_start + i * segment_length
            sub_s_end = sub_s_start + segment_length

            # Get positions and tangents at sub-segment boundaries
            start_pos = spline.evaluate(sub_s_start, derivative=0)
            start_tangent = spline.evaluate(sub_s_start, derivative=1)
            end_pos = spline.evaluate(sub_s_end, derivative=0)
            end_tangent = spline.evaluate(sub_s_end, derivative=1)

            # Extract 2D coordinates
            x0, y0 = start_pos[0], start_pos[1]

            # Calculate heading from tangent vector
            hdg = np.arctan2(start_tangent[1], start_tangent[0])

            # Transform to local coordinate system
            cos_hdg = np.cos(hdg)
            sin_hdg = np.sin(hdg)

            # Transform end position to local coordinates
            dx = end_pos[0] - x0
            dy = end_pos[1] - y0
            u_end = dx * cos_hdg + dy * sin_hdg
            v_end = -dx * sin_hdg + dy * cos_hdg

            # Transform tangent vectors to local coordinates
            du_start = start_tangent[0] * cos_hdg + start_tangent[1] * sin_hdg
            dv_start = -start_tangent[0] * sin_hdg + start_tangent[1] * cos_hdg

            du_end = end_tangent[0] * cos_hdg + end_tangent[1] * sin_hdg
            dv_end = -end_tangent[0] * sin_hdg + end_tangent[1] * cos_hdg

            # Fit cubic polynomials using boundary conditions
            L = segment_length

            aU = 0.0
            bU = du_start
            cU = (3 * u_end - 2 * du_start * L - du_end * L) / (L * L)
            dU = (-2 * u_end + (du_start + du_end) * L) / (L * L * L)

            aV = 0.0
            bV = dv_start
            cV = (3 * v_end - 2 * dv_start * L - dv_end * L) / (L * L)
            dV = (-2 * v_end + (dv_start + dv_end) * L) / (L * L * L)

            segment = cls(
                s=sub_s_start,
                x=x0,
                y=y0,
                hdg=hdg,
                length=segment_length,
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

            subsegments.append(segment)

        return subsegments

    @classmethod
    def from_spline_adaptive(
        cls,
        spline: "Splines",
        initial_num_segments: Optional[int] = None,
        max_heading_change_deg: Optional[float] = None,
        max_iterations: Optional[int] = None,
    ) -> List["ParamPoly3"]:
        """
        Convert spline to ParamPoly3 with adaptive segment subdivision.

        This method automatically detects segments with excessive heading changes
        and subdivides them to maintain smooth heading continuity. Segments with
        heading changes exceeding max_heading_change_deg are automatically split
        into smaller segments until the threshold is met or max_iterations is reached.

        Args:
            spline: The Splines object to convert
            initial_num_segments: Initial number of segments (uses config if None)
            max_heading_change_deg: Maximum allowed heading change per segment in degrees
                                   (uses config if None)
            max_iterations: Maximum number of subdivision iterations (uses config if None)

        Returns:
            List of ParamPoly3 segments with adaptive subdivision

        Example:
            >>> spline = Splines(points)
            >>> segments = ParamPoly3.from_spline_adaptive(
            ...     spline,
            ...     initial_num_segments=30,
            ...     max_heading_change_deg=30.0,
            ... )
            >>> # Segments with >30° heading change are automatically subdivided
        """
        from ..config import (
            get_param_poly3_num_segments,
            get_max_heading_change_deg,
            get_max_subdivision_iterations,
        )
        import logging

        logger = logging.getLogger(__name__)

        # Use config defaults if not specified
        if initial_num_segments is None:
            initial_num_segments = get_param_poly3_num_segments()

        if max_heading_change_deg is None:
            max_heading_change_deg = get_max_heading_change_deg()

        if max_iterations is None:
            max_iterations = get_max_subdivision_iterations()

        max_heading_change = np.radians(max_heading_change_deg)

        # Generate initial segments
        segments = cls.from_spline(spline, initial_num_segments)
        total_subdivisions = 0

        for iteration in range(max_iterations):
            # Detect segments with excessive heading changes
            problematic_indices = []
            for i, segment in enumerate(segments):
                heading_change = cls.calculate_heading_change(segment)
                if abs(heading_change) > max_heading_change:
                    problematic_indices.append((i, heading_change))

            if not problematic_indices:
                # No more subdivisions needed
                break

            logger.info(
                f"Adaptive subdivision iteration {iteration + 1}/{max_iterations}: "
                f"Found {len(problematic_indices)} segments exceeding "
                f"{max_heading_change_deg:.1f}° threshold"
            )

            # Build new segment list with subdivisions
            new_segments = []
            problematic_set = {idx for idx, _ in problematic_indices}

            for i, segment in enumerate(segments):
                if i in problematic_set:
                    # Subdivide this segment into 2 parts
                    sub_segments = cls.subdivide_spline_range(
                        spline, segment.s, segment.s + segment.length, num_subsegments=2
                    )
                    new_segments.extend(sub_segments)

                    heading_change = next(
                        hc for idx, hc in problematic_indices if idx == i
                    )
                    logger.debug(
                        f"  Subdivided segment {i} at s={segment.s:.2f} "
                        f"(heading change: {np.degrees(heading_change):.1f}°)"
                    )
                    total_subdivisions += 1
                else:
                    new_segments.append(segment)

            segments = new_segments

        if total_subdivisions > 0:
            logger.info(
                f"Adaptive subdivision complete: {len(segments)} segments "
                f"(initial: {initial_num_segments}, added: {len(segments) - initial_num_segments})"
            )

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
