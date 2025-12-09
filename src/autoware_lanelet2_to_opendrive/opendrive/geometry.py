"""OpenDRIVE geometry definitions."""

from dataclasses import dataclass
from typing import List, TYPE_CHECKING
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

    @classmethod
    def from_spline(
        cls, spline: "Splines", num_segments: int = 10
    ) -> List["ParamPoly3"]:
        """
        Convert a B-spline to a list of ParamPoly3 segments.

        This method divides the spline into segments and fits a cubic polynomial
        to each segment using local coordinate systems.

        Args:
            spline: The Splines object to convert
            num_segments: Number of ParamPoly3 segments to create

        Returns:
            List of ParamPoly3 objects representing the spline
        """
        segments = []
        total_length = spline.total_length

        if total_length <= 0:
            # Handle degenerate case
            return []

        # Divide the spline into segments
        segment_length = total_length / num_segments

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
