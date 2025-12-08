"""OpenDRIVE geometry definitions."""

from dataclasses import dataclass
from typing import List, TYPE_CHECKING
import lxml.etree as ET
import numpy as np

from .enums import GeometryType

if TYPE_CHECKING:
    from ..geometry import ArcLengthParameterizedCatmullRomSpline
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
    def from_spline_old(
        cls, spline: "ArcLengthParameterizedCatmullRomSpline"
    ) -> List["ParamPoly3"]:
        """
        Create ParamPoly3 list from ArcLengthParameterizedCatmullRomSpline.

        Args:
            spline: Arc length parameterized Catmull-Rom spline

        Returns:
            List of ParamPoly3 instances with normalized pRange for all segments
        """

        def solve_cubic_coeffs(
            p0: float, v0: float, p1: float, v1: float, dt: float
        ) -> tuple[float, float, float, float]:
            """
            Calculate cubic polynomial coefficients from start/end positions and velocities.
            f(t) = a + bt + ct^2 + dt^3
            """
            if dt == 0:
                print("Warning: dt is zero, returning zero coefficients.")
                return p0, v0, 0.0, 0.0

            inv_dt = 1.0 / dt
            inv_dt2 = inv_dt * inv_dt
            inv_dt3 = inv_dt2 * inv_dt

            delta_p = p1 - p0

            a = p0
            b = v0
            c = (3 * delta_p * inv_dt2) - ((2 * v0 + v1) * inv_dt)
            d = (2 * -delta_p * inv_dt3) + ((v0 + v1) * inv_dt2)
            return a, b, c, d

        # Get spline segments data
        segments = spline.as_cubic_spline_parameters()

        if not segments:
            raise ValueError("Spline has no segments.")

        param_poly_list = []

        for segment_index, segment in enumerate(segments):
            # Get segment start and end arc lengths
            s_start = segment["s_start"]
            s_end = segment["s_end"]
            segment_length = s_end - s_start

            if segment_length <= 0:
                raise ValueError(
                    f"Invalid segment length: {segment_length} for segment {segment_index}"
                )

            # Evaluate spline at segment boundaries to get positions and velocities
            start_frame = spline.evaluate(s_start, frenet=True)
            end_frame = spline.evaluate(s_end, frenet=True)

            start_position = start_frame["position"]
            start_tangent = start_frame["tangent"]
            start_normal = start_frame["normal"]

            end_position = end_frame["position"]
            end_tangent = end_frame["tangent"]
            end_normal = end_frame["normal"]

            # Calculate local U/V coordinates using Frenet frame at segment start
            start_global = start_position
            end_global = end_position

            # Project end position onto start frame
            delta = end_global - start_global
            u_end = np.dot(delta, start_tangent)  # longitudinal
            v_end = np.dot(delta, start_normal)  # lateral

            # Calculate proper velocities using spline derivatives
            # For normalized parameterization, dt = 1.0 and we need du/dp, dv/dp where p ∈ [0,1]
            dt_normalized = 1.0

            # Get velocity vectors at start and end points in global coordinates
            start_velocity = spline._arc_length_spline.evaluate_derivative(
                s_start, order=1
            )
            end_velocity = spline._arc_length_spline.evaluate_derivative(s_end, order=1)

            # Project velocities onto local Frenet frame and scale for normalized parameter
            # For normalized parameter, we need to scale by segment_length
            u_vel_start = np.dot(start_velocity, start_tangent) * segment_length
            v_vel_start = np.dot(start_velocity, start_normal) * segment_length

            u_vel_end = np.dot(end_velocity, end_tangent) * segment_length
            v_vel_end = np.dot(end_velocity, end_normal) * segment_length

            # Calculate cubic coefficients for U and V coordinates with normalized parameter
            aU, bU, cU, dU = solve_cubic_coeffs(
                0.0, u_vel_start, u_end, u_vel_end, dt_normalized
            )
            aV, bV, cV, dV = solve_cubic_coeffs(
                0.0, v_vel_start, v_end, v_vel_end, dt_normalized
            )

            param_poly = cls(
                s=s_start,
                x=start_position[0],
                y=start_position[1],
                hdg=float(np.arctan2(start_tangent[1], start_tangent[0])),
                length=segment_length,
                aU=aU,
                bU=bU,
                cU=cU,
                dU=dU,
                aV=aV,
                bV=bV,
                cV=cV,
                dV=dV,
                pRange="normalized",
            )

            param_poly_list.append(param_poly)

        return param_poly_list

    @classmethod
    def from_spline(
        cls, spline: "Splines", num_segments: int = 10
    ) -> List["ParamPoly3"]:
        """
        Create ParamPoly3 list from Splines class using B-spline interpolation.

        Args:
            spline: B-spline object with arc length parameterization
            num_segments: Number of segments to divide the spline into

        Returns:
            List of ParamPoly3 instances with normalized pRange
        """

        def solve_cubic_coeffs(
            p0: float, v0: float, p1: float, v1: float, dt: float
        ) -> tuple[float, float, float, float]:
            """
            Calculate cubic polynomial coefficients from start/end positions and velocities.
            f(t) = a + bt + ct^2 + dt^3
            """
            if dt == 0:
                return p0, v0, 0.0, 0.0

            inv_dt = 1.0 / dt
            inv_dt2 = inv_dt * inv_dt
            inv_dt3 = inv_dt2 * inv_dt

            delta_p = p1 - p0

            a = p0
            b = v0
            c = (3 * delta_p * inv_dt2) - ((2 * v0 + v1) * inv_dt)
            d = (2 * -delta_p * inv_dt3) + ((v0 + v1) * inv_dt2)
            return a, b, c, d

        total_length = spline.total_length
        if total_length <= 0:
            raise ValueError("Spline has zero length")

        segment_length = total_length / num_segments
        param_poly_list = []

        for i in range(num_segments):
            # Calculate segment boundaries in arc length
            s_start = i * segment_length
            s_end = min((i + 1) * segment_length, total_length)
            current_segment_length = s_end - s_start

            if current_segment_length <= 0:
                continue

            # Get Frenet frames at segment boundaries
            start_frame = spline.get_frenet_frame(s_start)
            end_frame = spline.get_frenet_frame(s_end)

            start_position = start_frame["position"]
            start_tangent = start_frame["tangent"]
            start_normal = start_frame["normal"]

            end_position = end_frame["position"]
            end_tangent = end_frame["tangent"]
            end_normal = end_frame["normal"]

            # Calculate local U/V coordinates using Frenet frame at segment start
            delta = end_position - start_position
            u_end = np.dot(delta, start_tangent)  # longitudinal
            v_end = np.dot(delta, start_normal)  # lateral

            # Get velocity vectors at start and end points
            start_velocity = spline.evaluate(s_start, derivative=1)
            end_velocity = spline.evaluate(s_end, derivative=1)

            # Project velocities onto local Frenet frame and scale for normalized parameter
            # For normalized parameter [0,1], we need to scale by segment_length
            u_vel_start = np.dot(start_velocity, start_tangent) * current_segment_length
            v_vel_start = np.dot(start_velocity, start_normal) * current_segment_length

            u_vel_end = np.dot(end_velocity, end_tangent) * current_segment_length
            v_vel_end = np.dot(end_velocity, end_normal) * current_segment_length

            # Calculate cubic coefficients for U and V coordinates with normalized parameter
            dt_normalized = 1.0
            aU, bU, cU, dU = solve_cubic_coeffs(
                0.0, u_vel_start, u_end, u_vel_end, dt_normalized
            )
            aV, bV, cV, dV = solve_cubic_coeffs(
                0.0, v_vel_start, v_end, v_vel_end, dt_normalized
            )

            param_poly = cls(
                s=s_start,
                x=start_position[0],
                y=start_position[1],
                hdg=float(np.arctan2(start_tangent[1], start_tangent[0])),
                length=current_segment_length,
                aU=aU,
                bU=bU,
                cU=cU,
                dU=dU,
                aV=aV,
                bV=bV,
                cV=cV,
                dV=dV,
                pRange="normalized",
            )

            param_poly_list.append(param_poly)

        return param_poly_list

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
