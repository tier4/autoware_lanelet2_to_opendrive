"""OpenDRIVE geometry definitions."""

from dataclasses import dataclass
from typing import List
import lxml.etree as ET

from .enums import GeometryType


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
class PlanView:
    """Plan view container for geometry."""

    geometries: List[GeometryBase]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("planView")
        for geometry in self.geometries:
            elem.append(geometry.to_xml())
        return elem
