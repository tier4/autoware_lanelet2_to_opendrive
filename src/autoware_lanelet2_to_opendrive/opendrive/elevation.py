"""OpenDRIVE elevation definitions."""

from dataclasses import dataclass
from typing import List
import lxml.etree as ET


@dataclass
class Elevation:
    """Elevation definition."""

    s: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("elevation")
        elem.set("s", str(self.s))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem


@dataclass
class ElevationProfile:
    """Elevation profile container."""

    elevations: List[Elevation]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("elevationProfile")
        for elevation in self.elevations:
            elem.append(elevation.to_xml())
        return elem
