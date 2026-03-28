"""OpenDRIVE elevation definitions."""

from dataclasses import dataclass
from typing import List
import lxml.etree as ET

from .xml_utils import replace_subnormal


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
        for attr in ("b", "c", "d"):
            elem.set(attr, str(replace_subnormal(getattr(self, attr))))
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
