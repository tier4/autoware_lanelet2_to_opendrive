"""OpenDRIVE elevation definitions."""

from dataclasses import dataclass
from typing import List
import lxml.etree as ET

from .xml_utils import replace_subnormal
from .profile_redundancy import without_redundant_records


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
        for attr in ("a", "b", "c", "d"):
            elem.set(attr, str(replace_subnormal(getattr(self, attr))))
        return elem


@dataclass
class ElevationProfile:
    """Elevation profile container."""

    elevations: List[Elevation]

    def to_xml(self) -> ET.Element:
        """Convert to XML element.

        A record that restates its predecessor's polynomial is not written:
        it covers the same s with the same height, and ASAM reports it
        (performance.avoid_redundant_info). See
        :mod:`.profile_redundancy` for why only that case is safe to drop.
        """
        elem = ET.Element("elevationProfile")
        for elevation in without_redundant_records(self.elevations, "s"):
            elem.append(elevation.to_xml())
        return elem
