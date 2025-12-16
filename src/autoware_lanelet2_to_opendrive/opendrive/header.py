"""OpenDRIVE header definitions."""

from dataclasses import dataclass
from typing import Optional
import lxml.etree as ET


@dataclass
class Header:
    """OpenDRIVE header."""

    rev_major: str = "1"
    rev_minor: str = "4"
    name: Optional[str] = None
    version: Optional[str] = None
    date: Optional[str] = None
    north: Optional[str] = None
    south: Optional[str] = None
    east: Optional[str] = None
    west: Optional[str] = None
    geo_reference: Optional[str] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("header")
        elem.set("revMajor", self.rev_major)
        elem.set("revMinor", self.rev_minor)

        if self.name:
            elem.set("name", self.name)
        if self.version:
            elem.set("version", self.version)
        if self.date:
            elem.set("date", self.date)
        if self.north:
            elem.set("north", self.north)
        if self.south:
            elem.set("south", self.south)
        if self.east:
            elem.set("east", self.east)
        if self.west:
            elem.set("west", self.west)

        if self.geo_reference:
            geo_ref_elem = ET.SubElement(elem, "geoReference")
            # Wrap the geo_reference string in a CDATA section
            geo_ref_elem.text = ET.CDATA(self.geo_reference)

        return elem
