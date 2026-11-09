"""OpenDRIVE header definitions."""

from dataclasses import dataclass
from typing import Optional
import lxml.etree as ET


@dataclass
class Header:
    """OpenDRIVE header.

    ``rev_major``/``rev_minor`` default to 1.7, the version whose schema
    actually admits everything this converter emits — ``<road rule>``
    (``e_trafficRule``, added in 1.7), ``<signal><positionInertial>``
    (present in 1.7, deprecated in 1.8) and the full ``e_accessRestrictionType``
    enumeration (``passengerCar``/``bus``/``taxi``/… , added in 1.7). It is
    also the highest declaration that keeps the whole asam-qc-opendrive
    bundle applicable: 25 of its 26 checkers have a rule definition-setting
    of <= 1.7.0, and ``junctions.connection.one_connection_element`` is
    additionally marked ``APPLICABLE_VERSION = "<=1.7.0"``, so declaring
    1.8 would gain one checker and lose another while switching the
    validator to the XSD 1.1 schema that deprecates ``positionInertial``.

    Raising the declaration from the historical 1.4 is safe for CARLA:
    ``OpenDriveParser::Load`` never reads ``@revMajor``/``@revMinor``
    (LibCarla/source/carla/opendrive/OpenDriveParser.cpp, tag 0.10.0 —
    the version pinned in ``carla_wheels/``).
    """

    rev_major: str = "1"
    rev_minor: str = "7"
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
