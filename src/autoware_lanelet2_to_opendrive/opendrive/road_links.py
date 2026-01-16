"""Data classes for OpenDRIVE road link elements."""

from dataclasses import dataclass
from typing import Optional
import lxml.etree as ET

from .enums import ContactPoint, ElementType


@dataclass
class Predecessor:
    """Predecessor link element."""

    element_type: ElementType
    element_id: int
    contact_point: Optional[ContactPoint] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("predecessor")
        elem.set("elementType", self.element_type.value)
        elem.set("elementId", str(self.element_id))
        if self.contact_point:
            elem.set("contactPoint", self.contact_point.value)
        return elem


@dataclass
class Successor:
    """Successor link element."""

    element_type: ElementType
    element_id: int
    contact_point: Optional[ContactPoint] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("successor")
        elem.set("elementType", self.element_type.value)
        elem.set("elementId", str(self.element_id))
        if self.contact_point:
            elem.set("contactPoint", self.contact_point.value)
        return elem


@dataclass
class RoadLink:
    """Road link element containing predecessor and successor."""

    predecessor: Optional[Predecessor] = None
    successor: Optional[Successor] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("link")
        if self.predecessor:
            elem.append(self.predecessor.to_xml())
        if self.successor:
            elem.append(self.successor.to_xml())
        return elem
