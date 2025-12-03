"""OpenDRIVE lane element definitions."""

from dataclasses import dataclass
import lxml.etree as ET

from .enums import RoadMarkType, RoadMarkColor


@dataclass
class LaneWidth:
    """Lane width definition with cubic polynomial coefficients."""

    s_offset: float
    a: float
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("width")
        elem.set("sOffset", str(self.s_offset))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem


@dataclass
class RoadMark:
    """Road marking definition."""

    s_offset: float
    type: RoadMarkType
    color: RoadMarkColor

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("roadMark")
        elem.set("sOffset", str(self.s_offset))
        elem.set("type", self.type.value)
        elem.set("color", self.color.value)
        return elem


@dataclass
class LaneLink:
    """Lane link definition for predecessor/successor."""

    id: int

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("link")
        elem.set("id", str(self.id))
        return elem


@dataclass
class LaneBorder:
    """Lane border definition."""

    s_offset: float
    a: float
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("border")
        elem.set("sOffset", str(self.s_offset))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem


@dataclass
class LaneHeight:
    """Lane height definition."""

    s_offset: float
    inner: float
    outer: float

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("height")
        elem.set("sOffset", str(self.s_offset))
        elem.set("inner", str(self.inner))
        elem.set("outer", str(self.outer))
        return elem
