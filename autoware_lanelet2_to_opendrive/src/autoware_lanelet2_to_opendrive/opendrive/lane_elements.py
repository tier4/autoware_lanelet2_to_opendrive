"""OpenDRIVE lane element definitions."""

from dataclasses import dataclass
from typing import Optional
import lxml.etree as ET

from .enums import RoadMarkType, RoadMarkColor, RoadMarkLaneChange, SpeedUnit, RoadType
from .xml_utils import replace_subnormal


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
        for attr in ("a", "b", "c", "d"):
            elem.set(attr, str(replace_subnormal(getattr(self, attr))))
        return elem


@dataclass
class RoadMark:
    """Road marking definition."""

    s_offset: float
    type: RoadMarkType
    color: RoadMarkColor
    lane_change: Optional[RoadMarkLaneChange] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("roadMark")
        elem.set("sOffset", str(self.s_offset))
        elem.set("type", self.type.value)
        elem.set("color", self.color.value)
        if self.lane_change is not None:
            elem.set("laneChange", self.lane_change.value)
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
        for attr in ("a", "b", "c", "d"):
            elem.set(attr, str(replace_subnormal(getattr(self, attr))))
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


@dataclass
class LaneSpeed:
    """Lane speed limit definition.

    Represents individual lane speed limits in OpenDRIVE format.
    Corresponds to <lane><speed> element.
    """

    s_offset: float
    max: float
    unit: SpeedUnit = SpeedUnit.KMH

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("speed")
        elem.set("sOffset", str(self.s_offset))
        elem.set("max", str(int(self.max)))
        elem.set("unit", self.unit.value)
        return elem


@dataclass
class RoadTypeSpeed:
    """Road type speed limit definition.

    Represents speed limits for a road type in OpenDRIVE format.
    Corresponds to <road><type><speed> element.
    """

    max: float
    unit: SpeedUnit = SpeedUnit.KMH

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("speed")
        elem.set("max", str(int(self.max)))
        elem.set("unit", self.unit.value)
        return elem


@dataclass
class RoadTypeDefinition:
    """Road type definition.

    Represents road type information in OpenDRIVE format.
    Corresponds to <road><type> element.
    """

    s: float
    type: RoadType
    speed: Optional[RoadTypeSpeed] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("type")
        elem.set("s", str(self.s))
        elem.set("type", self.type.value)

        if self.speed:
            elem.append(self.speed.to_xml())

        return elem
