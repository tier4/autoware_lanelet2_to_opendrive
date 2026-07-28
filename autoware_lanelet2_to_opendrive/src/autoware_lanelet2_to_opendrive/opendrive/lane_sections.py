"""OpenDRIVE lane section definitions."""

from dataclasses import dataclass, field
from typing import Dict, List, TYPE_CHECKING
import lxml.etree as ET

from .xml_utils import replace_subnormal

if TYPE_CHECKING:
    from ..lane import Lane
    from .lane_section import LaneSection


@dataclass
class Left:
    """Left lane section."""

    lanes: List["Lane"]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("left")
        for lane in self.lanes:
            # Check if lane has to_xml method (supports both Lane types)
            if hasattr(lane, "to_xml"):
                elem.append(lane.to_xml())
        return elem


@dataclass
class Center:
    """Center lane section."""

    lanes: List["Lane"]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("center")
        for lane in self.lanes:
            # Check if lane has to_xml method (supports both Lane types)
            if hasattr(lane, "to_xml"):
                elem.append(lane.to_xml())
        return elem


@dataclass
class Right:
    """Right lane section."""

    lanes: List["Lane"]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("right")
        for lane in self.lanes:
            # Check if lane has to_xml method (supports both Lane types)
            if hasattr(lane, "to_xml"):
                elem.append(lane.to_xml())
        return elem


@dataclass
class Lanes:
    """Lanes container."""

    lane_sections: List["LaneSection"]
    lane_offsets: List[Dict[str, float]] = field(default_factory=list)

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("lanes")
        for lane_offset in self.lane_offsets:
            offset_elem = ET.SubElement(elem, "laneOffset")
            offset_elem.set("s", str(lane_offset["s"]))
            for key in ("a", "b", "c", "d"):
                offset_elem.set(key, str(replace_subnormal(lane_offset[key])))
        for lane_section in self.lane_sections:
            elem.append(lane_section.to_xml())
        return elem
