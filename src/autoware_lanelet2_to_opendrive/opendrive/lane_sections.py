"""OpenDRIVE lane section definitions."""

from dataclasses import dataclass
from typing import List, Optional, TYPE_CHECKING
import lxml.etree as ET

if TYPE_CHECKING:
    from ..lane import Lane


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
class LaneSection:
    """Lane section definition."""

    s: float = 0.0
    left: Optional[Left] = None
    center: Optional[Center] = None
    right: Optional[Right] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("laneSection")
        elem.set("s", str(self.s))

        if self.left:
            elem.append(self.left.to_xml())
        if self.center:
            elem.append(self.center.to_xml())
        if self.right:
            elem.append(self.right.to_xml())

        return elem


@dataclass
class Lanes:
    """Lanes container."""

    lane_sections: List[LaneSection]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("lanes")
        for lane_section in self.lane_sections:
            elem.append(lane_section.to_xml())
        return elem
