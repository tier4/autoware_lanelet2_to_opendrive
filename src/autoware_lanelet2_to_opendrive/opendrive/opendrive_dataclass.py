"""OpenDRIVE dataclass definitions."""

from enum import Enum
from dataclasses import dataclass
from typing import Optional, List
import lxml.etree as ET


class LaneType(Enum):
    """Lane types according to OpenDRIVE standard."""

    DRIVING = "driving"
    STOP = "stop"
    SHOULDER = "shoulder"
    BIKING = "biking"
    SIDEWALK = "sidewalk"
    BORDER = "border"
    RESTRICTED = "restricted"
    PARKING = "parking"
    BIDIRECTIONAL = "bidirectional"
    MEDIAN = "median"
    SPECIAL1 = "special1"
    SPECIAL2 = "special2"
    SPECIAL3 = "special3"
    ROADWORKS = "roadWorks"
    TRAM = "tram"
    RAIL = "rail"
    ENTRY = "entry"
    EXIT = "exit"
    OFF_RAMP = "offRamp"
    ON_RAMP = "onRamp"
    CONNECTING_RAMP = "connectingRamp"


class RoadMarkType(Enum):
    """Road mark types according to OpenDRIVE standard."""

    SOLID = "solid"
    BROKEN = "broken"
    SOLID_SOLID = "solid_solid"
    SOLID_BROKEN = "solid_broken"
    BROKEN_SOLID = "broken_solid"


class RoadMarkColor(Enum):
    """Road mark colors according to OpenDRIVE standard."""

    STANDARD = "standard"
    WHITE = "white"
    YELLOW = "yellow"
    RED = "red"
    BLUE = "blue"
    GREEN = "green"
    ORANGE = "orange"


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


class GeometryType(Enum):
    """Geometry types supported in OpenDRIVE planView."""

    LINE = "line"
    ARC = "arc"
    SPIRAL = "spiral"
    POLY3 = "poly3"
    PARAMPOLY3 = "paramPoly3"


@dataclass
class GeometryBase:
    """Base class for geometry records in planView."""

    s: float = 0.0  # s-coordinate of the start position
    x: float = 0.0  # x-coordinate of the start position
    y: float = 0.0  # y-coordinate of the start position
    hdg: float = 0.0  # heading at the start position (radians)
    length: float = 0.0  # length of the geometry segment

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("geometry")
        elem.set("s", str(self.s))
        elem.set("x", str(self.x))
        elem.set("y", str(self.y))
        elem.set("hdg", str(self.hdg))
        elem.set("length", str(self.length))
        return elem


@dataclass
class Line(GeometryBase):
    """Straight line geometry."""

    geometry_type = GeometryType.LINE

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = super().to_xml()
        ET.SubElement(elem, "line")
        return elem


@dataclass
class Arc(GeometryBase):
    """Arc geometry with constant curvature."""

    curvature: float = 0.0  # constant curvature (1/radius)
    geometry_type = GeometryType.ARC

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = super().to_xml()
        arc_elem = ET.SubElement(elem, "arc")
        arc_elem.set("curvature", str(self.curvature))
        return elem


@dataclass
class Spiral(GeometryBase):
    """Spiral/clothoid geometry."""

    curvStart: float = 0.0  # curvature at start
    curvEnd: float = 0.0  # curvature at end
    geometry_type = GeometryType.SPIRAL

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = super().to_xml()
        spiral_elem = ET.SubElement(elem, "spiral")
        spiral_elem.set("curvStart", str(self.curvStart))
        spiral_elem.set("curvEnd", str(self.curvEnd))
        return elem


@dataclass
class PlanView:
    """Plan view container for geometry."""

    geometries: List[GeometryBase]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("planView")
        for geometry in self.geometries:
            elem.append(geometry.to_xml())
        return elem


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


@dataclass
class Lane:
    """Lane definition."""

    id: int = 0
    type: str = "driving"
    level: str = "false"
    link: Optional[LaneLink] = None
    widths: Optional[List[LaneWidth]] = None
    road_marks: Optional[List[RoadMark]] = None
    borders: Optional[List[LaneBorder]] = None
    heights: Optional[List[LaneHeight]] = None

    def __post_init__(self):
        if self.widths is None:
            self.widths = []
        if self.road_marks is None:
            self.road_marks = []
        if self.borders is None:
            self.borders = []
        if self.heights is None:
            self.heights = []

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("lane")
        elem.set("id", str(self.id))
        elem.set("type", self.type)
        elem.set("level", self.level)

        if self.link:
            elem.append(self.link.to_xml())

        if self.widths:
            for width in self.widths:
                elem.append(width.to_xml())

        if self.road_marks:
            for road_mark in self.road_marks:
                elem.append(road_mark.to_xml())

        if self.borders:
            for border in self.borders:
                elem.append(border.to_xml())

        if self.heights:
            for height in self.heights:
                elem.append(height.to_xml())

        return elem


@dataclass
class Left:
    """Left lane section."""

    lanes: List[Lane]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("left")
        for lane in self.lanes:
            elem.append(lane.to_xml())
        return elem


@dataclass
class Center:
    """Center lane section."""

    lanes: List[Lane]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("center")
        for lane in self.lanes:
            elem.append(lane.to_xml())
        return elem


@dataclass
class Right:
    """Right lane section."""

    lanes: List[Lane]

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("right")
        for lane in self.lanes:
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


@dataclass
class Road:
    """Road definition."""

    id: int = 0
    name: Optional[str] = None
    length: float = 0.0
    junction: int = -1
    plan_view: Optional[PlanView] = None
    elevation_profile: Optional[ElevationProfile] = None
    lanes: Optional[Lanes] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("road")
        elem.set("id", str(self.id))
        elem.set("length", str(self.length))
        elem.set("junction", str(self.junction))

        if self.name:
            elem.set("name", self.name)

        if self.plan_view:
            elem.append(self.plan_view.to_xml())
        if self.elevation_profile:
            elem.append(self.elevation_profile.to_xml())
        if self.lanes:
            elem.append(self.lanes.to_xml())

        return elem


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

        return elem


@dataclass
class OpenDRIVE:
    """Root OpenDRIVE element."""

    header: Optional[Header] = None
    roads: Optional[List[Road]] = None

    def __post_init__(self):
        if self.roads is None:
            self.roads = []

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("OpenDRIVE")

        if self.header:
            elem.append(self.header.to_xml())

        if self.roads:
            for road in self.roads:
                elem.append(road.to_xml())

        return elem


# Export functionality


def export_to_xml(opendrive: OpenDRIVE) -> str:
    """
    Export OpenDRIVE object to XML string.

    Args:
        opendrive: OpenDRIVE object to export

    Returns:
        XML string representation
    """
    xml_element = opendrive.to_xml()

    # Pretty print
    xml_str = ET.tostring(
        xml_element, pretty_print=True, xml_declaration=True, encoding="UTF-8"
    ).decode("utf-8")

    return xml_str


def save_opendrive_to_file(opendrive: OpenDRIVE, filepath: str) -> None:
    """
    Save OpenDRIVE object to XML file.

    Args:
        opendrive: OpenDRIVE object to save
        filepath: Path to save the XML file
    """
    xml_str = export_to_xml(opendrive)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(xml_str)
