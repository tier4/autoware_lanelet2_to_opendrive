"""OpenDRIVE lane element definitions."""

from dataclasses import dataclass
from typing import Mapping, Optional
import lxml.etree as ET

from .enums import (
    RoadMarkType,
    RoadMarkColor,
    RoadMarkLaneChange,
    RoadMarkWeight,
    SpeedUnit,
    RoadType,
)
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
    weight: Optional[RoadMarkWeight] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("roadMark")
        elem.set("sOffset", str(self.s_offset))
        elem.set("type", self.type.value)
        elem.set("color", self.color.value)
        # OpenDRIVE spec defaults <roadMark weight> to "standard"; only emit when
        # we have something non-default to record (e.g. BOLD for line_thick).
        if self.weight is not None and self.weight is not RoadMarkWeight.STANDARD:
            elem.set("weight", self.weight.value)
        if self.lane_change is not None:
            elem.set("laneChange", self.lane_change.value)
        return elem


# ---------------------------------------------------------------------------
# LineString → RoadMark mapping
# ---------------------------------------------------------------------------
#
# Mapping tables for converting Lanelet2 LineString boundary attributes
# (type/subtype/color/lane_change) into OpenDRIVE RoadMark fields.
#
# References:
#   - docs/spec-mapping/lanelet2-autoware-profile.md §"Boundary marking types"
#   - docs/spec-mapping/opendrive-14-carla-profile.md §"Road marks"
#
# Notes:
#   - OpenDRIVE's `laneChange` attribute is lane-id-direction-relative
#     (``increase`` = toward more-positive lane IDs, ``decrease`` = toward
#     more-negative lane IDs). Lanelet2's ``lane_change`` attribute is
#     world-relative (``left``/``right``). The mapping depends on the
#     handedness of the road:
#       * RHT: right-side lanes have negative IDs, left-side lanes have
#         positive IDs → lane_change=left → INCREASE, right → DECREASE.
#       * LHT: right-side lanes have positive IDs, left-side lanes have
#         negative IDs → lane_change=left → DECREASE, right → INCREASE.
#     Callers must pass ``is_lht`` when the mapping should respect
#     handedness; it defaults to False (RHT).

_TYPE_TABLE: dict[tuple[str, str], tuple[RoadMarkType, RoadMarkWeight]] = {
    ("line_thin", "solid"): (RoadMarkType.SOLID, RoadMarkWeight.STANDARD),
    ("line_thin", "dashed"): (RoadMarkType.BROKEN, RoadMarkWeight.STANDARD),
    ("line_thin", "solid_solid"): (RoadMarkType.SOLID_SOLID, RoadMarkWeight.STANDARD),
    ("line_thin", "solid_dashed"): (RoadMarkType.SOLID_BROKEN, RoadMarkWeight.STANDARD),
    ("line_thin", "dashed_solid"): (RoadMarkType.BROKEN_SOLID, RoadMarkWeight.STANDARD),
    ("line_thin", "dashed_dashed"): (
        RoadMarkType.BROKEN_BROKEN,
        RoadMarkWeight.STANDARD,
    ),
    ("line_thick", "solid"): (RoadMarkType.SOLID, RoadMarkWeight.BOLD),
    ("line_thick", "dashed"): (RoadMarkType.BROKEN, RoadMarkWeight.BOLD),
    ("line_thick", "solid_solid"): (RoadMarkType.SOLID_SOLID, RoadMarkWeight.BOLD),
    ("line_thick", "solid_dashed"): (RoadMarkType.SOLID_BROKEN, RoadMarkWeight.BOLD),
    ("line_thick", "dashed_solid"): (RoadMarkType.BROKEN_SOLID, RoadMarkWeight.BOLD),
    ("line_thick", "dashed_dashed"): (RoadMarkType.BROKEN_BROKEN, RoadMarkWeight.BOLD),
    ("road_border", ""): (RoadMarkType.CURB, RoadMarkWeight.STANDARD),
    ("curbstone", ""): (RoadMarkType.CURB, RoadMarkWeight.STANDARD),
    ("virtual", ""): (RoadMarkType.NONE, RoadMarkWeight.STANDARD),
    ("guard_rail", ""): (RoadMarkType.EDGE, RoadMarkWeight.STANDARD),
}

_COLOR_TABLE: dict[str, RoadMarkColor] = {
    "white": RoadMarkColor.WHITE,
    "yellow": RoadMarkColor.YELLOW,
    "red": RoadMarkColor.RED,
    "blue": RoadMarkColor.BLUE,
    "green": RoadMarkColor.GREEN,
    "orange": RoadMarkColor.ORANGE,
}


def road_mark_from_linestring_attrs(
    s_offset: float,
    attrs: Mapping[str, str],
    is_lht: bool = False,
) -> RoadMark:
    """Map Lanelet2 LineString attributes to an OpenDRIVE RoadMark.

    Args:
        s_offset: s-coordinate at which this road mark begins.
        attrs: Dict of Lanelet2 LineString attributes. Typically
            ``dict(linestring.attributes)``. Recognised keys:
            ``type``, ``subtype``, ``color``, ``lane_change``.
        is_lht: Whether the parent map uses left-hand traffic. Affects
            the ``lane_change`` world-relative → lane-id-relative
            mapping (see module docstring).

    Returns:
        A :class:`RoadMark` whose fields are populated from the mapping
        tables. Unknown combinations fall back to ``solid`` / ``white`` /
        ``standard`` and no ``laneChange``.

    References:
        - docs/spec-mapping/lanelet2-autoware-profile.md §"Boundary marking types"
        - docs/spec-mapping/opendrive-14-carla-profile.md §"Road marks"
    """
    ls_type = attrs.get("type", "") or ""
    ls_sub = attrs.get("subtype", "") or ""
    ls_col = (attrs.get("color", "") or "").lower()
    ls_lc = (attrs.get("lane_change", "") or "").lower()

    rm_type, rm_weight = _TYPE_TABLE.get(
        (ls_type, ls_sub),
        (RoadMarkType.SOLID, RoadMarkWeight.STANDARD),  # fallback
    )

    rm_color = _COLOR_TABLE.get(ls_col, RoadMarkColor.WHITE)

    # lane_change: map world-relative Lanelet2 value to lane-id-relative
    # OpenDRIVE value, flipping for LHT handedness.
    rm_lc: Optional[RoadMarkLaneChange]
    if ls_lc in ("yes", "both"):
        rm_lc = RoadMarkLaneChange.BOTH
    elif ls_lc == "no":
        rm_lc = RoadMarkLaneChange.NONE
    elif ls_lc == "left":
        rm_lc = RoadMarkLaneChange.DECREASE if is_lht else RoadMarkLaneChange.INCREASE
    elif ls_lc == "right":
        rm_lc = RoadMarkLaneChange.INCREASE if is_lht else RoadMarkLaneChange.DECREASE
    else:
        rm_lc = None

    return RoadMark(
        s_offset=s_offset,
        type=rm_type,
        color=rm_color,
        lane_change=rm_lc,
        weight=rm_weight,
    )


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
class LaneAccess:
    """OpenDRIVE 1.4 lane access restriction.

    Corresponds to a single ``<lane><access>`` element. ``restriction``
    must be a value from the OpenDRIVE 1.4 ``e_accessRestrictionType``
    enumeration (e.g. ``passengerCar``, ``pedestrian``, ``bicycle``,
    ``bus``, ``taxi``, ``truck``, ``motorcycle``).
    """

    s_offset: float
    rule: str
    restriction: str

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("access")
        elem.set("sOffset", str(self.s_offset))
        elem.set("rule", self.rule)
        elem.set("restriction", self.restriction)
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
