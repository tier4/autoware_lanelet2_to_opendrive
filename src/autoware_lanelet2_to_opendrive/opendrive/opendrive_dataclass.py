"""OpenDRIVE dataclass definitions."""

from enum import Enum
from dataclasses import dataclass


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


@dataclass
class RoadMark:
    """Road marking definition."""

    s_offset: float
    type: RoadMarkType
    color: RoadMarkColor


@dataclass
class LaneLink:
    """Lane link definition for predecessor/successor."""

    id: int


@dataclass
class LaneBorder:
    """Lane border definition."""

    s_offset: float
    a: float
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0


@dataclass
class LaneHeight:
    """Lane height definition."""

    s_offset: float
    inner: float
    outer: float
