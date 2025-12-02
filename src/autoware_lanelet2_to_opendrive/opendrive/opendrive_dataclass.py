"""Data classes for OpenDRIVE conversion."""

from dataclasses import dataclass
from enum import Enum


class LaneType(Enum):
    """OpenDRIVE lane types."""

    NONE = "none"
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
    ROAD_WORKS = "roadWorks"
    TRAM = "tram"
    RAIL = "rail"
    ENTRY = "entry"
    EXIT = "exit"
    OFF_RAMP = "offRamp"
    ON_RAMP = "onRamp"
    CONNECTING_RAMP = "connectingRamp"
    BUS = "bus"
    TAXI = "taxi"
    HOV = "hov"


class RoadMarkType(Enum):
    """OpenDRIVE road mark types."""

    NONE = "none"
    SOLID = "solid"
    BROKEN = "broken"
    SOLID_SOLID = "solid solid"
    SOLID_BROKEN = "solid broken"
    BROKEN_SOLID = "broken solid"
    BROKEN_BROKEN = "broken broken"
    BOTTS_DOTS = "botts dots"
    REFLECTORS = "reflectors"
    GRASS = "grass"
    COBBLE = "cobble"
    PAVING_STONES = "paving stones"
    PAVING = "paving"
    CONCRETE = "concrete"
    ASPHALT = "asphalt"


class RoadMarkColor(Enum):
    """OpenDRIVE road mark colors."""

    STANDARD = "standard"
    BLUE = "blue"
    GREEN = "green"
    RED = "red"
    WHITE = "white"
    YELLOW = "yellow"
    ORANGE = "orange"


class LaneChangeType(Enum):
    """OpenDRIVE lane change types."""

    INCREASE = "increase"
    DECREASE = "decrease"
    BOTH = "both"
    NONE = "none"


@dataclass
class LaneWidth:
    """Lane width definition."""

    s_offset: float
    a: float  # Width at s_offset
    b: float = 0.0  # Linear coefficient
    c: float = 0.0  # Quadratic coefficient
    d: float = 0.0  # Cubic coefficient


@dataclass
class RoadMark:
    """Road mark definition."""

    s_offset: float
    type: RoadMarkType
    weight: str = "standard"
    color: RoadMarkColor = RoadMarkColor.STANDARD
    width: float = 0.12
    lane_change: LaneChangeType = LaneChangeType.BOTH


@dataclass
class LaneLink:
    """Lane link definition for predecessors and successors."""

    id: int


@dataclass
class LaneBorder:
    """Lane border definition."""

    s_offset: float
    a: float  # Border position at s_offset
    b: float = 0.0  # Linear coefficient
    c: float = 0.0  # Quadratic coefficient
    d: float = 0.0  # Cubic coefficient


@dataclass
class LaneHeight:
    """Lane height definition."""

    s_offset: float
    inner: float
    outer: float
