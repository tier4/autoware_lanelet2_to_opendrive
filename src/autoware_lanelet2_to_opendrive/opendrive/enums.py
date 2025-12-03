"""OpenDRIVE enum definitions."""

from enum import Enum


class LaneType(Enum):
    """Lane types according to OpenDRIVE standard."""

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


class GeometryType(Enum):
    """Geometry types supported in OpenDRIVE planView."""

    LINE = "line"
    ARC = "arc"
    SPIRAL = "spiral"
    POLY3 = "poly3"
    PARAMPOLY3 = "paramPoly3"
