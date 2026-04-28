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

    NONE = "none"
    SOLID = "solid"
    BROKEN = "broken"
    SOLID_SOLID = "solid_solid"
    SOLID_BROKEN = "solid_broken"
    BROKEN_SOLID = "broken_solid"
    BROKEN_BROKEN = "broken_broken"
    BOTTS_DOTS = "botts dots"  # space per OpenDRIVE spec
    GRASS = "grass"
    CURB = "curb"
    CUSTOM = "custom"
    EDGE = "edge"


class RoadMarkWeight(Enum):
    """Road mark weights according to OpenDRIVE standard."""

    STANDARD = "standard"
    BOLD = "bold"


class RoadMarkColor(Enum):
    """Road mark colors according to OpenDRIVE standard."""

    STANDARD = "standard"
    WHITE = "white"
    YELLOW = "yellow"
    RED = "red"
    BLUE = "blue"
    GREEN = "green"
    ORANGE = "orange"


class RoadMarkLaneChange(Enum):
    """Lane change permission for road markings according to OpenDRIVE standard."""

    INCREASE = "increase"  # Lane change only in direction of increasing lane IDs
    DECREASE = "decrease"  # Lane change only in direction of decreasing lane IDs
    BOTH = "both"  # Lane change allowed in both directions
    NONE = "none"  # No lane change allowed


class GeometryType(Enum):
    """Geometry types supported in OpenDRIVE planView."""

    LINE = "line"
    ARC = "arc"
    SPIRAL = "spiral"
    POLY3 = "poly3"
    PARAMPOLY3 = "paramPoly3"


class ContactPoint(Enum):
    """Contact point for road link connections."""

    START = "start"
    END = "end"


class ElementType(Enum):
    """Element type for road link connections."""

    ROAD = "road"
    JUNCTION = "junction"


class RoadType(Enum):
    """Road type classification according to OpenDRIVE standard."""

    UNKNOWN = "unknown"
    RURAL = "rural"
    MOTORWAY = "motorway"
    TOWN = "town"
    LOW_SPEED = "lowSpeed"
    PEDESTRIAN = "pedestrian"
    BICYCLE = "bicycle"


class SpeedUnit(Enum):
    """Speed unit for speed limits."""

    MS = "m/s"  # meters per second
    MPH = "mph"  # miles per hour
    KMH = "km/h"  # kilometers per hour


class TrafficRule(Enum):
    """Traffic rule according to OpenDRIVE standard (e_trafficRule)."""

    RHT = "RHT"  # Right-hand traffic
    LHT = "LHT"  # Left-hand traffic


class LaneMode(Enum):
    """Selects between <lane><width> and <lane><border> emission paths.

    WIDTH: Existing default. The lane is described by a <width> polynomial
        whose value is the lateral distance from the previous lane outward.
    BORDER: Used by connecting roads when their reference-line endpoints
        are pinned to linked regular roads (issue #437). The lane is
        described by an absolute <border> polynomial t(s) so endpoint
        constraints can pin the lane edge to match the linked road's lane
        edge exactly.
    """

    WIDTH = "width"
    BORDER = "border"
