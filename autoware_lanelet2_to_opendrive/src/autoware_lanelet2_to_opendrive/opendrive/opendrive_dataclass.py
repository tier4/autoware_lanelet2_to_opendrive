"""OpenDRIVE dataclass definitions - Main module for backward compatibility."""

# Import all classes from the split modules for backward compatibility
from .enums import (
    LaneType,
    RoadMarkType,
    RoadMarkColor,
    RoadMarkLaneChange,
    RoadMarkWeight,
    GeometryType,
    RoadType,
    SpeedUnit,
)
from .lane_elements import (
    LaneWidth,
    RoadMark,
    LaneLink,
    LaneBorder,
    LaneHeight,
    LaneSpeed,
    RoadTypeSpeed,
    RoadTypeDefinition,
)
from .geometry import GeometryBase, Line, Arc, Spiral, PlanView
from .elevation import Elevation, ElevationProfile

# Import LaneSection directly for code that needs it
# Note: Lane is imported separately to avoid circular imports
from .lane_sections import Left, Center, Right, Lanes
from .road import Road
from .header import Header
from .junction import Junction
from .opendrive import OpenDRIVE, export_to_xml, save_opendrive_to_file
from .signal import Signal, Validity, SignalUserData, SignalType

# Re-export everything for backward compatibility
__all__ = [
    # Enums
    "LaneType",
    "RoadMarkType",
    "RoadMarkColor",
    "RoadMarkLaneChange",
    "RoadMarkWeight",
    "GeometryType",
    "RoadType",
    "SpeedUnit",
    # Lane elements
    "LaneWidth",
    "RoadMark",
    "LaneLink",
    "LaneBorder",
    "LaneHeight",
    "LaneSpeed",
    "RoadTypeSpeed",
    "RoadTypeDefinition",
    # Geometry
    "GeometryBase",
    "Line",
    "Arc",
    "Spiral",
    "PlanView",
    # Elevation
    "Elevation",
    "ElevationProfile",
    # Lane sections
    "Left",
    "Center",
    "Right",
    "Lanes",
    # Road and structure
    "Road",
    "Header",
    "Junction",
    "OpenDRIVE",
    # Signals
    "Signal",
    "Validity",
    "SignalUserData",
    "SignalType",
    # Export functions
    "export_to_xml",
    "save_opendrive_to_file",
]
