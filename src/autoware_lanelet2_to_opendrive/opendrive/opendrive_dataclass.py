"""OpenDRIVE dataclass definitions - Main module for backward compatibility."""

# Import all classes from the split modules for backward compatibility
from .enums import LaneType, RoadMarkType, RoadMarkColor, GeometryType
from .lane_elements import LaneWidth, RoadMark, LaneLink, LaneBorder, LaneHeight
from .geometry import GeometryBase, Line, Arc, Spiral, PlanView
from .elevation import Elevation, ElevationProfile
from .lane_sections import Left, Center, Right, LaneSection, Lanes
from .road import Road
from .header import Header
from .opendrive import OpenDRIVE, export_to_xml, save_opendrive_to_file

# Re-export everything for backward compatibility
__all__ = [
    # Enums
    "LaneType",
    "RoadMarkType",
    "RoadMarkColor",
    "GeometryType",
    # Lane elements
    "LaneWidth",
    "RoadMark",
    "LaneLink",
    "LaneBorder",
    "LaneHeight",
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
    "LaneSection",
    "Lanes",
    # Road and structure
    "Road",
    "Header",
    "OpenDRIVE",
    # Export functions
    "export_to_xml",
    "save_opendrive_to_file",
]
