"""OpenDRIVE signal definitions."""

from dataclasses import dataclass
from typing import Optional, List
import lxml.etree as ET


@dataclass
class Validity:
    """Signal validity element defining which lanes the signal applies to."""

    from_lane: int
    to_lane: int

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("validity")
        elem.set("fromLane", str(self.from_lane))
        elem.set("toLane", str(self.to_lane))
        return elem


@dataclass
class SignalUserData:
    """User data for signal (custom extensions)."""

    data: dict

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("userData")
        # Add custom user data elements
        for key, value in self.data.items():
            child = ET.SubElement(elem, key)
            if isinstance(value, dict):
                for attr_key, attr_value in value.items():
                    child.set(attr_key, str(attr_value))
            else:
                child.text = str(value)
        return elem


@dataclass
class Signal:
    """
    OpenDRIVE signal representation.

    Signals are traffic signs, traffic lights, and specific road markings
    for the control and regulation of road traffic.

    Reference: ASAM OpenDRIVE v1.8.1 - Section 14: Signals
    """

    id: int
    name: str
    s: float  # s-coordinate along the road reference line
    t: float  # t-coordinate lateral offset from the road reference line
    dynamic: str  # "yes" or "no" - whether signal changes during simulation
    orientation: str  # "+" or "-" - orientation with respect to road direction
    country: str  # Country code (e.g., "OpenDRIVE", "US", "DE")
    type: int  # Signal type ID
    subtype: int  # Signal subtype ID
    z_offset: float = 0.0  # Height offset
    h_offset: float = 0.0  # Heading offset
    roll: float = 0.0  # Roll angle
    pitch: float = 0.0  # Pitch angle
    value: float = -1.0  # Signal value (e.g., speed limit value)
    text: str = ""  # Signal text content
    height: float = 0.0  # Signal height
    width: float = 0.0  # Signal width
    validities: Optional[List[Validity]] = None  # Lane validity definitions
    user_data: Optional[SignalUserData] = None  # Custom user data

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("signal")

        # Set required attributes
        elem.set("id", str(self.id))
        elem.set("name", self.name)
        elem.set("s", f"{self.s:.16e}")  # Scientific notation with 16 decimals
        elem.set("t", f"{self.t:.16e}")
        elem.set("zOffset", f"{self.z_offset:.16e}")
        elem.set("hOffset", f"{self.h_offset:.16e}")
        elem.set("roll", f"{self.roll:.16e}")
        elem.set("pitch", f"{self.pitch:.16e}")
        elem.set("orientation", self.orientation)
        elem.set("dynamic", self.dynamic)
        elem.set("country", self.country)
        elem.set("type", str(self.type))
        elem.set("subtype", str(self.subtype))
        elem.set("value", f"{self.value:.16e}")
        elem.set("text", self.text)
        elem.set("height", f"{self.height:.16e}")
        elem.set("width", f"{self.width:.16e}")

        # Add validities if present
        if self.validities:
            for validity in self.validities:
                elem.append(validity.to_xml())

        # Add user data if present
        if self.user_data:
            elem.append(self.user_data.to_xml())

        return elem

    def __repr__(self) -> str:
        """String representation of the signal."""
        return (
            f"Signal(id={self.id}, name='{self.name}', type={self.type}, "
            f"s={self.s:.2f}, t={self.t:.2f})"
        )


# Common signal type constants (from ASAM OpenDRIVE specification)
class SignalType:
    """Common signal type IDs for OpenDRIVE signals."""

    # Traffic lights (type 1000001-1000003 commonly used with country="OpenDRIVE")
    TRAFFIC_LIGHT_3_LIGHTS = (
        1000001  # Standard 3-light traffic signal (red, yellow, green)
    )
    TRAFFIC_LIGHT_PEDESTRIAN = 1000002  # Pedestrian traffic light
    TRAFFIC_LIGHT_ARROW = 1000003  # Arrow traffic light

    # Custom types should use appropriate country codes and follow
    # national regulations or use country="OpenDRIVE" for simulation-specific signals
