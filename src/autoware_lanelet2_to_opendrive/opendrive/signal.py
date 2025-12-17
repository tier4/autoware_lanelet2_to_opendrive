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
class ControlEntry:
    """Control entry linking a signal to a controller."""

    signal_id: int  # ID of the controlled signal
    type: str = ""  # Optional: type of control

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("control")
        elem.set("signalId", str(self.signal_id))
        if self.type:
            elem.set("type", self.type)
        return elem


@dataclass
class Controller:
    """
    OpenDRIVE signal controller representation.

    Controllers group multiple signals together for coordinated control,
    such as traffic light phases at an intersection.

    Reference: ASAM OpenDRIVE v1.8.1 - Section 14.3: Signal Controllers
    """

    id: int  # Controller ID
    name: str  # Controller name
    sequence: Optional[int] = None  # Optional sequence number
    controls: Optional[List[ControlEntry]] = None  # List of controlled signals

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("controller")

        # Set required attributes
        elem.set("id", str(self.id))
        elem.set("name", self.name)

        # Set optional sequence attribute
        if self.sequence is not None:
            elem.set("sequence", str(self.sequence))

        # Add control entries if present
        if self.controls:
            for control in self.controls:
                elem.append(control.to_xml())

        return elem

    def __repr__(self) -> str:
        """String representation of the controller."""
        num_controls = len(self.controls) if self.controls else 0
        return f"Controller(id={self.id}, name='{self.name}', controls={num_controls})"


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

    @staticmethod
    def construct_from_lanelet2_traffic_signal(
        traffic_light,  # lanelet2.core.TrafficLight regulatory element
        signal_id: int,
        s: float,
        t: float,
        lane_ids: Optional[List[int]] = None,
    ) -> "Signal":
        """Construct a Signal from a lanelet2 TrafficLight regulatory element.

        This method converts a lanelet2 TrafficLight regulatory element into an
        OpenDRIVE Signal object. The traffic light's position, type, and other
        attributes are extracted and mapped to OpenDRIVE format.

        Args:
            traffic_light: lanelet2 TrafficLight regulatory element containing
                          traffic light geometry and attributes
            signal_id: Unique signal identifier for the OpenDRIVE signal
            s: Position along road reference line (m). This must be computed
               by the caller based on the road's reference line.
            t: Lateral offset from road reference line (m). Negative values
               indicate signals on the right side of the road.
            lane_ids: List of lane IDs this signal applies to. If None, applies
                     to lane -1 (rightmost lane). Use negative IDs for right lanes.

        Returns:
            Signal object constructed from the traffic light with OpenDRIVE format

        Example:
            >>> # Get traffic light from lanelet
            >>> lanelet = lanelet_map.laneletLayer.get(lanelet_id)
            >>> traffic_lights = [reg for reg in lanelet.regulatoryElements
            ...                   if isinstance(reg, lanelet2.core.TrafficLight)]
            >>> traffic_light = traffic_lights[0]
            >>>
            >>> # Create signal with computed s, t coordinates
            >>> signal = Signal.construct_from_lanelet2_traffic_signal(
            ...     traffic_light=traffic_light,
            ...     signal_id=100,
            ...     s=50.0,  # Computed position along road
            ...     t=-4.5,  # Computed lateral offset
            ...     lane_ids=[-1]  # Applies to lane -1
            ... )
        """
        # Get traffic light geometry (position)
        # TrafficLight regulatory element contains trafficLights attribute
        # which is a list of LineString3d objects
        traffic_light_geometry = traffic_light.trafficLights
        if not traffic_light_geometry:
            raise ValueError(
                f"Traffic light with ID {traffic_light.id} has no geometry"
            )

        # Get the first traffic light linestring (most traffic lights have one)
        light_linestring = traffic_light_geometry[0]

        # Get position from the first point of the linestring
        if len(light_linestring) == 0:
            raise ValueError(
                f"Traffic light linestring for ID {traffic_light.id} is empty"
            )

        # Extract position (we'll use the first point as the signal position)
        position = light_linestring[0]
        z_offset = float(position.z) if hasattr(position, "z") else 0.0

        # Determine signal type from attributes
        # Check if traffic light has a 'subtype' or 'type' attribute
        signal_type = SignalType.TRAFFIC_LIGHT_3_LIGHTS  # Default type
        signal_subtype = -1

        # Try to extract type from attributes if available
        if hasattr(traffic_light, "attributes"):
            attrs = traffic_light.attributes
            # Check for common attribute keys used in lanelet2
            if "subtype" in attrs:
                subtype_str = attrs["subtype"]
                # Map lanelet2 subtypes to OpenDRIVE types
                if "red_yellow_green" in subtype_str or "3_lights" in subtype_str:
                    signal_type = SignalType.TRAFFIC_LIGHT_3_LIGHTS
                elif "pedestrian" in subtype_str:
                    signal_type = SignalType.TRAFFIC_LIGHT_PEDESTRIAN
                elif "arrow" in subtype_str:
                    signal_type = SignalType.TRAFFIC_LIGHT_ARROW
            elif "type" in attrs:
                type_str = attrs["type"]
                if "pedestrian" in type_str:
                    signal_type = SignalType.TRAFFIC_LIGHT_PEDESTRIAN
                elif "arrow" in type_str:
                    signal_type = SignalType.TRAFFIC_LIGHT_ARROW

        # Determine signal dimensions (width x height)
        # Default dimensions for a standard traffic light
        signal_height = 1.2  # meters
        signal_width = 0.6  # meters

        # Create lane validities
        validities = []
        if lane_ids is None:
            # Default to lane -1 (rightmost lane)
            validities.append(Validity(from_lane=-1, to_lane=-1))
        else:
            # Create validity for specified lanes
            if len(lane_ids) > 0:
                min_lane = min(lane_ids)
                max_lane = max(lane_ids)
                validities.append(Validity(from_lane=min_lane, to_lane=max_lane))

        # Determine orientation based on t-coordinate
        # Negative t means signal is on the right side (orientation "-")
        # Positive t means signal is on the left side (orientation "+")
        orientation = "-" if t < 0 else "+"

        # Create signal name from traffic light ID
        signal_name = f"TrafficLight_{traffic_light.id}"

        # Create and return the Signal object
        signal = Signal(
            id=signal_id,
            name=signal_name,
            s=s,
            t=t,
            z_offset=z_offset,
            h_offset=0.0,
            roll=0.0,
            pitch=0.0,
            orientation=orientation,
            dynamic="yes",  # Traffic lights are always dynamic
            country="OpenDRIVE",  # Use OpenDRIVE country code for simulation
            type=signal_type,
            subtype=signal_subtype,
            value=-1.0,  # No speed limit value for traffic lights
            text="",
            height=signal_height,
            width=signal_width,
            validities=validities if validities else None,
        )

        return signal


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
