"""OpenDRIVE signal definitions."""

from dataclasses import dataclass
from typing import Any, Optional, List
import lxml.etree as ET

from ..config import COORDINATE_OFFSET


@dataclass
class Dependency:
    """Dependency reference from a stop line signal to a controlling traffic light.

    Per ASAM OpenDRIVE specification, stop line signals shall have a dependency
    to each of the corresponding traffic lights.

    Reference: ASAM OpenDRIVE Junction Guideline - Section 9: Traffic Lights
    """

    id: int  # ID of the dependent signal (e.g., traffic light signal ID)
    type: str  # Type of dependency (e.g., "trafficLight")

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("dependency")
        elem.set("id", str(self.id))
        elem.set("type", self.type)
        return elem


@dataclass
class Reference:
    """Reference from a traffic light signal to its associated stop line.

    Per ASAM OpenDRIVE specification, traffic light signals should reference
    their associated stop lines.

    Reference: ASAM OpenDRIVE Junction Guideline - Section 9: Traffic Lights
    """

    id: int  # ID of the referenced signal (e.g., stop line signal ID)
    element_type: str  # Type of referenced element (e.g., "signal")
    type: str  # Type qualifier (e.g., "stopLine")

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("reference")
        elem.set("id", str(self.id))
        elem.set("elementType", self.element_type)
        elem.set("type", self.type)
        return elem


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
class PositionInertial:
    """Physical position of a signal in inertial (world) coordinates.

    Per ASAM OpenDRIVE specification, positionInertial provides the absolute
    position of a signal independent of the road coordinate system.
    """

    x: float
    y: float
    z: float
    hdg: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("positionInertial")
        elem.set("x", f"{self.x:.16e}")
        elem.set("y", f"{self.y:.16e}")
        elem.set("z", f"{self.z:.16e}")
        elem.set("hdg", f"{self.hdg:.16e}")
        elem.set("pitch", f"{self.pitch:.16e}")
        elem.set("roll", f"{self.roll:.16e}")
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
    dependencies: Optional[List[Dependency]] = (
        None  # Dependencies on other signals (e.g., stop line -> traffic light)
    )
    references: Optional[List[Reference]] = (
        None  # References to related signals (e.g., traffic light -> stop line)
    )
    position_inertial: Optional[PositionInertial] = (
        None  # Physical position in inertial coordinates
    )

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

        # Add dependencies if present (e.g., stop line -> traffic light)
        if self.dependencies:
            for dependency in self.dependencies:
                elem.append(dependency.to_xml())

        # Add references if present (e.g., traffic light -> stop line)
        if self.references:
            for reference in self.references:
                elem.append(reference.to_xml())

        # Add positionInertial if present (physical position in world coordinates)
        if self.position_inertial is not None:
            elem.append(self.position_inertial.to_xml())

        # Add user data if present
        if self.user_data:
            elem.append(self.user_data.to_xml())

        return elem

    def to_signal_reference_xml(self) -> ET.Element:
        """Convert to XML signalReference element.

        SignalReferences are simplified versions of signals that reference
        the signal position on the reference line (t=0.0) for lane assignment.
        They share the same id, s, orientation, and validity as the signal.

        Returns:
            XML element for signalReference
        """
        elem = ET.Element("signalReference")

        # Set required attributes (subset of signal attributes)
        elem.set("id", str(self.id))
        elem.set("s", f"{self.s:.16e}")  # Same scientific notation as Signal
        elem.set("t", "0.0000000000000000e+00")  # Always on reference line
        elem.set("orientation", self.orientation)

        # Add validities (same as signal)
        if self.validities:
            for validity in self.validities:
                elem.append(validity.to_xml())

        return elem

    def __repr__(self) -> str:
        """String representation of the signal."""
        return (
            f"Signal(id={self.id}, name='{self.name}', type={self.type}, "
            f"s={self.s:.2f}, t={self.t:.2f})"
        )

    @staticmethod
    def construct_from_lanelet2_traffic_signal(
        traffic_light: Any,  # lanelet2.core.TrafficLight regulatory element
        signal_id: int,
        s: float,
        t: float,
        lane_ids: Optional[List[int]] = None,
        road_elevation_at_s: Optional[float] = None,
        light_linestring: Any = None,
        position_inertial: Optional["PositionInertial"] = None,
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
            road_elevation_at_s: Elevation of the road surface at position s (m).
                If provided, used to calculate relative z_offset from
                signal's absolute height. If None, uses absolute height.
            light_linestring: Specific Light Bulb LineString to use for this
                signal. If None, falls back to trafficLights[0].
            position_inertial: Physical position of the signal in inertial
                coordinates. If provided, attached to the Signal object.

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
        # Determine which linestring to use
        if light_linestring is None:
            # Fallback: use the first linestring from the regulatory element
            traffic_light_geometry = traffic_light.trafficLights
            if not traffic_light_geometry:
                raise ValueError(
                    f"Traffic light with ID {traffic_light.id} has no geometry"
                )
            light_linestring = traffic_light_geometry[0]

        if len(light_linestring) == 0:
            raise ValueError(
                f"Traffic light linestring for ID {traffic_light.id} is empty"
            )

        # Calculate z_offset from the centroid z of all points in the linestring
        n = len(light_linestring)
        centroid_z = sum(float(light_linestring[i].z) for i in range(n)) / n
        signal_absolute_z = centroid_z - COORDINATE_OFFSET.z
        if road_elevation_at_s is not None:
            z_offset = signal_absolute_z - road_elevation_at_s
        else:
            z_offset = signal_absolute_z

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

        # Create signal name from traffic light ID and linestring ID
        linestring_id = getattr(light_linestring, "id", None)
        if linestring_id is not None:
            signal_name = f"TrafficLight_{traffic_light.id}_{linestring_id}"
        else:
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
            position_inertial=position_inertial,
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

    # Yield sign (type 205 corresponds to German StVO sign 205 - "Vorfahrt gewähren")
    # Used in OpenDRIVE to represent yield signs at intersections
    YIELD_SIGN = 205

    # Stop sign (type 206 corresponds to German StVO sign 206 - "Halt! Vorfahrt gewähren")
    # Used in OpenDRIVE to represent mandatory stop signs at intersections
    STOP_SIGN = 206

    # Stop line (type 294 corresponds to German StVO sign 294 - stop line)
    # Used in OpenDRIVE to represent painted stop lines with signal dependencies
    STOP_LINE = 294

    # Custom types should use appropriate country codes and follow
    # national regulations or use country="OpenDRIVE" for simulation-specific signals
