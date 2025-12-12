"""OpenDRIVE junction definitions."""

from dataclasses import dataclass, field
from typing import List, Optional
import lxml.etree as ET

from .enums import ContactPoint


@dataclass
class LaneLink:
    """Lane link within a junction connection.

    Defines how a lane from the incoming road connects to a lane
    on the connecting road.
    """

    from_lane: int  # Lane ID on the incoming road
    to_lane: int  # Lane ID on the connecting road

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("laneLink")
        elem.set("from", str(self.from_lane))
        elem.set("to", str(self.to_lane))
        return elem


@dataclass
class Connection:
    """Connection within a junction.

    Defines a connection between an incoming road and a connecting road
    through the junction.
    """

    id: int  # Unique ID within this junction
    incoming_road: int  # ID of the incoming road
    connecting_road: int  # ID of the connecting road
    contact_point: ContactPoint  # Which end of connecting road is used
    lane_links: List[LaneLink] = field(default_factory=list)

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("connection")
        elem.set("id", str(self.id))
        elem.set("incomingRoad", str(self.incoming_road))
        elem.set("connectingRoad", str(self.connecting_road))
        elem.set("contactPoint", self.contact_point.value)

        for lane_link in self.lane_links:
            elem.append(lane_link.to_xml())

        return elem

    def add_lane_link(self, from_lane: int, to_lane: int) -> None:
        """Add a lane link to this connection.

        Args:
            from_lane: Lane ID on the incoming road
            to_lane: Lane ID on the connecting road
        """
        self.lane_links.append(LaneLink(from_lane=from_lane, to_lane=to_lane))


@dataclass
class Junction:
    """Junction definition.

    A junction represents an intersection or complex road connection
    where multiple roads meet.
    """

    id: int
    name: Optional[str] = None
    connections: List[Connection] = field(default_factory=list)

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("junction")
        elem.set("id", str(self.id))

        if self.name:
            elem.set("name", self.name)

        for connection in self.connections:
            elem.append(connection.to_xml())

        return elem

    def add_connection(
        self,
        connection_id: int,
        incoming_road: int,
        connecting_road: int,
        contact_point: ContactPoint,
    ) -> Connection:
        """Add a connection to this junction.

        Args:
            connection_id: Unique ID for this connection within the junction
            incoming_road: ID of the incoming road
            connecting_road: ID of the connecting road
            contact_point: Which end of connecting road is used

        Returns:
            The created Connection object
        """
        connection = Connection(
            id=connection_id,
            incoming_road=incoming_road,
            connecting_road=connecting_road,
            contact_point=contact_point,
        )
        self.connections.append(connection)
        return connection
