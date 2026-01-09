"""OpenDRIVE junction definitions."""

from dataclasses import dataclass, field
from typing import List, Optional
import lxml.etree as ET
import lanelet2

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
    controller_ids: List[int] = field(default_factory=list)

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("junction")
        elem.set("id", str(self.id))

        if self.name:
            elem.set("name", self.name)

        # Add controller references (OpenDRIVE 1.4+ specification)
        # These reference the controllers that manage signals at this junction
        for controller_id in self.controller_ids:
            controller_elem = ET.SubElement(elem, "controller")
            controller_elem.set("id", str(controller_id))

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

    @staticmethod
    def construct_from_lanelet_groups(
        junction_id: int,
        lanelet_group: List[lanelet2.core.Lanelet],
        name: Optional[str] = None,
    ) -> "Junction":
        """Construct a Junction from a group of lanelets.

        This method creates a Junction instance from a single lanelet group
        as returned by find_junction_groups(). The connections are not
        automatically created - they should be added separately using
        add_connection() based on the road network structure.

        Args:
            junction_id: Unique ID for this junction
            lanelet_group: List of lanelets that form this junction
                          (one group from find_junction_groups result)
            name: Optional name for this junction. If not provided,
                  will be generated from the lanelet IDs

        Returns:
            Junction instance with the specified ID and name, but no connections yet

        Example:
            >>> from autoware_lanelet2_to_opendrive.junction import find_junction_groups
            >>> junction_groups = find_junction_groups(junction_lanelets)
            >>> junctions = []
            >>> for i, group in enumerate(junction_groups):
            >>>     junction = Junction.construct_from_lanelet_groups(
            >>>         junction_id=i,
            >>>         lanelet_group=group,
            >>>         name=f"Junction_{i}"
            >>>     )
            >>>     junctions.append(junction)
        """
        # Generate a name if not provided
        if name is None:
            if lanelet_group:
                lanelet_ids = sorted([ll.id for ll in lanelet_group])
                name = f"junction_{lanelet_ids[0]}"
            else:
                name = f"junction_{junction_id}"

        # Create the junction without connections
        junction = Junction(id=junction_id, name=name, connections=[])

        return junction

    @staticmethod
    def construct_from_lanelet_map(
        lanelet_map: lanelet2.core.LaneletMap,
        starting_junction_id: int = 0,
    ) -> List["Junction"]:
        """Construct a list of Junctions from a lanelet2 map.

        This method automatically extracts all junction lanelets from the map,
        groups them into separate junctions based on their spatial relationships,
        and creates Junction instances for each group.

        Args:
            lanelet_map: The lanelet2 map to extract junctions from
            starting_junction_id: The starting ID for junction numbering (default: 0)

        Returns:
            List of Junction instances, one for each detected junction group.
            The connections are not automatically created - they should be added
            separately using add_connection() based on the road network structure.

        Example:
            >>> from autoware_lanelet2_to_opendrive.opendrive.junction import Junction
            >>> junctions = Junction.construct_from_lanelet_map(lanelet_map)
            >>> print(f"Found {len(junctions)} junctions")
            >>> for junction in junctions:
            ...     print(f"Junction {junction.id}: {junction.name}")
        """
        # Import here to avoid circular dependency
        from ..junction import filter_lanelets_inside_junction, find_junction_groups

        # Get all lanelets from the map
        all_lanelets = list(lanelet_map.laneletLayer)

        # Filter lanelets that are inside junctions (have turn_direction attribute)
        junction_lanelets = filter_lanelets_inside_junction(all_lanelets)

        # Group junction lanelets into separate junction groups
        junction_groups = find_junction_groups(junction_lanelets)

        # Create Junction instances from each group
        junctions = []
        for i, group in enumerate(junction_groups):
            junction_id = starting_junction_id + i
            junction = Junction.construct_from_lanelet_groups(
                junction_id=junction_id, lanelet_group=group
            )
            junctions.append(junction)

        return junctions

    @staticmethod
    def build_connections_from_roads(
        lanelet_map: lanelet2.core.LaneletMap,
        junction_lanelet_group: List[lanelet2.core.Lanelet],
        junction_id: int,
        lanelet_to_road_id: dict[int, int],
        connecting_road_ids: List[int],
    ) -> List[Connection]:
        """Build junction connections from road topology.

        Analyzes the routing graph to find how incoming roads connect to
        connecting roads (roads inside the junction), and creates appropriate
        Connection and LaneLink elements.

        Args:
            lanelet_map: The lanelet2 map for routing graph analysis
            junction_lanelet_group: List of lanelets in this junction
            junction_id: ID of this junction
            lanelet_to_road_id: Mapping from lanelet ID to road ID for ALL lanelets
            connecting_road_ids: List of road IDs that are inside this junction

        Returns:
            List of Connection objects for this junction

        Note:
            This method uses the routing graph to find predecessor lanelets
            for each junction lanelet, determines which roads they belong to,
            and creates connections with appropriate lane links.
        """
        from lanelet2.routing import RoutingGraph, RoutingCostDistance
        import lanelet2

        # Create routing graph
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

        # Track connections: (incoming_road_id, connecting_road_id) -> Connection
        connection_map: dict[tuple[int, int], Connection] = {}
        connection_id_counter = 0

        junction_lanelet_ids = {ll.id for ll in junction_lanelet_group}

        # For each lanelet in this junction
        for junction_lanelet in junction_lanelet_group:
            # Get the connecting road ID for this junction lanelet
            if junction_lanelet.id not in lanelet_to_road_id:
                continue

            connecting_road_id = lanelet_to_road_id[junction_lanelet.id]

            # Find predecessor lanelets (incoming to junction)
            previous_lanelets = routing_graph.previous(junction_lanelet)

            for prev_lanelet in previous_lanelets:
                # Skip if predecessor is also in the junction
                if prev_lanelet.id in junction_lanelet_ids:
                    continue

                # Get the incoming road ID
                if prev_lanelet.id not in lanelet_to_road_id:
                    continue

                incoming_road_id = lanelet_to_road_id[prev_lanelet.id]

                # Create or get connection for this (incoming, connecting) pair
                connection_key = (incoming_road_id, connecting_road_id)

                if connection_key not in connection_map:
                    # Create new connection
                    # ContactPoint determination: we connect to the START of the connecting road
                    # This is a simplification - proper implementation would check geometry
                    connection = Connection(
                        id=connection_id_counter,
                        incoming_road=incoming_road_id,
                        connecting_road=connecting_road_id,
                        contact_point=ContactPoint.START,
                        lane_links=[],
                    )
                    connection_map[connection_key] = connection
                    connection_id_counter += 1

                # Add lane link
                # Note: This is a simplified mapping assuming lane IDs correspond
                # A proper implementation would need to map OpenDRIVE lane IDs
                # which may differ from lanelet IDs
                connection = connection_map[connection_key]

                # Get lane IDs - in OpenDRIVE, negative for right lanes
                # For simplicity, we use a basic mapping
                # TODO: Implement proper lane ID mapping from lanelet to OpenDRIVE lane
                from_lane_id = -1  # Simplified: assume single lane
                to_lane_id = -1  # Simplified: assume single lane

                # Check if this lane link already exists
                lane_link_exists = any(
                    ll.from_lane == from_lane_id and ll.to_lane == to_lane_id
                    for ll in connection.lane_links
                )

                if not lane_link_exists:
                    connection.add_lane_link(from_lane=from_lane_id, to_lane=to_lane_id)

        return list(connection_map.values())
