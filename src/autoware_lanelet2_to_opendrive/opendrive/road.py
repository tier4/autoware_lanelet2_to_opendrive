"""OpenDRIVE road definitions."""

from dataclasses import dataclass
from typing import Optional, Union, Set, List, cast
import lxml.etree as ET
import lanelet2
from tqdm import tqdm

from .geometry import PlanView, ParamPoly3, GeometryBase
from .elevation import ElevationProfile
from .lane_sections import Lanes
from .reference_line import ReferenceLine
from .enums import ContactPoint, ElementType
from ..centerline import AsymmetryLaneletException
from ..util import filter_lanelets_by_subtype

# Import for type hints only
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .signal import Signal


@dataclass
class Predecessor:
    """Predecessor link element."""

    element_type: ElementType
    element_id: int
    contact_point: Optional[ContactPoint] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("predecessor")
        elem.set("elementType", self.element_type.value)
        elem.set("elementId", str(self.element_id))
        if self.contact_point:
            elem.set("contactPoint", self.contact_point.value)
        return elem


@dataclass
class Successor:
    """Successor link element."""

    element_type: ElementType
    element_id: int
    contact_point: Optional[ContactPoint] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("successor")
        elem.set("elementType", self.element_type.value)
        elem.set("elementId", str(self.element_id))
        if self.contact_point:
            elem.set("contactPoint", self.contact_point.value)
        return elem


@dataclass
class RoadLink:
    """Road link element containing predecessor and successor."""

    predecessor: Optional[Predecessor] = None
    successor: Optional[Successor] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("link")
        if self.predecessor:
            elem.append(self.predecessor.to_xml())
        if self.successor:
            elem.append(self.successor.to_xml())
        return elem


@dataclass
class Road:
    """Road definition."""

    id: int = 0
    name: Optional[str] = None
    length: float = 0.0
    junction: int = -1
    plan_view: Optional[PlanView] = None
    elevation_profile: Optional[ElevationProfile] = None
    lanes: Optional[Lanes] = None
    link: Optional[RoadLink] = None
    signals: Optional[List["Signal"]] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("road")
        elem.set("id", str(self.id))
        elem.set("length", str(self.length))
        elem.set("junction", str(self.junction))

        if self.name:
            elem.set("name", self.name)

        if self.link:
            elem.append(self.link.to_xml())
        if self.plan_view:
            elem.append(self.plan_view.to_xml())
        if self.elevation_profile:
            elem.append(self.elevation_profile.to_xml())
        if self.lanes:
            elem.append(self.lanes.to_xml())
        if self.signals:
            signals_elem = ET.SubElement(elem, "signals")
            for signal in self.signals:
                signals_elem.append(signal.to_xml())

        return elem

    def add_predecessor(
        self,
        element_id: int,
        element_type: ElementType = ElementType.ROAD,
        contact_point: Optional[ContactPoint] = None,
    ) -> None:
        """Add a predecessor link to this road.

        Args:
            element_id: ID of the predecessor element
            element_type: Type of the predecessor element (road or junction)
            contact_point: Contact point of the predecessor (start or end)
        """
        if self.link is None:
            self.link = RoadLink()
        self.link.predecessor = Predecessor(
            element_type=element_type,
            element_id=element_id,
            contact_point=contact_point,
        )

    def add_successor(
        self,
        element_id: int,
        element_type: ElementType = ElementType.ROAD,
        contact_point: Optional[ContactPoint] = None,
    ) -> None:
        """Add a successor link to this road.

        Args:
            element_id: ID of the successor element
            element_type: Type of the successor element (road or junction)
            contact_point: Contact point of the successor (start or end)
        """
        if self.link is None:
            self.link = RoadLink()
        self.link.successor = Successor(
            element_type=element_type,
            element_id=element_id,
            contact_point=contact_point,
        )

    @staticmethod
    def construct_from_lanelet_groups(
        lanelet_map: lanelet2.core.LaneletMap,
        lanelet_group: Union[
            Set[lanelet2.core.Lanelet],
            List[lanelet2.core.Lanelet],
            lanelet2.core.LaneletLayer,
        ],
        road_id: int,
        s_offset: float = 0.0,
    ) -> "Road":
        """Construct a Road from a group of lanelets.

        Args:
            lanelet_map: The lanelet2 map containing the lanelets
            lanelet_group: Group of lanelets to convert to a road
            s_offset: Starting s-coordinate offset for the road

        Returns:
            Road object constructed from the lanelet group

        Raises:
            ValueError: If lanelet_group is empty or contains non-adjacent lanelets
        """
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        # Convert input to list for consistent processing
        if isinstance(lanelet_group, (set, lanelet2.core.LaneletLayer)):
            lanelet_list = list(lanelet_group)
        else:
            lanelet_list = lanelet_group

        centerline_spline = ReferenceLine.construct_from_lanelet_groups(
            lanelet_map, lanelet_list
        ).centerline_spline

        # Create paramPoly3 geometries from spline using from_spline method
        geometries: List[GeometryBase] = cast(
            List[GeometryBase], ParamPoly3.from_spline(centerline_spline)
        )

        # Create plan view with the paramPoly3 geometries
        plan_view = PlanView(geometries=geometries)

        # Calculate total road length from spline
        road_length = centerline_spline.total_length

        def get_lanes() -> Lanes:
            """Create Lanes object from lanelet group."""
            from .lane_section import LaneSection

            lane_section = LaneSection.construct_from_lanelet_groups(
                lanelet_map, lanelet_list, s_offset=s_offset
            )
            lanes = Lanes(lane_sections=[lane_section])
            return lanes

        # Create a basic road with the extracted information
        # Note: This is a simplified implementation
        # A complete implementation would also need to:
        # - Create proper lane sections from the lanelets
        # - Handle elevation profile
        # - Set appropriate road ID and other attributes
        road = Road(
            id=road_id,
            name=f"Road_{road_id}",
            length=road_length,
            junction=-1,  # Not in a junction by default
            plan_view=plan_view,
            elevation_profile=None,  # TODO: Extract elevation from lanelets
            lanes=get_lanes(),
        )

        return road

    @staticmethod
    def construct_from_lanelet_map(
        lanelet_map: lanelet2.core.LaneletMap,
    ) -> List["Road"]:
        """Construct Roads from a lanelet map.

        Args:
            lanelet_map: The lanelet2 map containing all lanelets

        Returns:
            List of Road objects constructed from non-junction lanelets grouped by adjacency

        Raises:
            ValueError: If lanelet_map is empty or contains no valid lanelets
        """
        if not lanelet_map or not lanelet_map.laneletLayer:
            raise ValueError("Lanelet map is empty or contains no lanelets")

        # Get all lanelets from the map
        all_lanelets = list(lanelet_map.laneletLayer)

        # Filter out lanelets inside junctions
        from ..junction import filter_lanelets_outside_junction

        road_lanelets = filter_lanelets_outside_junction(
            filter_lanelets_by_subtype(all_lanelets, ["road"])
        )

        if not road_lanelets:
            raise ValueError("No lanelets found outside junctions")

        # Find adjacent groups of lanelets
        from ..util import find_adjacent_groups

        # Pass the road_lanelets set to find_adjacent_groups so it only groups these
        adjacent_groups = find_adjacent_groups(lanelet_map, set(road_lanelets))

        # Create roads from adjacent groups
        roads = []
        # Store mapping from lanelet ID to road ID and adjacent group for each road
        lanelet_to_road: dict[int, int] = {}
        road_to_group: dict[int, Set[lanelet2.core.Lanelet]] = {}

        print(f"Creating roads from {len(adjacent_groups)} lanelet groups...")
        for road_id, adjacent_group in tqdm(
            enumerate(adjacent_groups),
            total=len(adjacent_groups),
            desc="Building roads",
        ):
            # ll_ids = [ll.id for ll in adjacent_group]
            # target_ids = [112, 114, 115, 116]
            # target_ids = [3013239]
            # target_ids = [3013249, 3001324]
            # target_ids = [3013259]
            # if set(ll_ids).isdisjoint(target_ids):
            #     continue
            try:
                road = Road.construct_from_lanelet_groups(
                    lanelet_map=lanelet_map,  # Use original map for proper connectivity
                    lanelet_group=adjacent_group,
                    road_id=road_id,
                    s_offset=0.0,
                )
                roads.append(road)

                # Store lanelet ID to road ID mapping
                for lanelet in adjacent_group:
                    lanelet_to_road[lanelet.id] = road_id

                # Store road ID to adjacent group mapping
                road_to_group[road_id] = adjacent_group

            except AsymmetryLaneletException as e:
                # Log asymmetric lanelet warning and skip this road
                tqdm.write(
                    f"Warning: Skipping road {road_id} due to asymmetric lanelet: {e}"
                )
                continue
            except Exception as e:
                # Log warning but continue with other groups
                if "has no len()" in str(e):
                    import traceback

                    full_trace = traceback.format_exc()
                    # Find the line that actually called len()
                    for line in full_trace.split("\n"):
                        if "len(" in line and "autoware_lanelet2_to_opendrive" in line:
                            tqdm.write(
                                f"len() error in group {road_id}: {line.strip()}"
                            )
                            break
                tqdm.write(f"Warning: Failed to create road from group {road_id}: {e}")
                continue

        if not roads:
            raise ValueError("No valid roads could be constructed from the lanelet map")

        # Build road links based on lanelet previous/following relationships
        from ..util import find_connecting_lanelet_groups, ConnectionDirection

        print(f"Building road links for {len(roads)} roads...")
        for road in tqdm(roads, desc="Building road links"):
            adjacent_group = road_to_group[road.id]

            # Find preceding lanelet groups
            try:
                preceding_groups = find_connecting_lanelet_groups(
                    lanelet_map, adjacent_group, ConnectionDirection.PREVIOUS
                )

                # Find which roads these preceding lanelets belong to
                for preceding_group in preceding_groups:
                    # Get the road ID for any lanelet in this group
                    for lanelet in preceding_group:
                        if lanelet.id in lanelet_to_road:
                            predecessor_road_id = lanelet_to_road[lanelet.id]
                            # Add predecessor link with contactPoint=END
                            # (the end of the predecessor road connects to the start of current road)
                            road.add_predecessor(
                                element_id=predecessor_road_id,
                                element_type=ElementType.ROAD,
                                contact_point=ContactPoint.END,
                            )
                            break  # Only need to find one lanelet from the group
            except Exception as e:
                tqdm.write(
                    f"Warning: Failed to find predecessors for road {road.id}: {e}"
                )

            # Find following lanelet groups
            try:
                following_groups = find_connecting_lanelet_groups(
                    lanelet_map, adjacent_group, ConnectionDirection.FOLLOWING
                )

                # Find which roads these following lanelets belong to
                for following_group in following_groups:
                    # Get the road ID for any lanelet in this group
                    for lanelet in following_group:
                        if lanelet.id in lanelet_to_road:
                            successor_road_id = lanelet_to_road[lanelet.id]
                            # Add successor link with contactPoint=START
                            # (the end of current road connects to the start of the successor road)
                            road.add_successor(
                                element_id=successor_road_id,
                                element_type=ElementType.ROAD,
                                contact_point=ContactPoint.START,
                            )
                            break  # Only need to find one lanelet from the group
            except Exception as e:
                tqdm.write(
                    f"Warning: Failed to find successors for road {road.id}: {e}"
                )

        return roads

    @staticmethod
    def construct_connecting_roads_from_junctions(
        lanelet_map: lanelet2.core.LaneletMap,
        junction_groups: List[List[lanelet2.core.Lanelet]],
        starting_road_id: int = 0,
    ) -> tuple[List["Road"], dict[int, List[int]], dict[int, int]]:
        """Construct connecting roads from junction lanelet groups.

        Creates roads from lanelets inside junctions. These roads have their
        junction field set to the appropriate junction ID.

        Args:
            lanelet_map: The lanelet2 map containing all lanelets
            junction_groups: List of junction lanelet groups from find_junction_groups()
            starting_road_id: Starting ID for road numbering (default: 0)

        Returns:
            Tuple of:
            - List of Road objects (connecting roads with junction field set)
            - Dict mapping junction_id -> list of road IDs in that junction
            - Dict mapping lanelet_id -> road_id for all junction lanelets

        Example:
            >>> from autoware_lanelet2_to_opendrive.junction import find_junction_groups, filter_lanelets_inside_junction
            >>> junction_lanelets = filter_lanelets_inside_junction(lanelet_map.laneletLayer)
            >>> junction_groups = find_junction_groups(junction_lanelets)
            >>> roads, junction_to_roads, lanelet_to_road = Road.construct_connecting_roads_from_junctions(
            ...     lanelet_map, junction_groups, starting_road_id=1000
            ... )
        """
        from ..util import find_adjacent_groups

        connecting_roads = []
        junction_to_roads: dict[int, List[int]] = {}
        lanelet_to_road: dict[int, int] = {}

        current_road_id = starting_road_id

        print(
            f"Creating connecting roads from {len(junction_groups)} junction groups..."
        )

        for junction_id, junction_group in tqdm(
            enumerate(junction_groups),
            total=len(junction_groups),
            desc="Building junction roads",
        ):
            # Find adjacent groups within this junction
            adjacent_groups_in_junction = find_adjacent_groups(
                lanelet_map, set(junction_group)
            )

            junction_road_ids = []

            # Create one road per adjacent group within the junction
            for adjacent_group in adjacent_groups_in_junction:
                try:
                    road = Road.construct_from_lanelet_groups(
                        lanelet_map=lanelet_map,
                        lanelet_group=adjacent_group,
                        road_id=current_road_id,
                        s_offset=0.0,
                    )

                    # Set the junction field to mark this as a connecting road
                    road.junction = junction_id

                    connecting_roads.append(road)
                    junction_road_ids.append(current_road_id)

                    # Store lanelet ID to road ID mapping
                    for lanelet in adjacent_group:
                        lanelet_to_road[lanelet.id] = current_road_id

                    current_road_id += 1

                except AsymmetryLaneletException as e:
                    tqdm.write(
                        f"Warning: Skipping connecting road in junction {junction_id} "
                        f"due to asymmetric lanelet: {e}"
                    )
                    continue
                except Exception as e:
                    tqdm.write(
                        f"Warning: Failed to create connecting road in junction {junction_id}: {e}"
                    )
                    continue

            # Store the mapping from junction ID to its road IDs
            junction_to_roads[junction_id] = junction_road_ids

        print(
            f"Created {len(connecting_roads)} connecting roads across {len(junction_groups)} junctions"
        )

        return connecting_roads, junction_to_roads, lanelet_to_road
