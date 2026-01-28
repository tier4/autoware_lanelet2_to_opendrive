"""OpenDRIVE road definitions."""

from dataclasses import dataclass
from typing import Optional, Set, List, cast, Dict
import lxml.etree as ET
import lanelet2
from lanelet2.routing import RoutingGraph, RoutingCostDistance
from tqdm import tqdm

from .geometry import PlanView, ParamPoly3, GeometryBase
from .elevation import ElevationProfile
from .lane_sections import Lanes
from .reference_line import ReferenceLine
from .enums import ContactPoint, ElementType, TrafficRule
from .lane_elements import LaneLink
from .road_links import Predecessor, Successor, RoadLink
from ..centerline import AsymmetryLaneletException
from ..util import filter_lanelets_by_subtype, to_lanelet_list, LaneletInput

# Import for type hints only
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .signal import Signal
    from .lane import Lane
    from .junction import Junction


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
    elevation_offset: float = 0.0  # Absolute elevation at road start (s=0)
    rule: Optional[TrafficRule] = None  # Traffic direction rule (RHT/LHT)

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("road")
        elem.set("id", str(self.id))
        elem.set("length", str(self.length))
        elem.set("junction", str(self.junction))

        if self.name:
            elem.set("name", self.name)

        if self.rule is not None:
            elem.set("rule", self.rule.value)

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

            # Generate signal elements
            for signal in self.signals:
                signals_elem.append(signal.to_xml())

            # Generate signalReference elements (Issue #135)
            # Each signal gets a corresponding signalReference on the reference line
            for signal in self.signals:
                signals_elem.append(signal.to_signal_reference_xml())

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

    def get_lanelet_to_lane_mapping(self) -> Dict[int, int]:
        """Get mapping from lanelet ID to lane ID for all lanes in this road.

        Returns:
            Dictionary mapping lanelet_id -> lane_id
        """
        mapping: Dict[int, int] = {}

        if self.lanes is None:
            return mapping

        for lane_section in self.lanes.lane_sections:
            section_mapping = lane_section.get_lanelet_to_lane_mapping()
            mapping.update(section_mapping)

        return mapping

    def set_lane_links(
        self,
        lanelet_map: lanelet2.core.LaneletMap,
        lanelet_to_road_and_lane: Dict[int, tuple[int, int]],
        routing_graph: Optional[RoutingGraph] = None,
        road_lane_ids: Optional[Dict[int, Set[int]]] = None,
        road_id_to_road: Optional[Dict[int, "Road"]] = None,
    ) -> None:
        """Set lane predecessor and successor links based on lanelet connections.

        Args:
            lanelet_map: The Lanelet2 map containing connectivity information
            lanelet_to_road_and_lane: Global mapping from lanelet_id to (road_id, lane_id)
            routing_graph: Optional pre-built routing graph. If None, creates a new one.
            road_lane_ids: Optional mapping from road_id to set of existing lane_ids.
                          Used to validate that lane links reference existing lanes.
            road_id_to_road: Optional mapping from road_id to Road objects.
                            Used to check if target roads are connecting roads in junctions.
        """
        if self.lanes is None:
            return

        # Use provided routing graph or create a new one
        if routing_graph is None:
            traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle,
            )
            routing_graph = RoutingGraph(
                lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
            )

        for lane_section in self.lanes.lane_sections:
            # Process left lanes
            for lane in lane_section.left_lanes.values():
                self._set_single_lane_links(
                    lane,
                    lanelet_map,
                    routing_graph,
                    lanelet_to_road_and_lane,
                    road_lane_ids,
                    road_id_to_road,
                )

            # Process right lanes
            for lane in lane_section.right_lanes.values():
                self._set_single_lane_links(
                    lane,
                    lanelet_map,
                    routing_graph,
                    lanelet_to_road_and_lane,
                    road_lane_ids,
                    road_id_to_road,
                )

    @staticmethod
    def _find_closest_lane(target_lane: int, available_lanes: list[int]) -> int:
        """Find the closest lane ID when exact match doesn't exist.

        This handles cases where a lane references a non-existent lane in the
        target road, typically when the number of lanes changes (e.g., 3 lanes
        merging into 2 lanes).

        Args:
            target_lane: The desired lane ID that doesn't exist
            available_lanes: Sorted list of existing lane IDs in the target road

        Returns:
            Closest existing lane ID

        Example:
            >>> _find_closest_lane(-3, [-1, -2])  # Lane -3 merges into -2
            -2
        """
        if not available_lanes:
            return -1  # Fallback to default lane

        # Find the lane with minimum distance to target_lane
        # For right lanes (negative IDs): -3 is closer to -2 than -1
        # For left lanes (positive IDs): 3 is closer to 2 than 1
        closest = min(available_lanes, key=lambda x: abs(x - target_lane))
        return closest

    def _set_single_lane_links(
        self,
        lane: "Lane",
        lanelet_map: lanelet2.core.LaneletMap,
        routing_graph: RoutingGraph,
        lanelet_to_road_and_lane: Dict[int, tuple[int, int]],
        road_lane_ids: Optional[Dict[int, Set[int]]] = None,
        road_id_to_road: Optional[Dict[int, "Road"]] = None,
    ) -> None:
        """Set predecessor and successor for a single lane.

        Args:
            lane: The lane to set links for
            lanelet_map: The Lanelet2 map
            routing_graph: Routing graph for connectivity analysis
            lanelet_to_road_and_lane: Global mapping from lanelet_id to (road_id, lane_id)
            road_lane_ids: Optional mapping from road_id to set of existing lane_ids.
                          Used to validate that lane links reference existing lanes.
            road_id_to_road: Optional mapping from road_id to Road objects.
                            Used to check if target roads are connecting roads in junctions.
        """

        if lane.lanelet_id is None:
            return

        # Get the lanelet corresponding to this lane
        try:
            lanelet = lanelet_map.laneletLayer.get(lane.lanelet_id)
        except Exception:
            return

        # Get road link predecessor/successor for consistency check
        road_link_predecessor = self.link.predecessor if self.link else None
        road_link_successor = self.link.successor if self.link else None

        # Issue #124 Part 1 fix: For connecting roads (junction >= 0), allow lane
        # links even without road links
        is_connecting_road = self.junction is not None and self.junction >= 0

        # Find predecessor lanelets
        previous_lanelets = routing_graph.previous(lanelet)
        if previous_lanelets:
            # Take the first predecessor that maps to the road link's predecessor road
            for prev_ll in previous_lanelets:
                if prev_ll.id in lanelet_to_road_and_lane:
                    pred_road_id, pred_lane_id = lanelet_to_road_and_lane[prev_ll.id]
                    # Only set predecessor if it's in a different road
                    # (same road connections would be within lane sections)
                    if pred_road_id != self.id:
                        # Check consistency with road link
                        # For connecting roads, skip road link validation
                        if road_link_predecessor is None:
                            if not is_connecting_road:
                                # Road link is None - check if predecessor is a connecting road
                                # This allows lane branching scenarios (e.g., lane -2 comes from junction)
                                if road_id_to_road is not None:
                                    pred_road = road_id_to_road.get(pred_road_id)
                                    # If predecessor is a connecting road (junction member), allow lane link
                                    if (
                                        pred_road is None
                                        or pred_road.junction is None
                                        or pred_road.junction < 0
                                    ):
                                        # Predecessor is a regular road but no road link - skip
                                        continue
                                    # Predecessor is a connecting road - proceed with lane link creation
                                else:
                                    # Cannot validate - skip for safety
                                    continue
                            # For connecting roads, proceed with lane link creation

                        # Check if road link predecessor is a junction
                        if road_link_predecessor is not None:
                            if (
                                road_link_predecessor.element_type
                                == ElementType.JUNCTION
                            ):
                                # This road's predecessor is a junction
                                # The lane's predecessor should be a connecting road in that
                                # junction
                                if road_id_to_road is not None:
                                    pred_road = road_id_to_road.get(pred_road_id)
                                    if pred_road is None:
                                        if not is_connecting_road:
                                            continue
                                    # Check if predecessor road is a connecting road in
                                    # the junction
                                    elif (
                                        pred_road.junction
                                        != road_link_predecessor.element_id
                                    ):
                                        if not is_connecting_road:
                                            continue
                                # If no road_id_to_road, we can't validate - skip for safety
                                # (unless this is a connecting road)
                                elif not is_connecting_road:
                                    continue
                            else:
                                # Road link predecessor is a regular road
                                # Lane link must reference the same road
                                # (unless this is a connecting road)
                                if (
                                    pred_road_id != road_link_predecessor.element_id
                                    and not is_connecting_road
                                ):
                                    # Lane predecessor differs from road link - check if it's a branching scenario
                                    # Allow if predecessor is a connecting road (junction branching)
                                    if road_id_to_road is not None:
                                        pred_road = road_id_to_road.get(pred_road_id)
                                        if (
                                            pred_road is not None
                                            and pred_road.junction is not None
                                            and pred_road.junction >= 0
                                        ):
                                            # Predecessor is a connecting road - allow lane branching
                                            pass
                                        else:
                                            # Predecessor is a regular road but doesn't match road link - skip
                                            continue
                                    else:
                                        # Cannot validate - skip for safety
                                        continue

                        # Validate that the lane exists in the predecessor road
                        if road_lane_ids is not None:
                            existing_lanes = road_lane_ids.get(pred_road_id, set())
                            if pred_lane_id not in existing_lanes:
                                # Lane doesn't exist, find closest lane
                                # This handles lane reduction scenarios (e.g., 3 lanes -> 2 lanes)
                                if existing_lanes:
                                    available_lanes = sorted(existing_lanes)
                                    pred_lane_id = Road._find_closest_lane(
                                        pred_lane_id, available_lanes
                                    )
                                else:
                                    # No lanes available, skip this link
                                    continue

                        # Check for self-reference (lane linking to itself)
                        if pred_lane_id == lane.lane_id and pred_road_id == self.id:
                            import warnings

                            warnings.warn(
                                f"Skipping self-referencing predecessor: "
                                f"Road {self.id}, Lane {lane.lane_id} → "
                                f"Road {pred_road_id}, Lane {pred_lane_id}",
                                UserWarning,
                            )
                            continue

                        lane.predecessor = LaneLink(id=pred_lane_id)
                        break

        # Find successor lanelets
        following_lanelets = routing_graph.following(lanelet)
        if following_lanelets:
            # Take the first successor that maps to the road link's successor road
            for next_ll in following_lanelets:
                if next_ll.id in lanelet_to_road_and_lane:
                    succ_road_id, succ_lane_id = lanelet_to_road_and_lane[next_ll.id]
                    # Only set successor if it's in a different road
                    if succ_road_id != self.id:
                        # Check consistency with road link
                        # For connecting roads, skip road link validation
                        if road_link_successor is None:
                            if not is_connecting_road:
                                # Road link is None - check if successor is a connecting road
                                # This allows lane branching scenarios (e.g., lane -2 turns into junction)
                                if road_id_to_road is not None:
                                    succ_road = road_id_to_road.get(succ_road_id)
                                    # If successor is a connecting road (junction member), allow lane link
                                    if (
                                        succ_road is None
                                        or succ_road.junction is None
                                        or succ_road.junction < 0
                                    ):
                                        # Successor is a regular road but no road link - skip
                                        continue
                                    # Successor is a connecting road - proceed with lane link creation
                                else:
                                    # Cannot validate - skip for safety
                                    continue
                            # For connecting roads, proceed with lane link creation

                        # Check if road link successor is a junction
                        if road_link_successor is not None:
                            if road_link_successor.element_type == ElementType.JUNCTION:
                                # This road's successor is a junction
                                # The lane's successor should be a connecting road in that
                                # junction
                                if road_id_to_road is not None:
                                    succ_road = road_id_to_road.get(succ_road_id)
                                    if succ_road is None:
                                        if not is_connecting_road:
                                            continue
                                    # Check if successor road is a connecting road in
                                    # the junction
                                    elif (
                                        succ_road.junction
                                        != road_link_successor.element_id
                                    ):
                                        if not is_connecting_road:
                                            continue
                                # If no road_id_to_road, we can't validate - skip for safety
                                # (unless this is a connecting road)
                                elif not is_connecting_road:
                                    continue
                            else:
                                # Road link successor is a regular road
                                # Lane link must reference the same road
                                # (unless this is a connecting road)
                                if (
                                    succ_road_id != road_link_successor.element_id
                                    and not is_connecting_road
                                ):
                                    # Lane successor differs from road link - check if it's a branching scenario
                                    # Allow if successor is a connecting road (junction branching)
                                    if road_id_to_road is not None:
                                        succ_road = road_id_to_road.get(succ_road_id)
                                        if (
                                            succ_road is not None
                                            and succ_road.junction is not None
                                            and succ_road.junction >= 0
                                        ):
                                            # Successor is a connecting road - allow lane branching
                                            pass
                                        else:
                                            # Successor is a regular road but doesn't match road link - skip
                                            continue
                                    else:
                                        # Cannot validate - skip for safety
                                        continue

                        # Validate that the lane exists in the successor road
                        if road_lane_ids is not None:
                            existing_lanes = road_lane_ids.get(succ_road_id, set())
                            if succ_lane_id not in existing_lanes:
                                # Lane doesn't exist, find closest lane
                                # This handles lane reduction scenarios (e.g., 3 lanes -> 2 lanes)
                                if existing_lanes:
                                    available_lanes = sorted(existing_lanes)
                                    succ_lane_id = Road._find_closest_lane(
                                        succ_lane_id, available_lanes
                                    )
                                else:
                                    # No lanes available, skip this link
                                    continue

                        # Check for self-reference (lane linking to itself)
                        if succ_lane_id == lane.lane_id and succ_road_id == self.id:
                            import warnings

                            warnings.warn(
                                f"Skipping self-referencing successor: "
                                f"Road {self.id}, Lane {lane.lane_id} → "
                                f"Road {succ_road_id}, Lane {succ_lane_id}",
                                UserWarning,
                            )
                            continue

                        lane.successor = LaneLink(id=succ_lane_id)
                        break

    @staticmethod
    def construct_from_lanelet_groups(
        lanelet_map: lanelet2.core.LaneletMap,
        lanelet_group: LaneletInput,
        road_id: int,
        s_offset: float = 0.0,
        traffic_rule: Optional[TrafficRule] = None,
        use_spec_compliant_lane_positioning: bool = True,
    ) -> "Road":
        """Construct a Road from a group of lanelets.

        Args:
            lanelet_map: The lanelet2 map containing the lanelets
            lanelet_group: Group of lanelets to convert to a road
            road_id: Road ID to assign
            s_offset: Starting s-coordinate offset for the road
            traffic_rule: Optional traffic rule (RHT or LHT) to apply to road geometry
            use_spec_compliant_lane_positioning: If True, use LEFT lanes for LHT and RIGHT
                lanes for RHT (spec-compliant). If False, use RIGHT lanes for all traffic
                rules (CARLA compatibility mode)

        Returns:
            Road object constructed from the lanelet group

        Raises:
            ValueError: If lanelet_group is empty or contains non-adjacent lanelets
        """
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        # Convert input to list for consistent processing
        lanelet_list = to_lanelet_list(lanelet_group)

        reference_line = ReferenceLine.construct_from_lanelet_groups(
            lanelet_map, lanelet_list, traffic_rule=traffic_rule
        )
        centerline_2d = reference_line.centerline_2d

        # Create paramPoly3 geometries from 2D spline using from_spline method
        # ParamPoly3 only uses XY coordinates, so 2D spline is appropriate
        geometries: List[GeometryBase] = cast(
            List[GeometryBase], ParamPoly3.from_spline(centerline_2d)
        )

        # Create plan view with the paramPoly3 geometries
        plan_view = PlanView(geometries=geometries)

        # Calculate total road length from ParamPoly3 geometries (XY projection)
        # IMPORTANT: Use XY-plane length from geometries, not 3D spline length
        # ParamPoly3.from_spline() uses XY coordinates only, ignoring Z
        road_length = sum(geometry.length for geometry in geometries)

        def get_lanes() -> Lanes:
            """Create Lanes object from lanelet group."""
            from .lane_section import LaneSection

            lane_section = LaneSection.construct_from_lanelet_groups(
                lanelet_map,
                lanelet_list,
                s_offset=s_offset,
                traffic_rule=traffic_rule,
                use_spec_compliant_lane_positioning=use_spec_compliant_lane_positioning,
            )
            lanes = Lanes(lane_sections=[lane_section])
            return lanes

        # Extract geometry segment boundaries (s-coordinates)
        # This ensures elevation profile segments align with ParamPoly3 segments
        geometry_s_values = [g.s for g in geometries]

        # Get elevation profile from reference line, aligned with geometry boundaries
        elevation_profile = reference_line.get_elevation_profile(geometry_s_values)

        # Create a basic road with the extracted information
        # Note: This is a simplified implementation
        # A complete implementation would also need to:
        # - Create proper lane sections from the lanelets
        # - Set appropriate road ID and other attributes
        road = Road(
            id=road_id,
            name=f"Road_{road_id}",
            length=road_length,
            junction=-1,  # Not in a junction by default
            plan_view=plan_view,
            elevation_profile=elevation_profile,
            lanes=get_lanes(),
            elevation_offset=reference_line.elevation_offset,
            rule=traffic_rule,  # Apply traffic rule to road
        )

        return road

    @staticmethod
    def construct_from_lanelet_map(
        lanelet_map: lanelet2.core.LaneletMap,
        traffic_rule: Optional[TrafficRule] = None,
        use_spec_compliant_lane_positioning: bool = True,
    ) -> List["Road"]:
        """Construct Roads from a lanelet map.

        Args:
            lanelet_map: The lanelet2 map containing all lanelets
            traffic_rule: Optional traffic rule (RHT or LHT) to apply to all roads
            use_spec_compliant_lane_positioning: If True, use LEFT lanes for LHT and RIGHT
                lanes for RHT (spec-compliant). If False, use RIGHT lanes for all traffic
                rules (CARLA compatibility mode)

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

        # Create routing graph once and reuse for all operations
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

        # Find adjacent groups of lanelets
        from ..util import find_adjacent_groups

        # Pass the road_lanelets set to find_adjacent_groups so it only groups these
        adjacent_groups = find_adjacent_groups(
            lanelet_map, set(road_lanelets), routing_graph
        )

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
                    traffic_rule=traffic_rule,
                    use_spec_compliant_lane_positioning=use_spec_compliant_lane_positioning,
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
                    lanelet_map,
                    adjacent_group,
                    ConnectionDirection.PREVIOUS,
                    routing_graph,
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
                    lanelet_map,
                    adjacent_group,
                    ConnectionDirection.FOLLOWING,
                    routing_graph,
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

        # Build lane links based on lanelet previous/following relationships
        # Note: This only sets links between regular roads.
        # For complete lane links including junction roads, use set_all_lane_links()
        # after combining regular roads and connecting roads from junctions.
        Road.set_all_lane_links(lanelet_map, roads, routing_graph)

        return roads

    @staticmethod
    def construct_connecting_roads_from_junctions(
        lanelet_map: lanelet2.core.LaneletMap,
        junction_groups: List[List[lanelet2.core.Lanelet]],
        starting_road_id: int = 0,
        junction_id_offset: int = 0,
        traffic_rule: Optional[TrafficRule] = None,
        use_spec_compliant_lane_positioning: bool = True,
    ) -> tuple[List["Road"], dict[int, List[int]], dict[int, int]]:
        """Construct connecting roads from junction lanelet groups.

        Creates roads from lanelets inside junctions. These roads have their
        junction field set to the appropriate junction ID.

        Args:
            lanelet_map: The lanelet2 map containing all lanelets
            junction_groups: List of junction lanelet groups from find_junction_groups()
            starting_road_id: Starting ID for road numbering (default: 0)
            junction_id_offset: Offset to add to junction IDs to avoid conflicts
                               with road IDs (default: 0). Issue #132 fix.
            traffic_rule: Optional traffic rule (RHT or LHT) to apply to all connecting roads
            use_spec_compliant_lane_positioning: If True, use LEFT lanes for LHT and RIGHT
                lanes for RHT (spec-compliant). If False, use RIGHT lanes for all traffic
                rules (CARLA compatibility mode)

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
            ...     lanelet_map, junction_groups, starting_road_id=1000, junction_id_offset=1000
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

        for junction_index, junction_group in tqdm(
            enumerate(junction_groups),
            total=len(junction_groups),
            desc="Building junction roads",
        ):
            # Issue #132 fix: Apply offset to junction ID
            junction_id = junction_index + junction_id_offset
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
                        traffic_rule=traffic_rule,
                        use_spec_compliant_lane_positioning=use_spec_compliant_lane_positioning,
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

    @staticmethod
    def set_all_lane_links(
        lanelet_map: lanelet2.core.LaneletMap,
        roads: List["Road"],
        routing_graph: Optional[RoutingGraph] = None,
    ) -> None:
        """Set lane links for all roads based on lanelet connections.

        This method builds a global mapping from lanelet IDs to (road_id, lane_id)
        and sets predecessor/successor links for all lanes in all roads.

        Args:
            lanelet_map: The Lanelet2 map containing connectivity information
            roads: List of all roads (both regular and connecting roads from junctions)
            routing_graph: Optional pre-built routing graph. If None, creates a new one.

        Example:
            >>> # After creating all roads
            >>> all_roads = regular_roads + connecting_roads
            >>> Road.set_all_lane_links(lanelet_map, all_roads)
        """
        # Build global mapping from lanelet_id to (road_id, lane_id)
        lanelet_to_road_and_lane: Dict[int, tuple[int, int]] = {}
        for road in roads:
            lane_mapping = road.get_lanelet_to_lane_mapping()
            for lanelet_id, lane_id in lane_mapping.items():
                lanelet_to_road_and_lane[lanelet_id] = (road.id, lane_id)

        # Build mapping from road_id to set of existing lane_ids
        # This is used to validate that lane links reference existing lanes
        road_lane_ids: Dict[int, Set[int]] = {}
        for road in roads:
            lane_ids: Set[int] = set()
            if road.lanes:
                for lane_section in road.lanes.lane_sections:
                    lane_ids.update(lane_section.left_lanes.keys())
                    lane_ids.update(lane_section.right_lanes.keys())
            road_lane_ids[road.id] = lane_ids

        # Build mapping from road_id to Road object
        # This is used to check if target roads are connecting roads in junctions
        road_id_to_road: Dict[int, "Road"] = {road.id: road for road in roads}

        # Use provided routing graph or create a new one
        if routing_graph is None:
            traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle,
            )
            routing_graph = RoutingGraph(
                lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
            )

        # Set lane links for each road
        print(f"Building lane links for {len(roads)} roads...")
        for road in tqdm(roads, desc="Building lane links"):
            try:
                road.set_lane_links(
                    lanelet_map,
                    lanelet_to_road_and_lane,
                    routing_graph,
                    road_lane_ids,
                    road_id_to_road,
                )
            except Exception as e:
                tqdm.write(f"Warning: Failed to set lane links for road {road.id}: {e}")

    @staticmethod
    def set_connecting_road_links(
        lanelet_map: lanelet2.core.LaneletMap,
        connecting_roads: List["Road"],
        lanelet_to_road_id: Dict[int, int],
        road_to_lanelet_ids: Dict[int, List[int]],
    ) -> None:
        """Set predecessor/successor links for connecting roads inside junctions.

        For each connecting road, finds the incoming road (predecessor) and
        outgoing road (successor) by analyzing the routing graph connections
        of the lanelets that make up the road.

        Args:
            lanelet_map: The Lanelet2 map containing connectivity information
            connecting_roads: List of roads inside junctions (junction >= 0)
            lanelet_to_road_id: Mapping from lanelet ID to road ID for ALL lanelets
            road_to_lanelet_ids: Mapping from road ID to list of lanelet IDs
        """

        # Create routing graph
        import lanelet2 as ll2
        from lanelet2.routing import RoutingGraph, RoutingCostDistance

        traffic_rules = ll2.traffic_rules.create(
            ll2.traffic_rules.Locations.Germany,
            ll2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

        # Get junction lanelet IDs (all lanelets belonging to connecting roads)
        junction_lanelet_ids: set[int] = set()
        for road in connecting_roads:
            if road.id in road_to_lanelet_ids:
                junction_lanelet_ids.update(road_to_lanelet_ids[road.id])

        print(f"Setting road links for {len(connecting_roads)} connecting roads...")
        for road in tqdm(connecting_roads, desc="Building connecting road links"):
            if road.id not in road_to_lanelet_ids:
                continue

            road_lanelet_ids = road_to_lanelet_ids[road.id]
            if not road_lanelet_ids:
                continue

            # Get lanelet objects for this road
            road_lanelets = [
                lanelet_map.laneletLayer.get(lid)
                for lid in road_lanelet_ids
                if lid in lanelet_map.laneletLayer
            ]

            if not road_lanelets:
                continue

            # Find predecessor: look at all lanelets' previous connections
            # that are OUTSIDE the junction
            predecessor_road_id = None
            for lanelet in road_lanelets:
                previous_lanelets = routing_graph.previous(lanelet)
                for prev_ll in previous_lanelets:
                    # Skip if predecessor is also in a junction
                    if prev_ll.id in junction_lanelet_ids:
                        continue
                    # Find the road ID for this predecessor
                    if prev_ll.id in lanelet_to_road_id:
                        predecessor_road_id = lanelet_to_road_id[prev_ll.id]
                        break
                if predecessor_road_id is not None:
                    break

            if predecessor_road_id is not None:
                road.add_predecessor(
                    element_id=predecessor_road_id,
                    element_type=ElementType.ROAD,
                    contact_point=ContactPoint.END,
                )

            # Find successor: look at all lanelets' following connections
            # that are OUTSIDE the junction
            successor_road_id = None
            for lanelet in road_lanelets:
                following_lanelets = routing_graph.following(lanelet)
                for next_ll in following_lanelets:
                    # Skip if successor is also in a junction
                    if next_ll.id in junction_lanelet_ids:
                        continue
                    # Find the road ID for this successor
                    if next_ll.id in lanelet_to_road_id:
                        successor_road_id = lanelet_to_road_id[next_ll.id]
                        break
                if successor_road_id is not None:
                    break

            if successor_road_id is not None:
                road.add_successor(
                    element_id=successor_road_id,
                    element_type=ElementType.ROAD,
                    contact_point=ContactPoint.START,
                )

    @staticmethod
    def set_incoming_road_junction_links(
        roads: List["Road"],
        junctions: List["Junction"],
    ) -> None:
        """Set junction links for incoming roads.

        For roads that connect to junctions (as incoming roads), this method
        sets the appropriate successor/predecessor link to the junction.

        This is required for CARLA compatibility and ensures correct routing
        through junctions. Issue #132 fix: Use connection.contactPoint directly
        instead of checking connecting road links, which may only reference
        one of multiple incoming roads.

        Args:
            roads: List of all roads (both regular and connecting roads)
            junctions: List of all junctions with their connections
        """
        # Build a map of road_id to road for quick lookup
        road_map: Dict[int, "Road"] = {road.id: road for road in roads}

        # For each junction, find incoming roads and set their junction links
        for junction in junctions:
            for connection in junction.connections:
                incoming_road_id = connection.incoming_road

                if incoming_road_id not in road_map:
                    continue

                incoming_road = road_map[incoming_road_id]

                # Issue #132 fix: Use contactPoint to determine which side connects
                # contactPoint indicates which end of the connecting road is used
                # - START: connecting road starts at this junction
                #   → incoming road ends at junction → set successor
                # - END: connecting road ends at this junction
                #   → incoming road starts at junction → set predecessor
                if connection.contact_point == ContactPoint.START:
                    # The end of incoming road connects to junction
                    # Set successor to junction (if not already set)
                    if (
                        incoming_road.link is None
                        or incoming_road.link.successor is None
                    ):
                        incoming_road.add_successor(
                            element_id=junction.id,
                            element_type=ElementType.JUNCTION,
                            contact_point=None,  # Junction links don't have contact point
                        )
                elif connection.contact_point == ContactPoint.END:
                    # The start of incoming road connects to junction
                    # Set predecessor to junction (if not already set)
                    if (
                        incoming_road.link is None
                        or incoming_road.link.predecessor is None
                    ):
                        incoming_road.add_predecessor(
                            element_id=junction.id,
                            element_type=ElementType.JUNCTION,
                            contact_point=None,  # Junction links don't have contact point
                        )
