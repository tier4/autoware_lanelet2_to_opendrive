"""LaneSection implementation for OpenDRIVE conversion."""

from typing import List, Optional, Dict, Tuple, Union, Set, TYPE_CHECKING
import lanelet2
import lxml.etree as ET
from lanelet2.routing import RoutingGraph

if TYPE_CHECKING:
    from .lane import Lane
from .reference_line import ReferenceLine
from ..conversion_config import WidthEstimationConfig
from .enums import RoadMarkType, RoadMarkLaneChange
from .lane_elements import road_mark_from_linestring_attrs
from .xml_utils import replace_subnormal


class LaneSection:
    """
    OpenDRIVE LaneSection representation.

    A LaneSection contains lanes for a specific longitudinal section of a road.
    It includes left lanes, right lanes, and a center (reference) lane.
    """

    def __init__(
        self,
        s_offset: float = 0.0,
    ):
        """
        Initialize a LaneSection object.

        Args:
            s_offset: Start position of the lane section along the reference line
        """
        self.s_offset = s_offset

        # Lanes are stored by ID: negative for right, positive for left, 0 for center
        self.left_lanes: Dict[int, "Lane"] = {}  # Positive IDs
        # The centre slot accepts either a ``ReferenceLine`` (the lanelet-driven
        # path wraps a ``Lane`` inside it) or a bare ``Lane`` (the synthetic
        # parking-lot road path needs no fitted spline).  Both expose
        # ``lane_id`` and ``to_xml``; ``get_all_lanes`` unwraps the
        # ``ReferenceLine`` form via ``_lane`` when present.
        self.center_lane: Optional[Union[ReferenceLine, "Lane"]] = None  # ID = 0
        self.right_lanes: Dict[int, "Lane"] = {}  # Negative IDs

        # Lane offset for single lane sections
        self.lane_offset: Optional[Dict[str, float]] = None

    def _add_left_lane(self, lane: "Lane") -> None:
        """Add a left lane to the section."""
        if lane.lane_id is None:
            raise ValueError("Lane added to LaneSection must have a resolved lane_id")
        if lane.lane_id <= 0:
            raise ValueError(f"Left lane must have positive ID, got {lane.lane_id}")
        self.left_lanes[lane.lane_id] = lane

    def _add_right_lane(self, lane: "Lane") -> None:
        """Add a right lane to the section."""
        if lane.lane_id is None:
            raise ValueError("Lane added to LaneSection must have a resolved lane_id")
        if lane.lane_id >= 0:
            raise ValueError(f"Right lane must have negative ID, got {lane.lane_id}")
        self.right_lanes[lane.lane_id] = lane

    def _set_center_lane(self, reference_line: ReferenceLine) -> None:
        """Set the center/reference lane."""
        if reference_line.lane_id is None or reference_line.lane_id != 0:
            raise ValueError(
                f"Center lane must have ID 0, got {reference_line.lane_id}"
            )
        self.center_lane = reference_line

    @staticmethod
    def construct_from_lanelet_groups(
        lanelet_map: lanelet2.core.LaneletMap,
        lanelet_group: Union[
            Set[lanelet2.core.Lanelet],
            List[lanelet2.core.Lanelet],
            lanelet2.core.LaneletLayer,
        ],
        s_offset: float = 0.0,
        traffic_rule: Optional[str] = None,
        width_config: Optional[WidthEstimationConfig] = None,
        routing_graph: Optional[RoutingGraph] = None,
        start_xyz_override: Optional[Tuple[float, float, float]] = None,
        end_xyz_override: Optional[Tuple[float, float, float]] = None,
    ) -> "LaneSection":
        """
        Construct a LaneSection from a group of Lanelet2 lanelets.

        Args:
            lanelet_map: The Lanelet2 map containing the lanelets
            lanelet_group: Group of lanelets representing lanes in a road section
            s_offset: Start position of the lane section
            traffic_rule: Traffic rule for lanes (RHT or LHT)
            width_config: Configuration for width spline sampling
            routing_graph: Optional pre-built routing graph for lane-change detection.
                If None, creates a new one.
            start_xyz_override: Optional world-frame (x, y, z) coordinate that
                pins the reference line start.  Mirrors the override on
                ``Road.construct_from_lanelet_groups`` and is also forwarded
                to the OUTERMOST lane's width calculation (P0-2 junction
                endpoint fidelity, lane-width side).
            end_xyz_override: Optional analogous override for the s=length end.

        Returns:
            LaneSection instance constructed from the lanelet group
        """
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        # Create the LaneSection instance
        lane_section = LaneSection(s_offset=s_offset)

        # Import Lane here to avoid circular import
        from .lane import Lane

        # Sort lanelets from left to right
        from ..util import sort_adjacent_groups

        # Convert to set if needed
        if isinstance(lanelet_group, set):
            lanelet_set = lanelet_group
        elif isinstance(lanelet_group, list):
            lanelet_set = set(lanelet_group)
        else:  # lanelet2.core.LaneletLayer
            lanelet_set = set(lanelet_group)

        # Use provided routing graph or create a new one
        if routing_graph is None:
            from ..util import create_routing_graph

            routing_graph = create_routing_graph(lanelet_map)

        sorted_lanelets = sort_adjacent_groups(lanelet_map, lanelet_set, routing_graph)

        # Create and set the reference line.  When the caller supplies
        # endpoint overrides (junction endpoint pinning, P0-2) the centre
        # lane must use the SAME pinned spline as the road's planView so
        # width s-coordinates and total length agree.
        reference_line = ReferenceLine.construct_from_lanelet_groups(
            lanelet_map,
            lanelet_group,
            traffic_rule=traffic_rule,
            start_xyz_override=start_xyz_override,
            end_xyz_override=end_xyz_override,
        )
        lane_section._set_center_lane(reference_line)

        # Normalize traffic_rule to uppercase, default to RHT
        traffic_rule_normalized = (traffic_rule or "RHT").upper()

        # Validate traffic_rule
        if traffic_rule_normalized not in ("RHT", "LHT"):
            raise ValueError(
                f"Invalid traffic_rule: '{traffic_rule}'. Must be 'RHT' or 'LHT'."
            )

        # Extract reference line spline for width calculation
        reference_line_spline = reference_line.centerline_2d

        # Create lanes and collect them for road mark assignment
        lanes_built: List["Lane"] = []

        # For RHT: lanes are in the right section with negative IDs (-1, -2, ...)
        #   sorted_lanelets[0] = innermost (adjacent to reference line) → lane -1
        # For LHT: lanes are in the left section with positive IDs (+1, +2, ...)
        #   sorted_lanelets[-1] = innermost (adjacent to reference line) → lane +1
        #   Iteration is reversed so the innermost lanelet gets the smallest positive ID.
        is_lht = traffic_rule_normalized == "LHT"
        lanelets_ordered = (
            list(reversed(sorted_lanelets)) if is_lht else sorted_lanelets
        )

        # The OUTERMOST lanelet is the one whose anchor boundary became the
        # reference line.  For RHT that is sorted_lanelets[0] (leftmost) and
        # in lanelets_ordered it is at index 0.  For LHT the rightmost
        # lanelet is the reference; lanelets_ordered is reversed, so it is
        # also at index 0.  Only this lanelet's anchor boundary endpoints
        # are pinned to the regular-road overrides — inner lanelets share
        # boundaries with neighbours and must keep their original positions
        # so cumulative widths still close on the outer side.
        for i, lanelet in enumerate(lanelets_ordered):
            lane_id = (i + 1) if is_lht else -(i + 1)
            is_outermost = i == 0
            lane = Lane.construct_from_lanelet(
                lanelet_map,
                lanelet,
                lane_id=lane_id,
                rule=traffic_rule_normalized,
                width_config=width_config,
                reference_line_spline=reference_line_spline,
                anchor_start_override=(start_xyz_override if is_outermost else None),
                anchor_end_override=(end_xyz_override if is_outermost else None),
            )
            if is_lht:
                lane_section._add_left_lane(lane)
            else:
                lane_section._add_right_lane(lane)
            lanes_built.append(lane)

        # Assign road marks by combining the boundary LineString attributes
        # with the routing-graph-derived lane-change permission.
        #
        # In OpenDRIVE, the road mark on a lane describes its INNER (center-side) boundary:
        #   RHT – lane -(i+1) road mark = boundary between lane -i and lane -(i+1)
        #   LHT – lane +(i+1) road mark = boundary between lane +i and lane +(i+1)
        #
        # The inner boundary LineString for lanelet i is:
        #   RHT – leftBound  (shared with lanelet i-1 or with reference line)
        #   LHT – rightBound (shared with lanelet i-1 or with reference line)
        # Its type/subtype/color/lane_change attributes drive the roadMark
        # type, weight, and colour via ``road_mark_from_linestring_attrs``.
        #
        # The routing graph provides the authoritative outward lane-change
        # permission; we preserve that decision while allowing the helper
        # to supply the visual mark description.
        #
        # References:
        #   - docs/spec-mapping/lanelet2-autoware-profile.md §"Boundary marking types"
        #   - docs/spec-mapping/opendrive-14-carla-profile.md §"Road marks"
        for i, lane in enumerate(lanes_built):
            lanelet_here = lanelets_ordered[i]
            if is_lht:
                can_change = routing_graph.left(lanelet_here) is not None
                inner_bound = lanelet_here.rightBound
            else:
                can_change = routing_graph.right(lanelet_here) is not None
                inner_bound = lanelet_here.leftBound

            # ``AttributeMap`` supports __contains__ and __getitem__ but not
            # ``.get``; convert to dict once and reuse for both the helper
            # call and the subtype lookup below.
            inner_attrs = dict(inner_bound.attributes)
            rm = road_mark_from_linestring_attrs(
                s_offset=0.0,
                attrs=inner_attrs,
                is_lht=is_lht,
            )

            # If the LineString attributes did not specify a lane_change,
            # fall back to the routing-graph-derived permission so
            # simulators such as CARLA do not block a legal manoeuvre.
            if rm.lane_change is None:
                rm.lane_change = (
                    RoadMarkLaneChange.BOTH if can_change else RoadMarkLaneChange.NONE
                )

            # If the LineString did not specify a type (fallback solid) but
            # the routing graph says a lane change is permitted, relax the
            # mark to BROKEN to remain visually consistent with behaviour.
            if (
                rm.type == RoadMarkType.SOLID
                and not inner_attrs.get("subtype")
                and can_change
            ):
                rm.type = RoadMarkType.BROKEN

            lane._add_road_mark(rm)

        return lane_section

    def get_all_lanes(self) -> List["Lane"]:
        """Get all lanes in the section (left + center + right)."""
        all_lanes = []

        # Add left lanes (sorted)
        for lane_id in sorted(self.left_lanes.keys()):
            all_lanes.append(self.left_lanes[lane_id])

        # Add center lane.  ``ReferenceLine`` wraps the underlying ``Lane`` in
        # ``_lane``; the parking-lot path stores a bare ``Lane`` directly.
        if self.center_lane is not None:
            inner = getattr(self.center_lane, "_lane", None)
            all_lanes.append(inner if inner is not None else self.center_lane)

        # Add right lanes (sorted)
        for lane_id in sorted(self.right_lanes.keys(), reverse=True):
            all_lanes.append(self.right_lanes[lane_id])

        return all_lanes

    def get_lanelet_to_lane_mapping(self) -> Dict[int, int]:
        """Get mapping from lanelet ID to lane ID for all lanes in this section.

        Returns:
            Dictionary mapping lanelet_id -> lane_id
        """
        mapping: Dict[int, int] = {}

        # Add left lanes
        for lane in self.left_lanes.values():
            if lane.lanelet_id is not None:
                mapping[lane.lanelet_id] = lane.lane_id

        # Add right lanes
        for lane in self.right_lanes.values():
            if lane.lanelet_id is not None:
                mapping[lane.lanelet_id] = lane.lane_id

        return mapping

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("laneSection")
        elem.set("s", str(self.s_offset))

        # Add laneOffset if present (for single lane sections)
        if self.lane_offset:
            offset_elem = ET.SubElement(elem, "laneOffset")
            offset_elem.set("s", str(self.lane_offset["s"]))
            for key in ("a", "b", "c", "d"):
                offset_elem.set(key, str(replace_subnormal(self.lane_offset[key])))

        # Add left lanes
        if self.left_lanes:
            left_elem = ET.SubElement(elem, "left")
            for lane_id in sorted(self.left_lanes.keys()):
                left_elem.append(self.left_lanes[lane_id].to_xml())

        # Add center lane
        if self.center_lane:
            center_elem = ET.SubElement(elem, "center")
            center_elem.append(self.center_lane.to_xml())

        # Add right lanes
        if self.right_lanes:
            right_elem = ET.SubElement(elem, "right")
            for lane_id in sorted(self.right_lanes.keys(), reverse=True):
                right_elem.append(self.right_lanes[lane_id].to_xml())

        return elem

    def __repr__(self) -> str:
        """String representation of the lane section."""
        return (
            f"LaneSection(s_offset={self.s_offset}, "
            f"left_lanes={len(self.left_lanes)}, "
            f"center_lane={'Yes' if self.center_lane else 'No'}, "
            f"right_lanes={len(self.right_lanes)})"
        )
