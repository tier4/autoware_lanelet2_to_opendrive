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
from ..centerline import AsymmetryLaneletException
from ..util import filter_lanelets_by_subtype


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

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("road")
        elem.set("id", str(self.id))
        elem.set("length", str(self.length))
        elem.set("junction", str(self.junction))

        if self.name:
            elem.set("name", self.name)

        if self.plan_view:
            elem.append(self.plan_view.to_xml())
        if self.elevation_profile:
            elem.append(self.elevation_profile.to_xml())
        if self.lanes:
            elem.append(self.lanes.to_xml())

        return elem

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

        print(f"Creating roads from {len(adjacent_groups)} lanelet groups...")
        for road_id, adjacent_group in tqdm(
            enumerate(adjacent_groups),
            total=len(adjacent_groups),
            desc="Building roads",
        ):
            # ll_ids = [ll.id for ll in adjacent_group]
            # if 124253 not in ll_ids:
            #     continue
            try:
                road = Road.construct_from_lanelet_groups(
                    lanelet_map=lanelet_map,  # Use original map for proper connectivity
                    lanelet_group=adjacent_group,
                    road_id=road_id,
                    s_offset=0.0,
                )
                roads.append(road)
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

        return roads
