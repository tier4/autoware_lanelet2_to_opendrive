"""OpenDRIVE signals and controllers management."""

from dataclasses import dataclass, field
from typing import List, Dict, Set, Optional, TYPE_CHECKING
import lanelet2
import lxml.etree as ET

from .signal import Signal, Controller, ControlEntry
from .enums import TrafficRule
from ..util import RoadLaneletMapping, filter_regulatory_element_by_type
from ..config import COORDINATE_OFFSET

if TYPE_CHECKING:
    from .road import Road


@dataclass
class SignalsAndControllers:
    """
    Container for OpenDRIVE signals and controllers.

    This class manages the relationship between signals (traffic lights) and
    controllers (coordinated signal control for intersections). When multiple
    roads share the same traffic light, a controller is created to synchronize
    their behavior.

    Attributes:
        signals: List of all Signal objects
        controllers: List of Controller objects for coordinated signal control
        signal_to_road_id: Mapping from signal ID to road ID
    """

    signals: List[Signal] = field(default_factory=list)
    controllers: List[Controller] = field(default_factory=list)
    signal_to_road_id: Dict[int, int] = field(default_factory=dict)
    lanelet2_tl_id_to_signal_ids: Dict[int, List[int]] = field(default_factory=dict)

    def add_signal(self, signal: Signal) -> None:
        """Add a signal to the collection.

        Args:
            signal: Signal object to add
        """
        self.signals.append(signal)

    def add_controller(self, controller: Controller) -> None:
        """Add a controller to the collection.

        Args:
            controller: Controller object to add
        """
        self.controllers.append(controller)

    def to_xml(self) -> ET.Element:
        """Convert signals and controllers to XML structure.

        Returns:
            XML element containing both signals and controllers sections
        """
        root = ET.Element("signalsAndControllers")

        # Add signals section if signals exist
        if self.signals:
            signals_elem = ET.SubElement(root, "signals")
            for signal in self.signals:
                signals_elem.append(signal.to_xml())

        # Add controllers section if controllers exist
        if self.controllers:
            controllers_elem = ET.SubElement(root, "controllers")
            for controller in self.controllers:
                controllers_elem.append(controller.to_xml())

        return root

    @staticmethod
    def construct_from_lanelet_map(
        lanelet_map: lanelet2.core.LaneletMap,
        road_lanelet_mapping: RoadLaneletMapping,
        roads: Optional[List] = None,
        exclude_non_junction_signals: bool = False,
        junction_lanelet_ids: Optional[Set[int]] = None,
    ) -> "SignalsAndControllers":
        """
        Construct signals and controllers from Lanelet2 map.

        This method extracts traffic lights from the Lanelet2 map and converts them
        to OpenDRIVE signals. When a traffic light affects multiple roads (e.g., at
        an intersection), a controller is created to coordinate those signals.

        Args:
            lanelet_map: Lanelet2 map containing traffic light information
            road_lanelet_mapping: Mapping between OpenDRIVE roads and Lanelet2 lanelets
            roads: List of Road objects to calculate elevation at signal positions.
                  If None, signal z_offset will use absolute elevation values.
            exclude_non_junction_signals: If True, exclude traffic signals that are
                  not associated with junction lanelets. This is required for CARLA
                  compatibility, as CARLA does not support signals outside junctions.
            junction_lanelet_ids: Set of lanelet IDs that belong to junctions.
                  Required when exclude_non_junction_signals is True.

        Returns:
            SignalsAndControllers object with populated signals and controllers

        Algorithm:
            1. Extract all traffic lights from lanelets
            2. Group traffic lights by their lanelet2 ID (same physical signal)
            3. For each traffic light group:
               - Determine which roads it affects
               - Create Signal objects for each affected road
               - If it affects multiple roads, create a Controller to synchronize them
        """
        result = SignalsAndControllers()

        # Validate parameters
        if exclude_non_junction_signals and junction_lanelet_ids is None:
            junction_lanelet_ids = set()  # Empty set means no junctions

        # Step 1: Collect all traffic lights from all lanelets
        traffic_light_map = filter_regulatory_element_by_type(
            lanelet_map, element_type="trafficLights"
        )

        # Step 2: Process each traffic light and determine affected roads
        signal_id_counter = 0
        controller_id_counter = 0

        # Pre-build a dict for O(1) road lookup inside the signal loop.
        road_id_to_road: Dict[int, "Road"] = (
            {road.id: road for road in roads} if roads is not None else {}
        )

        for lanelet2_traffic_light_id, (
            traffic_light,
            lanelet_ids,
        ) in traffic_light_map.items():
            # CARLA compatibility: Skip signals not associated with junction lanelets
            if exclude_non_junction_signals and junction_lanelet_ids is not None:
                # Check if any of the lanelets associated with this signal are in a junction
                has_junction_lanelet = any(
                    ll_id in junction_lanelet_ids for ll_id in lanelet_ids
                )
                if not has_junction_lanelet:
                    # This signal is not associated with any junction lanelet, skip it
                    continue

            # Determine which roads are affected by this traffic light
            affected_roads: Set[int] = set()
            for lanelet_id in lanelet_ids:
                road_id = road_lanelet_mapping.get_road_for_lanelet(lanelet_id)
                if road_id is not None:
                    affected_roads.add(road_id)

            if not affected_roads:
                # No roads found for this traffic light, skip
                continue

            # Get traffic light position for s,t calculation
            # We'll use a simplified approach: place at the beginning of the first road
            # In a real implementation, you'd calculate the actual s,t from geometry
            # traffic_light_geometry = traffic_light.trafficLights[0]
            # first_point = traffic_light_geometry[0]

            # Create signals for each affected road
            created_signal_ids: List[int] = []

            for road_id in sorted(affected_roads):
                # Get lanelets for this road
                road_lanelets = road_lanelet_mapping.get_lanelets_for_road(road_id)

                # Find which lanelets in this road have this traffic light
                road_lanelets_with_signal = [
                    ll_id for ll_id in road_lanelets if ll_id in lanelet_ids
                ]

                if not road_lanelets_with_signal:
                    continue

                # Find the corresponding Road object (reused for lane IDs, elevation, and width)
                matching_road: Optional["Road"] = road_id_to_road.get(road_id)

                # Calculate s, t coordinates from traffic light geometry
                s, t = SignalsAndControllers._calculate_signal_position(
                    traffic_light=traffic_light,
                    road_id=road_id,
                    lanelet_map=lanelet_map,
                    road_lanelet_mapping=road_lanelet_mapping,
                    road=matching_road,
                )

                # Determine lane IDs for the validity element.
                # Use the road's lanelet-to-lane mapping so that the IDs are correct
                # for both RHT (negative IDs) and LHT (positive IDs).
                lane_ids: List[int] = SignalsAndControllers._get_signal_lane_ids(
                    road_lanelets_with_signal=road_lanelets_with_signal,
                    matching_road=matching_road,
                )

                # Calculate road elevation at signal position
                road_elevation_at_s = None
                if matching_road is not None:
                    # Use the Road's public method to get elevation at position s.
                    # Note: elevation_profile already contains absolute inertial
                    # z-coordinates (elevation_offset was added to 'a' coefficient
                    # in get_elevation_profile).
                    road_elevation_at_s = matching_road.get_elevation_at_s(s)

                # Create Signal object
                signal = Signal.construct_from_lanelet2_traffic_signal(
                    traffic_light=traffic_light,
                    signal_id=signal_id_counter,
                    s=s,
                    t=t,
                    lane_ids=lane_ids,
                    road_elevation_at_s=road_elevation_at_s,
                )

                result.add_signal(signal)
                created_signal_ids.append(signal_id_counter)

                # Track signal to road mapping
                result.signal_to_road_id[signal_id_counter] = road_id

                # Track lanelet2 traffic light ID to OpenDRIVE signal ID mapping
                if lanelet2_traffic_light_id not in result.lanelet2_tl_id_to_signal_ids:
                    result.lanelet2_tl_id_to_signal_ids[lanelet2_traffic_light_id] = []
                result.lanelet2_tl_id_to_signal_ids[lanelet2_traffic_light_id].append(
                    signal_id_counter
                )

                signal_id_counter += 1

            # Step 3: Create a controller for all traffic lights with signals
            # OpenDRIVE requires controllers to be defined for proper signal management,
            # regardless of whether the signal affects one or multiple roads
            if created_signal_ids:
                # Create control entries for all signals
                control_entries = [
                    ControlEntry(signal_id=sig_id) for sig_id in created_signal_ids
                ]

                # Create controller
                controller = Controller(
                    id=controller_id_counter,
                    name=f"Controller_TL_{lanelet2_traffic_light_id}",
                    controls=control_entries,
                )

                result.add_controller(controller)
                controller_id_counter += 1

        return result

    def __len__(self) -> int:
        """Return total number of signals and controllers."""
        return len(self.signals) + len(self.controllers)

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"SignalsAndControllers(signals={len(self.signals)}, "
            f"controllers={len(self.controllers)})"
        )

    @staticmethod
    def _get_signal_lane_ids(
        road_lanelets_with_signal: List[int],
        matching_road: Optional["Road"],
    ) -> List[int]:
        """Return lane IDs for the validity element of a signal.

        Looks up the actual lane IDs for the lanelets that carry the signal,
        using the road's lanelet-to-lane mapping.  This is correct for both
        RHT (negative IDs, right section) and LHT (positive IDs, left section).

        Falls back to a traffic-rule-aware default when the road or mapping is
        unavailable.

        Args:
            road_lanelets_with_signal: Lanelet IDs in this road that have the
                traffic light regulatory element attached.
            matching_road: Road object for this road_id, or None if not available.

        Returns:
            List of integer lane IDs to use in the validity element.
        """
        if matching_road is not None:
            lanelet_to_lane = matching_road.get_lanelet_to_lane_mapping()
            mapped = [
                lane_id
                for ll_id in road_lanelets_with_signal
                if (lane_id := lanelet_to_lane.get(ll_id)) is not None
            ]
            if mapped:
                return mapped

            # Lane mapping exists but the specific lanelets were not found —
            # fall back to the innermost lane for the road's traffic rule.
            if matching_road.rule == TrafficRule.LHT:
                return [1]

        # Default: RHT innermost lane
        return [-1]

    @staticmethod
    def _calculate_signal_position(
        traffic_light,  # lanelet2 TrafficLight regulatory element
        road_id: int,
        lanelet_map: lanelet2.core.LaneletMap,
        road_lanelet_mapping: RoadLaneletMapping,
        road: Optional["Road"] = None,
    ) -> tuple[float, float]:
        """
        Calculate s,t coordinates for a traffic signal on a road.

        The s coordinate is calculated by projecting the signal position onto
        the road reference line (Frenet s). The t coordinate is set to half
        the total lane width at that s position, placing the signal at the
        lateral center of the driving lanes.

        Args:
            traffic_light: Lanelet2 TrafficLight regulatory element
            road_id: ID of the road the signal is on
            lanelet_map: Lanelet2 map containing the lanelets
            road_lanelet_mapping: Mapping between roads and lanelets
            road: Optional Road object for computing lane width at s

        Returns:
            Tuple of (s, t) coordinates where:
                s: arc length along road reference line
                t: half of total lane width at s (signed by lane side)
        """
        # Get traffic light geometry (position)
        traffic_light_geometry = traffic_light.trafficLights
        if not traffic_light_geometry:
            # No geometry, return default position
            return (0.0, -4.0)

        # Get the first traffic light linestring
        light_linestring = traffic_light_geometry[0]
        if len(light_linestring) == 0:
            # Empty linestring, return default position
            return (0.0, -4.0)

        # Extract 3D position (use the first point as the signal position)
        # Apply coordinate offset to convert to local coordinates
        position = light_linestring[0]
        x = float(position.x) - COORDINATE_OFFSET.x
        y = float(position.y) - COORDINATE_OFFSET.y
        z = float(position.z) - COORDINATE_OFFSET.z

        # Build spline for this road
        from .reference_line import ReferenceLine

        # Get lanelet IDs for this road
        lanelet_ids = road_lanelet_mapping.get_lanelets_for_road(road_id)
        if not lanelet_ids:
            # No lanelets found for this road, return default position
            return (0.0, -4.0)

        # Get lanelet objects from IDs
        lanelets = []
        for lanelet_id in lanelet_ids:
            try:
                lanelet = lanelet_map.laneletLayer.get(lanelet_id)
                lanelets.append(lanelet)
            except Exception:
                # Lanelet not found, skip
                continue

        if not lanelets:
            # No valid lanelets found, return default position
            return (0.0, -4.0)

        try:
            # Construct spline from lanelets using ReferenceLine
            reference_line = ReferenceLine.construct_from_lanelet_groups(
                lanelet_map, lanelets
            )
            spline = reference_line.centerline_2d

            # Use cartesian_to_frenet to get s coordinate only
            s, _frenet_t = spline.cartesian_to_frenet(x, y, z)

            # Use half of total lane width at s for t
            if road is not None:
                t = road.get_half_width_at_s(s)
            else:
                t = _frenet_t

            return (s, t)
        except Exception as e:
            # If conversion fails, log warning and return default position
            print(
                f"Warning: Failed to calculate signal position for traffic light {traffic_light.id}: {e}"
            )
            return (0.0, -4.0)
