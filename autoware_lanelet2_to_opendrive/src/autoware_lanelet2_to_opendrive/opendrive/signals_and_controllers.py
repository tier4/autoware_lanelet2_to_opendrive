"""OpenDRIVE signals and controllers management."""

import math
from dataclasses import dataclass, field
from typing import List, Dict, Set, Optional, TYPE_CHECKING
import lanelet2
import lxml.etree as ET

from .signal import Signal, Controller, ControlEntry, PositionInertial
from .enums import TrafficRule
from ..util import RoadLaneletMapping, filter_regulatory_element_by_type
from ..config import COORDINATE_OFFSET
from ..conversion_config import TrafficLightConfig

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
        traffic_light_config: Optional[TrafficLightConfig] = None,
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
            traffic_light_config: Configuration for traffic light actor spawn offset.
                  If provided, the offsets are rotated by hdg and subtracted from
                  positionInertial coordinates.

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

            # Create signals for each LineString × road combination
            all_created_signal_ids: List[int] = []

            for light_linestring in traffic_light.trafficLights:
                if len(light_linestring) == 0:
                    continue

                for road_id in sorted(affected_roads):
                    # Get lanelets for this road
                    road_lanelets = road_lanelet_mapping.get_lanelets_for_road(road_id)

                    # Find which lanelets in this road have this traffic light
                    road_lanelets_with_signal = [
                        ll_id for ll_id in road_lanelets if ll_id in lanelet_ids
                    ]

                    if not road_lanelets_with_signal:
                        continue

                    # Find the corresponding Road object
                    matching_road: Optional["Road"] = road_id_to_road.get(road_id)

                    # Calculate logical s, t (stop-line based, fallback to linestring centroid)
                    s, t = SignalsAndControllers._calculate_signal_position(
                        traffic_light=traffic_light,
                        light_linestring=light_linestring,
                        road_id=road_id,
                        lanelet_map=lanelet_map,
                        road_lanelet_mapping=road_lanelet_mapping,
                        road=matching_road,
                    )

                    # Calculate physical position from linestring centroid
                    position_inertial = (
                        SignalsAndControllers._calculate_physical_position(
                            light_linestring=light_linestring,
                            road=matching_road,
                            road_s=s,
                            traffic_light_config=traffic_light_config,
                        )
                    )

                    # Determine lane IDs for the validity element.
                    lane_ids: List[int] = SignalsAndControllers._get_signal_lane_ids(
                        road_lanelets_with_signal=road_lanelets_with_signal,
                        matching_road=matching_road,
                    )

                    # Calculate road elevation at signal position
                    road_elevation_at_s = None
                    if matching_road is not None:
                        road_elevation_at_s = matching_road.get_elevation_at_s(s)

                    # Create Signal object
                    signal = Signal.construct_from_lanelet2_traffic_signal(
                        traffic_light=traffic_light,
                        light_linestring=light_linestring,
                        signal_id=signal_id_counter,
                        s=s,
                        t=t,
                        lane_ids=lane_ids,
                        road_elevation_at_s=road_elevation_at_s,
                        position_inertial=position_inertial,
                    )

                    result.add_signal(signal)
                    all_created_signal_ids.append(signal_id_counter)

                    # Track signal to road mapping
                    result.signal_to_road_id[signal_id_counter] = road_id

                    # Track lanelet2 traffic light ID to OpenDRIVE signal ID mapping
                    if (
                        lanelet2_traffic_light_id
                        not in result.lanelet2_tl_id_to_signal_ids
                    ):
                        result.lanelet2_tl_id_to_signal_ids[
                            lanelet2_traffic_light_id
                        ] = []
                    result.lanelet2_tl_id_to_signal_ids[
                        lanelet2_traffic_light_id
                    ].append(signal_id_counter)

                    signal_id_counter += 1

            # Step 3: Create a controller for all signals from this regulatory element
            if all_created_signal_ids:
                control_entries = [
                    ControlEntry(signal_id=sig_id) for sig_id in all_created_signal_ids
                ]

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
        light_linestring,  # specific Light Bulb LineString
        road_id: int,
        lanelet_map: lanelet2.core.LaneletMap,
        road_lanelet_mapping: RoadLaneletMapping,
        road: Optional["Road"] = None,
    ) -> tuple[float, float]:
        """
        Calculate logical s,t coordinates for a traffic signal on a road.

        Prefers the stop line position (if available) for logical placement.
        Falls back to the Light Bulb LineString centroid when no stop line exists.

        The t coordinate is set to half the total lane width at that s position.

        Args:
            traffic_light: Lanelet2 TrafficLight regulatory element
            light_linestring: Specific Light Bulb LineString for this signal
            road_id: ID of the road the signal is on
            lanelet_map: Lanelet2 map containing the lanelets
            road_lanelet_mapping: Mapping between roads and lanelets
            road: Optional Road object for computing lane width at s

        Returns:
            Tuple of (s, t) coordinates where:
                s: arc length along road reference line
                t: half of total lane width at s (signed by lane side)
        """
        if len(light_linestring) == 0:
            return (0.0, -4.0)

        # Determine the projection point: prefer stop line centroid, fallback to linestring centroid
        stop_line = getattr(traffic_light, "stopLine", None)
        if stop_line is not None and len(stop_line) > 0:
            # Use stop line centroid
            n = len(stop_line)
            x = sum(float(stop_line[i].x) for i in range(n)) / n - COORDINATE_OFFSET.x
            y = sum(float(stop_line[i].y) for i in range(n)) / n - COORDINATE_OFFSET.y
            z = sum(float(stop_line[i].z) for i in range(n)) / n - COORDINATE_OFFSET.z
        else:
            # Fallback: use linestring centroid (all points, not just [0])
            n = len(light_linestring)
            x = (
                sum(float(light_linestring[i].x) for i in range(n)) / n
                - COORDINATE_OFFSET.x
            )
            y = (
                sum(float(light_linestring[i].y) for i in range(n)) / n
                - COORDINATE_OFFSET.y
            )
            z = (
                sum(float(light_linestring[i].z) for i in range(n)) / n
                - COORDINATE_OFFSET.z
            )

        # Build spline for this road
        from .reference_line import ReferenceLine

        # Get lanelet IDs for this road
        lanelet_ids = road_lanelet_mapping.get_lanelets_for_road(road_id)
        if not lanelet_ids:
            return (0.0, -4.0)

        # Get lanelet objects from IDs
        lanelets = []
        for lanelet_id in lanelet_ids:
            try:
                lanelet = lanelet_map.laneletLayer.get(lanelet_id)
                lanelets.append(lanelet)
            except Exception:
                continue

        if not lanelets:
            return (0.0, -4.0)

        try:
            reference_line = ReferenceLine.construct_from_lanelet_groups(
                lanelet_map, lanelets
            )
            spline = reference_line.centerline_2d

            s, _frenet_t = spline.cartesian_to_frenet(x, y, z)

            # Use half of total lane width at s for t
            if road is not None:
                t = road.get_half_width_at_s(s)
            else:
                t = _frenet_t

            return (s, t)
        except Exception as e:
            print(
                f"Warning: Failed to calculate signal position for traffic light {traffic_light.id}: {e}"
            )
            return (0.0, -4.0)

    @staticmethod
    def _calculate_physical_position(
        light_linestring,
        road: Optional["Road"] = None,
        road_s: Optional[float] = None,
        traffic_light_config: Optional[TrafficLightConfig] = None,
    ) -> PositionInertial:
        """Calculate physical position of a signal from Light Bulb LineString centroid.

        Args:
            light_linestring: Light Bulb LineString (list of 3D points)
            road: Optional Road object (reserved for future use)
            road_s: Optional s coordinate on road (reserved for future use)
            traffic_light_config: Optional traffic light config with spawn offsets.
                The (offset_x, offset_y) are rotated by hdg and subtracted from the
                centroid position to adjust signal placement.

        Returns:
            PositionInertial with centroid coordinates and heading
        """
        n = len(light_linestring)
        x = (
            sum(float(light_linestring[i].x) for i in range(n)) / n
            - COORDINATE_OFFSET.x
        )
        y = (
            sum(float(light_linestring[i].y) for i in range(n)) / n
            - COORDINATE_OFFSET.y
        )
        z = (
            sum(float(light_linestring[i].z) for i in range(n)) / n
            - COORDINATE_OFFSET.z
        )

        # hdg: facing direction of the traffic light.
        # The LineString direction (first→last) runs along the bulb arrangement
        # (the face of the traffic light). The facing direction is perpendicular
        # to this. hdg_offset (default +π/2) rotates the LineString direction
        # to the facing direction. Configurable via TrafficLightConfig.hdg_offset.
        hdg_offset = math.pi / 2
        if traffic_light_config is not None:
            hdg_offset = traffic_light_config.hdg_offset

        hdg = 0.0
        if n >= 2:
            first = light_linestring[0]
            last = light_linestring[n - 1]
            dx = float(last.x) - float(first.x)
            dy = float(last.y) - float(first.y)
            hdg = math.atan2(dy, dx) + hdg_offset

        # Apply traffic light spawn offset (hdg-aware coordinate transformation).
        # The offset is specified in the signal's local frame:
        #   offset_x: along the facing direction (hdg)
        #   offset_y: perpendicular to hdg (positive = left)
        # Transform to world coordinates and subtract.
        if traffic_light_config is not None:
            cos_hdg = math.cos(hdg)
            sin_hdg = math.sin(hdg)
            world_dx = (
                traffic_light_config.offset_x * cos_hdg
                - traffic_light_config.offset_y * sin_hdg
            )
            world_dy = (
                traffic_light_config.offset_x * sin_hdg
                + traffic_light_config.offset_y * cos_hdg
            )
            x -= world_dx
            y -= world_dy
            z -= traffic_light_config.offset_z

        return PositionInertial(x=x, y=y, z=z, hdg=hdg)
