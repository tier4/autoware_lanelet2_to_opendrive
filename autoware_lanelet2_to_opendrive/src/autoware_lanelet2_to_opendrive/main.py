#!/usr/bin/env python3
"""Main script to convert Lanelet2 maps to OpenDRIVE format."""

import re
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple
import tempfile
import logging
from datetime import datetime

import hydra
import mgrs as mgrs_lib
from omegaconf import DictConfig, OmegaConf

# Import autoware extensions before loading maps to ensure proper registration
# The order matters: projection module must be imported to register extensions
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2

from autoware_lanelet2_to_opendrive.projection import (
    mgrs_to_lanelet2_origin,
    mgrs_grid_with_offset_to_lanelet2_origin,
    mgrs_grid_with_offset_to_latlon,
    latlon_to_lanelet2_origin,
    latlon_to_proj_string,
    mgrs_to_proj_string,
)
from autoware_lanelet2_to_opendrive.util import (
    RoadLaneletMapping,
)
from autoware_lanelet2_to_opendrive.config import COORDINATE_OFFSET
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    PreprocessOperation,
    LaneletPreprocessor,
)

from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    OpenDRIVE,
    Header,
    save_opendrive_to_file,
)
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.junction import Junction
from autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers import (
    SignalsAndControllers,
)
from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
    ParamPoly3Config,
    StopLineConfig,
    TrafficLightConfig,
    WidthEstimationConfig,
)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def load_lanelet2_map(
    lanelet2_path: Path, origin: lanelet2.io.Origin
) -> lanelet2.core.LaneletMap:
    """
    Load Lanelet2 map from file with specified origin projection.

    Args:
        lanelet2_path: Path to the Lanelet2 OSM file
        origin: lanelet2.io.Origin object specifying the map origin

    Returns:
        Loaded Lanelet2 map

    Raises:
        FileNotFoundError: If the Lanelet2 file doesn't exist
        Exception: If map loading fails
    """
    if not lanelet2_path.exists():
        raise FileNotFoundError(f"Lanelet2 file not found: {lanelet2_path}")

    try:
        projector = MGRSProjector(origin)
        lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)
        print(f"Successfully loaded Lanelet2 map: {lanelet2_path}")
        print(f"  - Lanelets: {len(lanelet_map.laneletLayer)}")
        print(f"  - Linestrings: {len(lanelet_map.lineStringLayer)}")
        print(f"  - Points: {len(lanelet_map.pointLayer)}")
        return lanelet_map
    except Exception as e:
        raise Exception(f"Failed to load Lanelet2 map: {e}")


class _Lanelet2ToOpenDRIVEConverter:
    """
    Helper class for converting Lanelet2 maps to OpenDRIVE format.

    This class breaks down the conversion process into focused, testable methods.
    """

    def __init__(
        self,
        lanelet_map: lanelet2.core.LaneletMap,
        config: ConversionConfig,
    ):
        """
        Initialize the converter.

        Args:
            lanelet_map: Loaded Lanelet2 map
            config: ConversionConfig object containing all conversion parameters.
                The geoReference PROJ string is derived from ``config.origin``:
                lat/lon are used when available (e.g. when an MGRS offset is
                applied), otherwise ``config.origin.mgrs_code`` is used.
        """
        self.lanelet_map = lanelet_map
        self.config = config

    def _build_regular_roads(
        self,
    ) -> Tuple[List[Road], Dict[int, int], int]:
        """
        Build roads from non-junction lanelets.

        Returns:
            Tuple of:
                - List of Road objects for non-junction lanelets
                - Dictionary mapping lanelet ID to road ID (only successfully built roads)
                - Total number of adjacent groups (including failed ones)
        """
        print("\n=== Building regular roads ===")
        regular_roads, lanelet_to_road_id, num_groups = Road.construct_from_lanelet_map(
            self.lanelet_map,
            traffic_rule=self.config.traffic_rule,
            parampoly3_config=self.config.parampoly3,
            width_config=self.config.width_estimation,
        )

        return regular_roads, lanelet_to_road_id, num_groups

    def _build_junction_structure(
        self,
        regular_roads: List[Road],
        lanelet_to_road_id: Dict[int, int],
        num_regular_groups: int,
    ) -> Tuple[
        List[Road],
        List[Junction],
        Dict[int, List[int]],
        Dict[int, int],
        List[lanelet2.core.Lanelet],
    ]:
        """
        Build junction structure from junction lanelets.

        Args:
            regular_roads: Already-built regular roads
            lanelet_to_road_id: Existing lanelet-to-road mapping from regular roads
            num_regular_groups: Total number of regular road groups (including failed ones),
                used to assign non-overlapping IDs to junction roads

        Returns:
            Tuple of:
                - List of connecting Road objects
                - List of Junction objects
                - Dictionary mapping junction ID to connecting road IDs
                - Dictionary mapping junction lanelet IDs to road IDs
                - List of junction lanelets
        """
        from autoware_lanelet2_to_opendrive.junction import (
            _filter_lanelets_inside_junction,
            find_junction_groups,
        )

        # Get junction groups
        print("\n=== Finding junctions ===")
        all_lanelets = list(self.lanelet_map.laneletLayer)
        junction_lanelets = _filter_lanelets_inside_junction(all_lanelets)
        junction_groups = find_junction_groups(junction_lanelets)
        print(f"Found {len(junction_groups)} junctions")

        # Create connecting roads
        # Use total number of regular groups (not just successful ones) to avoid
        # ID collisions between junction roads and regular roads when some groups fail.
        print("\n=== Building connecting roads inside junctions ===")
        starting_junction_road_id = num_regular_groups
        junction_id_offset = self.config.junction_id_offset

        (
            connecting_roads,
            junction_to_roads,
            junction_lanelet_to_road,
        ) = Road.construct_connecting_roads_from_junctions(
            lanelet_map=self.lanelet_map,
            junction_groups=junction_groups,
            starting_road_id=starting_junction_road_id,
            junction_id_offset=junction_id_offset,
            traffic_rule=self.config.traffic_rule,
            parampoly3_config=self.config.parampoly3,
            width_config=self.config.width_estimation,
        )

        # Merge lanelet-to-road mappings
        lanelet_to_road_id.update(junction_lanelet_to_road)

        # Build junctions with connections
        print("\n=== Building junction connections ===")
        print(
            f"Using junction ID offset: {junction_id_offset} "
            f"(junction IDs will be {junction_id_offset}+)"
        )

        junctions = []
        for junction_index, junction_group in enumerate(junction_groups):
            junction_id = junction_index + junction_id_offset

            # Create base junction
            junction = Junction.construct_from_lanelet_groups(
                junction_id=junction_id,
                lanelet_group=junction_group,
            )

            # Build connections for this junction
            connecting_road_ids = junction_to_roads.get(junction_id, [])
            all_roads_for_junction = regular_roads + connecting_roads
            connections = Junction.build_connections_from_roads(
                lanelet_map=self.lanelet_map,
                junction_lanelet_group=junction_group,
                junction_id=junction_id,
                lanelet_to_road_id=lanelet_to_road_id,
                connecting_road_ids=connecting_road_ids,
                roads=all_roads_for_junction,
            )

            junction.connections = connections
            junctions.append(junction)

        print(
            f"Built {len(junctions)} junctions with {sum(len(j.connections) for j in junctions)} total connections"
        )

        return (
            connecting_roads,
            junctions,
            junction_to_roads,
            junction_lanelet_to_road,
            junction_lanelets,
        )

    def _build_road_lanelet_mappings(
        self,
        lanelet_to_road_id: Dict[int, int],
    ) -> RoadLaneletMapping:
        """
        Create bidirectional mappings between roads and lanelets.

        Args:
            lanelet_to_road_id: Dictionary mapping lanelet IDs to road IDs

        Returns:
            RoadLaneletMapping dataclass with bidirectional maps
        """
        print("\n=== Building Road-Lanelet mapping ===")

        # Build reverse mapping (road_id -> list of lanelet IDs)
        road_to_lanelet_ids: Dict[int, List[int]] = {}
        for lanelet_id, road_id in lanelet_to_road_id.items():
            if road_id not in road_to_lanelet_ids:
                road_to_lanelet_ids[road_id] = []
            road_to_lanelet_ids[road_id].append(lanelet_id)

        # Sort lanelet IDs for each road for consistency
        for road_id in road_to_lanelet_ids:
            road_to_lanelet_ids[road_id].sort()

        print(
            f"Created mapping for {len(road_to_lanelet_ids)} roads covering {len(lanelet_to_road_id)} lanelets"
        )

        return RoadLaneletMapping(
            road_to_lanelets=road_to_lanelet_ids,
            lanelet_to_road=lanelet_to_road_id,
        )

    def _setup_connections(
        self,
        all_roads: List[Road],
        connecting_roads: List[Road],
        road_to_lanelet_ids: Dict[int, List[int]],
        lanelet_to_road_id: Dict[int, int],
        junctions: List[Junction],
    ) -> Dict[int, Tuple[int, int]]:
        """
        Set up predecessor/successor connections for roads and lanes.

        Args:
            all_roads: All roads (regular + connecting)
            connecting_roads: Connecting roads only
            road_to_lanelet_ids: Dictionary mapping road IDs to lanelet IDs
            lanelet_to_road_id: Dictionary mapping lanelet IDs to road IDs
            junctions: All junctions

        Returns:
            Mapping from lanelet ID to (road_id, lane_id) for all lanes.
        """
        # Set road links for connecting roads
        print("\n=== Building road links for connecting roads ===")
        Road.set_connecting_road_links(
            lanelet_map=self.lanelet_map,
            connecting_roads=connecting_roads,
            lanelet_to_road_id=lanelet_to_road_id,
            road_to_lanelet_ids=road_to_lanelet_ids,
        )

        # Set junction links for incoming roads
        print("\n=== Setting junction links for incoming roads ===")
        Road.set_incoming_road_junction_links(
            roads=all_roads,
            junctions=junctions,
        )

        # Set lane links for all roads
        print("\n=== Building lane links for all roads ===")
        lanelet_to_road_and_lane = Road.set_all_lane_links(self.lanelet_map, all_roads)
        return lanelet_to_road_and_lane

    def _extract_and_assign_signals(
        self,
        all_roads: List[Road],
        mapping: RoadLaneletMapping,
        junction_lanelets: List[lanelet2.core.Lanelet],
    ) -> SignalsAndControllers:
        """
        Extract traffic signals and assign to roads.

        Args:
            all_roads: All roads
            mapping: Road-lanelet bidirectional mapping
            junction_lanelets: List of junction lanelets

        Returns:
            SignalsAndControllers object with all signals and controllers
        """
        print("\n=== Extracting signals and controllers ===")

        # Get junction lanelet IDs for filtering
        junction_lanelet_ids = {ll.id for ll in junction_lanelets}
        if self.config.exclude_non_junction_signals:
            print(
                f"CARLA compatibility mode: excluding signals not in {len(junction_lanelet_ids)} junction lanelets"
            )

        # Extract signals and controllers
        signals_and_controllers = SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=self.lanelet_map,
            road_lanelet_mapping=mapping,
            roads=all_roads,
            exclude_non_junction_signals=self.config.exclude_non_junction_signals,
            junction_lanelet_ids=junction_lanelet_ids,
            traffic_light_config=self.config.traffic_light,
        )
        print(
            f"Extracted {len(signals_and_controllers.signals)} signals and "
            f"{len(signals_and_controllers.controllers)} controllers"
        )

        # Assign signals to roads
        print("\n=== Assigning signals to roads ===")
        road_signals: Dict[int, List] = {}
        for signal in signals_and_controllers.signals:
            signal_road_id: Optional[int] = (
                signals_and_controllers.signal_to_road_id.get(signal.id)
            )
            if signal_road_id is not None:
                if signal_road_id not in road_signals:
                    road_signals[signal_road_id] = []
                road_signals[signal_road_id].append(signal)

        # Assign signals to road objects
        signals_assigned_count = 0
        for road in all_roads:
            if road.id in road_signals:
                road.signals = road_signals[road.id]
                signals_assigned_count += len(road.signals)

        print(f"Assigned {signals_assigned_count} signals to {len(road_signals)} roads")

        return signals_and_controllers

    def _assign_controllers_to_junctions(
        self,
        signals_and_controllers: SignalsAndControllers,
        junctions: List[Junction],
        all_roads: List[Road],
    ) -> None:
        """
        Create controllers and assign to junctions.

        Args:
            signals_and_controllers: All extracted signals and controllers
            junctions: All junctions
            all_roads: All roads
        """
        print("\n=== Associating controllers with junctions ===")

        controllers_assigned_count = 0
        for junction in junctions:
            # Get all road IDs related to this junction
            junction_incoming_road_ids = {
                conn.incoming_road for conn in junction.connections
            }
            junction_connecting_road_ids = {
                conn.connecting_road for conn in junction.connections
            }

            # Include roads that belong to this junction by attribute
            junction_roads_by_attribute = {
                road.id for road in all_roads if road.junction == junction.id
            }

            junction_related_road_ids = (
                junction_incoming_road_ids
                | junction_connecting_road_ids
                | junction_roads_by_attribute
            )

            # Find controllers whose signals are on roads related to this junction
            junction_controller_ids: List[int] = []
            for controller in signals_and_controllers.controllers:
                if controller.controls:
                    # Get road IDs for all signals controlled by this controller
                    controller_road_ids = set()
                    for control_entry in controller.controls:
                        signal_road_id = signals_and_controllers.signal_to_road_id.get(
                            control_entry.signal_id
                        )
                        if signal_road_id is not None:
                            controller_road_ids.add(signal_road_id)

                    # If any of the controller's roads are related to this junction,
                    # associate the controller with the junction
                    if controller_road_ids & junction_related_road_ids:
                        junction_controller_ids.append(controller.id)

            junction.controller_ids = junction_controller_ids
            controllers_assigned_count += len(junction_controller_ids)

        print(
            f"Associated {controllers_assigned_count} controller references across {len(junctions)} junctions"
        )

    def _extract_and_assign_crosswalks(self, all_roads: List[Road]) -> None:
        """Extract crosswalk lanelets and assign them as objects to the nearest roads.

        For each lanelet with subtype="crosswalk", this method:
        1. Finds the nearest road within a distance threshold
        2. Constructs a CrosswalkObject with outline coordinates
        3. Assigns it to the road's objects list

        Args:
            all_roads: All roads (regular + connecting) to search and assign to.
        """
        from autoware_lanelet2_to_opendrive.util import filter_lanelets_by_subtype
        from autoware_lanelet2_to_opendrive.opendrive.objects import (
            CrosswalkObject,
            find_nearest_road,
        )

        print("\n=== Extracting crosswalks ===")
        all_lanelets = list(self.lanelet_map.laneletLayer)
        crosswalk_lanelets = list(
            filter_lanelets_by_subtype(all_lanelets, ["crosswalk"])
        )
        print(f"Found {len(crosswalk_lanelets)} crosswalk lanelets")

        road_objects: Dict[int, List] = {}

        for lanelet in crosswalk_lanelets:
            best_road = find_nearest_road(lanelet, all_roads)
            if best_road is None:
                continue
            obj = CrosswalkObject.construct_from_crosswalk_lanelet(
                lanelet, best_road, object_id=lanelet.id
            )
            if obj is not None:
                road_objects.setdefault(best_road.id, []).append(obj)

        crosswalk_count = sum(len(v) for v in road_objects.values())
        print(
            f"Assigned {crosswalk_count} crosswalk objects to {len(road_objects)} roads"
        )

        for road in all_roads:
            if road.id in road_objects:
                road.objects = road_objects[road.id]

    def _build_stop_line_to_tl_mapping(self) -> Dict[int, List[int]]:
        """Build mapping from stop line linestring ID to associated traffic light RE IDs.

        Iterates through all traffic light regulatory elements in the Lanelet2 map
        and extracts stop line references via the stopLine attribute.

        Returns:
            Dictionary mapping stop line linestring ID to list of traffic light
            regulatory element IDs that reference it.
        """
        stop_line_to_tl_ids: Dict[int, List[int]] = {}
        seen_tl_ids: set = set()

        for lanelet in self.lanelet_map.laneletLayer:
            for reg_elem in lanelet.regulatoryElements:
                if not (hasattr(reg_elem, "trafficLights") and reg_elem.trafficLights):
                    continue
                if reg_elem.id in seen_tl_ids:
                    continue
                seen_tl_ids.add(reg_elem.id)

                stop_line = None
                if hasattr(reg_elem, "stopLine"):
                    try:
                        stop_line = reg_elem.stopLine
                    except Exception:
                        pass

                if stop_line is not None:
                    sl_id = stop_line.id
                    if sl_id not in stop_line_to_tl_ids:
                        stop_line_to_tl_ids[sl_id] = []
                    stop_line_to_tl_ids[sl_id].append(reg_elem.id)

        return stop_line_to_tl_ids

    def _build_stop_sign_stop_line_ids(self) -> Set[int]:
        """Build a set of stop line linestring IDs associated with stop signs.

        Iterates through all regulatory elements with ``subtype="traffic_sign"``
        that reference a linestring with ``type="traffic_sign", subtype="stop_sign"``
        (via the ``refers`` role).  The stop line is found via the ``ref_line``
        (``stopLine``) attribute of that regulatory element.

        Per Autoware vector map spec, a 一時停止 regulatory element has:
        - refers → Linestring(type="traffic_sign", subtype="stop_sign")
        - ref_line → Linestring(type="stop_line")

        Returns:
            Set of stop line linestring IDs that belong to stop sign REs.
        """
        stop_sign_stop_line_ids: Set[int] = set()
        seen_re_ids: set = set()

        for lanelet in self.lanelet_map.laneletLayer:
            for reg_elem in lanelet.regulatoryElements:
                if reg_elem.id in seen_re_ids:
                    continue
                seen_re_ids.add(reg_elem.id)

                # Check subtype == "traffic_sign"
                attrs = reg_elem.attributes if hasattr(reg_elem, "attributes") else None
                if not (
                    attrs is not None
                    and "subtype" in attrs
                    and attrs["subtype"] == "traffic_sign"
                ):
                    continue

                # Check that refers contains a stop_sign linestring and
                # extract ref_line (stop line) IDs in a single parameters access
                try:
                    params = reg_elem.parameters

                    has_stop_sign = False
                    if "refers" in params:
                        for refers_ls in params["refers"]:
                            ls_attrs = (
                                refers_ls.attributes
                                if hasattr(refers_ls, "attributes")
                                else None
                            )
                            if ls_attrs is not None and (
                                "subtype" in ls_attrs
                                and ls_attrs["subtype"] == "stop_sign"
                            ):
                                has_stop_sign = True
                                break

                    if has_stop_sign and "ref_line" in params:
                        for rl in params["ref_line"]:
                            stop_sign_stop_line_ids.add(rl.id)
                except Exception:
                    pass

        return stop_sign_stop_line_ids

    def _build_road_marking_stop_line_ids(self) -> Set[int]:
        """Build a set of stop line linestring IDs from road_marking regulatory elements.

        Iterates through all regulatory elements with ``subtype="road_marking"``
        and collects the IDs of linestrings with ``type="stop_line"`` from the
        ``refers`` role.

        Returns:
            Set of stop line linestring IDs that belong to road_marking REs.
        """
        road_marking_stop_line_ids: Set[int] = set()
        seen_re_ids: set = set()

        for lanelet in self.lanelet_map.laneletLayer:
            for reg_elem in lanelet.regulatoryElements:
                if reg_elem.id in seen_re_ids:
                    continue
                seen_re_ids.add(reg_elem.id)

                attrs = reg_elem.attributes if hasattr(reg_elem, "attributes") else None
                if not (
                    attrs is not None
                    and "subtype" in attrs
                    and attrs["subtype"] == "road_marking"
                ):
                    continue

                try:
                    params = reg_elem.parameters
                    if "refers" in params:
                        for refers_ls in params["refers"]:
                            ls_attrs = (
                                refers_ls.attributes
                                if hasattr(refers_ls, "attributes")
                                else None
                            )
                            if (
                                ls_attrs is not None
                                and "type" in ls_attrs
                                and ls_attrs["type"] == "stop_line"
                            ):
                                road_marking_stop_line_ids.add(refers_ls.id)
                except Exception:
                    pass

        return road_marking_stop_line_ids

    def _extract_and_assign_stop_lines(
        self,
        all_roads: List[Road],
        stop_line_to_tl_signal_ids: Optional[Dict[int, List[int]]] = None,
        stop_sign_stop_line_ids: Optional[Set[int]] = None,
        starting_signal_id: int = 0,
        road_marking_stop_line_ids: Optional[Set[int]] = None,
    ) -> Tuple[Dict[int, List[int]], Dict, Dict]:
        """Extract stop line linestrings and assign them as objects to nearest roads.

        For each linestring with type="stop_line", this method:
        1. Finds the nearest road within a distance threshold
        2. Constructs a StopLineObject with position and heading
        3. Extends the road's objects list with the new object
        4. If traffic light associations exist, creates a Signal (type 294)
           with dependency elements referencing the associated traffic lights
        5. If associated with a stop sign regulatory element, creates a Signal
           (type 206 / StopSign)
        6. If associated with a road_marking regulatory element (and not already
           handled by traffic_light), creates a YieldSign (type 205) and a
           StopLine (type 294) with yieldSign dependency

        Args:
            all_roads: All roads (regular + connecting) to search and assign to.
            stop_line_to_tl_signal_ids: Mapping from stop line lanelet2 ID to list
                of OpenDRIVE traffic light signal IDs. If provided (and not in CARLA
                mode), stop line Signal elements are created with dependency references.
            stop_sign_stop_line_ids: Set of stop line linestring IDs that are
                associated with stop sign regulatory elements.  These will produce
                StopSign signals (type 206).
            starting_signal_id: Starting ID for generated stop line signals.
            road_marking_stop_line_ids: Set of stop line linestring IDs from
                road_marking regulatory elements.  These produce YieldSign (205)
                and StopLine (294) signal pairs.

        Returns:
            Tuple of:
            - Dictionary mapping traffic light signal ID to list of stop line signal
              IDs, used to add back-references (Reference elements) to traffic light
              signals.
            - Dictionary mapping linestring ID to StopLineMappingEntry for
              successfully converted stop lines.
            - Dictionary mapping linestring ID to SkippedStopLineEntry for
              stop lines that were skipped during conversion.
        """
        from autoware_lanelet2_to_opendrive.opendrive.objects import (
            StopLineObject,
            find_nearest_road_for_linestring,
        )
        from autoware_lanelet2_to_opendrive.opendrive.signal import (
            Signal,
            Dependency,
            SignalType,
        )
        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            StopLineMappingEntry,
            SkippedStopLineEntry,
        )

        print("\n=== Extracting stop lines ===")
        stop_line_ids_seen: set = set()
        road_objects: Dict[int, List] = {}
        road_stop_line_signals: Dict[int, List] = {}
        tl_signal_to_stop_line_signal_ids: Dict[int, List[int]] = {}
        stop_line_mapping: Dict[int, StopLineMappingEntry] = {}
        skipped_stop_lines: Dict[int, SkippedStopLineEntry] = {}
        stop_line_signal_id_counter = starting_signal_id
        # Resolve Optional to a concrete dict for type narrowing
        resolved_tl_signal_ids: Dict[int, List[int]] = (
            stop_line_to_tl_signal_ids
            if (
                not self.config.stopline.carla_stop_line
                and stop_line_to_tl_signal_ids is not None
            )
            else {}
        )
        resolved_stop_sign_ids: Set[int] = stop_sign_stop_line_ids or set()
        resolved_road_marking_ids: Set[int] = road_marking_stop_line_ids or set()
        stop_line_294_count = 0
        stop_sign_206_count = 0
        yield_sign_205_count = 0
        road_marking_294_count = 0

        for ls in self.lanelet_map.lineStringLayer:
            if "type" not in ls.attributes or ls.attributes["type"] != "stop_line":
                continue
            if ls.id in stop_line_ids_seen:
                continue
            stop_line_ids_seen.add(ls.id)

            best_road = find_nearest_road_for_linestring(ls, all_roads)
            if best_road is None:
                skipped_stop_lines[ls.id] = SkippedStopLineEntry(
                    reason="no_nearest_road"
                )
                continue

            obj = StopLineObject.construct_from_linestring(
                linestring=ls,
                road=best_road,
                object_id=ls.id,
                width=self.config.stopline.width,
                carla_format=self.config.stopline.carla_stop_line,
            )
            if obj is None:
                skipped_stop_lines[ls.id] = SkippedStopLineEntry(
                    reason="construction_failed"
                )
                continue

            road_objects.setdefault(best_road.id, []).append(obj)
            current_signal_types: List[int] = []

            # Use half of road width at s for signal t coordinate
            signal_t = best_road.get_half_width_at_s(obj.s)

            def _make_signal(
                signal_type: int,
                name: str,
                dependencies: Optional[List[Dependency]] = None,
            ) -> Signal:
                return Signal(
                    id=stop_line_signal_id_counter,
                    name=name,
                    s=obj.s,
                    t=signal_t,
                    z_offset=obj.z_offset,
                    h_offset=0.0,
                    roll=0.0,
                    pitch=0.0,
                    orientation="-" if signal_t < 0 else "+",
                    dynamic="no",
                    country="OpenDRIVE",
                    type=signal_type,
                    subtype=-1,
                    value=-1.0,
                    text="",
                    height=0.0,
                    width=obj.length,
                    dependencies=dependencies,
                )

            # Create stop line Signal (type 294) when traffic light associations exist
            if ls.id in resolved_tl_signal_ids:
                tl_signal_ids = resolved_tl_signal_ids[ls.id]
                stop_line_signal = _make_signal(
                    signal_type=SignalType.STOP_LINE,
                    name=f"StopLine_{ls.id}",
                    dependencies=[
                        Dependency(id=tl_sig_id, type="trafficLight")
                        for tl_sig_id in tl_signal_ids
                    ],
                )
                road_stop_line_signals.setdefault(best_road.id, []).append(
                    stop_line_signal
                )

                # Build reverse mapping for adding references to TL signals
                for tl_sig_id in tl_signal_ids:
                    tl_signal_to_stop_line_signal_ids.setdefault(tl_sig_id, []).append(
                        stop_line_signal_id_counter
                    )

                stop_line_signal_id_counter += 1
                stop_line_294_count += 1
                current_signal_types.append(SignalType.STOP_LINE)

            # Create StopSign signal (type 206) for stop lines referenced by
            # a traffic_sign regulatory element with a stop_sign refers member
            if ls.id in resolved_stop_sign_ids:
                stop_sign_signal = _make_signal(
                    signal_type=SignalType.STOP_SIGN,
                    name=f"StopSign_{ls.id}",
                )
                road_stop_line_signals.setdefault(best_road.id, []).append(
                    stop_sign_signal
                )
                stop_line_signal_id_counter += 1
                stop_sign_206_count += 1
                current_signal_types.append(SignalType.STOP_SIGN)

            # Create YieldSign (type 205) + StopLine (type 294) for road marking
            # stop lines.  Skip if already handled by traffic_light (avoids
            # duplicate type=294).
            if (
                ls.id in resolved_road_marking_ids
                and ls.id not in resolved_tl_signal_ids
            ):
                # 1. YieldSign signal (type 205)
                yield_sign_signal = _make_signal(
                    signal_type=SignalType.YIELD_SIGN,
                    name=f"YieldSign_{ls.id}",
                )
                road_stop_line_signals.setdefault(best_road.id, []).append(
                    yield_sign_signal
                )
                yield_sign_id = stop_line_signal_id_counter
                stop_line_signal_id_counter += 1
                yield_sign_205_count += 1

                # 2. StopLine signal (type 294) with dependency to YieldSign
                rm_stop_line_signal = _make_signal(
                    signal_type=SignalType.STOP_LINE,
                    name=f"StopLine_{ls.id}",
                    dependencies=[Dependency(id=yield_sign_id, type="yieldSign")],
                )
                road_stop_line_signals.setdefault(best_road.id, []).append(
                    rm_stop_line_signal
                )
                stop_line_signal_id_counter += 1
                road_marking_294_count += 1
                current_signal_types.extend(
                    [SignalType.YIELD_SIGN, SignalType.STOP_LINE]
                )

            # Record mapping for this successfully converted stop line
            stop_line_mapping[ls.id] = StopLineMappingEntry(
                road_id=best_road.id,
                signal_types=current_signal_types,
            )

        stop_line_count = sum(len(v) for v in road_objects.values())
        print(
            f"Assigned {stop_line_count} stop line objects to {len(road_objects)} roads"
        )
        if stop_line_294_count > 0:
            print(
                f"Created {stop_line_294_count} stop line signals (type 294) "
                f"with traffic light dependencies"
            )
        if stop_sign_206_count > 0:
            print(
                f"Created {stop_sign_206_count} stop sign signals (type 206) "
                f"for stop lines without traffic lights"
            )
        if yield_sign_205_count > 0:
            print(
                f"Created {yield_sign_205_count} yield sign signals (type 205) "
                f"and {road_marking_294_count} stop line signals (type 294) "
                f"for road marking stop lines"
            )

        for road in all_roads:
            if road.id in road_objects:
                if road.objects is None:
                    road.objects = []
                road.objects.extend(road_objects[road.id])
            if road.id in road_stop_line_signals:
                if road.signals is None:
                    road.signals = []
                road.signals.extend(road_stop_line_signals[road.id])

        return tl_signal_to_stop_line_signal_ids, stop_line_mapping, skipped_stop_lines

    def _write_opendrive_output(
        self,
        all_roads: List[Road],
        junctions: List[Junction],
        signals_and_controllers: SignalsAndControllers,
    ) -> OpenDRIVE:
        """
        Write final OpenDRIVE XML output.

        Args:
            all_roads: All roads to write
            junctions: All junctions to write
            signals_and_controllers: All signals and controllers

        Returns:
            OpenDRIVE object
        """
        # Generate PROJ string for geoReference.
        #
        # Priority:
        #   1. lat/lon  – most precise; set when an MGRS offset is applied or
        #                 when the origin was specified directly as lat/lon.
        #   2. mgrs_code – fallback; used when only the MGRS grid square is
        #                  known (origin = south-west corner of the square).
        if self.config.origin.lat is not None and self.config.origin.lon is not None:
            geo_reference_proj = latlon_to_proj_string(
                self.config.origin.lat, self.config.origin.lon
            )
        elif self.config.origin.mgrs_code is not None:
            geo_reference_proj = mgrs_to_proj_string(self.config.origin.mgrs_code)
        else:
            raise ValueError(
                "Cannot generate geoReference: config.origin must have lat/lon "
                "or mgrs_code set."
            )
        logger.info("geoReference (PROJ string): %s", geo_reference_proj)

        # Create header
        header = Header(
            rev_major="1",
            rev_minor="4",
            name="Converted from Lanelet2",
            version="1.0",
            date=datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
            north="0.0",
            south="0.0",
            east="0.0",
            west="0.0",
            geo_reference=geo_reference_proj,
        )

        # Create OpenDRIVE object
        opendrive = OpenDRIVE(
            header=header,
            roads=all_roads,
            junctions=junctions,
            controllers=signals_and_controllers.controllers,
        )

        logger.info("Conversion completed successfully!")

        # Save to file if output path is provided
        if self.config.output_path:
            save_opendrive_to_file(opendrive, self.config.output_path)
            print(f"OpenDRIVE file saved to: {self.config.output_path}")

        return opendrive

    def convert(
        self,
    ) -> Tuple[OpenDRIVE, RoadLaneletMapping, Dict[int, Tuple[int, int]]]:
        """
        Convert Lanelet2 map to OpenDRIVE format.

        High-level orchestration of conversion pipeline.

        Returns:
            Tuple of:
                - OpenDRIVE object representing the converted map
                - RoadLaneletMapping containing bidirectional mapping
                - Mapping from lanelet ID to (road_id, lane_id) for all lanes
        """
        print("Converting Lanelet2 map to OpenDRIVE format...")

        # Step 1: Build regular roads from non-junction lanelets
        regular_roads, lanelet_to_road_id, num_regular_groups = (
            self._build_regular_roads()
        )

        # Step 2: Build junction structure
        (
            connecting_roads,
            junctions,
            junction_to_roads,
            junction_lanelet_to_road,
            junction_lanelets,
        ) = self._build_junction_structure(
            regular_roads, lanelet_to_road_id, num_regular_groups
        )

        # Step 3: Create bidirectional mappings
        mapping = self._build_road_lanelet_mappings(lanelet_to_road_id)

        # Combine all roads
        all_roads = regular_roads + connecting_roads
        print(
            f"\nTotal roads: {len(all_roads)} ({len(regular_roads)} regular + {len(connecting_roads)} connecting)"
        )

        # Step 4: Set up road and lane connections
        lanelet_to_road_and_lane = self._setup_connections(
            all_roads,
            connecting_roads,
            mapping.road_to_lanelets,
            lanelet_to_road_id,
            junctions,
        )

        # Step 5: Extract and assign signals
        signals_and_controllers = self._extract_and_assign_signals(
            all_roads, mapping, junction_lanelets
        )

        # Step 6: Create and assign controllers
        self._assign_controllers_to_junctions(
            signals_and_controllers, junctions, all_roads
        )

        # Step 6.5: Extract crosswalks and assign as road objects
        self._extract_and_assign_crosswalks(all_roads)

        # Step 6.6: Build stop line -> traffic light signal associations
        print("\n=== Building stop line to traffic light associations ===")
        stop_line_to_tl_ids = self._build_stop_line_to_tl_mapping()
        print(
            f"Found {len(stop_line_to_tl_ids)} stop lines with traffic light references"
        )

        # Resolve Lanelet2 TL IDs to OpenDRIVE signal IDs
        stop_line_to_tl_signal_ids: Dict[int, List[int]] = {}
        for sl_id, tl_ids in stop_line_to_tl_ids.items():
            resolved_signal_ids: List[int] = []
            for tl_id in tl_ids:
                resolved_signal_ids.extend(
                    signals_and_controllers.lanelet2_tl_id_to_signal_ids.get(tl_id, [])
                )
            if resolved_signal_ids:
                stop_line_to_tl_signal_ids[sl_id] = resolved_signal_ids

        # Step 6.6b: Build stop sign stop line IDs
        stop_sign_stop_line_ids = self._build_stop_sign_stop_line_ids()
        if stop_sign_stop_line_ids:
            print(
                f"Found {len(stop_sign_stop_line_ids)} stop lines "
                f"associated with stop sign regulatory elements"
            )

        # Step 6.6c: Build road marking stop line IDs
        road_marking_stop_line_ids = self._build_road_marking_stop_line_ids()
        if road_marking_stop_line_ids:
            print(
                f"Found {len(road_marking_stop_line_ids)} stop lines "
                f"associated with road marking regulatory elements"
            )

        # Step 6.7: Extract stop lines and assign as road objects (with signal dependencies)
        next_signal_id = len(signals_and_controllers.signals)
        (
            tl_signal_to_stop_line_signal_ids,
            stop_line_mapping,
            skipped_stop_lines,
        ) = self._extract_and_assign_stop_lines(
            all_roads,
            stop_line_to_tl_signal_ids,
            stop_sign_stop_line_ids,
            next_signal_id,
            road_marking_stop_line_ids=road_marking_stop_line_ids,
        )

        # Step 6.8: Add back-references to traffic light signals pointing to stop lines
        if tl_signal_to_stop_line_signal_ids:
            from autoware_lanelet2_to_opendrive.opendrive.signal import Reference

            ref_count = 0
            for signal in signals_and_controllers.signals:
                stop_line_signal_ids = tl_signal_to_stop_line_signal_ids.get(
                    signal.id, []
                )
                if stop_line_signal_ids:
                    signal.references = [
                        Reference(
                            id=sl_sig_id,
                            element_type="signal",
                            type="stopLine",
                        )
                        for sl_sig_id in stop_line_signal_ids
                    ]
                    ref_count += 1
            print(f"Added stop line references to {ref_count} traffic light signals")

        # Step 6.7: Validate no duplicate road IDs (safety check for ID assignment bugs)
        from autoware_lanelet2_to_opendrive.opendrive.validation import (
            validate_no_duplicate_road_ids,
        )

        dup_result = validate_no_duplicate_road_ids(all_roads)
        if not dup_result.is_valid:
            print(f"\nWARNING: {dup_result.get_error_summary()}")

        # Step 7: Write OpenDRIVE output
        opendrive = self._write_opendrive_output(
            all_roads, junctions, signals_and_controllers
        )

        return (
            opendrive,
            mapping,
            lanelet_to_road_and_lane,
            stop_line_mapping,
            skipped_stop_lines,
        )


def convert_lanelet2_to_opendrive(
    lanelet_map: lanelet2.core.LaneletMap,
    config: ConversionConfig,
    mgrs_code: Optional[str] = None,
) -> Tuple[OpenDRIVE, RoadLaneletMapping, Dict[int, Tuple[int, int]], Dict, Dict]:
    """
    Convert Lanelet2 map to OpenDRIVE format.

    Args:
        lanelet_map: Loaded Lanelet2 map
        config: ConversionConfig object containing all conversion parameters.
            The geoReference PROJ string is derived from ``config.origin``:
            lat/lon are preferred (set when an MGRS offset is applied or when
            the origin was specified as lat/lon); ``config.origin.mgrs_code``
            is used as a fallback.
        mgrs_code: Deprecated.  Pass the MGRS code via
            ``config.origin.mgrs_code`` instead.  When provided here it
            overrides ``config.origin.mgrs_code`` so that callers using the
            old API continue to work.

    Returns:
        Tuple of:
            - OpenDRIVE object representing the converted map
            - RoadLaneletMapping containing bidirectional mapping
            - Mapping from lanelet ID to (road_id, lane_id) for all lanes
            - Stop line mapping (linestring_id -> StopLineMappingEntry)
            - Skipped stop lines (linestring_id -> SkippedStopLineEntry)
    """
    # Merge legacy mgrs_code argument into config.origin so the converter has
    # a single, consistent source of truth for the MGRS grid code.
    if mgrs_code is not None:
        config = config.with_mgrs_code(mgrs_code)

    converter = _Lanelet2ToOpenDRIVEConverter(lanelet_map, config)
    return converter.convert()


def parse_origin_from_config(
    cfg: DictConfig,
) -> Tuple[lanelet2.io.Origin, str, float, float, float, float, float]:
    """
    Parse origin specification from Hydra config with mutual exclusion validation.

    Supports three methods of origin specification (mutually exclusive):
    1. mgrs_grid: Simple MGRS grid code (e.g., "54SUE")
       - With optional offset: mgrs_grid + offset {x, y, z}
    2. lat_lon: Latitude/longitude {latitude, longitude, altitude}

    Args:
        cfg: Hydra configuration object with map settings

    Returns:
        Tuple of (lanelet2.io.Origin, mgrs_code_for_proj_string, origin_lat, origin_lon,
                  offset_x, offset_y, offset_z)
        The mgrs_code_for_proj_string is used for generating the OpenDRIVE geoReference
        The origin_lat and origin_lon are the actual origin coordinates (with offset applied)
        The offset values are used to convert coordinates to local coordinate system

    Raises:
        ValueError: If origin specification is invalid or multiple methods are specified
    """
    map_cfg = cfg.map

    # Check which origin specification methods are provided
    has_mgrs_grid = "mgrs_grid" in map_cfg and map_cfg.mgrs_grid is not None
    has_mgrs_code = (
        "mgrs_code" in map_cfg and map_cfg.mgrs_code is not None
    )  # Legacy support
    has_offset = "offset" in map_cfg and map_cfg.offset is not None
    has_lat_lon = "lat_lon" in map_cfg and map_cfg.lat_lon is not None

    # Support legacy mgrs_code field
    if has_mgrs_code and not has_mgrs_grid:
        has_mgrs_grid = True
        map_cfg.mgrs_grid = map_cfg.mgrs_code

    # Count how many base methods are specified (offset is an optional modifier for mgrs_grid)
    specified_methods = sum([has_mgrs_grid, has_lat_lon])

    if specified_methods == 0:
        raise ValueError(
            "Origin must be specified using one of: mgrs_grid (with optional offset), or lat_lon"
        )

    if specified_methods > 1:
        raise ValueError(
            "Multiple origin specification methods detected. "
            "Please specify only one of: mgrs_grid (with optional offset), or lat_lon"
        )

    # Offset can only be used with mgrs_grid
    if has_offset and not has_mgrs_grid:
        raise ValueError(
            "The 'offset' field can only be used together with 'mgrs_grid'"
        )

    # Parse the specified method
    if has_mgrs_grid:
        mgrs_grid = map_cfg.mgrs_grid

        # Check if offset is specified
        if has_offset:
            offset_cfg = map_cfg.offset
            offset_x = offset_cfg.x
            offset_y = offset_cfg.y
            offset_z = offset_cfg.get("z", 0.0)

            logger.info(
                f"Using MGRS grid with offset: {mgrs_grid}, "
                f"offset x={offset_x} y={offset_y} z={offset_z}"
            )
            # Get lat/lon with offset applied
            origin_lat, origin_lon = mgrs_grid_with_offset_to_latlon(
                mgrs_grid, offset_x, offset_y
            )
            origin = mgrs_grid_with_offset_to_lanelet2_origin(
                mgrs_grid, offset_x, offset_y, offset_z
            )
            logger.info(f"Origin coordinates: lat={origin_lat}, lon={origin_lon}")
            return (
                origin,
                mgrs_grid,
                origin_lat,
                origin_lon,
                offset_x,
                offset_y,
                offset_z,
            )
        else:
            logger.info(f"Using MGRS grid origin: {mgrs_grid}")
            origin = mgrs_to_lanelet2_origin(mgrs_grid)
            # Get lat/lon from MGRS grid origin (no offset)
            origin_lat, origin_lon = mgrs_grid_with_offset_to_latlon(
                mgrs_grid, 0.0, 0.0
            )
            logger.info(f"Origin coordinates: lat={origin_lat}, lon={origin_lon}")
            # No offset specified
            return origin, mgrs_grid, origin_lat, origin_lon, 0.0, 0.0, 0.0

    elif has_lat_lon:
        latlon_cfg = map_cfg.lat_lon
        latitude = latlon_cfg.latitude
        longitude = latlon_cfg.longitude
        altitude = latlon_cfg.get("altitude", 0.0)

        logger.info(
            f"Using lat/lon origin: latitude={latitude}, longitude={longitude}, altitude={altitude}"
        )
        origin = latlon_to_lanelet2_origin(latitude, longitude, altitude)

        # For lat/lon origin, we need to generate an approximate MGRS code for the PROJ string
        # Convert lat/lon back to MGRS to get the grid zone
        m = mgrs_lib.MGRS()
        mgrs_code = m.toMGRS(latitude, longitude)
        # Extract just the grid zone designator (first 5 characters: zone + band + square)
        # Format: 54SUE1234567890 -> we want 54SUE
        match = re.match(r"^(\d+[A-Z][A-Z][A-Z])", mgrs_code)
        if match:
            mgrs_grid = match.group(1)
        else:
            mgrs_grid = mgrs_code[:5]  # Fallback to first 5 chars

        logger.info(f"Derived MGRS grid for PROJ string: {mgrs_grid}")
        # No offset for lat/lon origin
        return origin, mgrs_grid, latitude, longitude, 0.0, 0.0, 0.0

    # Should never reach here due to the checks above
    raise ValueError("Invalid origin configuration")


def preprocess_and_convert_with_hydra(
    cfg: DictConfig,
    lanelet2_file: Path,
    output_file: Path,
) -> None:
    """
    Run preprocessing (if configured) and convert Lanelet2 to OpenDRIVE using Hydra config.

    Args:
        cfg: Hydra configuration object
        lanelet2_file: Path to input Lanelet2 OSM file
        output_file: Path to output OpenDRIVE file
    """
    input_map_path = lanelet2_file

    # Parse origin from config (with mutual exclusion validation)
    (
        origin,
        mgrs_code,
        origin_lat,
        origin_lon,
        offset_x,
        offset_y,
        offset_z,
    ) = parse_origin_from_config(cfg)

    # Set global coordinate offset for conversion
    # This will be applied to all coordinates during OpenDRIVE export
    COORDINATE_OFFSET.set(offset_x, offset_y, offset_z)
    if COORDINATE_OFFSET.is_active:
        logger.info(
            f"Coordinate offset enabled: x={offset_x}, y={offset_y}, z={offset_z}"
        )

    # Get target-specific settings
    exclude_non_junction_signals = cfg.target.get("exclude_non_junction_signals", False)
    # Priority: map config > target config > default (RHT)
    traffic_rule = cfg.map.get("traffic_rule") or cfg.target.get("traffic_rule", "RHT")

    # Build PreprocessOperation from Hydra map config
    config = PreprocessOperation.from_hydra_config(cfg.map)

    # Check if any preprocessing operations are configured
    has_preprocessing = any(
        [
            config.merge_operations,
            config.remove_operations,
            config.replace_operations,
            config.move_point_operations,
            config.delete_point_operations,
            config.remove_lanelet_operations,
            config.remove_turn_direction_operations,
        ]
    )

    preprocessing_log_dict: dict | None = None

    if has_preprocessing:
        logger.info("Running preprocessing operations...")

        # Set input/output paths for preprocessing
        config.input_map_path = str(lanelet2_file)

        # Create a temporary file for preprocessed output
        if config.output_map_path:
            preprocessed_path = Path(config.output_map_path)
        else:
            with tempfile.NamedTemporaryFile(suffix=".osm", delete=False) as tmp_file:
                preprocessed_path = Path(tmp_file.name)
                config.output_map_path = str(preprocessed_path)

        # Run preprocessing
        preprocessor = LaneletPreprocessor(config)
        _lanelet_map, preprocessing_log = preprocessor.process()
        preprocessing_log_dict = preprocessing_log.to_dict()

        # Update input path to use preprocessed map
        input_map_path = preprocessed_path
        logger.info(
            f"Preprocessing completed ({len(preprocessing_log.entries)} ops). "
            f"Using preprocessed map from: {input_map_path}"
        )

    # Load the (possibly preprocessed) Lanelet2 map
    logger.info(f"Loading Lanelet2 map from: {input_map_path}")
    logger.info(f"Using origin with MGRS code for PROJ: {mgrs_code}")

    lanelet_map = load_lanelet2_map(input_map_path, origin)

    # Convert to OpenDRIVE
    logger.info("Converting to OpenDRIVE format...")

    # Build ParamPoly3Config from Hydra config
    # Priority: map config > target config > default
    parampoly3_dict = cfg.map.get("parampoly3") or cfg.target.get("parampoly3", {})
    if parampoly3_dict:
        parampoly3_config = ParamPoly3Config(
            min_segment_length=parampoly3_dict.get("min_segment_length", 0.5),
            default_segment_length=parampoly3_dict.get("default_segment_length", 1.0),
            max_segments=parampoly3_dict.get("max_segments", 100),
            min_segments=parampoly3_dict.get("min_segments", 1),
            coefficient_epsilon=parampoly3_dict.get("coefficient_epsilon", 1e-8),
            enabled=parampoly3_dict.get("enabled", True),
        )
        logger.info(
            f"ParamPoly3 config: default_length={parampoly3_config.default_segment_length}m, "
            f"max_segments={parampoly3_config.max_segments}"
        )
    else:
        parampoly3_config = ParamPoly3Config()

    # Build WidthEstimationConfig from Hydra config
    # Priority: map config > target config > default
    width_dict = cfg.map.get("width_estimation") or cfg.target.get(
        "width_estimation", {}
    )
    if width_dict:
        width_config = WidthEstimationConfig(
            adaptive_sampling=width_dict.get("adaptive_sampling", False),
            min_samples=width_dict.get("min_samples", 5),
            max_samples=width_dict.get("max_samples", 50),
            default_sample_interval=width_dict.get("default_sample_interval", 5.0),
        )
        logger.info(
            f"Width sampling config: adaptive={width_config.adaptive_sampling}, "
            f"interval={width_config.default_sample_interval}m, "
            f"max_samples={width_config.max_samples}"
        )
    else:
        width_config = WidthEstimationConfig()

    # Build StopLineConfig from Hydra config
    # Priority: map config > target config > default
    stopline_dict = cfg.map.get("stopline") or cfg.target.get("stopline", {})
    stopline_config = StopLineConfig(
        width=stopline_dict.get("width", 0.1) if stopline_dict else 0.1,
        carla_stop_line=(
            stopline_dict.get("carla_stop_line", False) if stopline_dict else False
        ),
    )
    logger.info(
        f"Stop line config: width={stopline_config.width}m, "
        f"carla_stop_line={stopline_config.carla_stop_line}"
    )

    # Build TrafficLightConfig from Hydra config
    # Priority: map config > target config > default
    tl_dict = cfg.map.get("traffic_light") or cfg.target.get("traffic_light", {})
    tl_config = TrafficLightConfig(
        offset_x=tl_dict.get("offset_x", 0.0) if tl_dict else 0.0,
        offset_y=tl_dict.get("offset_y", 0.0) if tl_dict else 0.0,
        offset_z=tl_dict.get("offset_z", 0.0) if tl_dict else 0.0,
        hdg_offset=(
            tl_dict.get("hdg_offset", TrafficLightConfig.hdg_offset)
            if tl_dict
            else TrafficLightConfig.hdg_offset
        ),
    )
    logger.info(
        f"Traffic light config: offset=({tl_config.offset_x}, "
        f"{tl_config.offset_y}, {tl_config.offset_z}), "
        f"hdg_offset={tl_config.hdg_offset}"
    )

    # Build ConversionConfig from parameters
    conversion_config = ConversionConfig(
        output_path=output_file,
        origin=OriginSpec(
            mgrs_code=mgrs_code,
            lat=origin_lat,
            lon=origin_lon,
        ),
        exclude_non_junction_signals=exclude_non_junction_signals,
        traffic_rule=traffic_rule,
        parampoly3=parampoly3_config,
        width_estimation=width_config,
        stopline=stopline_config,
        traffic_light=tl_config,
    )

    # mgrs_code is already stored in conversion_config.origin.mgrs_code;
    # no need to pass it as a separate argument.
    (
        opendrive,
        mapping,
        lanelet_to_road_and_lane,
        stop_line_mapping,
        skipped_stop_lines,
    ) = convert_lanelet2_to_opendrive(lanelet_map, conversion_config)

    logger.info("Conversion completed successfully!")
    logger.info(
        f"Road-Lanelet mapping: {len(mapping.road_to_lanelets)} roads, "
        f"{len(mapping.lanelet_to_road)} lanelets"
    )

    # Save mapping JSON and cross-validate against geometric mapping
    if conversion_config.output_path:
        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            _preprocessed_osm_path_for,
            validate_and_save_mapping,
        )

        xodr_path = Path(conversion_config.output_path)

        validate_and_save_mapping(
            lanelet_to_road_and_lane=lanelet_to_road_and_lane,
            lanelet_map=lanelet_map,
            roads=opendrive.roads,
            xodr_path=xodr_path,
            osm_path=input_map_path,
            mgrs_offset=(offset_x, offset_y),
            preprocessing_log=preprocessing_log_dict,
            stop_line_mapping=stop_line_mapping,
            skipped_stop_lines=skipped_stop_lines,
        )

        # Save preprocessed OSM next to XODR so that standalone `analyze`
        # can reproduce the same lanelet map without re-running preprocessing.
        if has_preprocessing:
            preprocessed_osm_dest = _preprocessed_osm_path_for(xodr_path)
            shutil.copy2(input_map_path, preprocessed_osm_dest)
            logger.info(f"Preprocessed OSM saved to: {preprocessed_osm_dest}")

        # Run ASAM QC analysis + mapping cross-validation
        from autoware_lanelet2_to_opendrive.analyze_xodr import run_analysis

        logger.info("Running post-conversion analysis...")
        run_analysis(
            xodr_path=xodr_path,
            osm_path=input_map_path,
        )


@hydra.main(version_base=None, config_path="conf", config_name="config")
def main(cfg: DictConfig) -> None:
    """
    Main entry point for the conversion script using Hydra.

    Usage:
        # Basic usage with default settings:
        uv run python -m autoware_lanelet2_to_opendrive.main input_map_path=/path/to/map.osm

        # With CARLA target:
        uv run python -m autoware_lanelet2_to_opendrive.main \\
            input_map_path=/path/to/map.osm target=carla

        # With custom map config:
        uv run python -m autoware_lanelet2_to_opendrive.main \\
            input_map_path=/path/to/map.osm map=my_map target=carla

        # Override output path:
        uv run python -m autoware_lanelet2_to_opendrive.main \\
            input_map_path=/path/to/map.osm output_map_path=/path/to/output.xodr
    """
    # Print resolved configuration
    if cfg.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.debug("Resolved configuration:")
        logger.debug(OmegaConf.to_yaml(cfg))

    # Get input/output paths
    input_path = Path(cfg.input_map_path)

    # Determine output path
    if cfg.output_map_path:
        output_path = Path(cfg.output_map_path)
    else:
        output_path = input_path.with_suffix(".xodr")

    try:
        preprocess_and_convert_with_hydra(
            cfg=cfg,
            lanelet2_file=input_path,
            output_file=output_path,
        )

    except FileNotFoundError as e:
        logger.error(f"File not found: {e}")
        sys.exit(1)
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error during conversion: {e}")
        if cfg.verbose:
            import traceback

            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
