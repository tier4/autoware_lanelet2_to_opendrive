"""Equivalence tests for controller -> junction association.

``_assign_controllers_to_junctions`` used to rescan every road and every
controller entry once per junction (O(J x R) + O(J x C x E)), which on a
city-scale map means tens of millions of comparisons. The scans were replaced
by two indices built once. These tests pin the optimised implementation to the
original, straightforward one.
"""

from __future__ import annotations

import random
from typing import Dict, List
from unittest.mock import MagicMock

from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter
from autoware_lanelet2_to_opendrive.opendrive.junction import Connection, Junction
from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint
from autoware_lanelet2_to_opendrive.opendrive.signal import ControlEntry, Controller
from autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers import (
    SignalsAndControllers,
)


def _naive_controller_ids(
    signals_and_controllers: SignalsAndControllers,
    junction: Junction,
    all_roads: List,
) -> List[int]:
    """The pre-optimisation implementation, kept verbatim as the oracle."""
    junction_incoming_road_ids = {conn.incoming_road for conn in junction.connections}
    junction_connecting_road_ids = {
        conn.connecting_road for conn in junction.connections
    }
    junction_roads_by_attribute = {
        road.id for road in all_roads if road.junction == junction.id
    }
    junction_related_road_ids = (
        junction_incoming_road_ids
        | junction_connecting_road_ids
        | junction_roads_by_attribute
    )

    junction_controller_ids: List[int] = []
    for controller in signals_and_controllers.controllers:
        if controller.controls:
            controller_road_ids = set()
            for control_entry in controller.controls:
                signal_road_id = signals_and_controllers.signal_to_road_id.get(
                    control_entry.signal_id
                )
                if signal_road_id is not None:
                    controller_road_ids.add(signal_road_id)
            if controller_road_ids & junction_related_road_ids:
                junction_controller_ids.append(controller.id)
    return junction_controller_ids


def _make_road(road_id: int, junction_id: int):
    road = MagicMock()
    road.id = road_id
    road.junction = junction_id
    return road


def _build_scenario(seed: int):
    """Build a randomised but reproducible roads/junctions/controllers set."""
    rng = random.Random(seed)

    num_roads = 40
    num_junctions = 6
    num_signals = 30
    num_controllers = 12

    roads = [
        _make_road(road_id, rng.choice([-1, -1, *range(num_junctions)]))
        for road_id in range(num_roads)
    ]

    junctions = []
    for junction_id in range(num_junctions):
        connections = [
            Connection(
                id=index,
                incoming_road=rng.randrange(num_roads),
                connecting_road=rng.randrange(num_roads),
                contact_point=ContactPoint.START,
            )
            for index in range(rng.randrange(0, 4))
        ]
        junctions.append(Junction(id=junction_id, connections=connections))

    signal_to_road_id: Dict[int, int] = {
        signal_id: rng.randrange(num_roads) for signal_id in range(num_signals)
    }
    # A signal that maps to no road must be ignored, like in production.
    signal_to_road_id.pop(0, None)

    controllers = []
    for controller_id in range(num_controllers):
        controls = [
            ControlEntry(signal_id=rng.randrange(num_signals))
            for _ in range(rng.randrange(0, 4))
        ]
        controllers.append(
            Controller(id=controller_id, name=f"C{controller_id}", controls=controls)
        )

    sac = SignalsAndControllers(
        controllers=controllers,
        signal_to_road_id=signal_to_road_id,
    )
    return sac, junctions, roads


def _run(sac, junctions, roads) -> Dict[int, List[int]]:
    converter = _Lanelet2ToOpenDRIVEConverter(
        lanelet_map=MagicMock(), config=MagicMock()
    )
    converter._assign_controllers_to_junctions(sac, junctions, roads)
    return {junction.id: list(junction.controller_ids) for junction in junctions}


def test_controller_assignment_matches_naive_implementation():
    """Indexed assignment must reproduce the naive scan exactly, order included."""
    for seed in range(20):
        sac, junctions, roads = _build_scenario(seed)
        expected = {
            junction.id: _naive_controller_ids(sac, junction, roads)
            for junction in junctions
        }

        assert _run(sac, junctions, roads) == expected, f"mismatch for seed {seed}"


def test_controller_with_no_control_entries_is_never_assigned():
    """An empty ``controls`` list is skipped, as in the original loop."""
    roads = [_make_road(0, 5)]
    junction = Junction(id=5, connections=[])
    sac = SignalsAndControllers(
        controllers=[
            Controller(id=0, name="C0", controls=[]),
            Controller(id=1, name="C1", controls=[ControlEntry(signal_id=7)]),
        ],
        signal_to_road_id={7: 0},
    )

    assert _run(sac, [junction], roads) == {5: [1]}


def test_controller_order_follows_controller_list_not_id():
    """Emitted IDs keep the controller-list order even when IDs are unsorted."""
    roads = [_make_road(0, 5)]
    junction = Junction(id=5, connections=[])
    sac = SignalsAndControllers(
        controllers=[
            Controller(id=42, name="C42", controls=[ControlEntry(signal_id=7)]),
            Controller(id=7, name="C7", controls=[ControlEntry(signal_id=8)]),
        ],
        signal_to_road_id={7: 0, 8: 0},
    )

    assert _run(sac, [junction], roads) == {5: [42, 7]}
