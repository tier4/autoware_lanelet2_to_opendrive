"""Unit tests for lanelet-constraint sweeper bindings.

The binding logic is pure graph lookup, exercised here with a mock routing
graph and lanelet map (the module import needs ``lanelet2`` present, so these
run in CI rather than on a bare host).
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from autoware_carla_scenario.sweeper.bindings import (
    AdjacentLaneletBinding,
    parse_binding,
)


def _lanelet(lanelet_id: int) -> MagicMock:
    ll = MagicMock()
    ll.id = lanelet_id
    return ll


def _map(*ids: int) -> MagicMock:
    lanelets = {i: _lanelet(i) for i in ids}
    lanelet_map = MagicMock()
    lanelet_map.laneletLayer.__getitem__.side_effect = lambda i: lanelets[i]
    return lanelet_map


def _routing_graph(right: dict[int, int] | None = None) -> MagicMock:
    right = right or {}
    rg = MagicMock()
    rg.right.side_effect = lambda ll: _lanelet(right[ll.id]) if ll.id in right else None
    rg.left.side_effect = lambda ll: None
    return rg


def test_parse_and_resolve_right_neighbour() -> None:
    binding = parse_binding(
        "scenario.npc_1_spawn_lanelet_id",
        {"type": "adjacent_lanelet", "side": "right"},
    )
    assert isinstance(binding, AdjacentLaneletBinding)
    assert binding.target_key == "scenario.npc_1_spawn_lanelet_id"

    result = binding.resolve(183, _map(183, 184), _routing_graph({183: 184}))
    assert result.value == 184


def test_missing_neighbour_raises() -> None:
    binding = AdjacentLaneletBinding("scenario.npc_1_spawn_lanelet_id", side="right")
    with pytest.raises(ValueError, match="no right adjacent lane"):
        binding.resolve(184, _map(184), _routing_graph({}))


def test_invalid_side_rejected() -> None:
    with pytest.raises(ValueError, match="left.*right"):
        AdjacentLaneletBinding("x", side="up")
