"""End-to-end tests: adjacent connecting lanelets that come from or lead to
different roads become separate connecting roads, and every lane link
resolves in its road-level linked road.

``junction_fanout_mini.osm``: three adjacent connecting lanelets lead to two
outgoing roads (lane -1 into a one-lane road, lanes -2 and -3 into a two-lane
road).  Bundled into one connecting road they carried one road-level
``<successor>`` (the one-lane road) while lanes -2 and -3 emitted the
``<successor id>`` they have in the two-lane road; CARLA read those against
the one-lane road, linked lane -2 to its only lane and segfaulted on the
lane that lane -3 names.

``junction_fanin_mini.osm`` is the mirror image: a one-lane and a two-lane
road merge into three adjacent connecting lanelets that lead to one
three-lane road, so the bundled road named one ``<predecessor>`` while two
lanes carried ids from the other incoming road.
"""

import subprocess
from pathlib import Path
from typing import Any, Iterator

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.qc_validate import (
    load_ignore_patterns,
    validate,
)

DATA = Path(__file__).parent / "data"
FAN_OUT = (DATA / "junction_fanout_mini.osm").resolve()
FAN_IN = (DATA / "junction_fanin_mini.osm").resolve()
BOTH = pytest.mark.parametrize("fixture", [FAN_OUT, FAN_IN], ids=["fan-out", "fan-in"])


def _run_convert(tmp_path: Path, fixture: Path) -> Path:
    """Run the ``convert`` CLI on ``fixture`` and return the output path."""
    out = tmp_path / fixture.with_suffix(".xodr").name
    # Same minimal map config as walkway_mini / speed_change_mini.
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=example_mgrs_offset",
            "target=carla",
            f"input_map_path={fixture}",
            f"output_map_path={out}",
        ],
        check=True,
    )
    return out


def _lane_ids(road: ET._Element, at_start: bool) -> set[int]:
    sections = road.findall("lanes/laneSection")
    section = sections[0] if at_start else sections[-1]
    return {
        int(lane.get("id")) for lane in section.iter("lane") if lane.get("id") != "0"
    }


def _road_link(road: ET._Element, kind: str) -> tuple[str, str] | None:
    link = road.find(f"link/{kind}")
    if link is None:
        return None
    return link.get("elementType"), link.get("elementId")


def _lane_links(root: ET._Element) -> Iterator[tuple[str, int, str, str, int | None]]:
    """(road id, lane id, kind, linked road id, lane link id) for road-to-road links."""
    roads = {road.get("id"): road for road in root.findall("road")}
    for road in roads.values():
        for kind in ("predecessor", "successor"):
            link = _road_link(road, kind)
            if link is None or link[0] != "road":
                continue
            sections = road.findall("lanes/laneSection")
            section = sections[0] if kind == "predecessor" else sections[-1]
            for lane in section.iter("lane"):
                if lane.get("type") != "driving":
                    continue
                lane_link = lane.find(f"link/{kind}")
                yield (
                    road.get("id"),
                    int(lane.get("id")),
                    kind,
                    link[1],
                    None if lane_link is None else int(lane_link.get("id")),
                )


def _connecting_roads(
    root: ET._Element,
) -> tuple[dict[str, ET._Element], list[ET._Element]]:
    roads = {road.get("id"): road for road in root.findall("road")}
    return roads, [road for road in roads.values() if road.get("junction") != "-1"]


def test_fan_out_lanes_become_two_connecting_roads(tmp_path: Path) -> None:
    """One connecting road per outgoing road, both fed by the three-lane road."""
    roads, connecting = _connecting_roads(
        ET.parse(str(_run_convert(tmp_path, FAN_OUT))).getroot()
    )
    assert len(connecting) == 2
    by_lane_count = {len(_lane_ids(road, at_start=True)): road for road in connecting}
    assert set(by_lane_count) == {1, 2}
    for lanes, road in by_lane_count.items():
        successor = _road_link(road, "successor")
        assert successor is not None and successor[0] == "road"
        assert len(_lane_ids(roads[successor[1]], at_start=True)) == lanes
    predecessors = {_road_link(road, "predecessor") for road in connecting}
    assert len(predecessors) == 1
    predecessor = next(iter(predecessors))
    assert predecessor is not None and predecessor[0] == "road"
    assert len(_lane_ids(roads[predecessor[1]], at_start=False)) == 3


def test_fan_in_lanes_become_two_connecting_roads(tmp_path: Path) -> None:
    """One connecting road per incoming road, both leading into the three-lane road."""
    roads, connecting = _connecting_roads(
        ET.parse(str(_run_convert(tmp_path, FAN_IN))).getroot()
    )
    assert len(connecting) == 2
    by_lane_count = {len(_lane_ids(road, at_start=True)): road for road in connecting}
    assert set(by_lane_count) == {1, 2}
    for lanes, road in by_lane_count.items():
        predecessor = _road_link(road, "predecessor")
        assert predecessor is not None and predecessor[0] == "road"
        assert len(_lane_ids(roads[predecessor[1]], at_start=False)) == lanes
    successors = {_road_link(road, "successor") for road in connecting}
    assert len(successors) == 1
    successor = next(iter(successors))
    assert successor is not None and successor[0] == "road"
    assert len(_lane_ids(roads[successor[1]], at_start=True)) == 3


@BOTH
def test_every_lane_link_resolves_in_the_linked_road(
    tmp_path: Path, fixture: Path
) -> None:
    """Every lane of a road-to-road link is linked, into an existing lane."""
    root = ET.parse(str(_run_convert(tmp_path, fixture))).getroot()
    roads = {road.get("id"): road for road in root.findall("road")}
    for road_id, lane_id, kind, linked, target in _lane_links(root):
        assert target is not None, f"road {road_id} lane {lane_id} has no {kind}"
        assert target in _lane_ids(
            roads[linked], at_start=(kind == "successor")
        ), f"road {road_id} lane {lane_id} {kind} -> road {linked} lane {target}"


@BOTH
def test_junction_connections_cover_every_incoming_lane(
    tmp_path: Path, fixture: Path
) -> None:
    """Each incoming road enters the junction through connections covering all its lanes."""
    root = ET.parse(str(_run_convert(tmp_path, fixture))).getroot()
    roads = {road.get("id"): road for road in root.findall("road")}
    (junction,) = root.findall("junction")
    connections = junction.findall("connection")
    assert len(connections) == 2
    from_lanes: dict[str, list[int]] = {}
    for connection in connections:
        for link in connection.findall("laneLink"):
            from_lanes.setdefault(connection.get("incomingRoad"), []).append(
                int(link.get("from"))
            )
    for incoming, lanes in from_lanes.items():
        assert sorted(lanes) == sorted(_lane_ids(roads[incoming], at_start=False))
    assert sum(len(lanes) for lanes in from_lanes.values()) == 3


@BOTH
def test_fixture_passes_qc_validate(tmp_path: Path, fixture: Path) -> None:
    """The emitted OpenDRIVE must pass qc-framework with zero ERRORs."""
    errors = validate(_run_convert(tmp_path, fixture), load_ignore_patterns())
    assert errors == 0, f"qc-framework reported {errors} ERROR(s)"


@BOTH
def test_lane_links_are_continuous_in_carla(tmp_path: Path, fixture: Path) -> None:
    """CARLA follows every road-to-road successor link onto the linked lane without a jump."""
    carla = pytest.importorskip("carla")
    out = _run_convert(tmp_path, fixture)
    root = ET.parse(str(out)).getroot()
    carla_map = carla.Map(fixture.stem, out.read_text())
    last: dict[tuple[str, int], Any] = {}
    for waypoint in carla_map.generate_waypoints(1.0):
        key = (str(waypoint.road_id), waypoint.lane_id)
        if key not in last or waypoint.s > last[key].s:
            last[key] = waypoint
    for road_id, lane_id, kind, linked, target in _lane_links(root):
        if kind != "successor":
            continue
        waypoint = last[(road_id, lane_id)]
        following = waypoint.next(1.0)
        assert (
            len(following) == 1
        ), f"road {road_id} lane {lane_id}: {len(following)} next waypoints"
        assert (str(following[0].road_id), following[0].lane_id) == (linked, target)
        assert (
            waypoint.transform.location.distance(following[0].transform.location) < 1.5
        )
