"""Unit tests for the LaneAccess dataclass (to_xml serialization)."""

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneAccess
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneHeight, LaneSpeed
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    LaneType,
    SpeedUnit,
)
import lanelet2  # noqa: E402  (imported here so unit-test file remains self-contained)


def test_lane_access_to_xml_emits_attributes() -> None:
    """LaneAccess.to_xml() emits an <access> element with the three required attributes."""
    access = LaneAccess(s_offset=0.0, rule="allow", restriction="passengerCar")

    elem = access.to_xml()

    assert elem.tag == "access"
    assert elem.get("sOffset") == "0.0"
    assert elem.get("rule") == "allow"
    assert elem.get("restriction") == "passengerCar"


def test_lane_access_to_xml_supports_deny_rule() -> None:
    """LaneAccess supports rule="deny" with the same XML shape."""
    elem = LaneAccess(s_offset=0.0, rule="deny", restriction="pedestrian").to_xml()

    assert elem.tag == "access"
    assert elem.get("sOffset") == "0.0"
    assert elem.get("rule") == "deny"
    assert elem.get("restriction") == "pedestrian"


def test_lane_access_to_xml_serialises_nonzero_s_offset() -> None:
    """A non-zero s_offset is serialised as a float (not truncated to int)."""
    access = LaneAccess(s_offset=1.5, rule="allow", restriction="bicycle")
    assert access.to_xml().get("sOffset") == "1.5"


def test_lane_access_reexported_from_opendrive_dataclass() -> None:
    """LaneAccess is reachable from the back-compat ``opendrive_dataclass`` module."""
    from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import LaneAccess

    access = LaneAccess(s_offset=0.0, rule="allow", restriction="bicycle")
    assert access.restriction == "bicycle"


def test_lane_to_xml_emits_access_between_speed_and_height() -> None:
    """In <lane> output, <access> must appear after <speed> and before <height>."""
    lane = Lane(lane_id=-1, lane_type=LaneType.DRIVING)
    lane._add_speed(LaneSpeed(s_offset=0.0, max=50.0, unit=SpeedUnit.KMH))
    lane._add_access(LaneAccess(s_offset=0.0, rule="allow", restriction="passengerCar"))
    lane._add_height(LaneHeight(s_offset=0.0, inner=0.0, outer=0.0))

    children = [child.tag for child in lane.to_xml()]
    speed_idx = children.index("speed")
    access_idx = children.index("access")
    height_idx = children.index("height")
    assert speed_idx < access_idx < height_idx, children


def test_lane_with_no_accesses_emits_no_access_element() -> None:
    """Lanes without participant attributes must not gain an <access> element."""
    lane = Lane(lane_id=-1, lane_type=LaneType.DRIVING)
    xml_str = ET.tostring(lane.to_xml(), encoding="unicode")
    assert "<access" not in xml_str


def _make_lanelet(lanelet_id: int, attributes: dict) -> tuple:
    """Build a minimal lanelet + map for ``Lane.construct_from_lanelet`` tests."""
    left_points = [
        lanelet2.core.Point3d(lanelet_id * 100 + 1, 0.0, 0.0, 0.0),
        lanelet2.core.Point3d(lanelet_id * 100 + 2, 10.0, 0.0, 0.0),
    ]
    right_points = [
        lanelet2.core.Point3d(lanelet_id * 100 + 3, 0.0, 2.0, 0.0),
        lanelet2.core.Point3d(lanelet_id * 100 + 4, 10.0, 2.0, 0.0),
    ]
    left_bound = lanelet2.core.LineString3d(lanelet_id * 100 + 5, left_points)
    right_bound = lanelet2.core.LineString3d(lanelet_id * 100 + 6, right_points)
    lanelet = lanelet2.core.Lanelet(lanelet_id, left_bound, right_bound)
    for key, value in attributes.items():
        lanelet.attributes[key] = value
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


def test_construct_translates_vehicle_yes_to_passenger_car_allow() -> None:
    lanelet_map, lanelet = _make_lanelet(
        100, {"subtype": "road", "participant:vehicle": "yes"}
    )

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert len(lane.accesses) == 1
    assert lane.accesses[0].s_offset == 0.0
    assert lane.accesses[0].rule == "allow"
    assert lane.accesses[0].restriction == "passengerCar"


def test_construct_translates_pedestrian_no_to_deny() -> None:
    lanelet_map, lanelet = _make_lanelet(
        101, {"subtype": "road", "participant:pedestrian": "no"}
    )

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert len(lane.accesses) == 1
    assert lane.accesses[0].rule == "deny"
    assert lane.accesses[0].restriction == "pedestrian"


def test_construct_emits_one_access_per_participant() -> None:
    lanelet_map, lanelet = _make_lanelet(
        102,
        {
            "subtype": "road",
            "participant:vehicle": "yes",
            "participant:bicycle": "yes",
        },
    )

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert {(a.rule, a.restriction) for a in lane.accesses} == {
        ("allow", "passengerCar"),
        ("allow", "bicycle"),
    }


def test_construct_skips_unknown_participant() -> None:
    lanelet_map, lanelet = _make_lanelet(
        103, {"subtype": "road", "participant:emergency": "yes"}
    )

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.accesses == []


def test_construct_skips_non_yes_no_value() -> None:
    lanelet_map, lanelet = _make_lanelet(
        104, {"subtype": "road", "participant:vehicle": "true"}
    )

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.accesses == []


def test_construct_with_no_participant_attribute_yields_no_access() -> None:
    lanelet_map, lanelet = _make_lanelet(105, {"subtype": "road"})

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.accesses == []


def test_construct_translates_full_recognised_set() -> None:
    """All seven recognised Lanelet2 participants map to their OpenDRIVE values."""
    lanelet_map, lanelet = _make_lanelet(
        106,
        {
            "subtype": "road",
            "participant:vehicle": "yes",
            "participant:pedestrian": "yes",
            "participant:bicycle": "yes",
            "participant:bus": "yes",
            "participant:taxi": "yes",
            "participant:truck": "yes",
            "participant:motorcycle": "yes",
        },
    )

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    restrictions = {a.restriction for a in lane.accesses}
    assert restrictions == {
        "passengerCar",
        "pedestrian",
        "bicycle",
        "bus",
        "taxi",
        "truck",
        "motorcycle",
    }
    assert all(a.rule == "allow" for a in lane.accesses)


import subprocess  # noqa: E402
from pathlib import Path  # noqa: E402

NISHISHINJUKU_OSM = (Path(__file__).parent / "data" / "nishishinjuku.osm").resolve()


def _convert_nishishinjuku(tmp_path: Path) -> Path:
    """Run the CLI on the Shinjuku fixture and return the .xodr path."""
    out = tmp_path / "nishishinjuku.xodr"
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=nishishinjuku",
            "target=carla",
            f"input_map_path={NISHISHINJUKU_OSM}",
            f"output_map_path={out}",
        ],
        check=True,
    )
    return out


def test_nishishinjuku_emits_passenger_car_access_for_vehicle_lanelets(
    tmp_path: Path,
) -> None:
    """At least one <lane> in the converted output carries the new <access>."""
    out = _convert_nishishinjuku(tmp_path)
    root = ET.parse(out).getroot()

    matches = root.findall(".//lane/access[@rule='allow'][@restriction='passengerCar']")
    assert len(matches) >= 1, (
        "nishishinjuku has 602 lanelets with participant:vehicle=yes; "
        "the converted .xodr should contain at least one "
        "<access rule='allow' restriction='passengerCar'/>"
    )

    for access in matches:
        assert access.get("sOffset") == "0.0"
