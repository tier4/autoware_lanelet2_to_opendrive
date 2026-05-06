"""Unit tests for the LaneAccess dataclass (to_xml serialization).

Spec: docs/superpowers/specs/2026-05-06-issue-468-lane-access-design.md
"""

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneAccess
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneHeight, LaneSpeed
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    LaneType,
    SpeedUnit,
)


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
    access = LaneAccess(s_offset=0.0, rule="deny", restriction="pedestrian")
    xml_str = ET.tostring(access.to_xml(), encoding="unicode")

    assert xml_str == '<access sOffset="0.0" rule="deny" restriction="pedestrian"/>'


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
