"""Unit tests for the LaneAccess dataclass (to_xml serialization).

Spec: docs/superpowers/specs/2026-05-06-issue-468-lane-access-design.md
"""

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneAccess


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
