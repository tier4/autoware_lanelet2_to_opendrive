"""A record that restates its predecessor's polynomial must not be written.

``performance.avoid_redundant_info`` calls two consecutive records redundant
when the difference of their shifted cubics has every coefficient within
1e-6 -- the same curve written twice with different offsets
(``qc_opendrive/checks/performance/performance_avoid_redundant_info.py`` with
``base/utils.are_same_equations``). On nishishinjuku that is the whole ASAM
warning population: 341 of 342 issues, 108 in lane ``<width>`` and 233 in
road ``<elevation>``.

Such a record can be dropped outright, and only such a record can: a profile
record runs until the next one starts, so deleting one that states the same
polynomial leaves the covered s-range and the value at every s unchanged.
Deleting anything else changes the surface, and a lane whose ``<width>``
records do not cover its whole ``s`` makes consumers abort -- which is why
this is a redundancy test, not a thinning test.

The shifted case is the one worth pinning: ``a=3.0, b=0.1`` at sOffset 0 and
``a=4.0, b=0.1`` at sOffset 10 are the same line, and a comparison of raw
coefficients would keep both.
"""

from autoware_lanelet2_to_opendrive.opendrive.elevation import (
    Elevation,
    ElevationProfile,
)
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneWidth
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import LaneType


def _lane(widths) -> Lane:
    lane = Lane(lane_id=1, lane_type=LaneType.DRIVING)
    for width in widths:
        lane.widths.append(width)
    return lane


def _width_offsets(lane: Lane):
    return [float(w.get("sOffset")) for w in lane.to_xml().findall("width")]


def _elevation_offsets(profile: ElevationProfile):
    return [float(e.get("s")) for e in profile.to_xml().findall("elevation")]


def test_repeated_constant_width_is_written_once() -> None:
    lane = _lane([LaneWidth(s_offset=0.0, a=3.5), LaneWidth(s_offset=10.0, a=3.5)])
    assert _width_offsets(lane) == [0.0]


def test_width_that_restates_a_shifted_polynomial_is_dropped() -> None:
    """Same line, written twice against different offsets."""
    lane = _lane(
        [
            LaneWidth(s_offset=0.0, a=3.0, b=0.1),
            LaneWidth(s_offset=10.0, a=4.0, b=0.1),
        ]
    )
    assert _width_offsets(lane) == [0.0]


def test_width_that_states_a_different_polynomial_is_kept() -> None:
    lane = _lane(
        [
            LaneWidth(s_offset=0.0, a=3.0, b=0.1),
            LaneWidth(s_offset=10.0, a=4.0, b=0.2),
        ]
    )
    assert _width_offsets(lane) == [0.0, 10.0]


def test_only_the_run_collapses_not_the_whole_lane() -> None:
    """A redundant pair in the middle leaves the records around it alone."""
    lane = _lane(
        [
            LaneWidth(s_offset=0.0, a=3.0),
            LaneWidth(s_offset=5.0, a=3.0),
            LaneWidth(s_offset=10.0, a=3.0),
            LaneWidth(s_offset=15.0, a=3.4),
            LaneWidth(s_offset=20.0, a=3.4),
            LaneWidth(s_offset=25.0, a=3.9),
        ]
    )
    assert _width_offsets(lane) == [0.0, 15.0, 25.0]


def test_the_first_record_is_never_dropped() -> None:
    """Even a lane of one record keeps it: s=0 has to be covered."""
    lane = _lane([LaneWidth(s_offset=0.0, a=3.5)])
    assert _width_offsets(lane) == [0.0]


def test_repeated_elevation_is_written_once() -> None:
    profile = ElevationProfile(
        elevations=[
            Elevation(s=0.0, a=12.0, b=0.01),
            Elevation(s=20.0, a=12.2, b=0.01),
        ]
    )
    assert _elevation_offsets(profile) == [0.0]


def test_elevation_that_bends_is_kept() -> None:
    profile = ElevationProfile(
        elevations=[
            Elevation(s=0.0, a=12.0, b=0.01),
            Elevation(s=20.0, a=12.2, b=0.01, c=1e-4),
        ]
    )
    assert _elevation_offsets(profile) == [0.0, 20.0]
