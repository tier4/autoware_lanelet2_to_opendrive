"""The emitted file must state the successions the source map states.

Geometry has ``symmetric_point_to_segment_max`` and conformance has the ASAM
bundle, but nothing in the repository measures what the map means for
driving: which lane you may reach from which. A conversion can place every
reference line to a fraction of a millimetre and still have dropped a turn,
and no gate we own would notice.

These tests pin the instrument and the figure. Two of them are hand-built
files that fix the two readings a route measurement gets wrong if it is
written naively:

* **Travel direction is the road's ``rule``.** Under left-hand traffic a
  positive-id (left) lane travels along +s and so reads its successor at the
  road's successor end; under right-hand traffic the negative-id lanes do.
  Reading the wrong end reports every transition in the file as missing.
* **A road with no backing lanelet is walked through, not counted.** The
  converter inserts a synthetic connector wherever the map states a
  succession directly. Counting it as a node reports each connector as a
  transition the map never stated *and* the succession it stands for as
  missing -- which on the reference map was a 650-of-974 figure that measured
  only the instrument.
"""

from pathlib import Path

import pytest
from autoware_lanelet2_to_opendrive.route_fidelity import (
    EmittedNetwork,
    measure,
    transitions_required,
)

NISHISHINJUKU_MIN_SUCCESSIONS = 900
"""Below this the measurement did not run; the map states roughly 970."""

MAX_MISSING_SHARE = 0.02
"""The regression line, not a target.

Measured 1.55% here -- 15 of 969 successions on ``nishishinjuku`` as this
branch's base converts it. The same measurement reads 0.93% once the junction
and fork fixes land, so the bound is set above the worse of the two rather
than retightened every time the base moves.
"""


def _mapping_beside(xodr: Path) -> Path:
    """The conversion's mapping sidecar for ``xodr``.

    The converter names it after the output path it was given, and the
    nishishinjuku fixture converts to a staged name and renames the map
    afterwards -- so the sidecar can still carry the staged name. Both are
    accepted rather than pinning one, because which one appears is a property
    of the fixture, not of the thing under test.
    """
    direct = xodr.with_suffix(".mapping.json")
    if direct.is_file():
        return direct
    siblings = sorted(xodr.parent.glob("*.mapping.json"))
    assert (
        len(siblings) == 1
    ), f"expected one mapping sidecar beside {xodr}, found {siblings}"
    return siblings[0]


def _write(tmp_path: Path, name: str, body: str) -> Path:
    path = tmp_path / name
    path.write_text(body, encoding="utf-8")
    return path


def _road(road_id: int, rule: str, lanes: str, link: str = "") -> str:
    return f"""
  <road id="{road_id}" length="10.0" rule="{rule}">
    <link>{link}</link>
    <lanes>
      <laneSection s="0.0">{lanes}</laneSection>
    </lanes>
  </road>"""


def _lane(lane_id: int, link: str = "") -> str:
    side = "left" if lane_id > 0 else "right"
    return f"""
      <{side}>
        <lane id="{lane_id}" type="driving" level="false">
          <link>{link}</link>
        </lane>
      </{side}>"""


def test_a_left_hand_lane_leaves_by_the_successor_end(tmp_path: Path) -> None:
    """A positive-id lane under LHT travels along +s."""
    body = (
        '<?xml version="1.0"?><OpenDRIVE>'
        + _road(1, "LHT", _lane(1))
        + _road(2, "RHT", _lane(1))
        + "</OpenDRIVE>"
    )
    network = EmittedNetwork(_write(tmp_path, "rule.xodr", body))
    assert network.forward_end(1, 1) == "successor"
    assert network.forward_end(1, -1) == "predecessor"
    assert network.forward_end(2, 1) == "predecessor"
    assert network.forward_end(2, -1) == "successor"


def test_a_connector_with_no_backing_lanelet_is_walked_through(
    tmp_path: Path,
) -> None:
    """Road 1 -> connector 2 -> road 3 must be reported as 1 -> 3."""
    body = (
        '<?xml version="1.0"?><OpenDRIVE>'
        + _road(
            1,
            "LHT",
            _lane(1, '<successor id="1"/>'),
            '<successor elementType="road" elementId="2" contactPoint="start"/>',
        )
        + _road(
            2,
            "LHT",
            _lane(1, '<successor id="1"/>'),
            '<predecessor elementType="road" elementId="1" contactPoint="end"/>'
            '<successor elementType="road" elementId="3" contactPoint="start"/>',
        )
        + _road(
            3,
            "LHT",
            _lane(1),
            '<predecessor elementType="road" elementId="2" contactPoint="end"/>',
        )
        + "</OpenDRIVE>"
    )
    network = EmittedNetwork(_write(tmp_path, "connector.xodr", body))

    assert network.transitions(walk_through={2}) == {((1, 1), (3, 1))}
    # Counting the connector as a node is the mistake this pins against.
    assert ((1, 1), (2, 1)) in network.transitions(walk_through=set())


# ---------------------------------------------------------------------------
# Which successions need a transition at all
#
# A lanelet can be emitted on several roads: the mapping names one
# ``(road, lane)`` for it and lists the rest. Two consecutive lanelets inside a
# junction are routinely emitted *together* on one connecting road that spans
# the whole manoeuvre, and each of them on other connecting roads as well.
# Such a succession is realised inside the road they share. It needs no link,
# and no link exists for it.
# ---------------------------------------------------------------------------


def test_lanelets_on_different_roads_need_a_transition() -> None:
    """Two lanelets with one slot each, on different roads: one transition."""
    assert transitions_required([(1, 1)], [(2, 1)]) == {((1, 1), (2, 1))}


def test_lanelets_on_the_same_slot_need_none() -> None:
    """Both lanelets were merged into one lane of one road."""
    assert transitions_required([(1, 1)], [(1, 1)]) is None


def test_any_slot_pair_realises_a_succession() -> None:
    """A stage duplicated onto two roads may be reached on either copy."""
    assert transitions_required([(1, 1)], [(2, 1), (3, 1)]) == {
        ((1, 1), (2, 1)),
        ((1, 1), (3, 1)),
    }


@pytest.mark.xfail(
    strict=True,
    reason="a shared slot is only recognised when it is the lanelets' only slot",
)
def test_lanelets_sharing_one_of_several_slots_need_no_transition() -> None:
    """Sharing a slot is enough; the other slots do not make it a requirement.

    Road 10 spans a manoeuvre through both lanelets. The second lanelet is
    also the first stage of another manoeuvre, emitted on road 11. Roads 10
    and 11 are alternatives leaving the same junction entry, not consecutive
    roads, so no transition from road 10 to road 11 exists or should. Asking
    for one reports the succession as missing although road 10 carries it
    whole. Measured on a 36,249-lanelet map: 443 of the 446 successions
    reported missing were this case, and the other 3 were real.
    """
    assert transitions_required([(10, 1)], [(11, 1), (10, 1)]) is None
    assert transitions_required([(10, 1), (12, 1)], [(10, 1)]) is None


def test_shipped_conversion_preserves_the_stated_successions(
    nishishinjuku_xodr: Path,
) -> None:
    """The measured loss on the reference map stays where it is understood."""
    source = Path(__file__).parent / "data" / "nishishinjuku.osm"
    result = measure(source, nishishinjuku_xodr, _mapping_beside(nishishinjuku_xodr))

    assert result.stated >= NISHISHINJUKU_MIN_SUCCESSIONS, (
        f"only {result.stated} successions were read from the map;"
        " the measurement did not run"
    )
    assert result.missing_share < MAX_MISSING_SHARE, (
        f"{len(result.missing)} of {result.stated} successions"
        f" ({100.0 * result.missing_share:.2f}%) are absent from the file"
    )
