"""Structural invariants of the emitted OpenDRIVE, checked on real output.

Two halves, because either alone is worthless:

* The first half feeds the checker documents that each break exactly one
  invariant, so a checker that has quietly stopped looking fails here.  A
  clean-output assertion alone cannot tell "nothing is wrong" from "nothing
  is being checked".
* The second half runs the checker over genuinely converted maps.

The invariants are about internal consistency -- every link, lane link and
``<connection>`` resolves to something that exists, and a reference line is
as long and as continuous as it declares -- so a legitimate change to the
geometry or the junction decomposition leaves them true.  That is the point:
a golden file would have to be regenerated after such a change, and a
regenerated golden file asserts nothing.

**Measured against a real defect.**  Before "split connecting roads by their
incoming and outgoing roads" the converter bundled laterally adjacent
connecting lanelets into one connecting road and let its lanes keep successor
ids belonging to a *different* outgoing road.  CARLA read those against the
road the road-level link named, and segfaulted.  Run over that output the
checker reports it directly::

    [lane_link_target_missing] road 3 lane -3/successor:
        names lane -2 of road 2 at its start, which has lanes ['-1']

**What it does not catch.**  Every invariant here is about a reference that
resolves.  A *lost* maneuver -- a connecting road that should have been
emitted twice and was emitted once -- leaves no dangling reference behind,
and the checker reports nothing on it.  Detecting a loss needs the source map
to compare against, which is the separate semantics measurement, not this.

A second, narrower blind spot is worth stating because it is the largest
known population of the defect this module is named for: the lane-link checks
examine **road-typed ends only**.  Lane ``<link>``s at junction-typed ends --
which the format says to omit, and which the converter emits in the hundreds
on Nishi-Shinjuku -- are out of scope by construction, not because they are
clean.  See ``xodr_invariants``'s module docstring.

**Candidate invariants that were measured and rejected.**  These are all
false on Nishi-Shinjuku, so they are findings to look into rather than
invariants to assert.

* "a road links only to a road whose endpoint is within 1 cm" -- hundreds of
  violations, the worst over 10 m apart.  A reference line is anchored to its
  road's outermost lane boundary, so two linked roads with different lane
  counts legitimately start metres apart.  **Kept in weakened form**: where
  the two sides carry the same driving lanes the two reference lines describe
  the same edge and ought to meet, so ``linked_endpoints_apart`` asserts it
  there.  Even restricted that far it is not yet clean -- 22 violations over
  15 joins, worst 0.071 m, pinned below -- and the restriction is on the lane
  sets rather than on connecting-road-ness: measured, 19 of the 22 are
  regular-to-regular and 3 are connecting-to-regular.
* "every driving lane with a road-level successor has a lane-level one".  The
  cases are lane *gains*: a road with three driving lanes whose predecessor
  has two, so the third lane correctly has no predecessor.  The rule is
  wrong, not the output.
* "every connecting road is named by some ``<connection>``".  The converter
  logs each one: the connecting lanelet has no routing-graph predecessor in
  the source map.  That is a property of the input.

The per-candidate counts above are deliberately approximate.  They were taken
on a pre-migration basis whose commits are not reachable from this branch, so
quoting them to three digits would invite a comparison nobody can make; the
one figure stated exactly -- the 22 -- is measured on this branch and asserted
by a test.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path
from typing import List

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.xodr_invariants import (
    Violation,
    check_invariants,
    format_violations,
)

DATA = Path(__file__).parent / "data"
PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _convert(source: Path, out_dir: Path, map_config: str) -> Path:
    """Run the ``convert`` CLI on *source* and return the emitted ``.xodr``."""
    out = out_dir / source.with_suffix(".xodr").name
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            f"map={map_config}",
            "target=carla",
            f"input_map_path={source}",
            f"output_map_path={out}",
        ],
        cwd=PROJECT_ROOT,
        check=True,
    )
    return out


def _doc(body: str) -> ET._Element:
    """Wrap *body* in a minimal ``<OpenDRIVE>`` document."""
    return ET.fromstring(
        '<OpenDRIVE><header revMajor="1" revMinor="7"/>' + body + "</OpenDRIVE>"
    )


def _road(
    rid: str,
    *,
    junction: str = "-1",
    length: str = "10.0",
    link: str = "",
    lanes: str = '<left><lane id="1" type="driving"/></left>',
    x: str = "0",
    plan_view: str | None = None,
) -> str:
    """Render one ``<road>`` with the parts a test wants to vary.

    ``x`` places a default 10 m straight reference line, so two roads meant to
    be linked can be made to actually meet (``x="0"`` then ``x="10"``) without
    each test spelling out a ``<planView>``.
    """
    plan_view = plan_view or f'<geometry s="0" x="{x}" y="0" hdg="0" length="10.0"/>'
    return (
        f'<road id="{rid}" length="{length}" junction="{junction}">'
        f"<link>{link}</link>"
        f"<planView>{plan_view}</planView>"
        f'<lanes><laneSection s="0">{lanes}</laneSection></lanes>'
        f"</road>"
    )


def _codes(root: ET._Element) -> list[str]:
    """Return the violation codes the checker reports on *root*."""
    return [violation.code for violation in check_invariants(root)]


# --------------------------------------------------------------------------
# The checker is not vacuous: each document breaks exactly one invariant.
# --------------------------------------------------------------------------


def test_clean_document_has_no_violations() -> None:
    """The baseline the negative cases are derived from is itself clean."""
    root = _doc(
        _road(
            "1",
            link='<successor elementType="road" elementId="2" contactPoint="start"/>',
        )
        + _road(
            "2",
            x="10",
            link='<predecessor elementType="road" elementId="1" contactPoint="end"/>',
        )
    )
    assert check_invariants(root) == []


def test_link_to_a_road_that_does_not_exist() -> None:
    root = _doc(_road("1", link='<successor elementType="road" elementId="99"/>'))
    assert _codes(root) == ["link_target_missing"]


def test_link_to_a_junction_that_does_not_exist() -> None:
    root = _doc(_road("1", link='<successor elementType="junction" elementId="1000"/>'))
    assert _codes(root) == ["link_target_missing"]


def test_lane_names_a_successor_that_is_not_in_the_linked_road() -> None:
    """The shape of the defect CARLA segfaulted on."""
    root = _doc(
        _road(
            "1",
            link='<successor elementType="road" elementId="2" contactPoint="start"/>',
            lanes=(
                '<left><lane id="1" type="driving">'
                '<link><successor id="2"/></link></lane></left>'
            ),
        )
        + _road("2", x="10")
    )
    violations = check_invariants(root)
    assert [v.code for v in violations] == ["lane_link_target_missing"]
    assert "names lane 2 of road 2" in violations[0].detail


def test_connection_names_a_connecting_road_as_incoming() -> None:
    root = _doc(
        _road("1", junction="1000")
        + _road("2", junction="1000")
        + '<junction id="1000">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/>'
        "</connection></junction>"
    )
    assert _codes(root) == ["incoming_road_is_connecting"]


def test_connecting_road_declares_a_different_junction() -> None:
    root = _doc(
        _road("1") + _road("2", junction="1001") + '<junction id="1000">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/>'
        "</connection></junction>"
    )
    assert _codes(root) == ["connecting_road_junction_mismatch"]


def test_lane_link_names_a_lane_the_connecting_road_does_not_have() -> None:
    root = _doc(
        _road("1") + _road("2", junction="1000") + '<junction id="1000">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="7"/>'
        "</connection></junction>"
    )
    assert _codes(root) == ["lane_link_target_missing"]


def test_plain_road_used_as_a_connecting_road() -> None:
    root = _doc(
        _road("1") + _road("2") + '<junction id="1000">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/>'
        "</connection></junction>"
    )
    # A road that is not a connecting road necessarily also carries
    # junction="-1", so it disagrees with the junction that claims it; both
    # statements are true of this document.
    assert set(_codes(root)) == {
        "connecting_road_junction_mismatch",
        "plain_road_used_as_connecting",
    }


def test_connecting_road_claimed_by_two_junctions() -> None:
    root = _doc(
        _road("1") + _road("2", junction="1000") + '<junction id="1000">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/></connection></junction>'
        '<junction id="1001">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/></connection></junction>'
    )
    # The second junction also disagrees about @junction, and both junctions
    # claim road 1's end between them. All three are separate statements about
    # the same document; all three are expected.
    assert set(_codes(root)) == {
        "connecting_road_junction_mismatch",
        "connecting_road_shared",
        "junction_claim_contested",
    }


def test_two_junctions_claim_the_same_road_end() -> None:
    """Road 1 ends at a junction, and two of them hold a row for that end.

    Road 1's ``<link>`` names one junction, so the other's ``<connection>`` is
    unreachable: a consumer arriving along road 1 never opens the second table
    and nothing resolves into connecting road 3.
    """
    root = _doc(
        _road(
            "1",
            link='<successor elementType="junction" elementId="1000"/>',
        )
        + _road("2", junction="1000")
        + _road("3", junction="1001")
        + '<junction id="1000">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/></connection></junction>'
        '<junction id="1001">'
        '<connection id="0" incomingRoad="1" connectingRoad="3" contactPoint="start">'
        '<laneLink from="1" to="1"/></connection></junction>'
    )
    # Two statements about the same document: the end is contested, and the
    # row of the junction the slot does not name is the one that goes unread.
    assert set(_codes(root)) == {"junction_claim_contested", "connection_unreachable"}


def test_connection_whose_incoming_road_names_another_junction() -> None:
    """Unread without being contested, so the two checks are independent.

    Only junction 1000 holds a row for road 1, but road 1's ``<successor>``
    names junction 1001 -- which claims nothing. Nobody arriving along road 1
    ever opens 1000's table.
    """
    root = _doc(
        _road(
            "1",
            link='<successor elementType="junction" elementId="1001"/>',
        )
        + _road("2", junction="1000")
        + '<junction id="1000">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/></connection></junction>'
        '<junction id="1001"/>'
    )
    assert _codes(root) == ["connection_unreachable"]


def test_one_junction_at_each_end_of_a_road_is_not_contested() -> None:
    """The two ends of a road are two slots, so two junctions is correct."""
    root = _doc(
        _road(
            "1",
            link=(
                '<predecessor elementType="junction" elementId="1001"/>'
                '<successor elementType="junction" elementId="1000"/>'
            ),
        )
        + _road("2", junction="1000")
        + _road("3", junction="1001")
        + '<junction id="1000">'
        '<connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/></connection></junction>'
        '<junction id="1001">'
        '<connection id="0" incomingRoad="1" connectingRoad="3" contactPoint="end">'
        '<laneLink from="1" to="1"/></connection></junction>'
    )
    assert check_invariants(root) == []


@pytest.mark.parametrize("faults", [1, 2, 3])
def test_contested_end_count_tracks_the_number_seeded(faults: int) -> None:
    """Fault injection: the checker reports exactly as many as were seeded.

    A checker that flagged every junction would pass a "more than zero"
    assertion just as well, so the count is pinned rather than bounded.
    """
    body = ""
    for i in range(faults):
        incoming = str(10 + i)
        body += _road(
            incoming,
            link=f'<successor elementType="junction" elementId="{1000 + i}"/>',
        )
        body += _road(str(20 + i), junction=str(1000 + i))
        body += _road(str(30 + i), junction=str(2000 + i))
        body += (
            f'<junction id="{1000 + i}">'
            f'<connection id="0" incomingRoad="{incoming}" '
            f'connectingRoad="{20 + i}" contactPoint="start">'
            '<laneLink from="1" to="1"/></connection></junction>'
            f'<junction id="{2000 + i}">'
            f'<connection id="0" incomingRoad="{incoming}" '
            f'connectingRoad="{30 + i}" contactPoint="start">'
            '<laneLink from="1" to="1"/></connection></junction>'
        )
    # Control: one junction whose road end nothing else claims.
    body += _road("99", link='<successor elementType="junction" elementId="9000"/>')
    body += _road("98", junction="9000")
    body += (
        '<junction id="9000">'
        '<connection id="0" incomingRoad="99" connectingRoad="98" '
        'contactPoint="start"><laneLink from="1" to="1"/></connection></junction>'
    )

    codes = _codes(_doc(body))
    assert codes.count("junction_claim_contested") == faults
    # Each seeded conflict also leaves exactly one row unread: the one held by
    # the junction the incoming road's slot does not name.
    assert codes.count("connection_unreachable") == faults
    assert set(codes) == {"junction_claim_contested", "connection_unreachable"}


def test_linked_roads_with_the_same_lanes_must_meet() -> None:
    """Two roads carrying the same driving lanes describe the same edge."""
    root = _doc(
        _road(
            "1",
            link='<successor elementType="road" elementId="2" contactPoint="start"/>',
        )
        + _road(
            "2",
            x="10.5",
            link='<predecessor elementType="road" elementId="1" contactPoint="end"/>',
        )
    )
    violations = check_invariants(root)
    assert [v.code for v in violations] == [
        "linked_endpoints_apart",
        "linked_endpoints_apart",
    ]
    assert "0.500 m away" in violations[0].detail


def test_linked_roads_with_different_lanes_may_be_apart() -> None:
    """A lane gain moves the reference line, and that is not a defect.

    This is the case that makes the unrestricted version of the rule false on
    every map measured, so the checker has to stay silent here.
    """
    root = _doc(
        _road(
            "1",
            link='<successor elementType="road" elementId="2" contactPoint="start"/>',
        )
        + _road(
            "2",
            x="13.5",
            link='<predecessor elementType="road" elementId="1" contactPoint="end"/>',
            lanes=(
                '<left><lane id="1" type="driving"/>'
                '<lane id="2" type="driving"/></left>'
            ),
        )
    )
    assert check_invariants(root) == []


def test_reference_line_with_a_gap_between_geometries() -> None:
    root = _doc(
        _road(
            "1",
            length="20.0",
            plan_view=(
                '<geometry s="0" x="0" y="0" hdg="0" length="10.0"/>'
                '<geometry s="10" x="10.5" y="0" hdg="0" length="10.0"/>'
            ),
        )
    )
    assert _codes(root) == ["geometry_discontinuity"]


def test_road_length_disagrees_with_its_geometries() -> None:
    root = _doc(_road("1", length="17.0"))
    assert _codes(root) == ["road_length_mismatch"]


def test_paramPoly3_end_point_is_computed_not_assumed() -> None:
    """A curved piece is chained through its polynomial, not its ``hdg``.

    Treating a paramPoly3 as a straight line of its declared length would
    make this document look discontinuous; it is not.
    """
    root = _doc(
        _road(
            "1",
            length="2.0",
            plan_view=(
                '<geometry s="0" x="0" y="0" hdg="0" length="1.0">'
                '<paramPoly3 aU="0" bU="1" cU="0" dU="0"'
                ' aV="0" bV="0" cV="1" dV="0" pRange="normalized"/>'
                "</geometry>"
                '<geometry s="1" x="1.0" y="1.0" hdg="0" length="1.0"/>'
            ),
        )
    )
    assert check_invariants(root) == []


def test_format_violations_groups_and_caps() -> None:
    """The assertion message stays readable when a whole map is wrong."""
    root = _doc(
        "".join(
            _road(str(i), link=f'<successor elementType="road" elementId="9{i}"/>')
            for i in range(15)
        )
    )
    text = format_violations(check_invariants(root), max_shown=3)
    assert "[link_target_missing] 15" in text
    assert "... and 12 more" in text
    assert format_violations([]) == "no invariant violations"


# --------------------------------------------------------------------------
# The checker runs over genuinely converted output.
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def fanout_xodr(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Convert ``junction_fanout_mini.osm`` once for this module."""
    return _convert(
        DATA / "junction_fanout_mini.osm",
        tmp_path_factory.mktemp("invariants"),
        "example_mgrs_offset",
    )


def test_converted_junction_fixture_satisfies_invariants(fanout_xodr: Path) -> None:
    """Real converter output, on the fixture whose defect class this catches."""
    violations = check_invariants(fanout_xodr)
    assert not violations, format_violations(violations)


#: The one invariant Nishi-Shinjuku still breaks, tracked separately so the
#: others stay enforced on it.  See
#: :func:`test_nishishinjuku_linked_endpoints_still_gap`.
_KNOWN_NISHISHINJUKU_VIOLATION = "linked_endpoints_apart"


def test_nishishinjuku_output_satisfies_invariants(nishishinjuku_xodr: Path) -> None:
    """The project's reference conversion, converted by this invocation.

    Takes the session fixture rather than a path under ``test/data``: that
    path holds whatever an earlier checkout left behind, so a stale artifact
    would be checked instead of the current converter, and in a fresh clone
    the check would simply not run.

    ``linked_endpoints_apart`` is excluded and asserted separately, so that
    the one invariant this map is known to break does not mask a regression
    in any of the others.
    """
    violations = [
        v
        for v in check_invariants(nishishinjuku_xodr)
        if v.code != _KNOWN_NISHISHINJUKU_VIOLATION
    ]
    assert not violations, format_violations(violations)


#: What ``linked_endpoints_apart`` measures on Nishi-Shinjuku today: 22
#: violations over 15 distinct joins (a join is reported from both sides when
#: both ends name each other), the worst three at 0.071 m.  Both figures are
#: ceilings in :func:`test_nishishinjuku_linked_endpoints_do_not_get_worse`,
#: not just prose: an ``xfail`` is satisfied by a single violation, so without
#: them a change that turned 22 sub-decimetre gaps into 300 gaps and one 50 m
#: disconnection would leave this file entirely green.
_KNOWN_GAP_VIOLATIONS = 22
_KNOWN_GAP_WORST_M = 0.071


def _endpoint_gaps(xodr: Path) -> List[Violation]:
    """The ``linked_endpoints_apart`` violations, which this map still has."""
    return [
        v for v in check_invariants(xodr) if v.code == _KNOWN_NISHISHINJUKU_VIOLATION
    ]


def _measured_gap_m(violation: Violation) -> float:
    """The gap in metres, read out of the violation's own message.

    ``Violation`` carries only strings, so the magnitude has to be parsed back
    out.  A failed parse is raised rather than skipped: if ``detail``'s wording
    changes, the ceiling below must fail loudly instead of quietly measuring
    nothing.
    """
    match = re.search(r"([0-9.]+) m away", violation.detail)
    assert match is not None, (
        "cannot read the gap out of "
        f"{violation.detail!r} -- the message format changed, so "
        "_measured_gap_m needs updating with it"
    )
    return float(match.group(1))


def test_nishishinjuku_linked_endpoints_do_not_get_worse(
    nishishinjuku_xodr: Path,
) -> None:
    """Hold the known gap at its measured size, in count and in magnitude.

    This is the half an ``xfail`` cannot express.  The companion test below
    fails the day the gaps close; this one fails the day they widen or
    multiply, which is the direction a regression actually takes.

    Both ceilings are ``<=`` rather than ``==`` so that a partial fix does not
    fail the suite -- it just makes these numbers stale, and
    :func:`test_nishishinjuku_linked_endpoints_still_gap` is what forces them
    to be revisited once the last one goes.
    """
    violations = _endpoint_gaps(nishishinjuku_xodr)
    assert len(violations) <= _KNOWN_GAP_VIOLATIONS, format_violations(violations)

    worst = max((_measured_gap_m(v) for v in violations), default=0.0)
    assert worst <= _KNOWN_GAP_WORST_M + 1e-9, (
        f"worst endpoint gap is {worst:.3f} m, above the pinned "
        f"{_KNOWN_GAP_WORST_M} m: " + format_violations(violations)
    )


@pytest.mark.xfail(
    strict=True,
    reason=(
        "linked road endpoints on Nishi-Shinjuku are still up to 0.071 m "
        "apart against a 0.01 m tolerance (22 violations over 15 joins); the "
        "junction endpoint work is what closes this"
    ),
)
def test_nishishinjuku_linked_endpoints_still_gap(nishishinjuku_xodr: Path) -> None:
    """Pin the gap this map has, so closing it is visible as a change.

    Measured on the current converter: 22 violations over 15 distinct joins
    past the 1 cm tolerance, the worst three at 0.071 m.  Marked ``strict``,
    so the day the gaps close this test fails and the marker has to go -- the
    defect cannot be fixed silently.

    An ``xfail`` is satisfied by *any* violation, so this test alone would not
    notice the gap getting worse.
    :func:`test_nishishinjuku_linked_endpoints_do_not_get_worse` is the other
    half, and it is deliberately not marked ``xfail``.
    """
    violations = _endpoint_gaps(nishishinjuku_xodr)
    assert not violations, format_violations(violations)
