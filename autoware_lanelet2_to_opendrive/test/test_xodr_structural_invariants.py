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

**Measured against a real defect.**  At ``a2a48501^`` -- the commit before
"split connecting roads by their incoming and outgoing roads" -- the
converter bundled laterally adjacent connecting lanelets into one connecting
road and let its lanes keep successor ids belonging to a *different* outgoing
road.  CARLA read those against the road the road-level link named, and
segfaulted.  Running this checker over that commit's output reports it on all
three inputs tried, e.g.::

    [lane_link_target_missing] road 3 lane -3/successor:
        names lane -2 of road 2 at its start, which has lanes ['-1']

**What it does not catch.**  Every invariant here is about a reference that
resolves.  A *lost* maneuver -- a connecting road that should have been
emitted twice and was emitted once, as fixed in ``226caa11`` -- leaves no
dangling reference behind, and running this checker over that commit's parent
reports nothing on any input tried.  Detecting a loss needs the source map to
compare against, which is the separate semantics measurement, not this.

**Candidate invariants that were measured and rejected.**  These are all
*false today on Nishi-Shinjuku as well as on real-map output*, so they are
findings to look into rather than invariants to assert.  Counts are from
``target=carla`` at ``31e81c3f`` (Nishi-Shinjuku / a 190-lanelet real-map
excerpt):

* "a road links only to a road whose endpoint is within 1 cm" -- 317 / 34
  violations, up to 11.6 m / 6.6 m apart.  A reference line is anchored to
  its road's outermost lane boundary, so two linked roads with different lane
  counts legitimately start metres apart.  **Kept in weakened form**: where
  the two sides carry the same driving lanes the two reference lines describe
  the same edge and do meet, so ``linked_endpoints_apart`` asserts it there.
  That covers 575 of Nishi-Shinjuku's 1097 road-to-road links (max gap 5 mm,
  all of them synthetic divergence stubs at their 5 mm floor) and 80 of 146
  on ``junction_cluster`` (max 0). Split by kind on Nishi-Shinjuku:
  connecting->connecting 0 of 38 over 1 cm, regular->regular with equal lane
  counts 0 of 205, regular->regular with different lane counts 2 of 2,
  connecting->regular 315 of 852.
* "every driving lane with a road-level successor has a lane-level one" --
  0 / 1.  The single real-map case is a lane *gain*: road 11 has three
  driving lanes and its predecessor has two, so the third lane correctly has
  no predecessor.  The rule is wrong, not the output.
* "every connecting road is named by some ``<connection>``" -- 51 / 28.  The
  converter logs each one: the connecting lanelet has no routing-graph
  predecessor in the source map.  That is a property of the input.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.xodr_invariants import (
    check_invariants,
    format_violations,
)

DATA = Path(__file__).parent / "data"
PROJECT_ROOT = Path(__file__).resolve().parents[2]

#: A junction cluster rebuilt from a real map: renumbered, moved and stripped
#: of authoring metadata, but geometrically the original (see
#: ``data/tools/rebuild_as_synthetic.py`` and ``data/README.md``).  It carries
#: the structures a hand-drawn fixture cannot: multi-lane junctions, forking
#: and chained connectors, coarsely digitised boundaries, non-vehicle lanelets.
REAL_DERIVED_CLUSTER = DATA / "junction_cluster.osm"

#: Produced by the ``_ensure_nishishinjuku_xodr`` session fixture in
#: ``autoware_carla_scenario/test/conftest.py``.  Reused when it is already
#: there; never generated here, because converting Nishi-Shinjuku costs about
#: two minutes and this file is not worth that on its own.
NISHISHINJUKU_XODR = DATA / "nishishinjuku_carla.xodr"


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
    """The shape of the defect CARLA segfaulted on at ``a2a48501^``."""
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
    # The second junction also disagrees about @junction, which is a separate
    # statement about the same document; both are expected.
    assert set(_codes(root)) == {
        "connecting_road_junction_mismatch",
        "connecting_road_shared",
    }


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


@pytest.mark.skipif(
    not NISHISHINJUKU_XODR.exists(),
    reason=(
        "nishishinjuku_carla.xodr has not been generated in this run "
        "(see autoware_carla_scenario/test/conftest.py)"
    ),
)
def test_nishishinjuku_output_satisfies_invariants() -> None:
    """The project's reference conversion, when it is already on disk."""
    violations = check_invariants(NISHISHINJUKU_XODR)
    assert not violations, format_violations(violations)


@pytest.mark.slow
@pytest.mark.skipif(
    not REAL_DERIVED_CLUSTER.exists(),
    reason=(
        "junction_cluster.osm is not committed -- its geometry comes from a "
        "real urban map. Produce it with test/data/tools/crop_lanelet2_map.py "
        "and test/data/tools/rebuild_as_synthetic.py (see test/data/README.md) "
        "to enable this check."
    ),
)
def test_real_derived_cluster_satisfies_invariants(
    tmp_path_factory: pytest.TempPathFactory,
) -> None:
    """Geometry taken from a real map, which no hand-drawn fixture reproduces.

    The ``*_mini`` fixtures are drawn on a 20 m grid, and a fixture drawn on a
    grid cannot exhibit some of what this is here for: where two branches of a
    fork start at the same shared boundary by construction, a defect in how
    their start points are reconciled is a provable no-op.  Here they do not,
    because the source geometry is surveyed rather than drawn.

    Marked ``slow``: converting it costs about 40 s.

    **The fixture is not committed**, so by default this skips and the
    coverage it describes does not exist.  The two tools that produce it are
    committed next to it; ``test/data/README.md`` records the exact invocation
    and what the rebuild guarantees.  The skip names both, so a reader of the
    run sees which check is absent rather than a silent pass.

    The guard is keyed on the *source map*, not on the generated XODR, so it
    is outside what ``test_fixture_dependent_tests_never_skip`` forbids -- that
    module is about a test whose subject is rebuilt by a fixture, which this
    one is not.
    """
    xodr = _convert(
        REAL_DERIVED_CLUSTER, tmp_path_factory.mktemp("cluster"), "junction_cluster"
    )
    violations = check_invariants(xodr)
    assert not violations, format_violations(violations)
