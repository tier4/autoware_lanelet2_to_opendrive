"""The conversion's own analysis must report structural invariants.

``xodr_invariants.check_invariants`` names the defect CARLA dies on: a lane
whose ``<successor>`` points at a lane the linked road does not have. CARLA
exits 139 on one such id, and ``<predecessor>`` is inert, so the asymmetry
makes the failure look random. The checker exists and is exercised over the
shipped Nishishinjuku fixture by ``test_xodr_structural_invariants``, but
``run_analysis`` -- the post-conversion step every ``convert`` runs, and the
only gate an arbitrary customer map passes through -- does not call it. The
ASAM bundle does not cover it either: 25 of its 26 checkers complete on the
customer map and none of them reports a dangling lane link.

So a conversion of a map nobody has a test for can emit the one document
shape that kills the simulator, and say ``FINAL RESULT: PASS``.

Measured on 2026-11-06: a Nishi-Shinjuku conversion violates **nothing**, and
the customer map's output has **0** dangling links and 2 violations, both of
``linked_endpoints_apart`` over one join -- a real defect (see ``APART``).
So the section is at zero on the reference map and reports exactly one thing
on the customer map, which is what a gate is supposed to look like.
"""

from __future__ import annotations

from pathlib import Path

DANGLING = (
    '<OpenDRIVE><header revMajor="1" revMinor="7"/>'
    '<road id="1" length="10.0" junction="-1">'
    '<link><successor elementType="road" elementId="2" contactPoint="start"/></link>'
    '<planView><geometry s="0" x="0" y="0" hdg="0" length="10.0"/></planView>'
    '<lanes><laneSection s="0"><left>'
    '<lane id="1" type="driving"><link><successor id="2"/></link></lane>'
    "</left></laneSection></lanes>"
    "</road>"
    '<road id="2" length="10.0" junction="-1">'
    '<link><predecessor elementType="road" elementId="1" contactPoint="end"/></link>'
    '<planView><geometry s="0" x="10" y="0" hdg="0" length="10.0"/></planView>'
    '<lanes><laneSection s="0"><left><lane id="1" type="driving"/></left>'
    "</laneSection></lanes>"
    "</road>"
    "</OpenDRIVE>"
)
"""Road 1's lane names lane 2 of road 2, which has only lane 1."""

CLEAN = DANGLING.replace('<link><successor id="2"/></link>', "")

APART = CLEAN.replace('<geometry s="0" x="10" y="0"', '<geometry s="0" x="50" y="0"')
"""Road 2 sits 40 m from where road 1's link says it meets it.

``linked_endpoints_apart`` fires on this, and it is **enforced** like every
other invariant. The first version of this section exempted it, on the
strength of ``xodr_invariants``'s own docstring -- "22 violations over 15
joins on Nishi-Shinjuku, worst 0.071 m" -- reasoning that a gate failing
every real map on arrival is a gate nobody reads.

That prose is stale, and the repository had already said so: measured
2026-11-06, ``check_invariants`` reports **0** violations of every code on a
Nishi-Shinjuku conversion, and ``test_xodr_structural_invariants`` pins
``_KNOWN_GAP_VIOLATIONS = 0`` with the comment that excluding the code "would
only hide a reappearance". The junction endpoint work closed them.

So the exemption protected nothing on the reference map and, on the customer
map, hid the only thing the check found: one join where **both** link records
have the end and the role reversed, 39.830 m from where the two roads really
touch. That is exactly the defect the invariant is for.
"""


def test_a_dangling_lane_link_fails_the_analysis_section(tmp_path: Path) -> None:
    from autoware_lanelet2_to_opendrive.analyze_xodr import (
        run_structural_invariant_validation,
    )

    path = tmp_path / "dangling.xodr"
    path.write_text(DANGLING, encoding="utf-8")
    assert run_structural_invariant_validation(path) is False


def test_two_roads_that_claim_to_meet_far_apart_fail_the_section(
    tmp_path: Path, capsys
) -> None:
    from autoware_lanelet2_to_opendrive.analyze_xodr import (
        run_structural_invariant_validation,
    )

    path = tmp_path / "apart.xodr"
    path.write_text(APART, encoding="utf-8")
    assert run_structural_invariant_validation(path) is False
    printed = capsys.readouterr().out
    assert "linked_endpoints_apart" in printed, printed


def test_a_clean_document_passes_the_analysis_section(tmp_path: Path) -> None:
    """The guard rail: the section must not fail everything it is given."""
    from autoware_lanelet2_to_opendrive.analyze_xodr import (
        run_structural_invariant_validation,
    )

    path = tmp_path / "clean.xodr"
    path.write_text(CLEAN, encoding="utf-8")
    assert run_structural_invariant_validation(path) is True
