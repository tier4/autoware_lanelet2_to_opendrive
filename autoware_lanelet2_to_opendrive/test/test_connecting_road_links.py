"""Regression tests for connecting-road predecessor/successor links.

``Road.set_connecting_road_links`` is responsible for emitting the
``<link><predecessor>`` / ``<link><successor>`` of every junction
connecting road. It looked its lanelets up with ``lid in
lanelet_map.laneletLayer`` -- a membership test that is *always False*
for an integer id, because the layer's ``__contains__`` does not key on
id (``exists(id)`` does). The list comprehension therefore yielded an
empty ``road_lanelets`` for every road, the function ``continue``d past
all of them, and **no connecting road ever received a link**.

In the Nishishinjuku conversion that left 338 of 403 connecting roads
with no ``<link>`` at all. esmini/odrviewer routes a car into a
connecting road via the junction ``<connection>`` table but needs the
connecting road's own ``<successor>`` to leave it again; without it the
car reaches the end of the connecting road and is stranded, so traffic
piled up inside every junction.

These tests pin the linked-ness of connecting roads so the broken
membership test cannot silently return.
"""

import subprocess
from pathlib import Path

import lxml.etree as ET
import pytest


@pytest.fixture(scope="session")
def nishishinjuku_xodr(tmp_path_factory) -> Path:
    """Convert the Nishishinjuku fixture once per session and return the XODR.

    Only an end-to-end conversion exercises the ``_setup_connections``
    pipeline ``set_connecting_road_links`` lives in. The output is written
    into a fresh session-scoped temp directory rather than a fixed cached
    path: a regression test must exercise the *current* conversion code,
    never an XODR left behind by an earlier checkout.
    """
    fixture = Path(
        "autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm"
    ).resolve()
    if not fixture.is_file():
        pytest.skip(f"{fixture} not available; cannot build XODR")

    xodr_path = tmp_path_factory.mktemp("connecting_links") / "nishishinjuku.xodr"
    cmd = [
        "uv",
        "run",
        "convert",
        "map=nishishinjuku",
        "target=carla",
        f"input_map_path={fixture}",
        f"output_map_path={xodr_path}",
    ]
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError as exc:
        pytest.skip(f"converter unavailable: {exc}")

    if not xodr_path.is_file():
        pytest.fail(f"converter exited successfully but {xodr_path} was not produced")
    return xodr_path


def _connecting_roads(tree: ET._ElementTree) -> list:
    """All connecting roads -- roads whose ``junction`` attribute is not -1."""
    return [r for r in tree.iterfind(".//road") if r.get("junction") != "-1"]


def test_fixture_contains_many_connecting_roads(nishishinjuku_xodr):
    """Guard: the assertions below are meaningless on a tiny road set."""
    tree = ET.parse(str(nishishinjuku_xodr))
    assert len(_connecting_roads(tree)) > 100


def test_connecting_roads_receive_link_elements(nishishinjuku_xodr):
    """Almost every connecting road must carry a ``<link>``.

    With the broken membership test only ~16 % of connecting roads were
    linked (the handful reached by other passes). A correct
    ``set_connecting_road_links`` links the rest. The only roads that may
    legitimately stay link-less are connecting lanelets that are
    disconnected in the *source* map (no routing-graph neighbour at all);
    those are a small minority, so a 90 % floor both fails hard on the
    bug and leaves headroom for genuine source-map gaps.
    """
    tree = ET.parse(str(nishishinjuku_xodr))
    connecting = _connecting_roads(tree)
    linked = [r for r in connecting if r.find("link") is not None]

    assert len(linked) >= 0.9 * len(connecting), (
        f"only {len(linked)}/{len(connecting)} connecting roads have a "
        f"<link> element -- set_connecting_road_links emitted (almost) none"
    )


def test_connecting_roads_have_both_predecessor_and_successor(nishishinjuku_xodr):
    """A connecting road joins an incoming road to an outgoing road.

    A well-formed one therefore carries *both* a ``<predecessor>`` and a
    ``<successor>``; before the fix essentially none did. A 75 % floor
    distinguishes the fixed converter (most connecting roads fully
    linked) from the broken one (zero) without demanding perfection on
    source-map edge cases.
    """
    tree = ET.parse(str(nishishinjuku_xodr))
    connecting = _connecting_roads(tree)
    both = [
        r
        for r in connecting
        if r.find("link/predecessor") is not None
        and r.find("link/successor") is not None
    ]

    assert len(both) >= 0.75 * len(connecting), (
        f"only {len(both)}/{len(connecting)} connecting roads have both a "
        f"<predecessor> and a <successor> link"
    )
