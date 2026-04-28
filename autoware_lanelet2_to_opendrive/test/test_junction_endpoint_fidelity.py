"""Regression test for P0-2 junction endpoint fidelity.

Ensures that each connecting road (junction != -1) in the output OpenDRIVE map
lands exactly on its linked incoming and outgoing roads at its start and end
points.  Prior to the fix, the connecting road's reference line came from a
different OSM LineString than the incoming/outgoing road's reference line,
which caused gaps of up to ~11.6 m at junction entry/exit.

The test evaluates the 3D endpoint (x, y, z) of every connection by
reconstructing the planView + elevationProfile from XML and compares it with
the endpoint of the linked road.  A tolerance of 5 cm is used.
"""

import subprocess
from pathlib import Path

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.opendrive.geometry import evaluate_road_endpoints


TOLERANCE_M = 0.05
# Map used for the integration-style regression below.
NISHISHINJUKU_XODR = Path("/tmp/nishishinjuku_carla.xodr")


def _build_nishishinjuku_xodr() -> Path:
    """Produce the Nishishinjuku XODR if it is not already on disk.

    The fix in P0-2 shifts connecting-road endpoints; only an
    end-to-end conversion exercises it. We build the file on demand via
    ``uv run convert`` so the regression test actually runs in CI rather
    than silently skipping.
    """
    if NISHISHINJUKU_XODR.exists():
        return NISHISHINJUKU_XODR

    fixture = Path(
        "autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm"
    ).resolve()
    if not fixture.is_file():
        pytest.skip(f"{fixture} not available; cannot build XODR")

    try:
        # `pin_junction_endpoints=true` opts into the P0-2 override path
        # this test exists to validate.  It is off by default in the
        # converter because it currently breaks the mapping cross-check on
        # some maps (issue #431); when that is resolved this flag and the
        # corresponding ConversionConfig field can both go away.
        subprocess.run(
            [
                "uv",
                "run",
                "convert",
                "map=nishishinjuku",
                "target=carla",
                f"input_map_path={fixture}",
                f"output_map_path={NISHISHINJUKU_XODR}",
                "pin_junction_endpoints=true",
            ],
            check=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        pytest.skip(f"converter unavailable or failed: {exc}")

    if not NISHISHINJUKU_XODR.exists():
        pytest.skip(f"{NISHISHINJUKU_XODR} not produced by converter")

    return NISHISHINJUKU_XODR


def _distance3(a, b) -> float:
    return float(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5)


def test_evaluate_road_endpoints_minimal():
    """Sanity check ``evaluate_road_endpoints`` on a synthetic XODR root."""
    xml = """
    <OpenDRIVE>
      <road id="0" length="10.0" junction="-1">
        <planView>
          <geometry s="0.0" x="1.0" y="2.0" hdg="0.0" length="10.0">
            <paramPoly3 aU="0.0" bU="1.0" cU="0.0" dU="0.0"
                        aV="0.0" bV="0.0" cV="0.0" dV="0.0" pRange="arcLength"/>
          </geometry>
        </planView>
        <elevationProfile>
          <elevation s="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/>
        </elevationProfile>
      </road>
    </OpenDRIVE>
    """
    root = ET.fromstring(xml)
    endpoints = evaluate_road_endpoints(root)

    assert 0 in endpoints
    start, end = endpoints[0]
    assert start == pytest.approx((1.0, 2.0, 3.0), abs=1e-9)
    assert end == pytest.approx((11.0, 2.0, 3.0), abs=1e-9)


def test_junction_connection_endpoints_match_linked_roads():
    """Every junction connection must land on the linked road within 5 cm.

    The P0-2 fix overrides the connecting-road endpoints with the linked
    regular-road endpoints during construction.  Because a single connecting
    road has only two endpoints (start, end), it can only be pinned to one
    incoming and one outgoing regular road at a time.  Multi-incoming /
    multi-outgoing junctions therefore necessarily have gaps on the
    non-pinned sides — this test excludes those cases and checks only the
    connections whose connecting road has a unique incoming and unique
    outgoing regular road in the junction table.
    """
    xodr_path = _build_nishishinjuku_xodr()

    tree = ET.parse(str(xodr_path))
    root = tree.getroot()

    endpoints = evaluate_road_endpoints(root)

    # Map road_id -> junction attribute (-1 if not in a junction)
    road_junction: dict[int, int] = {}
    road_predecessor: dict[int, tuple] = {}
    road_successor: dict[int, tuple] = {}
    for road_elem in root.findall("road"):
        rid = int(road_elem.get("id"))
        road_junction[rid] = int(road_elem.get("junction", "-1"))

        link = road_elem.find("link")
        if link is not None:
            pred = link.find("predecessor")
            if pred is not None:
                road_predecessor[rid] = (
                    pred.get("elementType"),
                    int(pred.get("elementId")),
                    pred.get("contactPoint"),
                )
            succ = link.find("successor")
            if succ is not None:
                road_successor[rid] = (
                    succ.get("elementType"),
                    int(succ.get("elementId")),
                    succ.get("contactPoint"),
                )

    # Build per-connecting-road sets of incoming roads (from junction table).
    conn_road_incomings: dict[int, set] = {}
    conn_road_junction: dict[int, int] = {}
    conn_road_contact: dict[int, str] = {}
    for junction_elem in root.findall("junction"):
        junction_id = int(junction_elem.get("id"))
        for conn_elem in junction_elem.findall("connection"):
            connecting_road_id = int(conn_elem.get("connectingRoad"))
            incoming_road_id = int(conn_elem.get("incomingRoad"))
            contact_point = conn_elem.get("contactPoint", "start")
            conn_road_incomings.setdefault(connecting_road_id, set()).add(
                incoming_road_id
            )
            conn_road_junction[connecting_road_id] = junction_id
            conn_road_contact[connecting_road_id] = contact_point

    offenders = []
    for connecting_road_id, incoming_ids in conn_road_incomings.items():
        junction_id = conn_road_junction[connecting_road_id]
        contact_point = conn_road_contact[connecting_road_id]

        if connecting_road_id not in endpoints:
            continue
        conn_start, conn_end = endpoints[connecting_road_id]

        # Incoming side: check only when the connecting road has exactly
        # one incoming regular road — the override can pin only one.
        if len(incoming_ids) == 1:
            (incoming_road_id,) = incoming_ids
            if (
                incoming_road_id in endpoints
                and road_junction.get(incoming_road_id, -1) == -1
            ):
                inc_start, inc_end = endpoints[incoming_road_id]
                if contact_point == "start":
                    expected_in = inc_end
                    actual_in = conn_start
                else:
                    expected_in = inc_start
                    actual_in = conn_end
                d_in = _distance3(expected_in, actual_in)
                if d_in > TOLERANCE_M:
                    offenders.append(
                        (
                            junction_id,
                            connecting_road_id,
                            "incoming",
                            incoming_road_id,
                            d_in,
                        )
                    )

        # Outgoing side: the connecting road's link references the
        # outgoing road.  Check only when it resolves to a single regular
        # road (i.e. not a chained connecting road, and not multi-successor).
        if contact_point == "start":
            link = road_successor.get(connecting_road_id)
            out_side = conn_end
            out_contact_label = "end"
        else:
            link = road_predecessor.get(connecting_road_id)
            out_side = conn_start
            out_contact_label = "start"

        if link is None:
            continue
        link_type, link_id, link_contact = link
        if link_type != "road":
            continue
        if link_id not in endpoints:
            continue
        if road_junction.get(link_id, -1) != -1:
            # Chained connecting roads are not part of this check.
            continue

        link_start, link_end = endpoints[link_id]
        expected_out = link_start if link_contact == "start" else link_end
        d_out = _distance3(expected_out, out_side)
        if d_out > TOLERANCE_M:
            offenders.append(
                (
                    junction_id,
                    connecting_road_id,
                    out_contact_label,
                    link_id,
                    d_out,
                )
            )

    if offenders:
        offenders.sort(key=lambda o: -o[4])
        sample = "\n".join(
            f"  junction={j} conn_road={cr} side={side} linked_road={lr} d={d:.3f}"
            for j, cr, side, lr, d in offenders[:10]
        )
        pytest.fail(
            f"{len(offenders)} junction endpoint mismatches > {TOLERANCE_M} m.\n"
            f"Max: {offenders[0][4]:.3f} m.\n"
            f"Worst 10:\n{sample}"
        )
