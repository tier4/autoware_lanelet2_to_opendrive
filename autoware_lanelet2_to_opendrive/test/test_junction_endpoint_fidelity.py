"""Regression test for P0-2 junction endpoint fidelity.

Ensures that each connecting road (junction != -1) in the output OpenDRIVE map
lands exactly on its linked incoming and outgoing roads at the **lane edge**
shared between them.  Prior to the fix, the connecting road's reference line
came from a different OSM LineString than the incoming/outgoing road's
reference line, which caused gaps of up to ~11.6 m at junction entry/exit.

The test evaluates the 3D endpoint (x, y, z) of every connection by
reconstructing the planView + elevationProfile from XML and compares it
with the corresponding lane edge of the linked road, derived from the
junction ``<laneLink>`` mapping (incoming side) and the lane-level
``<successor>`` link (outgoing side).  A tolerance of 5 cm is used.

Note: comparing reference-line endpoints (the pre-#437 invariant) is only
correct when the connecting road's outermost lanelet links to the linked
road's outermost lanelet.  When it links to an inner lanelet — the case
that #437 fixes via lane-aware pinning — the reference lines differ by
one or more lane widths, but the **lane edges** still coincide.
"""

import math
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Optional, Tuple

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.opendrive.geometry import evaluate_road_endpoints


TOLERANCE_M = 0.05


def _evaluate_lane_inner_edge(
    road_elem: ET._Element, lane_id: int, at_start: bool
) -> Optional[Tuple[float, float, float]]:
    """Return ``(x, y, z)`` of lane ``lane_id``'s reference-side edge.

    The reference-side edge of lane ``±k`` is the boundary closer to the
    reference line — for ``|k| == 1`` this is the reference line itself
    (lateral offset ``t = 0``); for ``|k| >= 2`` it is at
    ``t = sign(k) * sum(widths of lanes between reference and lane k)``.

    Evaluated at ``s = 0`` if ``at_start`` is true, otherwise at
    ``s = sum(geometry.length)``.

    Returns ``None`` if the road is missing planView geometry, lane
    section, or the requested lane's width data.
    """
    plan_view = road_elem.find("planView")
    if plan_view is None:
        return None
    geom_elems = plan_view.findall("geometry")
    if not geom_elems:
        return None
    geom = geom_elems[0] if at_start else geom_elems[-1]
    geom_length = float(geom.get("length", "0.0"))
    p_local = 0.0 if at_start else geom_length

    geom_x = float(geom.get("x"))
    geom_y = float(geom.get("y"))
    hdg_base = float(geom.get("hdg"))
    cos_h = math.cos(hdg_base)
    sin_h = math.sin(hdg_base)

    pp3 = geom.find("paramPoly3")
    if pp3 is not None:
        a_u = float(pp3.get("aU", "0.0"))
        b_u = float(pp3.get("bU", "0.0"))
        c_u = float(pp3.get("cU", "0.0"))
        d_u = float(pp3.get("dU", "0.0"))
        a_v = float(pp3.get("aV", "0.0"))
        b_v = float(pp3.get("bV", "0.0"))
        c_v = float(pp3.get("cV", "0.0"))
        d_v = float(pp3.get("dV", "0.0"))
        local_u = a_u + b_u * p_local + c_u * p_local**2 + d_u * p_local**3
        local_v = a_v + b_v * p_local + c_v * p_local**2 + d_v * p_local**3
        x_ref = geom_x + local_u * cos_h - local_v * sin_h
        y_ref = geom_y + local_u * sin_h + local_v * cos_h
        du = b_u + 2.0 * c_u * p_local + 3.0 * d_u * p_local**2
        dv = b_v + 2.0 * c_v * p_local + 3.0 * d_v * p_local**2
        dx = du * cos_h - dv * sin_h
        dy = du * sin_h + dv * cos_h
        heading = math.atan2(dy, dx)
    else:
        x_ref = geom_x + p_local * cos_h
        y_ref = geom_y + p_local * sin_h
        heading = hdg_base

    s_road = float(geom_elems[0].get("s", "0.0")) + (
        0.0
        if at_start
        else (
            float(geom_elems[-1].get("s", "0.0"))
            - float(geom_elems[0].get("s", "0.0"))
            + geom_length
        )
    )

    lanes_elem = road_elem.find("lanes")
    if lanes_elem is None:
        return None
    section_elems = lanes_elem.findall("laneSection")
    if not section_elems:
        return None
    section = section_elems[0]
    s_in_section = s_road - float(section.get("s", "0.0"))

    t = 0.0
    if lane_id != 0:
        side_name = "left" if lane_id > 0 else "right"
        side_elem = section.find(side_name)
        if side_elem is None:
            return None
        lane_by_id = {int(le.get("id")): le for le in side_elem.findall("lane")}
        sign = 1 if lane_id > 0 else -1
        for j in range(1, abs(lane_id)):
            inner_lane = lane_by_id.get(sign * j)
            if inner_lane is None:
                return None
            width_elems = inner_lane.findall("width")
            if not width_elems:
                # Lane is missing required <width> data — refuse to
                # silently substitute zero, which would mask malformed
                # XODR and let the assertion pass for the wrong reason.
                return None
            seg = width_elems[0]
            for we in width_elems:
                if float(we.get("sOffset", "0")) <= s_in_section:
                    seg = we
                else:
                    break
            ds = s_in_section - float(seg.get("sOffset", "0"))
            w = (
                float(seg.get("a", "0"))
                + float(seg.get("b", "0")) * ds
                + float(seg.get("c", "0")) * ds**2
                + float(seg.get("d", "0")) * ds**3
            )
            t += sign * w

    nx = -math.sin(heading)
    ny = math.cos(heading)
    x = x_ref + t * nx
    y = y_ref + t * ny

    elev = road_elem.find("elevationProfile")
    z = 0.0
    if elev is not None:
        for elev_seg in elev.findall("elevation"):
            s_off = float(elev_seg.get("s", "0"))
            if s_off > s_road:
                break
            ds = s_road - s_off
            z = (
                float(elev_seg.get("a", "0"))
                + float(elev_seg.get("b", "0")) * ds
                + float(elev_seg.get("c", "0")) * ds**2
                + float(elev_seg.get("d", "0")) * ds**3
            )

    return (x, y, z)


def _outermost_lane_id(road_elem: ET._Element) -> Optional[int]:
    """Return the outermost lane ID of a road (``-1`` for RHT, ``+1`` for LHT).

    Determined by which side of the laneSection contains lanes; the
    reference-line side has the smallest absolute lane id (``±1``).
    """
    lanes_elem = road_elem.find("lanes")
    if lanes_elem is None:
        return None
    section_elems = lanes_elem.findall("laneSection")
    if not section_elems:
        return None
    section = section_elems[0]
    right = section.find("right")
    if right is not None and right.findall("lane"):
        return -1
    left = section.find("left")
    if left is not None and left.findall("lane"):
        return 1
    return None


def _outermost_lane_link_target(
    road_elem: ET._Element,
    outermost_lane_id: int,
    follow: str,
) -> Optional[int]:
    """Return the lane ID of the outermost lane's predecessor or successor.

    ``follow`` must be ``"predecessor"`` or ``"successor"``.  Returns
    ``None`` if the lane has no link of that kind.
    """
    lanes_elem = road_elem.find("lanes")
    if lanes_elem is None:
        return None
    section = lanes_elem.findall("laneSection")[0]
    side_name = "left" if outermost_lane_id > 0 else "right"
    side = section.find(side_name)
    if side is None:
        return None
    for lane in side.findall("lane"):
        if int(lane.get("id")) == outermost_lane_id:
            link = lane.find("link")
            if link is None:
                return None
            target = link.find(follow)
            if target is None:
                return None
            return int(target.get("id"))
    return None


def _nishishinjuku_xodr_path() -> Path:
    """Per-worker output path for the Nishishinjuku XODR.

    Using a worker-specific filename keeps ``pytest -n`` workers from
    racing to build / read the same file (the conversion is expensive,
    so we cache it on disk for the lifetime of the worker, but each
    worker needs its own copy).
    """
    worker_id = os.environ.get("PYTEST_XDIST_WORKER", "main")
    return Path(tempfile.gettempdir()) / f"nishishinjuku_carla_{worker_id}.xodr"


def _build_nishishinjuku_xodr() -> Path:
    """Produce the Nishishinjuku XODR if it is not already on disk.

    The fix in P0-2 shifts connecting-road endpoints; only an
    end-to-end conversion exercises it. We build the file on demand via
    ``uv run convert`` so the regression test actually runs in CI rather
    than silently skipping.
    """
    xodr_path = _nishishinjuku_xodr_path()
    if xodr_path.exists():
        return xodr_path

    fixture = Path(
        "autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm"
    ).resolve()
    if not fixture.is_file():
        pytest.skip(f"{fixture} not available; cannot build XODR")

    # Junction endpoint pinning is on by default since #437 — no flag is
    # required at the CLI.  The lateral-displacement gate inside
    # ``Road.construct_connecting_roads_from_junctions`` keeps the override
    # safe on maps where a connecting road is parallel to a regular road
    # at the junction boundary (the original #431 root cause).
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
        # Tooling missing — environmental, not a regression.
        pytest.skip(f"converter unavailable: {exc}")

    if not xodr_path.is_file():
        # The converter reported success (exit 0) but the expected file
        # is missing — fail with a clear, actionable message instead of
        # leaking out as an opaque XML parse error in the next step.
        pytest.fail(
            f"converter exited successfully but {xodr_path} was not "
            "produced; check that ``output_map_path`` is honoured."
        )

    return xodr_path


def _distance3(a, b) -> float:
    return float(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5)


def _connecting_road_lane_boundaries(
    road_elem: ET._Element,
) -> list[Tuple[float, float, float]]:
    """Every lane-boundary point at both ends of a road's first laneSection.

    A *chained* connector — one whose routing predecessor is itself an
    in-junction connecting lanelet (#492) — has its reference line pinned
    by the #437 override to one of its upstream connector's lane
    boundaries.  Enumerating every boundary (the reference line through
    the outer edge of the outermost lane) at both ends lets the
    incoming-side check recognise that pinning regardless of which lane
    the chain branches from.
    """
    lanes_elem = road_elem.find("lanes")
    if lanes_elem is None:
        return []
    section_elems = lanes_elem.findall("laneSection")
    if not section_elems:
        return []
    section = section_elems[0]
    boundaries: list[Tuple[float, float, float]] = []
    for side_name in ("left", "right"):
        side = section.find(side_name)
        if side is None:
            continue
        lane_count = sum(1 for le in side.findall("lane") if int(le.get("id")) != 0)
        sign = 1 if side_name == "left" else -1
        # Lane |k|'s reference-side edge sweeps the boundaries from the
        # reference line (|k| == 1) outward; |k| == lane_count + 1 is the
        # outer edge of the outermost lane.
        for k in range(1, lane_count + 2):
            for at_start in (True, False):
                point = _evaluate_lane_inner_edge(
                    road_elem, sign * k, at_start=at_start
                )
                if point is not None:
                    boundaries.append(point)
    return boundaries


def _incoming_endpoint_pinned_to_connector(
    point: Tuple[float, float, float],
    junction_id: int,
    connecting_road_id: int,
    road_by_id: dict[int, ET._Element],
    conn_road_junction: dict[int, int],
) -> bool:
    """True if ``point`` lands on another connector's lane boundary.

    A chained connector's ``<connection>`` resolves ``incomingRoad``
    transitively to a regular road so CARLA's OpenDRIVE parser accepts it
    (#500), but the connector is geometrically pinned to its *upstream
    connector*, not that regular road.  Such a connector is excluded from
    the regular-road incoming check — like multi-incoming connections —
    and is recognised by its incoming endpoint coinciding with a lane
    boundary of another connecting road in the same junction.
    """
    for other_id, other_junction in conn_road_junction.items():
        if other_id == connecting_road_id or other_junction != junction_id:
            continue
        other_road = road_by_id.get(other_id)
        if other_road is None:
            continue
        for boundary in _connecting_road_lane_boundaries(other_road):
            if _distance3(point, boundary) <= TOLERANCE_M:
                return True
    return False


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
    """Every junction connection must land on the linked lane edge within 5 cm.

    The P0-2 fix (made lane-aware in #437) overrides the connecting-road
    endpoints with the rendered XYZ of the lane edge that the connecting
    road's outermost lanelet shares with its linked predecessor /
    successor lanelet in the regular road.

    A connecting road has only two endpoints, so it can only be pinned to
    one incoming and one outgoing regular road; multi-incoming /
    multi-outgoing cases necessarily have gaps on the non-pinned sides
    and are excluded.

    Chained connectors — connecting roads fed only by another connector
    in the same junction (#492) — are likewise excluded on the incoming
    side: their ``<connection>`` resolves ``incomingRoad`` to a regular
    road so CARLA can parse the map (#500), but they are geometrically
    pinned to their upstream connector, recognised here by that
    coincidence.
    """
    xodr_path = _build_nishishinjuku_xodr()

    tree = ET.parse(str(xodr_path))
    root = tree.getroot()

    endpoints = evaluate_road_endpoints(root)

    road_by_id: dict[int, ET._Element] = {}
    road_junction: dict[int, int] = {}
    for road_elem in root.findall("road"):
        rid = int(road_elem.get("id"))
        road_by_id[rid] = road_elem
        road_junction[rid] = int(road_elem.get("junction", "-1"))

    # Build per-connecting-road junction info: incoming roads + outermost
    # ``<laneLink>`` (the one mapping to lane ±1) + connection contact point.
    # ``conn_road_outer_link[cr][incoming_road_id] = from_lane_id`` records
    # the regular-road lane that links to the connecting road's outermost
    # lane — the only one the override can pin.
    conn_road_incomings: dict[int, set[int]] = {}
    conn_road_outer_link: dict[int, dict[int, int]] = {}
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

            # Find the laneLink mapping to the connecting road's outermost
            # lane (``to == ±1``) — the lane the override pins.
            for lane_link in conn_elem.findall("laneLink"):
                to_id = int(lane_link.get("to"))
                if abs(to_id) != 1:
                    continue
                from_id = int(lane_link.get("from"))
                conn_road_outer_link.setdefault(connecting_road_id, {})[
                    incoming_road_id
                ] = from_id

    offenders: list[tuple[int, int, str, int, float]] = []

    for connecting_road_id, incoming_ids in conn_road_incomings.items():
        junction_id = conn_road_junction[connecting_road_id]
        contact_point = conn_road_contact[connecting_road_id]

        if connecting_road_id not in endpoints:
            continue
        if connecting_road_id not in road_by_id:
            continue
        conn_start, conn_end = endpoints[connecting_road_id]

        # ----- Incoming side -----
        # Skip multi-incoming connections: the override can pin only one.
        if len(incoming_ids) == 1:
            (incoming_road_id,) = incoming_ids
            if (
                incoming_road_id in road_by_id
                and road_junction.get(incoming_road_id, -1) == -1
            ):
                from_lane_id = conn_road_outer_link.get(connecting_road_id, {}).get(
                    incoming_road_id
                )
                if from_lane_id is not None:
                    inc_at_start = contact_point != "start"
                    expected_in = _evaluate_lane_inner_edge(
                        road_by_id[incoming_road_id],
                        from_lane_id,
                        at_start=inc_at_start,
                    )
                    actual_in = conn_start if contact_point == "start" else conn_end
                    if expected_in is not None:
                        d_in = _distance3(expected_in, actual_in)
                        # A chained connector resolves incomingRoad to a
                        # regular road for CARLA (#500) but is pinned to its
                        # upstream connector — exclude it from the
                        # regular-road check, as with multi-incoming cases.
                        if (
                            d_in > TOLERANCE_M
                            and not _incoming_endpoint_pinned_to_connector(
                                actual_in,
                                junction_id,
                                connecting_road_id,
                                road_by_id,
                                conn_road_junction,
                            )
                        ):
                            offenders.append(
                                (
                                    junction_id,
                                    connecting_road_id,
                                    "incoming",
                                    incoming_road_id,
                                    d_in,
                                )
                            )

        # ----- Outgoing side -----
        # Read the outgoing lane via the connecting road's outermost lane's
        # lane-level link (predecessor when contactPoint=end, successor when
        # contactPoint=start).
        conn_road_elem = road_by_id[connecting_road_id]
        outer_lane_id = _outermost_lane_id(conn_road_elem)
        if outer_lane_id is None:
            continue

        if contact_point == "start":
            road_link = conn_road_elem.find("link")
            link_kind = "successor"
        else:
            road_link = conn_road_elem.find("link")
            link_kind = "predecessor"
        if road_link is None:
            continue
        outgoing_link = road_link.find(link_kind)
        if outgoing_link is None or outgoing_link.get("elementType") != "road":
            continue
        outgoing_road_id = int(outgoing_link.get("elementId"))
        if road_junction.get(outgoing_road_id, -1) != -1:
            continue
        if outgoing_road_id not in road_by_id:
            continue
        outgoing_contact = outgoing_link.get("contactPoint")
        out_at_start = outgoing_contact == "start"

        target_lane_id = _outermost_lane_link_target(
            conn_road_elem, outer_lane_id, link_kind
        )
        if target_lane_id is None:
            continue

        expected_out = _evaluate_lane_inner_edge(
            road_by_id[outgoing_road_id], target_lane_id, at_start=out_at_start
        )
        actual_out = conn_end if contact_point == "start" else conn_start
        if expected_out is None:
            continue
        d_out = _distance3(expected_out, actual_out)
        out_label = "end" if contact_point == "start" else "start"
        if d_out > TOLERANCE_M:
            offenders.append(
                (
                    junction_id,
                    connecting_road_id,
                    out_label,
                    outgoing_road_id,
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


def test_nishishinjuku_emits_junction_priorities() -> None:
    """End-to-end: 85 right_of_way REs in nishishinjuku produce > 0 <priority> nodes.

    Also runs qc-framework against the produced .xodr so a malformed
    `<priority>` shape (wrong attribute order, schema-illegal placement, etc.)
    surfaces here rather than only via downstream consumers.
    """
    from autoware_lanelet2_to_opendrive.qc_validate import (
        load_ignore_patterns,
        validate,
    )

    xodr_path = _build_nishishinjuku_xodr()
    tree = ET.parse(str(xodr_path))

    priorities = tree.findall(".//junction/priority")
    assert len(priorities) > 0, (
        f"expected at least one <priority> from 85 right_of_way REs, "
        f"got {len(priorities)} in {xodr_path}"
    )

    for p in priorities:
        assert "high" in p.attrib
        assert "low" in p.attrib
        assert p.get("high") != p.get("low"), f"self-priority emitted: {p.attrib}"

    # Every referenced road must exist in the map.
    road_ids = {r.get("id") for r in tree.findall(".//road")}
    for p in priorities:
        assert p.get("high") in road_ids, p.attrib
        assert p.get("low") in road_ids, p.attrib

    # qc-framework must also accept the emitted XML.
    errors = validate(xodr_path, load_ignore_patterns())
    assert errors == 0, f"qc-framework reported {errors} ERROR(s) on {xodr_path}"
