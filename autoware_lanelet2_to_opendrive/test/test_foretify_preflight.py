"""Threshold unit tests for the Foretify documented-requirements preflight."""

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.foretify_preflight import (
    CLASS_CONVERTER,
    CLASS_PROXY_LIMITATION,
    PreflightConfig,
    check_border_connection_jitter,
    check_driving_non_driving_connections,
    check_lane_connections_geometry,
    check_lane_ref_line_jitter,
    check_missing_logical_connections,
    check_msp_length_proxy,
    check_neighbor_lanes,
    check_opposite_roads_overlap,
    developer_log_proxies,
    geometry_statistics,
    lane_connections,
    lane_registry,
    neighbor_border_deviation,
    opposite_penetration,
    parse_roads,
    run_preflight,
)

CONFIG = PreflightConfig()


def _road(
    road_id: str,
    *,
    x: float = 0.0,
    y: float = 0.0,
    hdg: float = 0.0,
    length: float = 20.0,
    z: float = 0.0,
    lane_types=("driving",),
    width: float = 3.5,
    junction: int = -1,
    succ: str = "",
    pred: str = "",
    lane_succ=None,
    lane_pred=None,
    geometries: str = "",
    left_lane_types=(),
) -> str:
    if not geometries:
        geometries = (
            f'<geometry s="0" x="{x}" y="{y}" hdg="{hdg}" length="{length}">'
            "<line/></geometry>"
        )
    right = ""
    for index, lane_type in enumerate(lane_types, start=1):
        link = ""
        if lane_succ is not None:
            link += f'<successor id="{lane_succ}"/>'
        if lane_pred is not None:
            link += f'<predecessor id="{lane_pred}"/>'
        if link:
            link = f"<link>{link}</link>"
        right += (
            f'<lane id="-{index}" type="{lane_type}" level="false">{link}'
            f'<width sOffset="0" a="{width}" b="0" c="0" d="0"/></lane>'
        )
    left = ""
    for index, lane_type in enumerate(left_lane_types, start=1):
        left += (
            f'<lane id="{index}" type="{lane_type}" level="false">'
            f'<width sOffset="0" a="{width}" b="0" c="0" d="0"/></lane>'
        )
    return (
        f'<road id="{road_id}" length="{length}" junction="{junction}">'
        f"<link>{pred}{succ}</link>"
        f"<planView>{geometries}</planView>"
        f'<elevationProfile><elevation s="0" a="{z}" b="0" c="0" d="0"/>'
        "</elevationProfile>"
        "<lanes><laneSection s=\"0\">"
        + (f"<left>{left}</left>" if left else "")
        + '<center><lane id="0" type="none" level="false"/></center>'
        f"<right>{right}</right>"
        "</laneSection></lanes></road>"
    )


def _doc(*roads: str, junctions: str = "") -> ET.Element:
    return ET.fromstring(f"<OpenDRIVE>{''.join(roads)}{junctions}</OpenDRIVE>")


def _pair_doc(gap_x: float = 0.0, gap_y: float = 0.0, gap_z: float = 0.0, bend: float = 0.0):
    """Road 1 [0..20] linked to road 2 starting at (20+gap_x, gap_y)."""
    first = _road(
        "1",
        length=20.0,
        succ='<successor elementType="road" elementId="2" contactPoint="start"/>',
        lane_succ=-1,
    )
    second = _road(
        "2",
        x=20.0 + gap_x,
        y=gap_y,
        hdg=bend,
        z=gap_z,
        length=20.0,
        pred='<predecessor elementType="road" elementId="1" contactPoint="end"/>',
        lane_pred=-1,
    )
    return _doc(first, second)


def _connection_findings(root, anomalies):
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    found = check_lane_connections_geometry(roads, registry, connections, CONFIG)
    return [f for f in found if f.anomaly in anomalies]


# ---------------------------------------------------------------------------
# LANE_REF_LINE_JITTER
# ---------------------------------------------------------------------------


def _jitter_doc(angle: float) -> ET.Element:
    # Zero lane width keeps the lane center exactly on the reference line,
    # so the sampled angle change equals the constructed kink.
    geometries = (
        '<geometry s="0" x="0" y="0" hdg="0" length="6"><line/></geometry>'
        f'<geometry s="6" x="6" y="0" hdg="{angle}" length="6"><line/></geometry>'
    )
    return _doc(_road("1", length=12.0, width=0.0, geometries=geometries))


@pytest.mark.parametrize(
    "angle,expected",
    [
        (0.0, 0),  # normal
        (CONFIG.jitter_angle_threshold - 0.05, 0),  # just below
        (CONFIG.jitter_angle_threshold, 1),  # at threshold (>= fails)
        (CONFIG.jitter_angle_threshold + 0.05, 1),  # just above
    ],
)
def test_lane_ref_line_jitter_thresholds(angle: float, expected: int) -> None:
    root = _jitter_doc(angle)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    findings, tiers = check_lane_ref_line_jitter(roads, registry, CONFIG)
    assert len(findings) == expected
    if angle > math.radians(1.0):
        assert tiers["angle_gt_1.0deg"] >= 1


def test_lane_ref_line_jitter_diagnostic_tiers() -> None:
    root = _jitter_doc(math.radians(0.7))
    roads = parse_roads(root)
    registry = lane_registry(roads)
    findings, tiers = check_lane_ref_line_jitter(roads, registry, CONFIG)
    assert not findings  # far below the documented threshold
    assert tiers["angle_gt_0.1deg"] >= 1
    assert tiers["angle_gt_0.5deg"] >= 1
    assert tiers["angle_gt_1.0deg"] == 0


# ---------------------------------------------------------------------------
# Connection geometry: NOT_CONNECTED_GEOMETRICALLY vs CONNECTION jitter
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "gap,geometric,jitter",
    [
        (0.0, 0, 0),  # normal
        (0.24, 0, 0),  # below connection gap
        (0.25, 0, 0),  # at connection gap boundary (0.25 < gap required)
        (0.26, 0, 1),  # just above -> jitter gap failure
        (0.50, 0, 1),  # at geometric boundary stays jitter
        (0.51, 1, 0),  # above 0.50 -> not connected geometrically
    ],
)
def test_connection_gap_classification_xy(gap, geometric, jitter) -> None:
    found_geo = _connection_findings(
        _pair_doc(gap_x=gap), {"LANES_NOT_CONNECTED_GEOMETRICALLY"}
    )
    found_jit = [
        f
        for f in _connection_findings(
            _pair_doc(gap_x=gap), {"LANE_CONNECTION_REF_LINE_JITTER"}
        )
        if "gap failure" in f.detail
    ]
    assert len(found_geo) == geometric
    assert len(found_jit) == jitter
    # A pair is never classified twice.
    assert len(found_geo) + len(found_jit) <= 1


@pytest.mark.parametrize(
    "gap_z,geometric,jitter",
    [(0.2, 0, 0), (0.3, 0, 1), (0.6, 1, 0)],
)
def test_connection_gap_classification_z(gap_z, geometric, jitter) -> None:
    found_geo = _connection_findings(
        _pair_doc(gap_z=gap_z), {"LANES_NOT_CONNECTED_GEOMETRICALLY"}
    )
    found_jit = [
        f
        for f in _connection_findings(
            _pair_doc(gap_z=gap_z), {"LANE_CONNECTION_REF_LINE_JITTER"}
        )
        if "gap failure" in f.detail
    ]
    assert len(found_geo) == geometric
    assert len(found_jit) == jitter


@pytest.mark.parametrize(
    "bend,expected",
    [
        (0.0, 0),
        (CONFIG.jitter_angle_threshold - 0.1, 0),
        # folded tangent difference saturates at pi/2, above the documented
        # threshold only for tighter configurations; verify with a tighter
        # config instead.
    ],
)
def test_connection_angle_normal_cases(bend, expected) -> None:
    found = [
        f
        for f in _connection_findings(
            _pair_doc(bend=bend), {"LANE_CONNECTION_REF_LINE_JITTER"}
        )
        if "angle failure" in f.detail
    ]
    assert len(found) == expected


def test_connection_angle_threshold_with_tight_config() -> None:
    config = PreflightConfig(min_expected_radius=150.0)  # threshold 0.02 rad
    for bend, expected in ((0.019, 0), (0.02, 1), (0.021, 1)):
        root = _pair_doc(bend=bend)
        roads = parse_roads(root)
        registry = lane_registry(roads)
        connections = lane_connections(root, roads)
        found = [
            f
            for f in check_lane_connections_geometry(
                roads, registry, connections, config
            )
            if "angle failure" in f.detail
        ]
        assert len(found) == expected, bend


# ---------------------------------------------------------------------------
# Border connection jitter
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "lateral,expected",
    [(0.0, 0), (0.2, 0), (0.24, 0), (0.26, 1)],
)
def test_border_connection_jitter_thresholds(lateral, expected) -> None:
    root = _pair_doc(gap_y=lateral)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    found = check_border_connection_jitter(roads, registry, connections, CONFIG)
    assert len(found) == expected


# ---------------------------------------------------------------------------
# Neighbor / opposite lanes (measurement helpers exercise the thresholds;
# single-road XODR lanes share borders by construction)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "offset,expected",
    [(0.0, 0), (0.29, 0), (0.30, 0), (0.31, 1)],
)
def test_neighbor_border_deviation_thresholds(offset, expected) -> None:
    stations = np.linspace(0.0, 30.0, 11)
    base = np.column_stack([stations, np.zeros_like(stations), np.zeros_like(stations)])
    shifted = base.copy()
    shifted[:, 1] += offset
    deviation = neighbor_border_deviation(base, shifted)
    assert (deviation > CONFIG.max_lateral_gap_or_overlap) == bool(expected)


def test_multi_lane_road_has_no_neighbor_findings() -> None:
    root = _doc(_road("1", lane_types=("driving", "driving", "driving")))
    findings = check_neighbor_lanes(parse_roads(root), CONFIG)
    assert findings == []


@pytest.mark.parametrize(
    "penetration,expected",
    [(-0.1, 0), (0.29, 0), (0.30, 0), (0.31, 1)],
)
def test_opposite_penetration_thresholds(penetration, expected) -> None:
    stations = np.linspace(0.0, 30.0, 11)
    left = np.zeros_like(stations)
    right = left + penetration
    value = opposite_penetration(left, right)
    assert (value > CONFIG.max_lateral_gap_or_overlap) == bool(expected)


def test_bidirectional_road_has_no_opposite_overlap() -> None:
    root = _doc(
        _road("1", lane_types=("driving",), left_lane_types=("driving",))
    )
    findings = check_opposite_roads_overlap(parse_roads(root), CONFIG)
    assert findings == []


# ---------------------------------------------------------------------------
# Driving / non-driving connections
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "second_type,expected",
    [("driving", 0), ("sidewalk", 1), ("biking", 1), ("shoulder", 1), ("border", 1), ("restricted", 1)],
)
def test_driving_non_driving_connection(second_type, expected) -> None:
    first = _road(
        "1",
        succ='<successor elementType="road" elementId="2" contactPoint="start"/>',
        lane_succ=-1,
    )
    second = _road(
        "2",
        x=20.0,
        lane_types=(second_type,),
        pred='<predecessor elementType="road" elementId="1" contactPoint="end"/>',
    )
    root = _doc(first, second)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    found = check_driving_non_driving_connections(registry, connections)
    assert len(found) == expected


# ---------------------------------------------------------------------------
# Missing logical connections (incl. junction connector and chains)
# ---------------------------------------------------------------------------


def test_missing_logical_connection_detected_without_links() -> None:
    first = _road("1")
    second = _road("2", x=20.0)
    root = _doc(first, second)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    found = check_missing_logical_connections(
        roads, registry, connections, CONFIG
    )
    assert [f.anomaly for f in found] == ["LANES_NOT_CONNECTED_LOGICALLY"]


def test_logical_connection_direct_link_suppresses_finding() -> None:
    root = _pair_doc()
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    assert (
        check_missing_logical_connections(roads, registry, connections, CONFIG)
        == []
    )


def test_logical_connection_through_short_intermediate_connector() -> None:
    first = _road(
        "1",
        succ='<successor elementType="road" elementId="9" contactPoint="start"/>',
        lane_succ=-1,
    )
    connector = _road(
        "9",
        x=20.0,
        length=0.6,
        junction=7,
        succ='<successor elementType="road" elementId="2" contactPoint="start"/>',
        lane_succ=-1,
        lane_pred=-1,
    )
    second = _road(
        "2",
        x=20.6,
        pred='<predecessor elementType="road" elementId="9" contactPoint="end"/>',
        lane_pred=-1,
    )
    junctions = (
        '<junction id="7"><connection id="0" incomingRoad="1"'
        ' connectingRoad="9" contactPoint="start">'
        '<laneLink from="-1" to="-1"/></connection></junction>'
    )
    root = _doc(first, connector, second, junctions=junctions)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    assert any(c.via == "junction" for c in connections)
    assert (
        check_missing_logical_connections(roads, registry, connections, CONFIG)
        == []
    )


def test_logical_connection_heading_mismatch_not_reported() -> None:
    # Continuation is geometric only if headings agree; a sharply bent
    # candidate is not a missing connection.
    first = _road("1")
    second = _road("2", x=20.0, hdg=0.5)
    root = _doc(first, second)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    assert (
        check_missing_logical_connections(roads, registry, connections, CONFIG)
        == []
    )


def test_logical_connection_left_lane_orientation_flag() -> None:
    # LHT output: left lanes linked end->start, travel along +s.
    first = _road(
        "1",
        lane_types=(),
        left_lane_types=("driving",),
        succ='<successor elementType="road" elementId="2" contactPoint="start"/>',
    ).replace('<lane id="1" type="driving" level="false">', (
        '<lane id="1" type="driving" level="false">'
        '<link><successor id="1"/></link>'
    ), 1)
    second = _road(
        "2",
        x=20.0,
        lane_types=(),
        left_lane_types=("driving",),
        pred='<predecessor elementType="road" elementId="1" contactPoint="end"/>',
    )
    root = _doc(first, second)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    config = PreflightConfig(left_lanes_travel_against_s=False)
    assert (
        check_missing_logical_connections(roads, registry, connections, config)
        == []
    )


def test_logical_connection_short_lanes_skipped() -> None:
    first = _road("1", length=0.9)
    second = _road("2", x=0.9, length=0.9)
    root = _doc(first, second)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    assert (
        check_missing_logical_connections(roads, registry, connections, CONFIG)
        == []
    )


# ---------------------------------------------------------------------------
# MSP length proxy
# ---------------------------------------------------------------------------


def _arc_doc(radius: float, *, left: bool) -> ET.Element:
    curvature = (1.0 if left else -1.0) / radius
    length = radius * math.pi / 2
    geometries = (
        f'<geometry s="0" x="0" y="0" hdg="0" length="{length}">'
        f'<arc curvature="{curvature}"/></geometry>'
    )
    return _doc(_road("1", length=length, geometries=geometries))


def test_msp_proxy_straight_lane_consistent() -> None:
    root = _doc(_road("1"))
    roads = parse_roads(root)
    registry = lane_registry(roads)
    assert check_msp_length_proxy(roads, registry, CONFIG) == []


@pytest.mark.parametrize(
    "radius,expected",
    [
        (30.0, 0),  # right lane center offset 1.75 m: ratio ~ 1.058 < 1.10
        (16.0, 1),  # ratio ~ 1.109 just above the 0.10 factor
    ],
)
def test_msp_proxy_offset_on_curvature(radius, expected) -> None:
    root = _arc_doc(radius, left=False)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    findings = check_msp_length_proxy(roads, registry, CONFIG)
    assert len(findings) == expected
    for finding in findings:
        # Fully explained by lateral offset progression -> proxy limitation.
        assert finding.classification == CLASS_PROXY_LIMITATION


def test_msp_proxy_declared_length_lie_is_converter_class() -> None:
    # A road whose declared length disagrees with its actual geometry span
    # produces an unexplained inconsistency.
    geometries = (
        '<geometry s="0" x="0" y="0" hdg="0" length="24"><line/></geometry>'
    )
    road = _road("1", length=20.0, geometries=geometries).replace(
        'length="20.0" junction', 'length="24" junction', 1
    )
    # declared road length 24 but lane section spans stations sampled over
    # 24 m of a straight line: consistent; instead lie via paramPoly3.
    poly = (
        '<geometry s="0" x="0" y="0" hdg="0" length="20">'
        '<paramPoly3 aU="0" bU="1.2" cU="0" dU="0" aV="0" bV="0" cV="0"'
        ' dV="0" pRange="arcLength"/></geometry>'
    )
    root = _doc(_road("1", length=20.0, geometries=poly))
    roads = parse_roads(root)
    registry = lane_registry(roads)
    findings = check_msp_length_proxy(roads, registry, CONFIG)
    assert len(findings) == 1
    assert findings[0].classification == CLASS_CONVERTER
    assert road  # silence unused warning for the illustrative variant


# ---------------------------------------------------------------------------
# Developer-log proxies and geometry statistics
# ---------------------------------------------------------------------------


def test_developer_log_proxies_on_clean_pair() -> None:
    root = _pair_doc()
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    proxies = developer_log_proxies(roads, registry, connections, CONFIG)
    assert all(not findings for findings in proxies.values())


def test_hiccup_1_and_2_and_6_detected() -> None:
    # Internal C0 gap + declared length mismatch.
    geometries = (
        '<geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry>'
        '<geometry s="10" x="10.5" y="0" hdg="0" length="10"><line/></geometry>'
    )
    broken = _road("1", length=21.0, geometries=geometries)
    root = _doc(broken)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    proxies = developer_log_proxies(roads, registry, connections, CONFIG)
    assert len(proxies["MAP_HICCUP_2_PROXY"]) == 1
    assert len(proxies["MAP_HICCUP_6_PROXY"]) == 1
    # Endpoint separation on a gapped connection.
    gapped = _pair_doc(gap_x=0.2)
    roads = parse_roads(gapped)
    registry = lane_registry(roads)
    connections = lane_connections(gapped, roads)
    proxies = developer_log_proxies(roads, registry, connections, CONFIG)
    assert len(proxies["MAP_HICCUP_1_PROXY"]) == 1


def test_possible_map_discrepancy_proxy() -> None:
    # Linked roads whose driving lane continues geometrically but carries
    # no laneLink.
    first = _road(
        "1",
        succ='<successor elementType="road" elementId="2" contactPoint="start"/>',
    )
    second = _road(
        "2",
        x=20.0,
        pred='<predecessor elementType="road" elementId="1" contactPoint="end"/>',
    )
    root = _doc(first, second)
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    proxies = developer_log_proxies(roads, registry, connections, CONFIG)
    assert len(proxies["POSSIBLE_MAP_DISCREPANCY_PROXY"]) == 1


def test_geometry_statistics_fields() -> None:
    root = _pair_doc()
    stats = geometry_statistics(parse_roads(root))
    counts = stats["counts"]
    assert isinstance(counts, dict)
    assert counts["line"] == 2
    assert stats["max_declared_vs_sum_gap"] == pytest.approx(0.0, abs=1e-9)
    assert stats["non_stub_leq_5cm"] == 0
    assert stats["per_road_geometry_max"] == 1


def test_run_preflight_reports_notes() -> None:
    report = run_preflight(_pair_doc())
    assert any("DOCUMENTATION_INSUFFICIENT" in note for note in report.notes)
    assert any("NOT REPRODUCIBLE WITHOUT" in note for note in report.notes)
    assert any("NOT EXECUTED" in note for note in report.notes)
