"""End-to-end test: synthetic spline -> road planView with <arc> (#466)."""

import lxml.etree as ET
import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import (
    ArcSpiralConfig,
    ParamPoly3Config,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    PlanView,
    evaluate_road_endpoints,
)
from autoware_lanelet2_to_opendrive.opendrive.road import (
    _build_planview_geometries,
)
from autoware_lanelet2_to_opendrive.spline import Splines


def _line_arc_line_spline(
    line1_len: float = 30.0,
    radius: float = 200.0,
    arc_rad: float = 0.3,
    line2_len: float = 30.0,
    n: int = 240,
):
    n1 = n // 3
    seg1 = np.column_stack(
        [np.linspace(0.0, line1_len, n1), np.zeros(n1), np.zeros(n1)]
    )
    n2 = n // 3
    theta = np.linspace(-np.pi / 2.0, -np.pi / 2.0 + arc_rad, n2)
    seg2 = np.column_stack(
        [
            line1_len + radius * np.cos(theta),
            radius + radius * np.sin(theta),
            np.zeros(n2),
        ]
    )
    end_pos = seg2[-1]
    end_tan_angle = theta[-1] + np.pi / 2.0
    n3 = n - n1 - n2
    ts = np.linspace(0.0, line2_len, n3)
    seg3 = np.column_stack(
        [
            end_pos[0] + ts * np.cos(end_tan_angle),
            end_pos[1] + ts * np.sin(end_tan_angle),
            np.zeros(n3),
        ]
    )
    pts = np.vstack([seg1[:-1], seg2[:-1], seg3])
    return Splines(pts)


def test_disabled_emits_paramPoly3_only():
    spline = _line_arc_line_spline()
    geoms = _build_planview_geometries(
        spline,
        parampoly3_config=ParamPoly3Config(),
        arcspiral_config=ArcSpiralConfig(enabled=False),
    )
    pv = PlanView(geometries=geoms)
    xml = pv.to_xml()
    assert xml.find(".//arc") is None
    assert xml.find(".//paramPoly3") is not None


def test_enabled_emits_at_least_one_arc():
    spline = _line_arc_line_spline()
    geoms = _build_planview_geometries(
        spline,
        parampoly3_config=ParamPoly3Config(),
        arcspiral_config=ArcSpiralConfig(enabled=True),
    )
    pv = PlanView(geometries=geoms)
    xml = pv.to_xml()
    arcs = xml.findall(".//arc")
    assert len(arcs) >= 1
    expected_kappa = 1.0 / 200.0
    arc_kappas = [abs(float(a.get("curvature"))) for a in arcs]
    assert min(abs(k - expected_kappa) for k in arc_kappas) < 5e-4


def test_endpoint_round_trip_through_evaluate_road_endpoints():
    spline = _line_arc_line_spline()
    geoms = _build_planview_geometries(
        spline,
        parampoly3_config=ParamPoly3Config(),
        arcspiral_config=ArcSpiralConfig(enabled=True),
    )
    # Wrap into a minimal <OpenDRIVE><road><planView/></road></OpenDRIVE>.
    root = ET.Element("OpenDRIVE")
    road = ET.SubElement(root, "road", id="1")
    pv = ET.SubElement(road, "planView")
    for g in geoms:
        pv.append(g.to_xml())
    endpoints = evaluate_road_endpoints(root)
    (start, end) = endpoints[1]
    expected_start = spline.evaluate(0.0, derivative=0)
    expected_end = spline.evaluate(spline.total_length, derivative=0)
    assert start[0] == pytest.approx(expected_start[0], abs=0.05)
    assert start[1] == pytest.approx(expected_start[1], abs=0.05)
    assert end[0] == pytest.approx(expected_end[0], abs=0.10)
    assert end[1] == pytest.approx(expected_end[1], abs=0.10)


def test_short_spline_becomes_single_straight_segment():
    """A spline shorter than min_segment_length must yield one straight segment."""
    points = np.array([[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]])
    spline = Splines(points, num_control_points=4)
    with pytest.warns(UserWarning, match="single straight segment"):
        geoms = _build_planview_geometries(
            spline,
            parampoly3_config=ParamPoly3Config(),
            arcspiral_config=ArcSpiralConfig(enabled=True),
        )
    assert len(geoms) == 1
    assert geoms[0].length == pytest.approx(0.3, rel=1e-3)
    assert geoms[0].bU == pytest.approx(1.0, rel=1e-3)
    assert (geoms[0].cU, geoms[0].dU, geoms[0].cV, geoms[0].dV) == (0.0, 0.0, 0.0, 0.0)


@pytest.mark.xfail(
    strict=True,
    reason="ArcSpiralConfig.enabled defaults to False, so every record is a paramPoly3",
)
def test_shipped_conversion_emits_line_and_arc_primitives(nishishinjuku_xodr):
    """The default conversion classifies its plan view, not just fits it.

    Every geometry record used to be a ``<paramPoly3>``, including the
    stretches that are exactly straight or exactly circular. OpenDRIVE has
    primitives for those, every published map uses them, and a consumer that
    reasons about road shape -- a lane-keeping controller, a curvature
    profile, a human reading the file -- gets a worse description from a
    cubic fit of a straight line than from ``<line>``.

    The counts are deliberately loose: what is pinned is that the classifier
    runs by default and finds both primitives on a real map, not how many of
    each a particular fit produces.
    """
    root = ET.parse(str(nishishinjuku_xodr)).getroot()

    lines = root.findall(".//planView/geometry/line")
    arcs = root.findall(".//planView/geometry/arc")
    poly3 = root.findall(".//planView/geometry/paramPoly3")
    total = len(lines) + len(arcs) + len(poly3)

    assert total > 0, "no plan-view geometry at all; the fixture is wrong"
    assert lines, (
        f"no <line> in {total} geometry records: the arc/line classifier is "
        f"disabled, so a straight road is emitted as a cubic fit of itself"
    )
    assert arcs, (
        f"no <arc> in {total} geometry records: the arc/line classifier is "
        f"disabled, so a circular road is emitted as a cubic fit of itself"
    )
