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
