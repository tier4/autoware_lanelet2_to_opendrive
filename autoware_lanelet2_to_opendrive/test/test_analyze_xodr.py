"""Tests for analyze_xodr helpers."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from autoware_lanelet2_to_opendrive.analyze_xodr import (
    _evaluate_road_xy,
    _load_validation_context,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry import Arc, Line, PlanView
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    GeoRoadLaneletMapping,
    ProjectionMetadata,
    build_mapping,
    parse_roads_from_xodr,
    save_mapping_json,
)


class TestEvaluateRoadXy:
    """``_evaluate_road_xy`` must evaluate line and arc geometry, not only
    paramPoly3 (#502).

    The analyze/QC path reconstructs roads from the XODR; once
    ``parse_roads_from_xodr`` keeps ``<arc>`` and ``<line>`` segments, the
    Frenet-to-world evaluator has to handle them too — previously it read
    paramPoly3 coefficients directly and would fail on other primitives.
    """

    def test_arc_geometry_evaluated_along_its_curve(self) -> None:
        # Quarter circle of radius 10 m, starting at the origin heading +x.
        radius = 10.0
        curvature = 1.0 / radius
        length = radius * math.pi / 2.0
        road = Road(
            id=1,
            plan_view=PlanView(
                geometries=[
                    Arc(
                        s=0.0,
                        x=0.0,
                        y=0.0,
                        hdg=0.0,
                        length=length,
                        curvature=curvature,
                    )
                ]
            ),
        )

        # A point partway along the arc lies on the analytic circle.
        p = length / 3.0
        assert _evaluate_road_xy(road, p) == pytest.approx(
            (
                math.sin(curvature * p) / curvature,
                (1.0 - math.cos(curvature * p)) / curvature,
            ),
            abs=1e-6,
        )
        # The quarter circle ends exactly at (radius, radius).
        assert _evaluate_road_xy(road, length) == pytest.approx(
            (radius, radius), abs=1e-6
        )

    def test_line_geometry_with_lateral_offset(self) -> None:
        # A 20 m line heading +x from the origin.
        road = Road(
            id=2,
            plan_view=PlanView(
                geometries=[Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=20.0)]
            ),
        )

        assert _evaluate_road_xy(road, 5.0) == pytest.approx((5.0, 0.0), abs=1e-6)
        # Positive t is to the left of travel; heading +x -> left is +y.
        assert _evaluate_road_xy(road, 5.0, 3.0) == pytest.approx((5.0, 3.0), abs=1e-6)


class TestValidationProjectionMetadata:
    def _write_xodr(self, tmp_path: Path) -> Path:
        xodr_path = tmp_path / "map.xodr"
        xodr_path.write_text(
            """\
<OpenDRIVE>
  <header>
    <geoReference>
      +proj=utm +zone=54 +lat_0=35.64594559004192 +lon_0=139.80712515470023 +datum=WGS84 +units=m +no_defs
    </geoReference>
  </header>
  <road id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <left>
          <lane id="1" type="driving" level="false"/>
        </left>
        <center>
          <lane id="0" type="none" level="false"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
""",
            encoding="utf-8",
        )
        return xodr_path

    def _lanelet_map(self, offset_x: float, offset_y: float):
        import lanelet2

        lanelet_map = lanelet2.core.LaneletMap()
        left = lanelet2.core.LineString3d(
            10,
            [
                lanelet2.core.Point3d(1, offset_x, offset_y + 3.0, 0.0),
                lanelet2.core.Point3d(2, offset_x + 10.0, offset_y + 3.0, 0.0),
            ],
        )
        right = lanelet2.core.LineString3d(
            11,
            [
                lanelet2.core.Point3d(3, offset_x, offset_y, 0.0),
                lanelet2.core.Point3d(4, offset_x + 10.0, offset_y, 0.0),
            ],
        )
        lanelet = lanelet2.core.Lanelet(100, left, right)
        lanelet.attributes["subtype"] = "road"
        lanelet_map.add(lanelet)
        return lanelet_map

    def test_sidecar_projection_metadata_reuses_exact_fractional_offset(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        offset_x = 100.5
        offset_y = 200.1
        lanelet_map = self._lanelet_map(offset_x, offset_y)
        xodr_path = self._write_xodr(tmp_path)
        osm_path = tmp_path / "map.osm"
        osm_path.write_text("<osm/>", encoding="utf-8")

        save_mapping_json(
            GeoRoadLaneletMapping(
                xodr_sha256="xodr",
                osm_sha256="osm",
                lanelet_to_road_and_lane={100: (1, 1)},
                projection_metadata=ProjectionMetadata(
                    projector_type="MGRSProjector",
                    mgrs_code="54SUE",
                    origin_lat=35.64594559004192,
                    origin_lon=139.80712515470023,
                    offset_x=offset_x,
                    offset_y=offset_y,
                ),
            ),
            xodr_path,
        )

        import lanelet2

        monkeypatch.setattr(lanelet2.io, "load", lambda *_args: lanelet_map)

        ctx = _load_validation_context(xodr_path, osm_path)
        assert ctx is not None
        assert ctx.offset_x == pytest.approx(offset_x)
        assert ctx.offset_y == pytest.approx(offset_y)

        roads = parse_roads_from_xodr(xodr_path, xodr_root=ctx.xodr_root)
        converter_geo = build_mapping(
            lanelet_map, roads, (offset_x, offset_y), "x", "o"
        )
        analyze_geo = build_mapping(
            ctx.lanelet_map, roads, (ctx.offset_x, ctx.offset_y), "x", "o"
        )

        assert converter_geo.lanelet_to_road_and_lane == {100: (1, 1)}
        assert (
            analyze_geo.lanelet_to_road_and_lane
            == converter_geo.lanelet_to_road_and_lane
        )

    def test_old_sidecar_falls_back_to_georeference_offset(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        lanelet_map = self._lanelet_map(0.0, 0.0)
        xodr_path = self._write_xodr(tmp_path)
        osm_path = tmp_path / "map.osm"
        osm_path.write_text("<osm/>", encoding="utf-8")

        save_mapping_json(
            GeoRoadLaneletMapping(
                xodr_sha256="xodr",
                osm_sha256="osm",
                lanelet_to_road_and_lane={100: (1, 1)},
            ),
            xodr_path,
        )

        import lanelet2

        monkeypatch.setattr(lanelet2.io, "load", lambda *_args: lanelet_map)

        ctx = _load_validation_context(xodr_path, osm_path)
        assert ctx is not None
        assert ctx.offset_x == pytest.approx(92007.99999999854)
        assert ctx.offset_y == pytest.approx(45335.00000000186)
