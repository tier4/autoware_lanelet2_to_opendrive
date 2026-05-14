from argparse import Namespace
import unittest
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ll2tofbx.cli import _build_parser, _resolve_blender_script_path, _resolve_origin_shift
from ll2tofbx.config import ExportConfig
from ll2tofbx.mesh_types import MeshLayer, TriangleMesh, Vec3
from ll2tofbx.solid_builder import compute_origin_shift
from ll2tofbx.validate import validate_layers


class OriginAndValidationTests(unittest.TestCase):
    def test_origin_shift_recenters_xy_and_grounds_z(self):
        road_top_vertices = [
            Vec3(10.0, 20.0, 5.0),
            Vec3(14.0, 22.0, 5.0),
            Vec3(12.0, 26.0, 6.0),
        ]
        layer = MeshLayer(
            name="lanelet_road",
            source_feature_count=1,
            top_vertices=list(road_top_vertices),
            mesh=TriangleMesh(vertices=[Vec3(10.0, 20.0, 4.5)], faces=[]),
        )
        shift = compute_origin_shift(road_top_vertices, [layer])
        self.assertEqual(shift, Vec3(12.0, 23.0, 4.5))

    def test_validation_reports_degenerate_triangle(self):
        layer = MeshLayer(
            name="broken",
            source_feature_count=1,
            mesh=TriangleMesh(
                vertices=[
                    Vec3(0.0, 0.0, 0.0),
                    Vec3(1.0, 0.0, 0.0),
                    Vec3(2.0, 0.0, 0.0),
                ],
                faces=[(0, 1, 2)],
            ),
        )
        report = validate_layers([layer], 1e-8)
        self.assertGreater(report.degenerate_triangles, 0)
        self.assertTrue(report.errors)

    def test_validation_warns_on_negative_signed_volume(self):
        layer = MeshLayer(
            name="reversed",
            source_feature_count=1,
            mesh=TriangleMesh(
                vertices=[
                    Vec3(0.0, 0.0, 0.0),
                    Vec3(1.0, 0.0, 0.0),
                    Vec3(0.0, 1.0, 0.0),
                    Vec3(0.0, 0.0, 1.0),
                ],
                faces=[
                    (0, 1, 2),
                    (0, 3, 1),
                    (0, 2, 3),
                    (1, 3, 2),
                ],
            ),
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])
        self.assertEqual(report.inward_faces, 1)
        self.assertEqual(len(report.warnings), 1)

    def test_explicit_origin_shift_uses_configured_values(self):
        road_top_vertices = [
            Vec3(10.0, 20.0, 5.0),
            Vec3(14.0, 22.0, 5.0),
        ]
        layer = MeshLayer(
            name="lanelet_road",
            source_feature_count=1,
            top_vertices=list(road_top_vertices),
            mesh=TriangleMesh(vertices=[Vec3(10.0, 20.0, 4.5)], faces=[]),
        )
        config = ExportConfig(
            input_path=Path("input.osm"),
            output_path=Path("output.fbx"),
            report_path=Path("report.json"),
            log_path=Path("export.log"),
            origin_policy="explicit",
            origin_shift_x=92008.5,
            origin_shift_y=45335.1,
            origin_shift_z=0.0,
        )

        shift = _resolve_origin_shift(config, road_top_vertices, [layer])

        self.assertEqual(shift, Vec3(92008.5, 45335.1, 0.0))

    def test_shift_args_require_explicit_origin(self):
        args = Namespace(
            input="input.osm",
            output="output.fbx",
            report="report.json",
            log=None,
            road_thickness=0.5,
            surface_style="solid",
            lanelet_side_overlap=0.0,
            wall_height=2.0,
            wall_thickness=0.1,
            marking_width=0.05,
            marking_thickness=0.01,
            marking_offset=0.002,
            marking_style="prism",
            keep_intermediate=False,
            tmp_dir=None,
            origin="center",
            shift_x=1.0,
            shift_y=0.0,
            shift_z=0.0,
        )

        with self.assertRaisesRegex(
            ValueError, "--shift-x/--shift-y/--shift-z require --origin explicit."
        ):
            ExportConfig.from_args(args)

    def test_surface_and_marking_style_defaults_and_flat_modes(self):
        parser = _build_parser()

        default_args = parser.parse_args(
            ["export", "--input", "input.osm", "--output", "output.fbx", "--report", "report.json"]
        )
        flat_args = parser.parse_args(
            [
                "export",
                "--input",
                "input.osm",
                "--output",
                "output.fbx",
                "--report",
                "report.json",
                "--surface-style",
                "flat",
                "--marking-style",
                "flat",
            ]
        )

        self.assertEqual(default_args.surface_style, "solid")
        self.assertEqual(default_args.marking_style, "prism")
        self.assertEqual(flat_args.surface_style, "flat")
        self.assertEqual(flat_args.marking_style, "flat")

    def test_blender_script_path_exists(self):
        self.assertTrue(_resolve_blender_script_path().exists())


if __name__ == "__main__":
    unittest.main()
