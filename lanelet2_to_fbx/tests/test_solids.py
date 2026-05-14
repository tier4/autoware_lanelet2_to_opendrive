import unittest
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ll2tofbx.mesh_types import LaneletFeature, MeshLayer, PolygonFeature, PolylineFeature, TriangleMesh, Vec3
from ll2tofbx.solid_builder import (
    _ensure_closed_feature_mesh,
    build_marking_layer,
    build_surface_layer,
    build_wall_layer,
)
from ll2tofbx.validate import mesh_component_signed_volumes, validate_layers


class SolidBuilderTests(unittest.TestCase):
    def test_surface_prism_is_closed(self):
        layer = build_surface_layer(
            layer_name="lanelet_road",
            features=[
                PolygonFeature(
                    feature_id=1,
                    points=[
                        Vec3(0.0, 0.0, 0.0),
                        Vec3(2.0, 0.0, 0.0),
                        Vec3(2.0, 2.0, 0.0),
                        Vec3(0.0, 2.0, 0.0),
                    ],
                )
            ],
            thickness=0.5,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])

    def test_lanelet_strip_with_shared_endpoints_is_closed(self):
        layer = build_surface_layer(
            layer_name="lanelet_road",
            features=[
                LaneletFeature(
                    feature_id=10,
                    left=[
                        Vec3(0.0, 0.0, 0.0),
                        Vec3(2.0, 0.0, 0.0),
                        Vec3(4.0, 0.0, 0.0),
                    ],
                    right=[
                        Vec3(0.0, 0.0, 0.0),
                        Vec3(2.0, 2.0, 0.0),
                        Vec3(4.0, 0.0, 0.0),
                    ],
                )
            ],
            thickness=0.5,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])

    def test_lanelet_side_overlap_expands_surface_width(self):
        layer = build_surface_layer(
            layer_name="lanelet_road",
            features=[
                LaneletFeature(
                    feature_id=13,
                    left=[
                        Vec3(0.0, 1.0, 0.0),
                        Vec3(4.0, 1.0, 0.0),
                    ],
                    right=[
                        Vec3(0.0, 0.0, 0.0),
                        Vec3(4.0, 0.0, 0.0),
                    ],
                )
            ],
            thickness=0.5,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
            side_overlap=0.5,
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])
        aabb = layer.mesh.aabb()
        self.assertIsNotNone(aabb)
        self.assertAlmostEqual(aabb[0].y, -0.5)
        self.assertAlmostEqual(aabb[1].y, 1.5)

    def test_lanelet_strip_with_near_shared_endpoints_is_closed(self):
        layer = build_surface_layer(
            layer_name="shoulder",
            features=[
                LaneletFeature(
                    feature_id=11,
                    left=[
                        Vec3(0.0, 0.0, 0.0),
                        Vec3(2.0, 0.0, 0.0),
                        Vec3(4.0, 0.0, 0.0),
                    ],
                    right=[
                        Vec3(0.0005, 0.0003, 0.0),
                        Vec3(2.0, 1.5, 0.0),
                        Vec3(4.0004, 0.0002, 0.0),
                    ],
                )
            ],
            thickness=0.5,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])

    def test_lanelet_strip_with_interior_shared_endpoint_is_closed(self):
        layer = build_surface_layer(
            layer_name="lanelet_road",
            features=[
                LaneletFeature(
                    feature_id=12,
                    left=[
                        Vec3(0.0, 1.0, 0.0),
                        Vec3(4.0, 0.0, 0.0),
                    ],
                    right=[
                        Vec3(0.0, 0.0, 0.0),
                        Vec3(4.0, 0.0, 0.0),
                        Vec3(5.0, -1.0, 0.0),
                    ],
                )
            ],
            thickness=0.5,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])

    def test_repeated_vertex_polygon_is_repaired(self):
        layer = build_surface_layer(
            layer_name="hatched_area",
            features=[
                PolygonFeature(
                    feature_id=2291963,
                    points=[
                        Vec3(87029.0326, 41575.1722, 10.686),
                        Vec3(87031.3193, 41574.9179, 10.705),
                        Vec3(87033.3131, 41574.5472, 10.729),
                        Vec3(87035.0182, 41573.9651, 10.748),
                        Vec3(87036.9328, 41573.2266, 10.769),
                        Vec3(87039.2013, 41572.2340, 10.792),
                        Vec3(87040.7929, 41571.4514, 10.795),
                        Vec3(87042.2017, 41570.4782, 10.8),
                        Vec3(87043.2189, 41569.1731, 10.789),
                        Vec3(87043.8420, 41567.6605, 10.762),
                        Vec3(87044.4377, 41566.2595, 10.734),
                        Vec3(87044.8987, 41564.7560, 10.711),
                        Vec3(87045.1513, 41563.0492, 10.692),
                        Vec3(87045.1042, 41562.5928, 10.688),
                        Vec3(87044.7432, 41560.1981, 10.68),
                        Vec3(87044.6382, 41562.0276, 10.683),
                        Vec3(87044.5841, 41563.0797, 10.697),
                        Vec3(87044.2857, 41564.5006, 10.694),
                        Vec3(87042.6356, 41567.9327, 10.775),
                        Vec3(87041.4591, 41569.3619, 10.766),
                        Vec3(87039.2451, 41571.0803, 10.7015),
                        Vec3(87035.4664, 41572.9583, 10.767),
                        Vec3(87031.8635, 41574.3298, 10.723),
                        Vec3(87029.0326, 41575.1722, 10.686),
                        Vec3(87025.4296, 41575.5555, 10.642),
                        Vec3(87020.0536, 41576.0055, 10.467),
                    ],
                )
            ],
            thickness=0.5,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        self.assertGreater(len(layer.mesh.faces), 0)
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])

    def test_wall_prism_is_closed(self):
        layer = build_wall_layer(
            layer_name="road_border_wall",
            features=[
                PolylineFeature(
                    feature_id=1,
                    points=[Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0)],
                )
            ],
            wall_height=2.0,
            wall_thickness=0.1,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])

    def test_marking_prism_is_closed(self):
        layer = build_marking_layer(
            layer_name="road_marking",
            features=[
                PolylineFeature(
                    feature_id=1,
                    points=[Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)],
                )
            ],
            marking_width=0.05,
            marking_thickness=0.01,
            marking_offset=0.002,
            marking_style="prism",
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])

    def test_flat_marking_layer_is_valid_and_nearly_flush(self):
        layer = build_marking_layer(
            layer_name="road_marking",
            features=[
                PolylineFeature(
                    feature_id=2,
                    points=[Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)],
                    feature_type="line_thin",
                )
            ],
            marking_width=0.05,
            marking_thickness=0.01,
            marking_offset=0.002,
            marking_style="flat",
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])
        self.assertFalse(layer.requires_closed_volume)
        self.assertEqual(len(layer.mesh.faces), 4)
        self.assertEqual({vertex.z for vertex in layer.mesh.vertices}, {0.002})

    def test_flat_surface_layer_is_valid_double_sided_and_unextruded(self):
        layer = build_surface_layer(
            layer_name="lanelet_road",
            features=[
                PolygonFeature(
                    feature_id=3,
                    points=[
                        Vec3(0.0, 0.0, 5.0),
                        Vec3(2.0, 0.0, 5.0),
                        Vec3(2.0, 2.0, 5.0),
                        Vec3(0.0, 2.0, 5.0),
                    ],
                )
            ],
            thickness=0.5,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
            surface_style="flat",
        )

        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])
        self.assertEqual(report.warnings, [])
        self.assertFalse(layer.requires_closed_volume)
        self.assertEqual(len(layer.mesh.faces), 4)
        self.assertEqual({vertex.z for vertex in layer.mesh.vertices}, {5.0})

    def test_flat_surface_layer_stays_valid_at_large_coordinates(self):
        layer = build_surface_layer(
            layer_name="lanelet_road",
            features=[
                PolygonFeature(
                    feature_id=4,
                    points=[
                        Vec3(92008.5, 45335.1, 10.0),
                        Vec3(92010.5, 45335.1, 10.0),
                        Vec3(92010.5, 45337.1, 10.0),
                        Vec3(92008.5, 45337.1, 10.0),
                    ],
                )
            ],
            thickness=0.5,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
            surface_style="flat",
        )

        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])
        self.assertEqual(report.warnings, [])
        self.assertFalse(layer.requires_closed_volume)
        self.assertEqual({vertex.z for vertex in layer.mesh.vertices}, {10.0})

    def test_inside_out_closed_component_is_flipped_outward(self):
        mesh = TriangleMesh(
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
        )

        _ensure_closed_feature_mesh(
            mesh=mesh,
            layer_name="lanelet_road",
            feature=PolygonFeature(
                feature_id=99,
                points=[
                    Vec3(0.0, 0.0, 0.0),
                    Vec3(1.0, 0.0, 0.0),
                    Vec3(0.0, 1.0, 0.0),
                ],
            ),
            patch_index=0,
            logger=None,
        )

        self.assertTrue(all(volume > 0.0 for volume in mesh_component_signed_volumes(mesh)))
        layer = MeshLayer(name="lanelet_road", mesh=mesh, source_feature_count=1)
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])
        self.assertEqual(report.warnings, [])

    def test_inside_out_closed_component_is_flipped_outward_at_large_coordinates(self):
        offset = Vec3(92008.5, 45335.1, 10.0)
        mesh = TriangleMesh(
            vertices=[
                offset + Vec3(0.0, 0.0, 0.0),
                offset + Vec3(1.0, 0.0, 0.0),
                offset + Vec3(0.0, 1.0, 0.0),
                offset + Vec3(0.0, 0.0, 1.0),
            ],
            faces=[
                (0, 1, 2),
                (0, 3, 1),
                (0, 2, 3),
                (1, 3, 2),
            ],
        )

        _ensure_closed_feature_mesh(
            mesh=mesh,
            layer_name="lanelet_road",
            feature=PolygonFeature(
                feature_id=100,
                points=[
                    offset + Vec3(0.0, 0.0, 0.0),
                    offset + Vec3(1.0, 0.0, 0.0),
                    offset + Vec3(0.0, 1.0, 0.0),
                ],
            ),
            patch_index=0,
            logger=None,
        )

        self.assertTrue(all(volume > 0.0 for volume in mesh_component_signed_volumes(mesh)))
        layer = MeshLayer(name="lanelet_road", mesh=mesh, source_feature_count=1)
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])
        self.assertEqual(report.warnings, [])


if __name__ == "__main__":
    unittest.main()
