import unittest
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ll2tofbx.mesh_types import Vec3
from ll2tofbx.triangulation import triangulate_polygon, triangulate_strip


class TriangulationTests(unittest.TestCase):
    def test_strip_matches_reference_order(self):
        patch = triangulate_strip(
            left=[Vec3(0.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)],
            right=[Vec3(1.0, 0.0, 0.0), Vec3(1.0, 1.0, 0.0)],
        )
        self.assertEqual(patch.faces, [(0, 2, 1), (1, 2, 3)])
        self.assertEqual(patch.boundary_loops, [[0, 1, 3, 2]])

    def test_polygon_triangulation_is_winding_invariant(self):
        ccw = [
            Vec3(0.0, 0.0, 0.0),
            Vec3(2.0, 0.0, 0.0),
            Vec3(2.0, 1.0, 0.0),
            Vec3(0.0, 1.0, 0.0),
        ]
        cw = list(reversed(ccw))
        ccw_patch = triangulate_polygon(ccw)
        cw_patch = triangulate_polygon(cw)
        ccw_area = _mesh_area(ccw_patch.vertices, ccw_patch.faces)
        cw_area = _mesh_area(cw_patch.vertices, cw_patch.faces)
        self.assertAlmostEqual(ccw_area, 2.0)
        self.assertAlmostEqual(cw_area, 2.0)
        self.assertEqual(len(ccw_patch.faces), 2)
        self.assertEqual(len(cw_patch.faces), 2)


def _mesh_area(vertices, faces):
    area = 0.0
    for a, b, c in faces:
        area += (vertices[b] - vertices[a]).cross(vertices[c] - vertices[a]).length() * 0.5
    return area


if __name__ == "__main__":
    unittest.main()
