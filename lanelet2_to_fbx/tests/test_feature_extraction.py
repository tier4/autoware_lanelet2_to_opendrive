import unittest
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ll2tofbx.godot_extract import extract_feature_set
from ll2tofbx.solid_builder import build_marking_layer
from ll2tofbx.validate import validate_layers
from ll2tofbx.mesh_types import Vec3


class _FakePoint:
    def __init__(self, point_id: int, x: float, y: float, z: float = 0.0) -> None:
        self.id = point_id
        self.x = x
        self.y = y
        self.z = z


class _FakeLineString:
    def __init__(self, line_id: int, points: list[_FakePoint], attributes: dict[str, str]) -> None:
        self.id = line_id
        self._points = points
        self.attributes = attributes

    def __iter__(self):
        return iter(self._points)


class _FakeLanelet:
    def __init__(
        self,
        lanelet_id: int,
        left_bound: _FakeLineString,
        right_bound: _FakeLineString,
        attributes: dict[str, str],
    ) -> None:
        self.id = lanelet_id
        self.leftBound = left_bound
        self.rightBound = right_bound
        self.attributes = attributes


class _FakeMap:
    def __init__(self, lanelets: list[_FakeLanelet], line_strings: list[_FakeLineString]) -> None:
        self.laneletLayer = lanelets
        self.lineStringLayer = line_strings
        self.polygonLayer = []


def _resolver(point: _FakePoint) -> Vec3:
    return Vec3(point.x, point.y, point.z)


class FeatureExtractionTests(unittest.TestCase):
    def test_extract_feature_set_keeps_non_shoulder_lanelets_and_infers_ground_fill(self):
        outer_road = _FakeLineString(
            10,
            [_FakePoint(1, 0.0, 0.0), _FakePoint(2, 10.0, 0.0), _FakePoint(3, 20.0, 0.0)],
            {"type": "virtual"},
        )
        inner_road = _FakeLineString(
            11,
            [_FakePoint(4, 0.0, 20.0), _FakePoint(5, 10.0, 20.0), _FakePoint(6, 20.0, 20.0)],
            {"type": "line_thin", "subtype": "solid"},
        )
        border_a = _FakeLineString(
            20,
            [_FakePoint(100, 0.0, -5.0), _FakePoint(101, 10.0, -5.0)],
            {"type": "road_border", "subtype": "high"},
        )
        border_b = _FakeLineString(
            21,
            [_FakePoint(101, 10.0, -5.0), _FakePoint(102, 20.0, -5.0)],
            {"type": "road_border", "subtype": "high"},
        )
        shoulder_left = _FakeLineString(
            30,
            [_FakePoint(200, 30.0, 0.0), _FakePoint(201, 40.0, 0.0)],
            {"type": "line_thin", "subtype": "solid"},
        )
        shoulder_right = _FakeLineString(
            31,
            [_FakePoint(202, 30.0, -4.0), _FakePoint(203, 40.0, -4.0)],
            {"type": "road_border", "subtype": "high"},
        )
        pedestrian_left = _FakeLineString(
            40,
            [_FakePoint(300, 50.0, 0.0), _FakePoint(301, 60.0, 0.0)],
            {"type": "line_thin", "subtype": "solid"},
        )
        pedestrian_right = _FakeLineString(
            41,
            [_FakePoint(302, 50.0, 4.0), _FakePoint(303, 60.0, 4.0)],
            {"type": "line_thin", "subtype": "solid"},
        )

        lanelet_map = _FakeMap(
            lanelets=[
                _FakeLanelet(
                    100,
                    outer_road,
                    inner_road,
                    {"type": "lanelet", "subtype": "road"},
                ),
                _FakeLanelet(
                    200,
                    shoulder_left,
                    shoulder_right,
                    {"type": "lanelet", "subtype": "road_shoulder"},
                ),
                _FakeLanelet(
                    300,
                    pedestrian_left,
                    pedestrian_right,
                    {"type": "lanelet", "subtype": "pedestrian_lane"},
                ),
            ],
            line_strings=[
                outer_road,
                inner_road,
                border_a,
                border_b,
                shoulder_left,
                shoulder_right,
                pedestrian_left,
                pedestrian_right,
            ],
        )

        features = extract_feature_set(lanelet_map, _resolver)

        self.assertEqual([feature.feature_id for feature in features.lanelet_roads], [100, 300])
        self.assertEqual({feature.feature_id for feature in features.shoulders}, {10, 200})

        inferred = next(feature for feature in features.shoulders if feature.feature_id == 10)
        self.assertEqual(
            inferred.right,
            [Vec3(0.0, -5.0, 0.0), Vec3(10.0, -5.0, 0.0), Vec3(20.0, -5.0, 0.0)],
        )

        explicit = next(feature for feature in features.shoulders if feature.feature_id == 200)
        self.assertEqual(
            explicit.right,
            [Vec3(30.0, -4.0, 0.0), Vec3(40.0, -4.0, 0.0)],
        )

    def test_extract_stop_line_builds_in_flat_marking_mode(self):
        stop_line = _FakeLineString(
            50,
            [_FakePoint(500, 0.0, 0.0), _FakePoint(501, 0.0, 3.0)],
            {"type": "stop_line", "subtype": "solid"},
        )
        lanelet_map = _FakeMap(lanelets=[], line_strings=[stop_line])

        features = extract_feature_set(lanelet_map, _resolver)

        self.assertEqual(len(features.road_markings), 1)
        self.assertEqual(features.road_markings[0].feature_type, "stop_line")

        layer = build_marking_layer(
            layer_name="road_marking",
            features=features.road_markings,
            marking_width=0.3,
            marking_thickness=0.01,
            marking_offset=0.002,
            marking_style="flat",
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
            filtered_counts={},
        )
        report = validate_layers([layer], 1e-8)
        self.assertEqual(report.errors, [])


if __name__ == "__main__":
    unittest.main()
