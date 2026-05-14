import unittest
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ll2tofbx.layer_names import LAYER_NAMES, OPTIONAL_LAYER_NAMES, ROAD_SURFACE_LAYER_NAMES


class LayerNamesTests(unittest.TestCase):
    def test_optional_layers_are_known_layers(self):
        self.assertLessEqual(OPTIONAL_LAYER_NAMES, set(LAYER_NAMES))

    def test_road_surface_layers_are_known_layers(self):
        self.assertLessEqual(ROAD_SURFACE_LAYER_NAMES, set(LAYER_NAMES))


if __name__ == "__main__":
    unittest.main()
