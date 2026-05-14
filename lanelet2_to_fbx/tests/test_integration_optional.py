import importlib.util
import unittest
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ll2tofbx.godot_extract import MARKING_TYPES
from ll2tofbx.lanelet_loader import load_lanelet_map
from ll2tofbx.osm_index import load_osm_index


HAS_LANELET2 = importlib.util.find_spec("lanelet2") is not None


@unittest.skipUnless(HAS_LANELET2, "lanelet2 is not installed")
class IntegrationOptionalTests(unittest.TestCase):
    def test_osm_index_loads_sample_map(self):
        osm_index = load_osm_index(Path("map/odaiba_ll2_raw.osm"))
        self.assertTrue(osm_index.nodes)
        self.assertIsNotNone(osm_index.first_geo_reference)

    def test_lanelet_loader_reads_sample_map(self):
        osm_index = load_osm_index(Path("map/odaiba_ll2_raw.osm"))
        loaded = load_lanelet_map(Path("map/odaiba_ll2_raw.osm"), osm_index)
        self.assertTrue(hasattr(loaded.lanelet_map, "laneletLayer"))
        self.assertIn("line_thin", MARKING_TYPES)


if __name__ == "__main__":
    unittest.main()
