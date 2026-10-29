"""Tests for preprocess_lanelet module."""

from pathlib import Path

import pytest
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    LaneletPreprocessor,
    LatLonOrigin,
    PreprocessOperation,
)

FIXTURE_DIR = Path(__file__).parent / "data"


class TestLatLonOrigin:
    """Tests for LatLonOrigin dataclass."""

    def test_valid_origin(self):
        """Test creating a valid LatLonOrigin."""
        origin = LatLonOrigin(lat=35.6762, lon=139.6503)
        assert origin.lat == 35.6762
        assert origin.lon == 139.6503

    def test_invalid_latitude_too_high(self):
        """Test that latitude > 90 raises ValueError."""
        with pytest.raises(ValueError, match="Latitude must be between -90 and 90"):
            LatLonOrigin(lat=91.0, lon=139.0)

    def test_invalid_latitude_too_low(self):
        """Test that latitude < -90 raises ValueError."""
        with pytest.raises(ValueError, match="Latitude must be between -90 and 90"):
            LatLonOrigin(lat=-91.0, lon=139.0)

    def test_invalid_longitude_too_high(self):
        """Test that longitude > 180 raises ValueError."""
        with pytest.raises(ValueError, match="Longitude must be between -180 and 180"):
            LatLonOrigin(lat=35.0, lon=181.0)

    def test_invalid_longitude_too_low(self):
        """Test that longitude < -180 raises ValueError."""
        with pytest.raises(ValueError, match="Longitude must be between -180 and 180"):
            LatLonOrigin(lat=35.0, lon=-181.0)

    def test_boundary_values(self):
        """Test boundary values for latitude and longitude."""
        # Should not raise
        origin = LatLonOrigin(lat=90.0, lon=180.0)
        assert origin.lat == 90.0
        assert origin.lon == 180.0

        origin = LatLonOrigin(lat=-90.0, lon=-180.0)
        assert origin.lat == -90.0
        assert origin.lon == -180.0


class TestPreprocessOperationOriginValidation:
    """Tests for PreprocessOperation origin validation."""

    def test_mgrs_code_only(self):
        """Test that specifying only mgrs_code is valid."""
        config = PreprocessOperation(
            input_map_path="/path/to/input.osm",
            output_map_path="/path/to/output.osm",
            mgrs_code="54SUE815501",
        )
        assert config.mgrs_code == "54SUE815501"
        assert config.origin is None

    def test_origin_only(self):
        """Test that specifying only origin (lat/lon) is valid."""
        origin = LatLonOrigin(lat=35.6762, lon=139.6503)
        config = PreprocessOperation(
            input_map_path="/path/to/input.osm",
            output_map_path="/path/to/output.osm",
            origin=origin,
        )
        assert config.mgrs_code is None
        assert config.origin == origin

    def test_load_map_ignores_the_projection_origin(self, tmp_path):
        """Every origin -- including none -- loads the same coordinates.

        ``MGRSProjector::forward()`` returns each point's offset from the
        south-west corner of its own grid square regardless of the
        projector's origin, so the origin a preprocessing config declares is
        inert. Pinned here because that is the premise for making it
        optional.
        """
        source = str(FIXTURE_DIR / "walkway_mini.osm")

        def _points(**origin_kwargs):
            config = PreprocessOperation(
                input_map_path=source,
                output_map_path=str(tmp_path / "out.osm"),
                **origin_kwargs,
            )
            lanelet_map = LaneletPreprocessor(config).load_map()
            return sorted(
                (p.id, round(p.x, 9), round(p.y, 9), round(p.z, 9))
                for p in lanelet_map.pointLayer
            )

        expected = _points(mgrs_code="54SUE")
        assert expected  # the fixture has points at all
        assert _points() == expected
        assert _points(mgrs_code="53SPU") == expected
        assert _points(origin=LatLonOrigin(lat=-33.123456, lon=151.234567)) == expected

    def test_both_mgrs_and_origin_raises_error(self):
        """Test that specifying both mgrs_code and origin raises ValueError."""
        origin = LatLonOrigin(lat=35.6762, lon=139.6503)
        with pytest.raises(
            ValueError,
            match="Cannot specify both mgrs_code and origin",
        ):
            PreprocessOperation(
                input_map_path="/path/to/input.osm",
                output_map_path="/path/to/output.osm",
                mgrs_code="54SUE815501",
                origin=origin,
            )

    def test_neither_mgrs_nor_origin_is_valid(self):
        """Preprocessing needs no projection origin.

        It loads and writes through ``MGRSProjector``, which ignores the
        origin it is given and emits coordinates relative to each point's own
        100 km grid square. See
        ``test_load_map_ignores_the_projection_origin``.
        """
        config = PreprocessOperation(
            input_map_path="/path/to/input.osm",
            output_map_path="/path/to/output.osm",
        )
        assert config.mgrs_code is None
        assert config.origin is None

    def test_empty_mgrs_code_without_origin_is_valid(self):
        """An empty mgrs_code is the same as none at all."""
        config = PreprocessOperation(
            input_map_path="/path/to/input.osm",
            output_map_path="/path/to/output.osm",
            mgrs_code="",
        )
        assert config.origin is None

    def test_empty_mgrs_code_with_origin_is_valid(self):
        """Test that empty mgrs_code with origin is valid (origin takes precedence)."""
        origin = LatLonOrigin(lat=35.6762, lon=139.6503)
        config = PreprocessOperation(
            input_map_path="/path/to/input.osm",
            output_map_path="/path/to/output.osm",
            mgrs_code="",
            origin=origin,
        )
        assert config.origin == origin
