"""Tests for preprocess_lanelet module."""

import pytest
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    LatLonOrigin,
    PreprocessOperation,
    RemoveOperation,
)


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

    def test_neither_mgrs_nor_origin_raises_error(self):
        """Neither one, with work to do, raises ValueError.

        The origin is what the map is read and written through, so it is
        required exactly when there is an operation to apply. This test used
        to assert the same thing for a configuration with no operations;
        that case is now accepted and pinned by
        ``test_no_operations_needs_no_origin`` below.
        """
        with pytest.raises(
            ValueError,
            match="Must specify projection origin using either mgrs_code or origin",
        ):
            PreprocessOperation(
                input_map_path="/path/to/input.osm",
                output_map_path="/path/to/output.osm",
                remove_operations=[RemoveOperation(lanelet_ids=[1])],
            )

    def test_no_operations_needs_no_origin(self):
        """With nothing to apply, no origin is required.

        ``convert`` builds a PreprocessOperation from ``cfg.map`` before it
        knows whether any preprocessing is configured. Demanding an origin
        there rejected map=example_latlon outright: it declares a lat_lon
        origin the projection resolver reads successfully, and carries no
        operations, so the keys this check wanted were ones nothing would
        have used.
        """
        config = PreprocessOperation(
            input_map_path="/path/to/input.osm",
            output_map_path="/path/to/output.osm",
        )
        assert not config.has_operations

    def test_empty_mgrs_code_without_origin_raises_error(self):
        """Test that empty mgrs_code without origin raises ValueError."""
        with pytest.raises(
            ValueError,
            match="Must specify projection origin using either mgrs_code or origin",
        ):
            PreprocessOperation(
                input_map_path="/path/to/input.osm",
                output_map_path="/path/to/output.osm",
                mgrs_code="",
                remove_operations=[RemoveOperation(lanelet_ids=[1])],
            )

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
