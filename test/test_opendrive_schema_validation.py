"""Tests for OpenDRIVE XML schema validation.

This module validates .xodr files against the official ASAM OpenDRIVE 1.7.0 XSD schema.
The schema files are bundled with the package in src/autoware_lanelet2_to_opendrive/schemas/.

Schema source: ASAM OpenDRIVE V1.7.0
https://www.asam.net/standards/detail/opendrive/
"""

from pathlib import Path

import pytest
import xmlschema

from autoware_lanelet2_to_opendrive.validation import (
    OpenDRIVEValidationError,
    get_opendrive_schema,
    get_opendrive_schema_path,
    validate_opendrive_file,
    validate_opendrive_file_or_raise,
    validate_opendrive_string,
    validate_opendrive_string_or_raise,
)


@pytest.fixture
def opendrive_schema() -> xmlschema.XMLSchema:
    """Load the OpenDRIVE 1.7 XML schema."""
    return get_opendrive_schema()


@pytest.fixture(scope="session")
def test_xodr_path(tmp_path_factory) -> Path:
    """Generate and return path to test .xodr file."""
    # Import here to avoid circular dependencies
    from autoware_lanelet2_extension_python.projection import MGRSProjector
    import lanelet2
    from autoware_lanelet2_to_opendrive.main import convert_lanelet2_to_opendrive
    from autoware_lanelet2_to_opendrive.util import mgrs_to_lanelet2_origin

    # Path to input OSM file
    osm_path = Path(__file__).parent / "data" / "lanelet2_map.osm"

    # Create output path in temporary directory
    tmp_dir = tmp_path_factory.mktemp("xodr_test")
    xodr_path = tmp_dir / "lanelet2_map.xodr"

    # Load and convert
    mgrs = "54SUE"  # Default MGRS code for test data
    projector = MGRSProjector(mgrs_to_lanelet2_origin(mgrs))
    lanelet_map = lanelet2.io.load(str(osm_path), projector)

    # Convert and save
    opendrive, _ = convert_lanelet2_to_opendrive(lanelet_map, mgrs, output_path=xodr_path)

    return xodr_path


class TestSchemaPath:
    """Tests for schema path functions."""

    def test_get_opendrive_schema_path_exists(self) -> None:
        """Verify that the schema path exists."""
        schema_path = get_opendrive_schema_path()
        assert schema_path.exists()
        assert schema_path.name == "opendrive_17_core.xsd"

    def test_get_opendrive_schema_loads(self) -> None:
        """Verify that the schema can be loaded."""
        schema = get_opendrive_schema()
        assert schema is not None


class TestOpenDRIVESchemaValidation:
    """Test suite for validating .xodr files against OpenDRIVE schema."""

    def test_xodr_file_exists(self, test_xodr_path: Path) -> None:
        """Verify that the test .xodr file exists."""
        assert test_xodr_path.exists(), f"Test file not found: {test_xodr_path}"

    def test_xodr_is_valid_xml(self, test_xodr_path: Path) -> None:
        """Verify that the .xodr file is valid XML."""
        from lxml import etree

        try:
            etree.parse(str(test_xodr_path))
        except etree.XMLSyntaxError as e:
            pytest.fail(f"Invalid XML: {e}")

    def test_xodr_validates_against_opendrive_schema(
        self, test_xodr_path: Path, opendrive_schema: xmlschema.XMLSchema
    ) -> None:
        """Validate the .xodr file against the official OpenDRIVE 1.7 schema."""
        errors = list(opendrive_schema.iter_errors(str(test_xodr_path)))

        if errors:
            error_messages = []
            for error in errors[:10]:  # Limit to first 10 errors
                error_messages.append(f"  - {error.reason} at {error.path}")

            total_errors = len(errors)
            shown_errors = min(10, total_errors)
            error_summary = "\n".join(error_messages)

            pytest.fail(
                f"OpenDRIVE schema validation failed with {total_errors} error(s).\n"
                f"First {shown_errors} errors:\n{error_summary}"
            )

    def test_xodr_has_required_root_element(self, test_xodr_path: Path) -> None:
        """Verify that the .xodr file has the required OpenDRIVE root element."""
        from lxml import etree

        tree = etree.parse(str(test_xodr_path))
        root = tree.getroot()

        assert (
            root.tag == "OpenDRIVE"
        ), f"Expected root element 'OpenDRIVE', got '{root.tag}'"

    def test_xodr_has_header(self, test_xodr_path: Path) -> None:
        """Verify that the .xodr file has a header element."""
        from lxml import etree

        tree = etree.parse(str(test_xodr_path))
        root = tree.getroot()

        header = root.find("header")
        assert header is not None, "OpenDRIVE file must contain a 'header' element"

    def test_xodr_has_at_least_one_road(self, test_xodr_path: Path) -> None:
        """Verify that the .xodr file has at least one road element."""
        from lxml import etree

        tree = etree.parse(str(test_xodr_path))
        root = tree.getroot()

        roads = root.findall("road")
        assert (
            len(roads) >= 1
        ), "OpenDRIVE file must contain at least one 'road' element"


class TestValidateOpenDRIVEFile:
    """Tests for file validation functions."""

    def test_validate_valid_file_returns_empty_list(self, test_xodr_path: Path) -> None:
        """Verify that a valid file returns no errors."""
        if not test_xodr_path.exists():
            pytest.skip("Test xodr file not found")
        errors = validate_opendrive_file(test_xodr_path)
        assert errors == []

    def test_validate_valid_file_or_raise_no_exception(
        self, test_xodr_path: Path
    ) -> None:
        """Verify that validate_or_raise does not raise for valid file."""
        if not test_xodr_path.exists():
            pytest.skip("Test xodr file not found")
        # Should not raise
        validate_opendrive_file_or_raise(test_xodr_path)


class TestValidateOpenDRIVEString:
    """Tests for string validation functions."""

    def test_validate_invalid_string_returns_errors(self) -> None:
        """Verify that an invalid XML string returns errors."""
        invalid_xml = """<?xml version="1.0" encoding="UTF-8"?>
        <OpenDRIVE>
            <invalid_element/>
        </OpenDRIVE>
        """
        errors = validate_opendrive_string(invalid_xml)
        assert len(errors) > 0

    def test_validate_invalid_string_or_raise_raises(self) -> None:
        """Verify that validate_or_raise raises for invalid XML."""
        invalid_xml = """<?xml version="1.0" encoding="UTF-8"?>
        <OpenDRIVE>
            <invalid_element/>
        </OpenDRIVE>
        """
        with pytest.raises(OpenDRIVEValidationError) as exc_info:
            validate_opendrive_string_or_raise(invalid_xml)

        assert len(exc_info.value.errors) > 0


class TestOpenDRIVEValidationError:
    """Tests for the validation error class."""

    def test_error_message_contains_error_count(self) -> None:
        """Verify that the error message contains error count."""
        errors = ["Error 1", "Error 2", "Error 3"]
        exc = OpenDRIVEValidationError(errors)

        assert "3 error(s)" in str(exc)
        assert "Error 1" in str(exc)
        assert "Error 2" in str(exc)
        assert "Error 3" in str(exc)

    def test_error_message_truncates_many_errors(self) -> None:
        """Verify that error messages are truncated when there are many."""
        errors = [f"Error {i}" for i in range(20)]
        exc = OpenDRIVEValidationError(errors)

        assert "20 error(s)" in str(exc)
        assert "10 more errors" in str(exc)
