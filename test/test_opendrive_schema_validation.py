"""Tests for OpenDRIVE XML schema validation.

This module validates .xodr files against the official ASAM OpenDRIVE 1.7.0 XSD schema.
The schema files are included in test/schemas/ directory.

Schema source: ASAM OpenDRIVE V1.7.0
https://www.asam.net/standards/detail/opendrive/
"""

from pathlib import Path

import pytest
import xmlschema


def get_opendrive_schema_path() -> Path:
    """Get the path to the OpenDRIVE 1.7 schema file.

    The schema is bundled with the project in test/schemas/.
    """
    schema_path = Path(__file__).parent / "schemas" / "opendrive_17_core.xsd"

    if not schema_path.exists():
        pytest.skip(f"OpenDRIVE schema not found at {schema_path}")

    return schema_path


@pytest.fixture
def opendrive_schema() -> xmlschema.XMLSchema:
    """Load the OpenDRIVE 1.7 XML schema."""
    schema_path = get_opendrive_schema_path()
    return xmlschema.XMLSchema(str(schema_path))


@pytest.fixture
def test_xodr_path() -> Path:
    """Get the path to the test .xodr file."""
    return Path(__file__).parent / "data" / "lanelet2_map.xodr"


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


def validate_xodr_file(xodr_path: Path) -> list[str]:
    """Validate an .xodr file against the OpenDRIVE 1.7 schema.

    Args:
        xodr_path: Path to the .xodr file to validate

    Returns:
        List of validation error messages. Empty list if valid.
    """
    schema_path = get_opendrive_schema_path()
    schema = xmlschema.XMLSchema(str(schema_path))

    errors = list(schema.iter_errors(str(xodr_path)))
    return [f"{error.reason} at {error.path}" for error in errors]
