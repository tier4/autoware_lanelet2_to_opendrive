"""OpenDRIVE XML schema validation module.

This module provides functions to validate OpenDRIVE XML files against the
official ASAM OpenDRIVE V1.7.0 XSD schema.
"""

from pathlib import Path
from typing import Union

import xmlschema


class OpenDRIVEValidationError(Exception):
    """Exception raised when OpenDRIVE XML validation fails."""

    def __init__(self, errors: list[str]):
        self.errors = errors
        error_count = len(errors)
        error_summary = "\n".join(f"  - {e}" for e in errors[:10])
        if error_count > 10:
            error_summary += f"\n  ... and {error_count - 10} more errors"
        message = (
            f"OpenDRIVE schema validation failed with {error_count} error(s):\n"
            f"{error_summary}"
        )
        super().__init__(message)


def get_opendrive_schema_path() -> Path:
    """Get the path to the OpenDRIVE 1.7 schema file.

    Returns:
        Path to the opendrive_17_core.xsd schema file.

    Raises:
        FileNotFoundError: If the schema file is not found.
    """
    schema_path = Path(__file__).parent / "schemas" / "opendrive_17_core.xsd"

    if not schema_path.exists():
        raise FileNotFoundError(f"OpenDRIVE schema not found at {schema_path}")

    return schema_path


def get_opendrive_schema() -> xmlschema.XMLSchema:
    """Load and return the OpenDRIVE 1.7 XML schema.

    Returns:
        XMLSchema object for OpenDRIVE validation.
    """
    schema_path = get_opendrive_schema_path()
    return xmlschema.XMLSchema(str(schema_path))


def validate_opendrive_file(xodr_path: Union[str, Path]) -> list[str]:
    """Validate an OpenDRIVE file against the XSD schema.

    Args:
        xodr_path: Path to the .xodr file to validate.

    Returns:
        List of validation error messages. Empty list if valid.
    """
    schema = get_opendrive_schema()
    errors = list(schema.iter_errors(str(xodr_path)))
    return [f"{error.reason} at {error.path}" for error in errors]


def validate_opendrive_string(xml_content: str) -> list[str]:
    """Validate an OpenDRIVE XML string against the XSD schema.

    Args:
        xml_content: XML string to validate.

    Returns:
        List of validation error messages. Empty list if valid.
    """
    schema = get_opendrive_schema()
    errors = list(schema.iter_errors(xml_content))
    return [f"{error.reason} at {error.path}" for error in errors]


def validate_opendrive_file_or_raise(xodr_path: Union[str, Path]) -> None:
    """Validate an OpenDRIVE file and raise an exception if invalid.

    Args:
        xodr_path: Path to the .xodr file to validate.

    Raises:
        OpenDRIVEValidationError: If validation fails.
    """
    errors = validate_opendrive_file(xodr_path)
    if errors:
        raise OpenDRIVEValidationError(errors)


def validate_opendrive_string_or_raise(xml_content: str) -> None:
    """Validate an OpenDRIVE XML string and raise an exception if invalid.

    Args:
        xml_content: XML string to validate.

    Raises:
        OpenDRIVEValidationError: If validation fails.
    """
    errors = validate_opendrive_string(xml_content)
    if errors:
        raise OpenDRIVEValidationError(errors)
