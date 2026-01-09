"""OpenDRIVE root element and export functionality."""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Union

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.validation import (
    validate_opendrive_string_or_raise,
)

from .header import Header
from .junction import Junction
from .road import Road
from .signal import Controller


@dataclass
class OpenDRIVE:
    """Root OpenDRIVE element."""

    header: Optional[Header] = None
    roads: Optional[List[Road]] = None
    junctions: Optional[List[Junction]] = None
    controllers: Optional[List[Controller]] = None

    def __post_init__(self):
        if self.roads is None:
            self.roads = []
        if self.junctions is None:
            self.junctions = []
        if self.controllers is None:
            self.controllers = []

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("OpenDRIVE")

        if self.header:
            elem.append(self.header.to_xml())

        if self.roads:
            for road in self.roads:
                elem.append(road.to_xml())

        if self.controllers:
            for controller in self.controllers:
                elem.append(controller.to_xml())

        if self.junctions:
            for junction in self.junctions:
                elem.append(junction.to_xml())

        return elem


# Export functionality


def export_to_xml(opendrive: OpenDRIVE) -> str:
    """
    Export OpenDRIVE object to XML string.

    Args:
        opendrive: OpenDRIVE object to export

    Returns:
        XML string representation
    """
    xml_element = opendrive.to_xml()

    # Pretty print
    xml_str = ET.tostring(
        xml_element, pretty_print=True, xml_declaration=True, encoding="UTF-8"
    ).decode("utf-8")

    return xml_str


def save_opendrive_to_file(
    opendrive: OpenDRIVE,
    filepath: Union[str, Path],
    validate: bool = True,
) -> None:
    """
    Save OpenDRIVE object to XML file.

    Args:
        opendrive: OpenDRIVE object to save
        filepath: Path to save the XML file
        validate: Whether to validate against XSD schema (default: True)

    Raises:
        OpenDRIVEValidationError: If validation is enabled and the output
            does not conform to the OpenDRIVE XSD schema.
    """
    xml_str = export_to_xml(opendrive)

    if validate:
        validate_opendrive_string_or_raise(xml_str)

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(xml_str)
