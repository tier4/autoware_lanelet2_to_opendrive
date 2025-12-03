"""OpenDRIVE road definitions."""

from dataclasses import dataclass
from typing import Optional
import lxml.etree as ET

from .geometry import PlanView
from .elevation import ElevationProfile
from .lane_sections import Lanes


@dataclass
class Road:
    """Road definition."""

    id: int = 0
    name: Optional[str] = None
    length: float = 0.0
    junction: int = -1
    plan_view: Optional[PlanView] = None
    elevation_profile: Optional[ElevationProfile] = None
    lanes: Optional[Lanes] = None

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("road")
        elem.set("id", str(self.id))
        elem.set("length", str(self.length))
        elem.set("junction", str(self.junction))

        if self.name:
            elem.set("name", self.name)

        if self.plan_view:
            elem.append(self.plan_view.to_xml())
        if self.elevation_profile:
            elem.append(self.elevation_profile.to_xml())
        if self.lanes:
            elem.append(self.lanes.to_xml())

        return elem
