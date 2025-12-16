"""OpenDRIVE signals container."""

from dataclasses import dataclass, field
from typing import List
import lxml.etree as ET

from .signal import Signal


@dataclass
class Signals:
    """
    Container for road signals (traffic lights, signs, etc.).

    This element contains all signals defined for a road.
    Signals are defined in the road coordinate system.

    Reference: ASAM OpenDRIVE v1.8.1 - Section 14: Signals
    """

    signals: List[Signal] = field(default_factory=list)

    def add_signal(self, signal: Signal) -> None:
        """
        Add a signal to the signals container.

        Args:
            signal: Signal object to add
        """
        self.signals.append(signal)

    def to_xml(self) -> ET.Element:
        """
        Convert to XML element.

        Returns:
            XML element representing the <signals> container
        """
        elem = ET.Element("signals")

        # Add all signal elements
        for signal in self.signals:
            elem.append(signal.to_xml())

        return elem

    def __repr__(self) -> str:
        """String representation of the signals container."""
        return f"Signals(count={len(self.signals)})"

    def __len__(self) -> int:
        """Return the number of signals."""
        return len(self.signals)

    def __iter__(self):
        """Iterate over signals."""
        return iter(self.signals)

    def __getitem__(self, index: int) -> Signal:
        """Get signal by index."""
        return self.signals[index]


def create_traffic_light(
    signal_id: int,
    name: str,
    s: float,
    t: float,
    orientation: str = "-",
    z_offset: float = 0.0,
    height: float = 1.0,
    width: float = 0.5,
    traffic_light_type: int = 1000001,  # Default to 3-light signal
    from_lane: int = 0,
    to_lane: int = 0,
) -> Signal:
    """
    Convenience function to create a standard traffic light signal.

    Args:
        signal_id: Unique signal ID
        name: Signal name
        s: s-coordinate along road reference line
        t: t-coordinate lateral offset from reference line
        orientation: "+" or "-" orientation with respect to road direction
        z_offset: Height offset above road surface
        height: Signal height
        width: Signal width
        traffic_light_type: Traffic light type (default: 1000001 = 3-light signal)
        from_lane: Starting lane ID for validity
        to_lane: Ending lane ID for validity

    Returns:
        Signal object configured as a traffic light
    """
    from .signal import Validity

    return Signal(
        id=signal_id,
        name=name,
        s=s,
        t=t,
        z_offset=z_offset,
        h_offset=0.0,
        roll=0.0,
        pitch=0.0,
        orientation=orientation,
        dynamic="yes",  # Traffic lights are dynamic
        country="OpenDRIVE",
        type=traffic_light_type,
        subtype=-1,
        value=-1.0,
        text="",
        height=height,
        width=width,
        validities=[Validity(from_lane=from_lane, to_lane=to_lane)],
    )
