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
