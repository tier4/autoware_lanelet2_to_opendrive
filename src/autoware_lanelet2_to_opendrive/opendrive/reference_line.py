"""ReferenceLine implementation for OpenDRIVE conversion."""

from typing import Optional
import lanelet2

from .lane import Lane
from .opendrive_dataclass import LaneType


class ReferenceLine(Lane):
    """
    OpenDRIVE reference line representation that inherits from Lane.

    The reference line is the center line of a road and serves as the basis
    for defining lane geometry in OpenDRIVE format. It inherits all lane
    functionality but is specifically designed for reference line purposes.
    """

    def __init__(
        self,
        predecessor: Optional[int] = None,
        successor: Optional[int] = None,
    ):
        """
        Initialize a ReferenceLine object.

        Args:
            predecessor: ID of predecessor reference line
            successor: ID of successor reference line
        """
        # Reference line always has lane_id = 0 and is always a driving type
        super().__init__(
            lane_id=0,
            lane_type=LaneType.DRIVING,
            level=False,
            predecessor=None,  # Reference lines don't use LaneLink objects
            successor=None,
        )

        # Store reference line specific connections
        self.predecessor_id = predecessor
        self.successor_id = successor

    @staticmethod
    def construct_from_lanelet(
        lanelet_map: lanelet2.core.LaneletMap, lanelet: lanelet2.core.Lanelet
    ) -> "ReferenceLine":
        """
        Construct a ReferenceLine from a Lanelet2 lanelet.

        Args:
            lanelet_map: The Lanelet2 map containing the lanelet
            lanelet: The lanelet to convert to ReferenceLine

        Returns:
            ReferenceLine instance constructed from the lanelet
        """
        # TODO: Determine predecessor and successor from lanelet connections
        predecessor_id = None
        successor_id = None

        # Create the ReferenceLine instance
        reference_line = ReferenceLine(
            predecessor=predecessor_id,
            successor=successor_id,
        )

        # Calculate reference line width using spline curve from estimate_lanelet_width_as_spline
        from ..centerline import estimate_lanelet_width_as_spline

        width_spline = estimate_lanelet_width_as_spline(lanelet)

        # Add width definitions using the inherited method
        reference_line._add_width_from_spline(width_spline)

        # TODO: Add road marks based on lanelet line types

        return reference_line

    def __repr__(self) -> str:
        """String representation of the reference line."""
        return (
            f"ReferenceLine(predecessor_id={self.predecessor_id}, "
            f"successor_id={self.successor_id}, "
            f"widths={len(self.widths)}, marks={len(self.road_marks)})"
        )
