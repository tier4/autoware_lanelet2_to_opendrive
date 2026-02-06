"""Abstract base class for 1D splines."""

from abc import ABC, abstractmethod
from typing import List, Tuple

import numpy as np


class Spline1DBase(ABC):
    """
    Abstract base class for 1D spline interpolation.

    This class defines the common interface for all 1D spline implementations,
    such as cubic splines, monotone splines, etc.
    """

    def __init__(self, arc_lengths: np.ndarray, values: np.ndarray):
        """
        Initialize a 1D spline.

        Args:
            arc_lengths: Array of arc length values (must be monotonically increasing)
            values: Array of values corresponding to arc_lengths

        Raises:
            ValueError: If input validation fails
        """
        self._validate_inputs(arc_lengths, values)

        self.arc_lengths = np.asarray(arc_lengths)
        self.values = np.asarray(values)
        self.total_length = self.arc_lengths[-1]

        # Store segment boundaries for efficient lookup
        self.segment_starts = self.arc_lengths[:-1]
        self.segment_ends = self.arc_lengths[1:]
        self.num_segments = len(self.segment_starts)

    @staticmethod
    def _validate_inputs(arc_lengths: np.ndarray, values: np.ndarray) -> None:
        """
        Validate input arrays.

        Args:
            arc_lengths: Array of arc length values
            values: Array of values

        Raises:
            ValueError: If validation fails
        """
        if len(arc_lengths) != len(values):
            raise ValueError("arc_lengths and values must have the same length")

        if len(arc_lengths) < 2:
            raise ValueError("At least 2 points are required for spline interpolation")

        # Ensure arc_lengths are monotonically increasing
        if not np.all(np.diff(arc_lengths) > 0):
            raise ValueError("arc_lengths must be monotonically increasing")

    @abstractmethod
    def evaluate(self, s: float, derivative: int = 0) -> float:
        """
        Evaluate the spline at a given arc length.

        Args:
            s: Arc length value
            derivative: Derivative order (0=value, 1=first derivative, etc.)

        Returns:
            Value or derivative at arc length s
        """
        pass

    @abstractmethod
    def get_polynomial_coefficients(
        self, segment_index: int
    ) -> Tuple[float, float, float, float]:
        """
        Get polynomial coefficients (a, b, c, d) for a specific segment.

        The polynomial is: f(s) = a + b*(s-s0) + c*(s-s0)^2 + d*(s-s0)^3
        where s0 is the start of the segment.

        Args:
            segment_index: Index of the segment (0 to num_segments-1)

        Returns:
            Tuple of (a, b, c, d) coefficients

        Raises:
            ValueError: If segment_index is out of range
        """
        pass

    def get_segments(self) -> List[Tuple[float, float, float, float, float]]:
        """
        Get all segment data for OpenDRIVE elements.

        Returns:
            List of tuples (sOffset, a, b, c, d) for each segment
        """
        segments = []
        for i in range(self.num_segments):
            s_offset = self.segment_starts[i]
            a, b, c, d = self.get_polynomial_coefficients(i)
            segments.append((s_offset, a, b, c, d))
        return segments

    @property
    def total_arc_length(self) -> float:
        """Get total arc length of the spline."""
        return self.total_length

    def evaluate_with_3d_compatibility(
        self, s: float, derivative: int = 0
    ) -> np.ndarray:
        """
        Evaluate and return in 3D format for compatibility with existing code.

        Args:
            s: Arc length value
            derivative: Derivative order (only 0 supported)

        Returns:
            3D array [s, value, 0] for compatibility

        Raises:
            NotImplementedError: If derivative != 0
        """
        if derivative != 0:
            raise NotImplementedError("Derivatives not supported in compatibility mode")

        value = self.evaluate(s, derivative=0)
        return np.array([s, value, 0.0])
