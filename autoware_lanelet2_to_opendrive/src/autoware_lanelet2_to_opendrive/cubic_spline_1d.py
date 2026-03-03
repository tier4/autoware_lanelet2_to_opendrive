"""1D cubic spline for width as a function of arc length."""

import numpy as np
from scipy.interpolate import CubicSpline
from typing import Tuple, List


class CubicSpline1D:
    """
    Generic 1D cubic spline that maps one variable to another.

    This class creates a proper 1D cubic spline interpolation for any 1D values
    (e.g., width, height, superelevation) and provides methods to get polynomial
    coefficients for each segment.
    """

    def __init__(
        self, arc_lengths: np.ndarray, widths: np.ndarray, bc_type: str = "not-a-knot"
    ):
        """
        Initialize a 1D width spline.

        Args:
            arc_lengths: Array of arc length values (must be monotonically increasing)
            widths: Array of width values corresponding to arc_lengths
            bc_type: Boundary condition type for spline ('not-a-knot', 'natural', 'clamped')
                    'not-a-knot' (default) provides smoother interpolation with less oscillation
        """
        if len(arc_lengths) != len(widths):
            raise ValueError("arc_lengths and widths must have the same length")

        if len(arc_lengths) < 2:
            raise ValueError("At least 2 points are required for spline interpolation")

        # Ensure arc_lengths are monotonically increasing
        if not np.all(np.diff(arc_lengths) > 0):
            raise ValueError("arc_lengths must be monotonically increasing")

        self.arc_lengths = np.asarray(arc_lengths)
        self.widths = np.asarray(widths)
        self.total_length = self.arc_lengths[-1]

        # Create cubic spline interpolation
        self.spline = CubicSpline(self.arc_lengths, self.widths, bc_type=bc_type)

        # Store segment boundaries for efficient lookup
        self.segment_starts = self.arc_lengths[:-1]
        self.segment_ends = self.arc_lengths[1:]
        self.num_segments = len(self.segment_starts)

    def evaluate(self, s: float, derivative: int = 0) -> float:
        """
        Evaluate the width at a given arc length.

        Args:
            s: Arc length value
            derivative: Derivative order (0=value, 1=first derivative, etc.)

        Returns:
            Width value or derivative at arc length s
        """
        # Clamp to valid range
        s = np.clip(s, 0, self.total_length)
        return float(self.spline(s, derivative))

    def get_polynomial_coefficients(
        self, segment_index: int
    ) -> Tuple[float, float, float, float]:
        """
        Get polynomial coefficients (a, b, c, d) for a specific segment.

        The polynomial is: w(s) = a + b*(s-s0) + c*(s-s0)^2 + d*(s-s0)^3
        where s0 is the start of the segment.

        Args:
            segment_index: Index of the segment (0 to num_segments-1)

        Returns:
            Tuple of (a, b, c, d) coefficients
        """
        if segment_index < 0 or segment_index >= self.num_segments:
            raise ValueError(f"Invalid segment index: {segment_index}")

        # Access scipy's CubicSpline internal coefficient matrix directly.
        # self.spline.c has shape (4, n-1) where n is number of knots.
        # Each column stores [d, c, b, a] for the polynomial:
        #   w(s) = a + b*(s-s0) + c*(s-s0)^2 + d*(s-s0)^3
        c_matrix = self.spline.c
        d = float(c_matrix[0, segment_index])  # Coefficient of (s-s0)^3
        c = float(c_matrix[1, segment_index])  # Coefficient of (s-s0)^2
        b = float(c_matrix[2, segment_index])  # Coefficient of (s-s0)
        a = float(c_matrix[3, segment_index])  # Constant term

        return a, b, c, d

    def get_segments(self) -> List[Tuple[float, float, float, float, float]]:
        """
        Get all segment data for OpenDRIVE width elements.

        Returns:
            List of tuples (sOffset, a, b, c, d) for each segment
        """
        segments = []
        for i in range(self.num_segments):
            s_offset = self.segment_starts[i]
            a, b, c, d = self.get_polynomial_coefficients(i)
            segments.append((s_offset, a, b, c, d))
        return segments

    def evaluate_with_3d_compatibility(
        self, s: float, derivative: int = 0
    ) -> np.ndarray:
        """
        Evaluate width and return in 3D format for compatibility with existing code.

        Args:
            s: Arc length value
            derivative: Derivative order (only 0 supported)

        Returns:
            3D array [s, width, 0] for compatibility
        """
        if derivative != 0:
            raise NotImplementedError("Derivatives not supported in compatibility mode")

        width = self.evaluate(s, derivative=0)
        return np.array([s, width, 0.0])

    @property
    def total_arc_length(self) -> float:
        """Get total arc length of the reference line."""
        return self.total_length
