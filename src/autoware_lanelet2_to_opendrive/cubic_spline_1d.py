"""1D cubic spline for width as a function of arc length."""

from typing import Tuple

import numpy as np
from scipy.interpolate import CubicSpline

from .spline_1d_base import Spline1DBase


class CubicSpline1D(Spline1DBase):
    """
    Generic 1D cubic spline that maps one variable to another.

    This class creates a proper 1D cubic spline interpolation for any 1D values
    (e.g., width, height, superelevation) and provides methods to get polynomial
    coefficients for each segment.

    Features C2 continuity (continuous second derivative) for maximum smoothness,
    but may produce overshoot/undershoot in some cases.
    """

    def __init__(
        self, arc_lengths: np.ndarray, values: np.ndarray, bc_type: str = "not-a-knot"
    ):
        """
        Initialize a 1D cubic spline.

        Args:
            arc_lengths: Array of arc length values (must be monotonically increasing)
            values: Array of values corresponding to arc_lengths
            bc_type: Boundary condition type for spline ('not-a-knot', 'natural', 'clamped')
                    'not-a-knot' (default) provides smoother interpolation with less oscillation
        """
        # Call parent class constructor (handles validation)
        super().__init__(arc_lengths, values)

        # Store boundary condition type (unique to cubic splines)
        self.bc_type = bc_type

        # Create cubic spline interpolation
        self.spline = CubicSpline(self.arc_lengths, self.values, bc_type=bc_type)

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

        # Get the polynomial piece for this segment
        # scipy's CubicSpline uses a different representation internally
        # We need to extract and convert the coefficients

        # Get the segment start point
        s0 = self.segment_starts[segment_index]

        # Evaluate the polynomial and its derivatives at the start of the segment
        a = self.spline(s0, 0)  # w(s0)
        b = self.spline(s0, 1)  # w'(s0)
        c = self.spline(s0, 2) / 2.0  # w''(s0) / 2!
        d = self.spline(s0, 3) / 6.0  # w'''(s0) / 3!

        # For cubic splines, the third derivative is piecewise constant within each segment
        # We need to get it from the spline's internal representation
        if hasattr(self.spline, "c"):
            # Access scipy's internal coefficient matrix
            # self.spline.c has shape (4, n-1) where n is number of knots
            # Each column represents [d, c, b, a] for that segment
            c_matrix = self.spline.c
            if segment_index < c_matrix.shape[1]:
                # The coefficients are stored in reverse order and for (x - x_i)
                d = c_matrix[0, segment_index]  # Coefficient of (s-s0)^3
                c = c_matrix[1, segment_index]  # Coefficient of (s-s0)^2
                b = c_matrix[2, segment_index]  # Coefficient of (s-s0)
                a = c_matrix[3, segment_index]  # Constant term

        return float(a), float(b), float(c), float(d)
